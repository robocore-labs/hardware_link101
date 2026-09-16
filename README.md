# hardware_link101

Board support for the RoboCore Link101 (RP2350A). Add it to a bare Pico
project and you have the board: pin map, serial ports over PIO and hardware
UART, the CAN controller, and the NeoPixel strip.

There is no umbrella header — include only what you use. A rev2 board, which
drops the RS485, CAN and NeoPixel hardware, simply never includes those
three; nothing moves between revisions, they are only depopulated.

```cmake
add_subdirectory(path/to/link-libraries/pico_serial       pico_serial)
add_subdirectory(path/to/link-libraries/hardware_link101  hardware_link101)
target_link_libraries(my_firmware hardware_link101)
```

| Header | What |
|---|---|
| `link101/pins.h` | Pin map |
| `link101/pio_serial.h` | Serial port over a PIO state machine, optional TXEN |
| `link101/uart_serial.h` | Serial port over a hardware UART, optional DE |
| `link101/buses.h` | Constructors for the board-fixed buses |
| `link101/neopixel.h` | The six-pixel WS2812 strip |
| `link101/can.h` | MCP2518FD CAN / CAN-FD controller |
| `link101/lsm6dsox.h` | The onboard 6-axis IMU (accel + gyro) |
| `link101/mmc5983.h` | The onboard magnetometer |

## Pins: fixed vs assigned

`pins.h` distinguishes two things, and the distinction is the point.

**Fixed** — something is soldered to this pin. The servo bus (GP7/GP8 with
direction on GP16), the RS485 transceiver (GP0–2), the CAN controller
(GP9–13 on spi1), I²C (GP14/15), the button (GP17), the LED strip (GP18).
These are the same on every board and get `LINK101_PIN_*` macros. The two
I²C addresses the board itself occupies are fixed in the same way, and are
in `pins.h` for the same reason.

**Assigned** — GP3–6 and GP19–26 are broken out to headers. What is on the
other end is a property of the robot, so the application picks and there are
deliberately no macros. The two conventions in use are documented in the
header: `link101-fw-transparent` runs UART bridge channels on GP3–6 and
GP19/20; `link-base101` runs a lidar on GP4/5 and four wheel motors on
GP19–26. Both are valid; neither belongs to the board.

## Serial ports

Bringing one up involves two types, and it is worth being clear about why.

`link101_pio_serial_t` (and `link101_uart_serial_t`) is the **concrete**
port: PIO instance and state machine numbers, or which hardware UART; the
TXEN/DE pin; the RX ring buffer. It is real storage and a real hardware
claim, which is why you declare one as a `static` variable — it has to live
somewhere for as long as the port is open.

[`serial_t`](../pico_serial) is the **interface** — one pointer to a vtable,
nothing else. It is what every driver in this repo actually takes: Feetech,
DDSM, and anything you write yourself. A driver given a `serial_t *` cannot
see whether it is talking to a PIO state machine or a hardware UART, and
does not need to.

So the two types have different jobs and different lifetimes:

```c
static link101_pio_serial_t servo_port;   // the storage: declare it once,
                                          // wherever it will live

serial_t *servo = link101_servo_bus_init(&servo_port, 1000000);
//        ^^^^^^                          ^^^^^^^^^^^
//        the handle you pass around      the storage you handed in

st3215_ping(servo, 1);   // takes serial_t* -- never sees link101_pio_serial_t
```

`link101_servo_bus_init()` takes a pointer to the concrete struct because it
has to initialize that storage (claim state machines, wire up the ring
buffer); it hands back a `serial_t *` because that pointer is all any driver
should ever hold. The cast between them is free — `serial_t` is the first
member of `link101_pio_serial_t`, so the two pointers have the same address;
see `pico_serial`'s ["Implementing one"](../pico_serial#implementing-one) if
you want the mechanics. In practice: keep the concrete type in a `static`
you never touch again after `init()`, and pass the `serial_t *` everywhere
else.

Both concrete types implement `serial_t`, so anything that takes a
`serial_t *` — the Feetech, DDSM and any other driver — works with either.

### Over PIO

```c
#include "link101/buses.h"

static link101_pio_serial_t servo_port;

// Board-fixed: the half-duplex Feetech/Dynamixel bus, 1 Mbaud.
serial_t *servo = link101_servo_bus_init(&servo_port, 1000000);
```

For anything on the breakout headers, give it pins directly. `txen_pin`
drives a transceiver's direction line and is held asserted until the last
stop bit has cleared the shift register; pass `LINK101_NO_TXEN` for a
full-duplex port:

```c
static link101_pio_serial_t port;
serial_t *s = link101_pio_serial_init(&port, /*tx*/ 19, /*rx*/ 20,
                                      LINK101_NO_TXEN, 115200);
```

Make as many as you need — they are ordinary objects. Four wheel motors is
four of them; two general-purpose UART channels is two.

### Over a hardware UART

```c
#include "link101/uart_serial.h"

static uint8_t  rs485_buf[256];
static link101_uart_serial_t rs485_port;

serial_t *rs485 = link101_rs485_init(&rs485_port, 115200,
                                     rs485_buf, sizeof(rs485_buf));
```

or, for a UART on the breakout headers:

```c
static uint8_t lidar_buf[2048];
static link101_uart_serial_t lidar_port;

serial_t *lidar = link101_uart_serial_init(&lidar_port, uart1,
                                           /*tx*/ 4, /*rx*/ 5,
                                           LINK101_NO_DE, 460800,
                                           lidar_buf, sizeof(lidar_buf));
```

The RX ring is caller-owned because the right size depends on the traffic:
a couple of hundred bytes for request/response, a couple of thousand for a
lidar streaming scans. RX is always interrupt-driven, so a slow main loop
cannot overrun the 32-byte hardware FIFO.

## State machines are claimed, not assigned

Every PIO port asks the SDK for a free state machine
(`pio_claim_free_sm_and_add_program_for_gpio_range`) rather than naming a
block. The old firmware hardcoded them by `#define`, which is why the
NeoPixel driver (pio2 SM0) and the fourth wheel motor (pio2 SM0/SM1) could
not exist in the same binary — and why the UART bridge (pio1 SM0–3) and the
front wheels (pio1 SM0–3) collided too. Nothing here needs to know which
block it got.

The budget is still real: 3 blocks × 4 state machines, and each port uses
two. Init returns `NULL` when nothing is free — check it.

## Keeping USB alive

Drivers block waiting for a reply and call `serial_task()` while they wait.
That is where the application keeps everything else running:

```c
static void servo_task(serial_t *s) {
    tud_task();
    link101_pio_serial_task(s);
}
```

No library below the application references TinyUSB.

## NeoPixels

Six pixels on one line. The buffer is caller-owned — a library has no
business calling `malloc`.

```c
#include "link101/neopixel.h"

static link101_rgb_t pixels[LINK101_NEOPIXEL_COUNT];

link101_neopixel_init(LINK101_PIN_NEOPIXELS, pixels,
                      LINK101_NEOPIXEL_COUNT, LINK101_NEOPIXEL_FREQ);

link101_neopixel_set(0, (link101_rgb_t){0, 32, 0});
link101_neopixel_show();
```

**What the pixels mean is yours to decide.** The activity controller from
`link101-fw-transparent` — the one that runs on core 1 and lights a pixel
per bus — stays in that firmware, because `LED_CAN` and `LED_RS485` are
names for a firmware's wiring, not for a board's hardware. `can.c` here
therefore does not blink anything.

`link101_neopixel_show()` blocks for roughly `10 µs × pixels + 500 µs`, and
staggers the updates so all six do not switch at once.

## The onboard IMU

Two chips on the I²C bus, both soldered to the board: an **LSM6DSOX**
(accelerometer + gyroscope) at `0x6B`, and an **MMC5983MA** magnetometer at
`0x30`. Together they are a 9-DOF IMU; neither collides with a BNO055 on the
Qwiic connector at `0x28`.

```c
#include "link101/lsm6dsox.h"
#include "link101/mmc5983.h"

static link101_lsm6dsox_t imu;
static link101_mmc5983_t  mag;

i2c_init(i2c1, 400000);
gpio_set_function(LINK101_PIN_SDA, GPIO_FUNC_I2C);
gpio_set_function(LINK101_PIN_SCL, GPIO_FUNC_I2C);
gpio_pull_up(LINK101_PIN_SDA);
gpio_pull_up(LINK101_PIN_SCL);

link101_lsm6dsox_init(&imu, i2c1, LINK101_LSM6DSOX_I2C_ADDR);
link101_mmc5983_init(&mag, i2c1, LINK101_MMC5983_I2C_ADDR);

float accel[3], gyro[3], field[3];
link101_lsm6dsox_read(&imu, accel, gyro);   // m/s^2 (gravity included), rad/s
link101_mmc5983_read(&mag, field);          // tesla
```

**There is no orientation.** The LSM6DSOX is a raw 6-axis sensor with no
fusion engine, so unlike a BNO055 there is no quaternion to read — fuse on
the host (`imu_filter_madgwick`, `robot_localization`) from the IMU and
magnetometer topics. A firmware publishing `sensor_msgs/Imu` from this
should set `orientation_covariance[0] = -1`, which is how that message says
"no orientation estimate here".

Both drivers poll, and both are configured for it: the IMU free-runs at
104 Hz and the magnetometer measures continuously at 100 Hz with automatic
set/reset, so a read is a register read and never waits on a conversion.
Their interrupt lines go to solder jumpers JP1 and JP2, open from the
factory.

## CAN

MCP2518FD on spi1 at 10 MHz, classic CAN and CAN FD. The interrupt line
(GP9) is polled, not wired to an IRQ, so `can_task()` must be called.

```c
#include "link101/can.h"

can_init();
can_set_bitrate(&(can_bitrate_t){
    .bitrate = 500000, .sample_point = 875,       // 87.5%
    .data_bitrate = 2000000, .data_sample_point = 875,
});
can_start();

can_frame_t tx = { .can_id = 0x123, .dlc = 8, .data = {1,2,3,4,5,6,7,8} };
can_send(&tx);

while (true) {
    can_task();
    can_frame_t rx;
    if (can_recv(&rx)) { /* ... */ }
}
```

`can_id` carries flags in its top bits — `CAN_EFF_FLAG` for 29-bit IDs,
`CAN_RTR_FLAG`, `CAN_ERR_FLAG` — masked with `CAN_EFF_MASK` / `CAN_SFF_MASK`.
`flags` carries `CANFD_BRS` and `CANFD_ESI`. `data` is 64 bytes for FD.

Also available: `can_stop`, `can_reset`, `can_tx_ready`, `can_rx_available`,
`can_get_error_counters`, and `can_start_loopback` / `can_self_test` /
`can_dump_regs` for bring-up.

## What is not here

**`uart_bridge.c`** needed no replacement: it was N PIO UARTs with ring
buffers, which is `link101_pio_serial_t`. Make N of them and hand each to
whatever consumes it.

**The core-1 LED controller** and **the wheel-motor port table** are
application layer, and stayed with the firmware that defines them.
