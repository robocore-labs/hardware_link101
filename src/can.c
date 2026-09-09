#include "link101/can.h"
#include "link101/pins.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <string.h>
#include <stdio.h>

// MCP2518FD SPI commands
#define MCP_CMD_RESET       0x00
#define MCP_CMD_READ        0x03
#define MCP_CMD_WRITE       0x02
#define MCP_CMD_READ_CRC    0x0B
#define MCP_CMD_WRITE_CRC   0x0A
#define MCP_CMD_WRITE_SAFE  0x0C

// MCP2518FD register addresses
#define REG_CiCON           0x000  // CAN Control Register
#define REG_CiNBTCFG        0x004  // Nominal Bit Time Config
#define REG_CiDBTCFG        0x008  // Data Bit Time Config
#define REG_CiTDC           0x00C  // Transmitter Delay Compensation
#define REG_CiTBC           0x010  // Time Base Counter
#define REG_CiTSCON         0x014  // Timestamp Control
#define REG_CiVEC           0x018  // Interrupt Code
#define REG_CiINT           0x01C  // Interrupt Register
#define REG_CiRXIF          0x020  // RX Interrupt Flag
#define REG_CiTXIF          0x024  // TX Interrupt Flag
#define REG_CiRXOVIF        0x028  // RX Overflow Interrupt Flag
#define REG_CiTXATIF        0x02C  // TX Attempt Interrupt Flag
#define REG_CiTXREQ         0x030  // TX Request Register
#define REG_CiTREC          0x034  // TX/RX Error Count
#define REG_CiBDIAG0        0x038  // Bus Diagnostic 0
#define REG_CiBDIAG1        0x03C  // Bus Diagnostic 1
#define REG_CiFIFOBA        0x04C  // FIFO Base Address
#define REG_CiFIFOCON(n)    (0x050 + (n) * 12)  // FIFO Control
#define REG_CiFIFOSTA(n)    (0x054 + (n) * 12)  // FIFO Status
#define REG_CiFIFOUA(n)     (0x058 + (n) * 12)  // FIFO User Address
#define REG_CiFLTCON(n)     (0x1D0 + (n))       // Filter Control (byte access)
#define REG_CiFLTOBJ(n)     (0x1F0 + (n) * 8)   // Filter Object
#define REG_CiMASK(n)       (0x1F4 + (n) * 8)   // Filter Mask
#define REG_OSC             0xE00  // Oscillator Control
#define REG_IOCON           0xE04  // I/O Control
#define REG_CRC             0xE08  // CRC Register
#define REG_ECCCON          0xE0C  // ECC Control
#define REG_ECCSTAT         0xE10  // ECC Status
#define REG_DEVID           0xE14  // Device ID
#define RAM_START           0x400  // Start of RAM
#define RAM_SIZE            2048   // 2KB RAM

// CiCON register bits
#define CiCON_TXBWS_MASK    (0xF << 28)
#define CiCON_ABAT          (1 << 27)  // Abort All Pending TX
#define CiCON_REQOP_MASK    (0x7 << 24)
#define CiCON_REQOP_NORMAL  (0x0 << 24)
#define CiCON_REQOP_SLEEP   (0x1 << 24)
#define CiCON_REQOP_INTLOOP (0x2 << 24)
#define CiCON_REQOP_LISONLY (0x3 << 24)
#define CiCON_REQOP_CONFIG  (0x4 << 24)
#define CiCON_REQOP_EXTLOOP (0x5 << 24)
#define CiCON_REQOP_CANFD   (0x6 << 24)
#define CiCON_REQOP_RESTRICT (0x7 << 24)
#define CiCON_OPMOD_MASK    (0x7 << 21)
#define CiCON_OPMOD_SHIFT   21
#define CiCON_TXQEN         (1 << 20)
#define CiCON_STEF          (1 << 19)  // Store in TX Event FIFO
#define CiCON_SERR2LOM      (1 << 18)
#define CiCON_ESIGM         (1 << 17)
#define CiCON_RTXAT         (1 << 16)
#define CiCON_BRSDIS        (1 << 12)  // Disable BRS
#define CiCON_BUSY          (1 << 11)
#define CiCON_WFT_MASK      (0x3 << 9)
#define CiCON_WAKFIL        (1 << 8)
#define CiCON_PXEDIS        (1 << 6)
#define CiCON_ISOCRCEN      (1 << 5)
#define CiCON_DNCNT_MASK    0x1F

// CiFIFOCON register bits
#define FIFOCON_PLSIZE_MASK (0x7 << 29)
#define FIFOCON_PLSIZE_8    (0x0 << 29)
#define FIFOCON_PLSIZE_12   (0x1 << 29)
#define FIFOCON_PLSIZE_16   (0x2 << 29)
#define FIFOCON_PLSIZE_20   (0x3 << 29)
#define FIFOCON_PLSIZE_24   (0x4 << 29)
#define FIFOCON_PLSIZE_32   (0x5 << 29)
#define FIFOCON_PLSIZE_48   (0x6 << 29)
#define FIFOCON_PLSIZE_64   (0x7 << 29)
#define FIFOCON_FSIZE_MASK  (0x1F << 24)
#define FIFOCON_FSIZE(n)    (((n) - 1) << 24)  // n = 1..32 messages
#define FIFOCON_TXAT_MASK   (0x3 << 21)
#define FIFOCON_TXAT_UNLIM  (0x3 << 21)
#define FIFOCON_TXPRI_MASK  (0x1F << 16)
#define FIFOCON_FRESET      (1 << 10)
#define FIFOCON_TXREQ       (1 << 9)
#define FIFOCON_UINC        (1 << 8)
#define FIFOCON_TXEN        (1 << 7)
#define FIFOCON_RTREN       (1 << 6)
#define FIFOCON_RXTSEN      (1 << 5)
#define FIFOCON_TXATIE      (1 << 4)
#define FIFOCON_RXOVIE      (1 << 3)
#define FIFOCON_TFERFFIE    (1 << 2)
#define FIFOCON_TFHRFHIE    (1 << 1)
#define FIFOCON_TFNRFNIE    (1 << 0)

// CiFIFOSTA register bits
#define FIFOSTA_FIFOCI_MASK (0x1F << 8)
#define FIFOSTA_TXABT       (1 << 7)
#define FIFOSTA_TXLARB      (1 << 6)
#define FIFOSTA_TXERR       (1 << 5)
#define FIFOSTA_TXATIF      (1 << 4)
#define FIFOSTA_RXOVIF      (1 << 3)
#define FIFOSTA_TFERFFIF    (1 << 2)
#define FIFOSTA_TFHRFHIF    (1 << 1)
#define FIFOSTA_TFNRFNIF    (1 << 0)

// OSC register bits
#define OSC_SCLKDIV         (1 << 4)  // System clock divider
#define OSC_CLKODIV_MASK    (0x3 << 5)
#define OSC_PLLRDY          (1 << 8)
#define OSC_OSCRDY          (1 << 10)
#define OSC_OSCDIS          (1 << 2)
#define OSC_PLLEN           (1 << 0)

// SPI configuration
#define CAN_SPI spi1
#define CAN_SPI_FREQ 10000000  // 10 MHz

// FIFO assignments
#define FIFO_TX 1   // TX FIFO index
#define FIFO_RX 2   // RX FIFO index
#define FIFO_TX_SIZE 8   // 8 messages deep
#define FIFO_RX_SIZE 16  // 16 messages deep

// Frame queues (software)
#define TX_QUEUE_SIZE 8
#define RX_QUEUE_SIZE 16

static can_frame_t tx_queue[TX_QUEUE_SIZE];
static volatile uint32_t tx_head = 0;
static volatile uint32_t tx_tail = 0;

static can_frame_t rx_queue[RX_QUEUE_SIZE];
static volatile uint32_t rx_head = 0;
static volatile uint32_t rx_tail = 0;

static bool initialized = false;
static bool running = false;

// RAM addresses for FIFOs (calculated during init)
static uint16_t fifo_tx_addr = 0;
static uint16_t fifo_rx_addr = 0;

//--------------------------------------------------------------------
// SPI Low-Level Functions
//--------------------------------------------------------------------

static inline void cs_select(void) {
    gpio_put(LINK101_PIN_SPI_CS, 0);
}

static inline void cs_deselect(void) {
    gpio_put(LINK101_PIN_SPI_CS, 1);
}

static void mcp_write_byte(uint16_t addr, uint8_t data) {
    uint8_t cmd[4] = {
        (MCP_CMD_WRITE << 4) | ((addr >> 8) & 0x0F),
        addr & 0xFF,
        data,
        0  // padding for alignment
    };
    cs_select();
    spi_write_blocking(CAN_SPI, cmd, 3);
    cs_deselect();
}

static void mcp_write_word(uint16_t addr, uint32_t data) {
    uint8_t cmd[6] = {
        (MCP_CMD_WRITE << 4) | ((addr >> 8) & 0x0F),
        addr & 0xFF,
        (data >> 0) & 0xFF,
        (data >> 8) & 0xFF,
        (data >> 16) & 0xFF,
        (data >> 24) & 0xFF
    };
    cs_select();
    spi_write_blocking(CAN_SPI, cmd, 6);
    cs_deselect();
}

static uint32_t mcp_read_word(uint16_t addr) {
    uint8_t cmd[2] = {
        (MCP_CMD_READ << 4) | ((addr >> 8) & 0x0F),
        addr & 0xFF
    };
    uint8_t data[4];
    cs_select();
    spi_write_blocking(CAN_SPI, cmd, 2);
    spi_read_blocking(CAN_SPI, 0, data, 4);
    cs_deselect();
    return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
}

static void mcp_write_array(uint16_t addr, const uint8_t *data, uint16_t len) {
    uint8_t cmd[2] = {
        (MCP_CMD_WRITE << 4) | ((addr >> 8) & 0x0F),
        addr & 0xFF
    };
    cs_select();
    spi_write_blocking(CAN_SPI, cmd, 2);
    spi_write_blocking(CAN_SPI, data, len);
    cs_deselect();
}

static void mcp_read_array(uint16_t addr, uint8_t *data, uint16_t len) {
    uint8_t cmd[2] = {
        (MCP_CMD_READ << 4) | ((addr >> 8) & 0x0F),
        addr & 0xFF
    };
    cs_select();
    spi_write_blocking(CAN_SPI, cmd, 2);
    spi_read_blocking(CAN_SPI, 0, data, len);
    cs_deselect();
}

static void mcp_reset(void) {
    uint8_t cmd[2] = { 0x00, 0x00 };  // Reset command
    cs_select();
    spi_write_blocking(CAN_SPI, cmd, 2);
    cs_deselect();
    sleep_ms(3);  // Wait for reset
}

//--------------------------------------------------------------------
// Configuration Helpers
//--------------------------------------------------------------------

// Wait for operation mode change
static bool mcp_wait_opmode(uint8_t expected, uint32_t timeout_ms) {
    uint32_t start = time_us_32();
    while ((time_us_32() - start) < (timeout_ms * 1000)) {
        uint32_t cicon = mcp_read_word(REG_CiCON);
        uint8_t opmod = (cicon >> 21) & 0x7;
        if (opmod == expected) return true;
        sleep_us(100);
    }
    return false;
}

// Set operation mode
static bool mcp_set_opmode(uint8_t mode) {
    uint32_t cicon = mcp_read_word(REG_CiCON);
    cicon &= ~CiCON_REQOP_MASK;
    cicon |= ((uint32_t)mode << 24);
    mcp_write_word(REG_CiCON, cicon);
    return mcp_wait_opmode(mode, 100);
}

// Calculate DLC to data length
static uint8_t dlc_to_len(uint8_t dlc) {
    static const uint8_t dlc_len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return (dlc < 16) ? dlc_len[dlc] : 64;
}

// Calculate data length to DLC
static uint8_t len_to_dlc(uint8_t len) {
    if (len <= 8) return len;
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15;
}

//--------------------------------------------------------------------
// Public API
//--------------------------------------------------------------------

bool can_init(void) {
    // Initialize SPI pins
    gpio_init(LINK101_PIN_SPI_CS);
    gpio_set_dir(LINK101_PIN_SPI_CS, GPIO_OUT);
    gpio_put(LINK101_PIN_SPI_CS, 1);  // CS high (inactive)

    // Initialize interrupt pin (for future use)
    gpio_init(LINK101_PIN_CAN_INT);
    gpio_set_dir(LINK101_PIN_CAN_INT, GPIO_IN);
    gpio_pull_up(LINK101_PIN_CAN_INT);

    // Initialize SPI
    spi_init(CAN_SPI, CAN_SPI_FREQ);
    gpio_set_function(LINK101_PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(LINK101_PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(LINK101_PIN_SPI_MISO, GPIO_FUNC_SPI);

    // Software reset (no hardware reset pin available)
    // Rely on software reset command instead
    mcp_reset();

    // Verify device is responding (read OSC register)
    uint32_t osc = mcp_read_word(REG_OSC);
    if ((osc & 0xFF) == 0x00 || (osc & 0xFF) == 0xFF) {
        return false;  // No response
    }

    // Read device ID to verify
    uint32_t devid = mcp_read_word(REG_DEVID);
    // MCP2518FD should return 0x00 in device ID field, revision in upper bits
    // Accept any non-0xFF value as valid
    if (devid == 0xFFFFFFFF) {
        return false;
    }

    // Should already be in config mode after reset, verify
    if (!mcp_wait_opmode(4, 100)) {  // 4 = config mode
        // Try to force config mode
        if (!mcp_set_opmode(4)) {
            return false;
        }
    }

    // Configure oscillator - assume 40MHz crystal, no PLL needed for now
    // Keep default settings after reset
    osc = mcp_read_word(REG_OSC);
    osc &= ~OSC_SCLKDIV;  // Don't divide system clock
    mcp_write_word(REG_OSC, osc);

    // Wait for oscillator ready
    sleep_ms(5);
    osc = mcp_read_word(REG_OSC);
    if (!(osc & OSC_OSCRDY)) {
        // Oscillator not ready - may still work
    }

    // Configure CAN Control register
    uint32_t cicon = mcp_read_word(REG_CiCON);
    cicon &= ~(CiCON_TXBWS_MASK | CiCON_STEF | CiCON_TXQEN);
    cicon |= CiCON_BRSDIS;  // Disable BRS for classic CAN mode by default
    cicon |= CiCON_ISOCRCEN;  // ISO CRC
    mcp_write_word(REG_CiCON, cicon);

    // Configure bit timing for 500kbps @ 40MHz clock
    // Nominal: Tq = 1/(40MHz) = 25ns
    // 500kbps = 2us bit time = 80 Tq
    // Prescaler = 1, so Tq = 25ns
    // Sync=1, Prop+Phase1=63, Phase2=16 -> 1+63+16=80 Tq
    // SJW = 16
    // NBTCFG: BRP=0 (div by 1), TSEG1=62 (63 Tq), TSEG2=15 (16 Tq), SJW=15
    uint32_t nbtcfg = ((15 << 24) |  // SJW-1
                       (15 << 16) |  // TSEG2-1
                       (62 << 8)  |  // TSEG1-1
                       (0 << 0));    // BRP-1
    mcp_write_word(REG_CiNBTCFG, nbtcfg);

    // Data bit timing (for CAN FD - same as nominal for now)
    uint32_t dbtcfg = ((3 << 24) |   // SJW-1
                       (3 << 16) |   // TSEG2-1
                       (14 << 8) |   // TSEG1-1
                       (0 << 0));    // BRP-1
    mcp_write_word(REG_CiDBTCFG, dbtcfg);

    // Transmitter delay compensation
    uint32_t tdc = (1 << 8) | 15;  // Enable TDC, offset = 15
    mcp_write_word(REG_CiTDC, tdc);

    // Set FIFO base address (0 = start of message RAM at SPI addr 0x400)
    mcp_write_word(REG_CiFIFOBA, 0);

    // Configure TX FIFO (FIFO 1)
    // 8 messages, 8 byte payload, TX mode
    uint32_t txfifocon = FIFOCON_PLSIZE_8 |
                         FIFOCON_FSIZE(FIFO_TX_SIZE) |
                         FIFOCON_TXAT_UNLIM |
                         FIFOCON_TXEN;
    mcp_write_word(REG_CiFIFOCON(FIFO_TX), txfifocon);

    // Reset TX FIFO
    txfifocon |= FIFOCON_FRESET;
    mcp_write_word(REG_CiFIFOCON(FIFO_TX), txfifocon);
    txfifocon &= ~FIFOCON_FRESET;
    mcp_write_word(REG_CiFIFOCON(FIFO_TX), txfifocon);

    // Calculate TX FIFO RAM address
    // Message object: 8 bytes header + 8 bytes data = 16 bytes
    fifo_tx_addr = RAM_START;

    // Configure RX FIFO (FIFO 2)
    // 16 messages, 8 byte payload, RX mode (TXEN=0)
    uint32_t rxfifocon = FIFOCON_PLSIZE_8 |
                         FIFOCON_FSIZE(FIFO_RX_SIZE) |
                         FIFOCON_RXTSEN;  // Enable RX timestamps
    mcp_write_word(REG_CiFIFOCON(FIFO_RX), rxfifocon);

    // Reset RX FIFO
    rxfifocon |= FIFOCON_FRESET;
    mcp_write_word(REG_CiFIFOCON(FIFO_RX), rxfifocon);
    rxfifocon &= ~FIFOCON_FRESET;
    mcp_write_word(REG_CiFIFOCON(FIFO_RX), rxfifocon);

    // Calculate RX FIFO RAM address (after TX FIFO)
    // TX FIFO: 8 messages × 16 bytes = 128 bytes
    fifo_rx_addr = RAM_START + (FIFO_TX_SIZE * 16);

    // Configure filter 0 to accept all messages into RX FIFO
    // Filter control: Enable filter 0, point to FIFO 2
    mcp_write_byte(REG_CiFLTCON(0), (FIFO_RX << 0) | (1 << 7));  // F0BP=FIFO_RX, FLTEN=1

    // Filter 0: Match all standard IDs
    mcp_write_word(REG_CiFLTOBJ(0), 0x00000000);  // Match ID 0
    mcp_write_word(REG_CiMASK(0), 0x00000000);    // Mask all bits (accept all)

    // Configure filter 1 to accept extended IDs
    mcp_write_byte(REG_CiFLTCON(1), (FIFO_RX << 0) | (1 << 7));
    mcp_write_word(REG_CiFLTOBJ(1), 0x40000000);  // EXIDE bit set
    mcp_write_word(REG_CiMASK(1), 0x00000000);    // Accept all extended

    // Configure INT0/GPIO0/XSTBY pin as GPIO0 output driving LOW
    // This controls the SN65HVD230 transceiver Rs pin (LOW = normal mode)
    uint32_t iocon = mcp_read_word(REG_IOCON);
    iocon &= ~(1 << 0);   // TRIS0 = 0 (output)
    iocon &= ~(1 << 8);   // LAT0 = 0 (drive LOW)
    iocon |=  (1 << 24);  // PM0 = 1 (GPIO mode, not INT)
    iocon &= ~(1 << 6);   // XSTBYEN = 0 (manual GPIO control)
    mcp_write_word(REG_IOCON, iocon);

    // Clear software queues
    tx_head = tx_tail = 0;
    rx_head = rx_tail = 0;

    initialized = true;
    return true;
}

bool can_set_bitrate(const can_bitrate_t *config) {
    if (!initialized || running) return false;
    if (!config) return false;

    // Must be in config mode
    if (!mcp_set_opmode(4)) return false;

    // Calculate bit timing parameters
    // Assuming 40MHz clock
    const uint32_t fclk = 40000000;
    uint32_t brp, tseg1, tseg2, sjw;

    // Target: bitrate = fclk / (brp * (1 + tseg1 + tseg2))
    // Try to find valid parameters
    uint32_t tq_target = fclk / config->bitrate;

    // Find a good prescaler
    for (brp = 1; brp <= 256; brp++) {
        uint32_t tq_count = tq_target / brp;
        if (tq_count < 8 || tq_count > 385) continue;

        // Calculate segments for target sample point
        // sample_point = (1 + tseg1) / tq_count * 1000
        uint32_t sp_tq = (config->sample_point * tq_count) / 1000;
        tseg1 = sp_tq - 1;
        tseg2 = tq_count - 1 - tseg1;

        if (tseg1 >= 1 && tseg1 <= 256 && tseg2 >= 1 && tseg2 <= 128) {
            sjw = (tseg2 < 128) ? tseg2 : 128;

            uint32_t nbtcfg = (((sjw - 1) << 24) |
                               ((tseg2 - 1) << 16) |
                               ((tseg1 - 1) << 8) |
                               ((brp - 1) << 0));
            mcp_write_word(REG_CiNBTCFG, nbtcfg);
            return true;
        }
    }

    return false;  // Could not find valid timing
}

bool can_start(void) {
    if (!initialized) return false;
    if (running) return true;

    // Enable transceiver right before switching to normal mode (critical timing)
    uint32_t iocon = mcp_read_word(REG_IOCON);
    iocon &= ~(1 << 0);   // TRIS0 = 0 (output)
    iocon &= ~(1 << 8);   // LAT0 = 0 (drive LOW for normal mode)
    iocon |=  (1 << 24);  // PM0 = 1 (GPIO mode, not INT)
    iocon &= ~(1 << 6);   // XSTBYEN = 0 (manual GPIO control)
    mcp_write_word(REG_IOCON, iocon);

    // Small delay to ensure transceiver is ready
    sleep_ms(1);

    // Switch to normal CAN 2.0 mode (mode 6 is CAN FD, mode 0 is CAN 2.0)
    if (!mcp_set_opmode(0)) {  // Normal CAN 2.0 mode
        return false;
    }

    running = true;
    return true;
}

void can_stop(void) {
    if (!initialized) return;

    // Switch back to configuration mode
    mcp_set_opmode(4);
    running = false;
}

void can_reset(void) {
    if (!initialized) return;

    // Software reset only (no hardware reset pin)
    mcp_reset();
    running = false;

    // Re-initialize
    can_init();
}

bool can_send(const can_frame_t *frame) {
    if (!initialized || !running) return false;
    if (!frame) return false;

    // Queue frame in software buffer
    uint32_t next_head = (tx_head + 1) % TX_QUEUE_SIZE;
    if (next_head == tx_tail) {
        return false;  // Queue full
    }

    tx_queue[tx_head] = *frame;
    tx_head = next_head;

    return true;
}

bool can_recv(can_frame_t *frame) {
    if (!initialized || !frame) return false;

    if (rx_head == rx_tail) {
        return false;  // Queue empty
    }

    *frame = rx_queue[rx_tail];
    rx_tail = (rx_tail + 1) % RX_QUEUE_SIZE;

    return true;
}

bool can_tx_ready(void) {
    uint32_t next_head = (tx_head + 1) % TX_QUEUE_SIZE;
    return next_head != tx_tail;
}

bool can_rx_available(void) {
    return rx_head != rx_tail;
}

void can_get_error_counters(uint8_t *tx_err, uint8_t *rx_err) {
    if (!initialized) {
        if (tx_err) *tx_err = 0;
        if (rx_err) *rx_err = 0;
        return;
    }

    uint32_t trec = mcp_read_word(REG_CiTREC);
    if (tx_err) *tx_err = (trec >> 8) & 0xFF;
    if (rx_err) *rx_err = trec & 0xFF;
}

void can_task(void) {
    if (!initialized || !running) return;

    // ----- Transmit pending frames -----
    while (tx_head != tx_tail) {
        // Check if TX FIFO has space
        uint32_t txsta = mcp_read_word(REG_CiFIFOSTA(FIFO_TX));
        if (!(txsta & FIFOSTA_TFNRFNIF)) {
            break;  // TX FIFO full
        }

        // Get the frame to send
        can_frame_t *frame = &tx_queue[tx_tail];

        // Get TX FIFO user address
        uint32_t ua = mcp_read_word(REG_CiFIFOUA(FIFO_TX));
        uint16_t addr = RAM_START + ua;

        // Build TX message object
        // Word 0: ID (T0)
        uint32_t t0;
        if (frame->can_id & CAN_EFF_FLAG) {
            // Extended ID
            t0 = (frame->can_id & CAN_EFF_MASK);
            t0 |= (1 << 29);  // IDE bit
        } else {
            // Standard ID (bits 28:18)
            t0 = (frame->can_id & CAN_SFF_MASK) << 18;
        }
        if (frame->can_id & CAN_RTR_FLAG) {
            t0 |= (1 << 29);  // RTR bit position differs for SID
        }

        // Word 1: Control (T1)
        uint8_t dlc = len_to_dlc(frame->dlc);
        uint32_t t1 = (dlc << 0);  // DLC
        // SEQ field can be used for tracking (optional)

        // Write message object header
        mcp_write_word(addr, t0);
        mcp_write_word(addr + 4, t1);

        // Write data bytes
        uint8_t len = dlc_to_len(dlc);
        if (len > 0 && len <= 8) {
            uint8_t data[8] = {0};
            memcpy(data, frame->data, len);
            mcp_write_array(addr + 8, data, 8);
        }

        // Increment FIFO pointer and request transmission
        uint32_t txcon = mcp_read_word(REG_CiFIFOCON(FIFO_TX));
        txcon |= FIFOCON_UINC | FIFOCON_TXREQ;
        mcp_write_word(REG_CiFIFOCON(FIFO_TX), txcon);

        tx_tail = (tx_tail + 1) % TX_QUEUE_SIZE;
    }

    // ----- Receive frames -----
    while (true) {
        // Check if RX FIFO has messages
        uint32_t rxsta = mcp_read_word(REG_CiFIFOSTA(FIFO_RX));
        if (!(rxsta & FIFOSTA_TFNRFNIF)) {
            break;  // RX FIFO empty
        }

        // Check software queue space
        uint32_t next_head = (rx_head + 1) % RX_QUEUE_SIZE;
        if (next_head == rx_tail) {
            break;  // Software queue full
        }

        // Get RX FIFO user address
        uint32_t ua = mcp_read_word(REG_CiFIFOUA(FIFO_RX));
        uint16_t addr = RAM_START + ua;

        // Read message object header
        uint32_t r0 = mcp_read_word(addr);
        uint32_t r1 = mcp_read_word(addr + 4);

        // Parse ID
        can_frame_t *frame = &rx_queue[rx_head];
        memset(frame, 0, sizeof(*frame));

        bool ide = (r0 >> 29) & 1;  // Extended ID flag
        if (ide) {
            frame->can_id = (r0 & CAN_EFF_MASK) | CAN_EFF_FLAG;
        } else {
            frame->can_id = (r0 >> 18) & CAN_SFF_MASK;
        }

        // RTR flag
        bool rtr = (r1 >> 5) & 1;
        if (rtr) {
            frame->can_id |= CAN_RTR_FLAG;
        }

        // DLC
        uint8_t dlc = r1 & 0x0F;
        frame->dlc = dlc_to_len(dlc);

        // Read data
        if (frame->dlc > 0 && !rtr) {
            uint8_t len = (frame->dlc <= 8) ? frame->dlc : 8;
            mcp_read_array(addr + 8, frame->data, len);
        }

        // Increment FIFO pointer
        uint32_t rxcon = mcp_read_word(REG_CiFIFOCON(FIFO_RX));
        rxcon |= FIFOCON_UINC;
        mcp_write_word(REG_CiFIFOCON(FIFO_RX), rxcon);

        rx_head = next_head;
    }
}

bool can_start_loopback(void) {
    if (!initialized) return false;
    if (running) return true;
    if (!mcp_set_opmode(2)) return false;  // Internal loopback mode
    running = true;
    return true;
}

void can_dump_regs(void) {
    if (!initialized) return;
    uint32_t cicon = mcp_read_word(REG_CiCON);
    uint32_t osc = mcp_read_word(REG_OSC);
    uint32_t devid = mcp_read_word(REG_DEVID);
    uint32_t nbtcfg = mcp_read_word(REG_CiNBTCFG);
    uint32_t trec = mcp_read_word(REG_CiTREC);
    uint32_t txsta = mcp_read_word(REG_CiFIFOSTA(FIFO_TX));
    uint32_t rxsta = mcp_read_word(REG_CiFIFOSTA(FIFO_RX));
    uint32_t txcon = mcp_read_word(REG_CiFIFOCON(FIFO_TX));
    uint32_t rxcon = mcp_read_word(REG_CiFIFOCON(FIFO_RX));
    uint32_t txua = mcp_read_word(REG_CiFIFOUA(FIFO_TX));
    uint32_t rxua = mcp_read_word(REG_CiFIFOUA(FIFO_RX));

    printf("  DEVID:    0x%08lX\n", devid);
    printf("  OSC:      0x%08lX\n", osc);
    printf("  CiCON:    0x%08lX  (OPMOD=%lu)\n", cicon, (cicon >> 21) & 0x7);
    printf("  CiNBTCFG: 0x%08lX\n", nbtcfg);
    printf("  CiTREC:   0x%08lX  (TXerr=%lu RXerr=%lu)\n", trec, (trec >> 8) & 0xFF, trec & 0xFF);
    printf("  TX FIFO:  CON=0x%08lX  STA=0x%08lX  UA=0x%04lX\n", txcon, txsta, txua);
    printf("  RX FIFO:  CON=0x%08lX  STA=0x%08lX  UA=0x%04lX\n", rxcon, rxsta, rxua);
}

bool can_self_test(void) {
    if (!initialized) {
        printf("  CAN not initialized\n");
        return false;
    }

    // Stop if running, enter loopback
    if (running) { mcp_set_opmode(4); running = false; }
    tx_head = tx_tail = 0;
    rx_head = rx_tail = 0;

    printf("  Entering loopback mode...\n");
    if (!mcp_set_opmode(2)) {
        printf("  FAIL: could not enter loopback mode\n");
        return false;
    }
    running = true;
    printf("  OPMOD = %lu\n", (mcp_read_word(REG_CiCON) >> 21) & 0x7);

    // Send a test frame using the software queue
    can_frame_t test_frame = {
        .can_id = 0x123,
        .dlc = 2,
        .data = {0xAA, 0xBB, 0, 0, 0, 0, 0, 0}
    };

    if (!can_send(&test_frame)) {
        printf("  FAIL: could not queue test frame\n");
        mcp_set_opmode(4);
        running = false;
        return false;
    }

    // Process the frame through hardware
    can_task();
    sleep_ms(10);  // Allow time for loopback

    // Check if we received it back
    can_frame_t rx_frame;
    if (can_recv(&rx_frame)) {
        if (rx_frame.can_id == 0x123 && rx_frame.dlc == 2 && 
            rx_frame.data[0] == 0xAA && rx_frame.data[1] == 0xBB) {
            printf("  PASS: loopback test OK\n");
            mcp_set_opmode(4);
            running = false;
            return true;
        } else {
            printf("  FAIL: data mismatch (ID=0x%03lX, data=%02X%02X)\n",
                   rx_frame.can_id & CAN_SFF_MASK, rx_frame.data[0], rx_frame.data[1]);
        }
    } else {
        printf("  FAIL: no frame received in loopback\n");
    }

    mcp_set_opmode(4);
    running = false;
    return false;
}
