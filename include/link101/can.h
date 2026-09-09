#ifndef LINK101_CAN_H
#define LINK101_CAN_H

#include <stdint.h>
#include <stdbool.h>

// CAN bus via MCP2518FD (CAN FD controller with SPI interface)
// Pins: RST=GPIO9, SCK=GPIO10, MOSI=GPIO11, MISO=GPIO12, CS=GPIO13
// Uses SPI1 hardware

// CAN frame structure (compatible with gs_usb)
typedef struct {
    uint32_t can_id;    // CAN identifier + flags (bit 31=ERR, bit 30=RTR, bit 29=EFF)
    uint8_t  dlc;       // Data length code (0-8 for classic, 0-64 for FD)
    uint8_t  flags;     // CAN FD flags: BRS, ESI
    uint8_t  reserved[2];
    uint8_t  data[64];  // Data payload (up to 64 bytes for CAN FD)
} can_frame_t;

// CAN ID flags
#define CAN_EFF_FLAG 0x80000000U  // Extended frame format
#define CAN_RTR_FLAG 0x40000000U  // Remote transmission request
#define CAN_ERR_FLAG 0x20000000U  // Error frame
#define CAN_EFF_MASK 0x1FFFFFFFU  // Extended ID mask
#define CAN_SFF_MASK 0x000007FFU  // Standard ID mask

// CAN FD frame flags
#define CANFD_BRS    0x01  // Bit rate switch
#define CANFD_ESI    0x02  // Error state indicator

// Bitrate configuration
typedef struct {
    uint32_t bitrate;       // Nominal bitrate (e.g., 500000)
    uint32_t sample_point;  // Sample point in 0.1% (e.g., 875 = 87.5%)
    // For CAN FD:
    uint32_t data_bitrate;  // Data phase bitrate (e.g., 2000000)
    uint32_t data_sample_point;
} can_bitrate_t;

// Initialize CAN controller
// Returns true on success
bool can_init(void);

// Configure bitrate (call after init, before start)
bool can_set_bitrate(const can_bitrate_t *config);

// Start CAN communication
bool can_start(void);

// Stop CAN communication
void can_stop(void);

// Reset CAN controller
void can_reset(void);

// Transmit a CAN frame
// Returns true if frame was queued successfully
bool can_send(const can_frame_t *frame);

// Receive a CAN frame (non-blocking)
// Returns true if a frame was available
bool can_recv(can_frame_t *frame);

// Check if transmit buffer has space
bool can_tx_ready(void);

// Check if receive buffer has frames
bool can_rx_available(void);

// Get error counters
void can_get_error_counters(uint8_t *tx_err, uint8_t *rx_err);

// Poll task - call from main loop
void can_task(void);

// Start in internal loopback mode (for testing)
bool can_start_loopback(void);

// Dump key registers for debugging  
void can_dump_regs(void);

// Self-test: enter loopback, send a frame, poll for it, print results
bool can_self_test(void);

#endif // LINK101_CAN_H
