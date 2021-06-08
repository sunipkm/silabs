#include <linux/kernel.h>
#include <linux/types.h>
#define SI446X_MAX_PACKET_LEN 128 ///< Maximum packet length

#define SI446X_MAX_TX_POWER 127 ///< Maximum TX power (+20dBm/100mW)

// Raspberry Pi pin assignments
#define SI446X_CSN 15 ///! Chip select pin
#define SI446X_SDN 13 ///! Shutdown pin
#define SI446X_IRQ 11 ///! Receive interrupt

/**
* @brief Data structure for storing chip info from ::si446x_getInfo()
*/
typedef struct
{
    u8 chipRev;   ///< Chip revision
    u16 part;     ///< Part ID
    u8 partBuild; ///< Part build
    u16 id;       ///< ID
    u8 customer;  ///< Customer
    u8 romId;     ///< ROM ID (3 = revB1B, 6 = revC2A)

    u8 revExternal; ///< Revision external
    u8 revBranch;   ///< Revision branch
    u8 revInternal; ///< Revision internal
    u16 patch;      ///< Patch
    u8 func;        ///< Function
} si446x_info_t;

/**
* @brief GPIOs for passing to ::si446x_writeGPIO(), or for masking when reading from ::si446x_readGPIO()
*/
typedef enum
{
    SI446X_GPIO0 = 0, ///< GPIO 1
    SI446X_GPIO1 = 1, ///< GPIO 2
    SI446X_GPIO2 = 2, ///< GPIO 3
    SI446X_GPIO3 = 3, ///< GPIO 4
    SI446X_NIRQ = 4,  ///< NIRQ
    SI446X_SDO = 5    ///< SDO
} si446x_gpio_t;

/**
* @brief Radio states, returned from ::si446x_getState()
*/
typedef enum
{
    SI446X_STATE_NOCHANGE = 0x00,
    SI446X_STATE_SLEEP = 0x01, ///< This will never be returned since SPI activity will wake the radio into ::SI446X_STATE_SPI_ACTIVE
    SI446X_STATE_SPI_ACTIVE = 0x02,
    SI446X_STATE_READY = 0x03,
    SI446X_STATE_READY2 = 0x04,  ///< Will return as ::SI446X_STATE_READY
    SI446X_STATE_TX_TUNE = 0x05, ///< Will return as ::SI446X_STATE_TX
    SI446X_STATE_RX_TUNE = 0x06, ///< Will return as ::SI446X_STATE_RX
    SI446X_STATE_TX = 0x07,
    SI446X_STATE_RX = 0x08
} si446x_state_t;

#ifndef DOXYGEN
static char *si446x_state_str[] = {
    "Si446x State: No change",
    "Si446x State: Sleep",
    "Si446x State: SPI active",
    "Si446x State: Ready",
    "Si446x State: Ready",
    "Si446x State: TX tune",
    "Si446x State: RX tune",
    "Si446x State: TX",
    "Si446x State: RX"};
#endif

enum SI446X_RETVAL
{
    SI446X_ERROR = -1,
    SI446X_TOUT = 0,
    SI446X_SUCCESS = 1,
    SI446X_CRC_INVALID = 2,
};

#ifndef DOXYGEN
static char *si446x_errstr[] = {
    "Si446x: Timed out",
    "Si446x: Success",
    "Si446x: CRC invalid on receiver"};
#endif
