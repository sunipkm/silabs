/**
 * @file si446x.h
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief 
 * @version 1.0a
 * @date 2021-06-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _SI446X_H
#define _SI446X_H
#ifndef __KERNEL__
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#else
#include <linux/kernel.h>
#include <linux/types.h>
#endif

#include "radio_config.h"

#define SI446X_MAX_PACKET_LEN 128 ///< Maximum packet length

#define SI446X_MAX_TX_POWER 127 ///< Maximum TX power (+20dBm/100mW)

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

enum SI446X_IOCTL
{
    SI446X_SET_STATE = 0xa,  // si446x_state_t state
    SI446X_GET_STATE,        // si446x_state_t state
    SI446X_GET_LATCHED_RSSI, // int16_t rssi
    SI446X_GET_RSSI,         // int16_t rssi
    SI446X_GET_INFO,         // si446x_info_t info
    SI446X_DISABLE_WUT,      // void
    SI446X_SETUP_WUT,        // struct SI446X_WUT_CONFIG
    SI446X_GET_TX_PWR,       // uint8_t tx power
    SI446X_SET_TX_PWR,       // uint8_t tx_power
    SI446X_GET_TEMP,         // int32_t get temperature
    SI446X_RD_GPIO,          // uint8_t gpio state
    SI446X_WR_GPIO,          // struct SI446X_GPIO_CONFIG
    SI446X_SET_LOW_BATT,     // uint16_t voltage
    SI446X_ADC_BATT,         // uint16_t voltage
    SI446X_ADC_GPIO,         // struct SI446X_ADC_GPIO_MEM
    SI446X_SLEEP,            // void
    SI446X_ADC_CONF,         // struct SI446X_ADC_CONFIG
    SI446X_RD_RX_BUF_SZ,     // int
    SI446X_INIT              // NULL for default config, or pointer to struct SI446X_INIT_PROPS
};

struct SI446X_WUT_CONFIG
{
    unsigned char r;
    unsigned short m;
    unsigned char ldc;
    unsigned char config;
};

struct SI446X_GPIO_CONFIG
{
    si446x_gpio_t pin;
    unsigned char val;
};

struct SI446X_ADC_GPIO_MEM
{
    unsigned char pin;
    unsigned short val;
};

struct SI446X_ADC_CONFIG
{
    unsigned char en;
    unsigned char cfg;
    unsigned char part;
    unsigned short ret;
};

struct SI446X_INIT_PROPS
{
    u32 len;      // length of config array
    void *config; // pointer to config array
};

/**
 * @brief Convert result of SI446X_GET_TEMP from int to float temperature in Celcius
 * 
 */
#define SI446X_CONVERT_TEMP(x) ((899.0 / 4096.0) * x - 293)
#endif // _SI446X_H
