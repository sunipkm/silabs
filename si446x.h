/* SPDX-License-Identifier: GPL-2.0-only */
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
#ifndef _SI446X_H_
#define _SI446X_H_
#ifndef __KERNEL__
#include <stdint.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#else
#include <linux/kernel.h>
#include <linux/types.h>
#endif

#ifndef DRIVER_GPL2 // if GPL2 driver is not needed, since WDS generated config is proprietary
#include "radio_config.h"
#endif

#define SI446X_MAX_PACKET_LEN 0x80 ///< Maximum packet length

#define SI446X_MAX_TX_POWER 0x7f ///< Maximum TX power (+20dBm/100mW)

/**
* @brief Data structure for storing chip info from ::si446x_getInfo()
*/
typedef struct
{
	u8 chip_rev;   ///< Chip revision
	u16 part;      ///< Part ID
	u8 part_build; ///< Part build
	u16 id;	       ///< ID
	u8 customer;   ///< Customer
	u8 rom_id;     ///< ROM ID (3 = revB1B, 6 = revC2A)

	u8 rev_external; ///< Revision external
	u8 rev_branch;	 ///< Revision branch
	u8 rev_internal; ///< Revision internal
	u16 patch;	 ///< Patch
	u8 func;	 ///< Function
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
	SI446X_SDO = 5	  ///< SDO
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

/**
* @brief GPIO pin modes (see the Si446x API docs for what they all mean)
*/
typedef enum
{
	SI446X_GPIO_MODE_DONOTHING = 0x00,
	SI446X_GPIO_MODE_TRISTATE = 0x01,
	SI446X_GPIO_MODE_DRIVE0 = 0x02,
	SI446X_GPIO_MODE_DRIVE1 = 0x03,
	SI446X_GPIO_MODE_INPUT = 0x04,
	SI446X_GPIO_MODE_32K_CLK = 0x05,
	SI446X_GPIO_MODE_BOOT_CLK = 0x06,
	SI446X_GPIO_MODE_DIV_CLK = 0x07,
	SI446X_GPIO_MODE_CTS = 0x08,
	SI446X_GPIO_MODE_INV_CTS = 0x09,
	SI446X_GPIO_MODE_CMD_OVERLAP = 0x0A,
	SI446X_GPIO_MODE_SDO = 0x0B,
	SI446X_GPIO_MODE_POR = 0x0C,
	SI446X_GPIO_MODE_CAL_WUT = 0x0D,
	SI446X_GPIO_MODE_WUT = 0x0E,
	SI446X_GPIO_MODE_EN_PA = 0x0F,
	SI446X_GPIO_MODE_TX_DATA_CLK = 0x10,
	SI446X_GPIO_MODE_RX_DATA_CLK = 0x11,
	SI446X_GPIO_MODE_EN_LNA = 0x12,
	SI446X_GPIO_MODE_TX_DATA = 0x13,
	SI446X_GPIO_MODE_RX_DATA = 0x14,
	SI446X_GPIO_MODE_RX_RAW_DATA = 0x15,
	SI446X_GPIO_MODE_ANTENNA_1_SW = 0x16,
	SI446X_GPIO_MODE_ANTENNA_2_SW = 0x17,
	SI446X_GPIO_MODE_VALID_PREAMBLE = 0x18,
	SI446X_GPIO_MODE_INVALID_PREAMBLE = 0x19,
	SI446X_GPIO_MODE_SYNC_WORD_DETECT = 0x1A,
	SI446X_GPIO_MODE_CCA = 0x1B,
	SI446X_GPIO_MODE_IN_SLEEP = 0x1C,
	SI446X_GPIO_MODE_PKT_TRACE = 0x1D,
	// Nothing for 0x1E (30)
	SI446X_GPIO_MODE_TX_RX_DATA_CLK = 0x1F,
	SI446X_GPIO_MODE_TX_STATE = 0x20,
	SI446X_GPIO_MODE_RX_STATE = 0x21,
	SI446X_GPIO_MODE_RX_FIFO_FULL = 0x22,
	SI446X_GPIO_MODE_TX_FIFO_EMPTY = 0x23,
	SI446X_GPIO_MODE_LOW_BATT = 0x24,
	SI446X_GPIO_MODE_CCA_LATCH = 0x25,
	SI446X_GPIO_MODE_HOPPED = 0x26,
	SI446X_GPIO_MODE_HOP_TABLE_WRAP = 0x27
} si446x_gpio_mode_t;

enum SI446X_IOCTL
{
	SI446X_SET_STATE = 0xa,		 // si446x_state_t state
	SI446X_GET_STATE,		 // si446x_state_t state
	SI446X_GET_LATCHED_RSSI,	 // int16_t rssi
	SI446X_GET_RSSI,		 // int16_t rssi
	SI446X_GET_INFO,		 // si446x_info_t info
	SI446X_DISABLE_WUT,		 // void
	SI446X_SETUP_WUT,		 // struct SI446X_WUT_CONFIG
	SI446X_GET_TX_PWR,		 // uint8_t tx power
	SI446X_SET_TX_PWR,		 // uint8_t tx_power
	SI446X_GET_TEMP,		 // int32_t get temperature
	SI446X_RD_GPIO,			 // uint8_t gpio state
	SI446X_WR_GPIO,			 // struct SI446X_GPIO_CONFIG
	SI446X_SET_LOW_BATT,		 // uint8_t voltage (scale by 1/50, offset by -30)
	SI446X_ADC_BATT,		 // uint16_t voltage (scale by 75/32)
	SI446X_ADC_GPIO,		 // struct SI446X_ADC_GPIO_MEM
	SI446X_SLEEP,			 // void, returns 0 on failure, 1 on success
	SI446X_ADC_CONF,		 // struct SI446X_ADC_CONFIG
	SI446X_RD_RX_BUF_SZ,		 // int
	SI446X_INIT,			 // NULL for default config, or pointer to struct SI446X_INIT_PROPS
	SI446X_DEBUG_TX_PACKETS,	 // Number of packets sent
	SI446X_DEBUG_RX_PACKETS,	 // Number of packets received
	SI446X_DEBUG_RX_CORRUPT_PACKETS, // Number of corrupt packets
	SI446X_SET_ONTX_STATE,		 // State the device goes to after TX, default: Sleep
	SI446X_SET_ONRX_STATE,		 // State the device goes to after RX, default: Sleep
	SI446X_GET_ONTX_STATE,		 // State the device goes to after TX (return value)
	SI446X_GET_ONRX_STATE,		 // State the device goes to after RX (return value)
};

struct SI446X_WUT_CONFIG
{
	unsigned char r;  // r = 1; while (((int) t >> (r++ + 2)) > 1);
	unsigned short m; // m = t * 2^(13 - r)
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
#endif // _SI446X_H_
