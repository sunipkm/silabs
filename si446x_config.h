/*
 * Project: Si4463 Radio Library for AVR and Arduino
 * Author: Zak Kemble, contact@zakkemble.co.uk
 * Copyright: (C) 2017 by Zak Kemble
 * License: GNU GPL v3 (see License.txt)
 * Web: http://blog.zakkemble.co.uk/si4463-radio-library-avr-arduino/
 */

#ifndef SI443X_CONFIG_H_
#define SI443X_CONFIG_H_

#define SI446X_WUT_RUN 1  ///< Wake the microcontroller when the WUT expires
#define SI446X_WUT_BATT 2 ///< Take a battery measurement when the WUT expires
#define SI446X_WUT_RX 4   ///< Go into RX mode for LDC time (not supported yet!)

#define SI446X_GPIO_DRV_HIGH		0x00 ///< GPIO drive strength high
#define SI446X_GPIO_DRV_MED_HIGH	0x20 ///< GPIO drive strength medium-high
#define SI446X_GPIO_DRV_MED_LOW		0x40 ///< GPIO drive strength medium-low
#define SI446X_GPIO_DRV_LOW			0x60 ///< GPIO drive strength low

#define SI446X_PROP_GROUP_GLOBAL 0x00       ///< Property group global
#define SI446X_PROP_GROUP_INT 0x01          ///< Property group interrupts
#define SI446X_PROP_GROUP_FRR 0x02          ///< Property group fast response registers
#define SI446X_PROP_GROUP_PREAMBLE 0x10     ///< Property group preamble
#define SI446X_PROP_GROUP_SYNC 0x11         ///< Property group sync
#define SI446X_PROP_GROUP_PKT 0x12          ///< Property group packet config
#define SI446X_PROP_GROUP_MODEM 0x20        ///< Property group modem
#define SI446X_PROP_GROUP_MODEM_CHFLT 0x21  ///< Property group RX coefficients
#define SI446X_PROP_GROUP_PA 0x22           ///< Property group power amp
#define SI446X_PROP_GROUP_SYNTH 0x23        ///< Property group synthesizer
#define SI446X_PROP_GROUP_MATCH 0x30        ///< Property group address match
#define SI446X_PROP_GROUP_FREQ_CONTROL 0x40 ///< Property group frequency control
#define SI446X_PROP_GROUP_RX_HOP 0x50       ///< Property group RX hop
#define SI446X_PROP_GROUP_PTI 0xF0          ///< Property group packet trace interface

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

/**
* @brief NIRQ pin modes (see the Si446x API docs for what they all mean)
*/
typedef enum
{
    SI446X_NIRQ_MODE_DONOTHING = 0x00,
    SI446X_NIRQ_MODE_TRISTATE = 0x01,
    SI446X_NIRQ_MODE_DRIVE0 = 0x02,
    SI446X_NIRQ_MODE_DRIVE1 = 0x03,
    SI446X_NIRQ_MODE_INPUT = 0x04,
    //	SI446X_NIRQ_MODE_32K_CLK	= 0x05,
    //	SI446X_NIRQ_MODE_BOOT_CLK	= 0x06,
    SI446X_NIRQ_MODE_DIV_CLK = 0x07,
    SI446X_NIRQ_MODE_CTS = 0x08,
    //	SI446X_NIRQ_MODE_INV_CTS	= 0x09,
    //	SI446X_NIRQ_MODE_CMD_OVERLAP	= 0x0A,
    SI446X_NIRQ_MODE_SDO = 0x0B,
    SI446X_NIRQ_MODE_POR = 0x0C,
    //	SI446X_NIRQ_MODE_CAL_WUT	= 0x0D,
    //	SI446X_NIRQ_MODE_WUT		= 0x0E,
    SI446X_NIRQ_MODE_EN_PA = 0x0F,
    SI446X_NIRQ_MODE_TX_DATA_CLK = 0x10,
    SI446X_NIRQ_MODE_RX_DATA_CLK = 0x11,
    SI446X_NIRQ_MODE_EN_LNA = 0x12,
    SI446X_NIRQ_MODE_TX_DATA = 0x13,
    SI446X_NIRQ_MODE_RX_DATA = 0x14,
    SI446X_NIRQ_MODE_RX_RAW_DATA = 0x15,
    SI446X_NIRQ_MODE_ANTENNA_1_SW = 0x16,
    SI446X_NIRQ_MODE_ANTENNA_2_SW = 0x17,
    SI446X_NIRQ_MODE_VALID_PREAMBLE = 0x18,
    SI446X_NIRQ_MODE_INVALID_PREAMBLE = 0x19,
    SI446X_NIRQ_MODE_SYNC_WORD_DETECT = 0x1A,
    SI446X_NIRQ_MODE_CCA = 0x1B,
    //	SI446X_NIRQ_MODE_IN_SLEEP		= 0x1C,
    SI446X_NIRQ_MODE_PKT_TRACE = 0x1D,
    // Nothing for 0x1E (30)
    SI446X_NIRQ_MODE_TX_RX_DATA_CLK = 0x1F,
    //	SI446X_NIRQ_MODE_TX_STATE		= 0x20,
    //	SI446X_NIRQ_MODE_RX_STATE		= 0x21,
    //	SI446X_NIRQ_MODE_RX_FIFO_FULL	= 0x22,
    //	SI446X_NIRQ_MODE_TX_FIFO_EMPTY	= 0x23,
    //	SI446X_NIRQ_MODE_LOW_BATT		= 0x24,
    //	SI446X_NIRQ_MODE_CCA_LATCH		= 0x25,
    //	SI446X_NIRQ_MODE_HOPPED			= 0x26,
    SI446X_NIRQ_MODE_NIRQ = 0x27
} si446x_nirq_mode_t;

/**
* @brief SDO pin modes (see the Si446x API docs for what they all mean)
*/
typedef enum
{
    SI446X_SDO_MODE_DONOTHING = 0x00,
    SI446X_SDO_MODE_TRISTATE = 0x01,
    SI446X_SDO_MODE_DRIVE0 = 0x02,
    SI446X_SDO_MODE_DRIVE1 = 0x03,
    SI446X_SDO_MODE_INPUT = 0x04,
    SI446X_SDO_MODE_32K_CLK = 0x05,
    //	SI446X_SDO_MODE_BOOT_CLK	= 0x06,
    SI446X_SDO_MODE_DIV_CLK = 0x07,
    SI446X_SDO_MODE_CTS = 0x08,
    //	SI446X_SDO_MODE_INV_CTS	= 0x09,
    //	SI446X_SDO_MODE_CMD_OVERLAP	= 0x0A,
    SI446X_SDO_MODE_SDO = 0x0B,
    SI446X_SDO_MODE_POR = 0x0C,
    //	SI446X_SDO_MODE_CAL_WUT	= 0x0D,
    SI446X_SDO_MODE_WUT = 0x0E,
    SI446X_SDO_MODE_EN_PA = 0x0F,
    SI446X_SDO_MODE_TX_DATA_CLK = 0x10,
    SI446X_SDO_MODE_RX_DATA_CLK = 0x11,
    SI446X_SDO_MODE_EN_LNA = 0x12,
    SI446X_SDO_MODE_TX_DATA = 0x13,
    SI446X_SDO_MODE_RX_DATA = 0x14,
    SI446X_SDO_MODE_RX_RAW_DATA = 0x15,
    SI446X_SDO_MODE_ANTENNA_1_SW = 0x16,
    SI446X_SDO_MODE_ANTENNA_2_SW = 0x17,
    SI446X_SDO_MODE_VALID_PREAMBLE = 0x18,
    SI446X_SDO_MODE_INVALID_PREAMBLE = 0x19,
    SI446X_SDO_MODE_SYNC_WORD_DETECT = 0x1A,
    SI446X_SDO_MODE_CCA = 0x1B,
    //	SI446X_SDO_MODE_IN_SLEEP		= 0x1C,
    //	SI446X_SDO_MODE_PKT_TRACE		= 0x1D,
    // Nothing for 0x1E (30)
    //	SI446X_SDO_MODE_TX_RX_DATA_CLK	= 0x1F,
    //	SI446X_SDO_MODE_TX_STATE		= 0x20,
    //	SI446X_SDO_MODE_RX_STATE		= 0x21,
    //	SI446X_SDO_MODE_RX_FIFO_FULL	= 0x22,
    //	SI446X_SDO_MODE_TX_FIFO_EMPTY	= 0x23,
    //	SI446X_SDO_MODE_LOW_BATT		= 0x24,
    //	SI446X_SDO_MODE_CCA_LATCH		= 0x25,
    //	SI446X_SDO_MODE_HOPPED			= 0x26,
    //	SI446X_SDO_MODE_HOP_TABLE_WRAP	= 0x27
} si446x_sdo_mode_t;

// If other libraries communicate with SPI devices while inside an interrupt then set this to 1, otherwise you can set this to 0
// If you're not sure then leave this at 1
// If this is 1 then global interrupts will be turned off when this library uses the SPI bus
#define SI446X_INT_SPI_COMMS 1

// ADC Conversion time
// 1 - 15
// RATE  = SYS_CLK / 12 / 2^(SI446X_ADC_SPEED + 1)
// SYS_CLK is usually 30MHz
// 10, 11 or 12 is recommended
// 10 = 0.82ms (1.22KHz)
// 11 = 1.64ms (610Hz)
// 12 = 3.27ms (305Hz)
// A slower conversion gives higher resolution
#define SI446X_ADC_SPEED 10

// Mode to enter when radio is idle
// The radio is put into idle mode when new data is being loaded for transmission, just before starting to receive and once a packet has been received
// This option effects response time to TX/RX mode and current consumption
// NOTE: After receiving an invalid packet the radio will be put into SLEEP mode instead of the option chosen here, this is to fix an issue with INVALID_SYNC causing the radio to get stuck
//
// SI446X_STATE_SPI_ACTIVE
//	Response time: 340us
//	Current consumption: 1.35mA
//
// SI446X_STATE_READY
//	Response time: 100us
//	Current consumption: 1.8mA
#define SI446X_IDLE_MODE SI446X_STATE_READY

// To use variable length packets set this to 0
// Otherwise for fixed length packets this should be set to the length. The len parameter in Si446x_TX() will then be ignored.
// Using fixed length packets will stop the length field from being transmitted, reducing the transmission by 3 bytes.
#define SI446X_FIXED_LENGTH 0

// Interrupt number
// This must match the INT that the NIRQ pin is connected to
#define SI446X_INTERRUPT_NUM	0



#define SI446X_CONCAT(a, b) a ## b
#define SI446X_INTCONCAT(num) SI446X_CONCAT(INT, num)

#ifndef SI446X_REG_EXTERNAL_INT
	#ifdef EIMSK
		#define SI446X_REG_EXTERNAL_INT EIMSK
	#elif defined GICR
		#define SI446X_REG_EXTERNAL_INT GICR
	#else
		#define SI446X_REG_EXTERNAL_INT GIMSK
	#endif
#endif

#ifndef SI446X_BIT_EXTERNAL_INT
	#define SI446X_BIT_EXTERNAL_INT SI446X_INTCONCAT(SI446X_INTERRUPT_NUM)
#endif



// NOT PROPERLY TESTED, KEEP 1
// what happens if:
// 1. Si446x_SERVICE()
// 2. *new packet RX after service()*
// 3. Si446x_TX(blah)
// RX packet is lost, what are the interrupt pending statuses at?
//
// Use pin interrupt
// If this is 1 and you have other devices that use the SPI bus then you will need to wrap areas of code that communicate with those devices with SI446X_NO_INTERRUPT()
// If this is 0 make sure to call Si446x_SERVICE() as often as possible so that the library can process radio events
// 0 = Off, run callbacks from Si446x_SERVICE()
// 1 = On, run callbacks from interrupt
#define SI446X_INTERRUPTS 1 // DO NOT CHANGE


#endif /* SI443X_CONFIG_H_ */
