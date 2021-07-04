/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * @file si446x.c
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief 
 * @version 1.0a
 * @date 2021-06-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/circ_buf.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/poll.h>

#define SI446X_DRIVER_GPL2

#include "si446x_kern.h"
#include "si446x_config.h"
#include "si446x_defs.h"
#include "si446x.h"

static int si446x_buffer_len = 16;
module_param(si446x_buffer_len, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(si446x_buffer_len, "RX buffer length. Default: 16 units (of 128 Bytes), minimum 128 bytes (1 unit), maximum 16 KiB (128 units)");

#define DRV_NAME "si446x"
#define DRV_VERSION "1.0b"
#define DEVICE_NAME "ttyUHF"

// Max devices to be probed by this driver
#ifndef SI446X_MAX_DEVICES
#define SI446X_MAX_DEVICES 5
#endif

#if SI446X_MAX_DEVICES <= 0
#undef SI446X_MAX_DEVICES
#define SI446X_MAX_DEVICES 1
#endif

// Default TX/RX finish state
#ifndef SI446X_FINISH_STATE
#define SI446X_FINISH_STATE SI446X_STATE_SLEEP
#endif

// Enable debug callback functions
#ifndef SI446X_DEBUG_CB
#define SI446X_DEBUG_CB 0
#endif

static dev_t device_num = 0; // major number
static dev_t minor_ct = 0;   // minor count

static struct class *si446x_class;

struct si446x
{
	struct cdev serdev;	      // char device
	struct spi_device *spibus;    // spi bus
	struct work_struct irq_work;  // IRQ handler
	struct work_struct init_work; // INIT handler
	int sdn_pin;		      // sdn pin
	int nirq_pin;		      // IRQ pin
	int cs_pin;		      // chip select gpio
	int tx_pk_ctr;		      // stat for tx packets
	int rx_pk_ctr;		      // stat for rx packets
	int rx_corrupt_ctr;	      // stat for corrupt rx packets
	bool device_avail;	      // Device available flag
	unsigned int isr_state;	      // ISR lock state
	struct mutex isr_lock;	      // mutex lock to prevent ISR from running
	bool isr_busy;		      // indicate ISR is busy
	u8 *config;		      // initialize as RADIO_CONFIGURATION_DATA_ARRAY
	u32 config_len;		      // length of RADIO_CONFIGURATION_DATA_ARRAY
	u8 enabledInterrupts[3];      // enabled interrupts
	s16 rssi;		      // RSSI value returned by ioctl
	struct circ_buf *rxbuf;	      // RX data buffer
	int rxbuf_len;		      // rx buffer length
	bool data_available;	      // indicate RX
	int open_ctr;		      // module use counter
	bool initd;		      // indicate initialization
	dev_t this_dev;		      // this device
	struct completion initq;      // init queue
	struct wait_queue_head rxq;   // RX queue
	si446x_state_t on_tx_state;   // State after TX
	si446x_state_t on_rx_state;   // State after RX
	/**
	 * @brief Mode to enter when radio is idle.
	 * The radio is put into idle mode when new data is being loaded for transmission,
	 * just before starting receiver and after receiving a packet. This option affects
	 * response time to TX/RX mode and power consumption.
	 * 
	 * NOTE: After receiving an invalid packet, the radio can be put into sleep mode
	 * instead of the option chosen here, depending on the SI446X_SLEEP_ON_INVALID option.
	 * Putting the radio into sleep mode temporarily fixes an issue with INVALID_SYNC
	 * causing the radio to lock up.
	 * 
	 * SI446X_STATE_SPI_ACTIVE:
	 * Response time: 340 us
	 * Current consumption: 1.35 mA 
	 * 
	 * SI446X_STATE_READY:
	 * Response time: 100 us
	 * Current consumption: 1.8 mA
	 * 
     	 */
	si446x_state_t SI446X_IDLE_MODE;
	/**
	 * @brief 
	 * of the interrupt handler. This option is disabled by default, and can be manipulated by
	 * performing an ioctl call with SI446X_SLEEP_ON_INVALID command, and an int as a parameter.
	 * If the integer is NULL or 0, sleep_on_invalid is false; else it is set to true.
	 * 
	 */
	bool sleep_on_invalid;
	/**
	 * @brief WUT timeout counter. Incremented at WUT interrupt. Does not trigger POLL.
	 * 
	 */
	int wut_counter;
	/**
	 * @brief Low battery indicator. Cleared by the ioctl call. Does not trigger POLL.
	 * 
	 */
	bool lowbatt;
};

static inline int interrupt_off(struct si446x *dev)
{
	int isr_state;
	bool isr_busy;
	isr_busy = READ_ONCE(dev->isr_busy); // check if ISR is busy first
	if (!isr_busy)
	{
		isr_state = READ_ONCE(dev->isr_state);
		WRITE_ONCE(dev->isr_state, isr_state + 1);
		if (isr_state == 0)
			mutex_lock(&(dev->isr_lock)); // prevent ISR from running
	}
	return 1;
}

static inline int interrupt_on(struct si446x *dev)
{
	int isr_state;
	bool isr_busy;
	isr_busy = READ_ONCE(dev->isr_busy); // check if ISR is busy first
	if (!isr_busy)
	{
		isr_state = READ_ONCE(dev->isr_state);
		if (isr_state > 0)
			WRITE_ONCE(dev->isr_state, isr_state - 1);
		isr_state = READ_ONCE(dev->isr_state);
		if (isr_state == 0)
			mutex_unlock(&(dev->isr_lock)); // Allow ISR to run
	}
	return 0;
}

void microsleep(int us)
{
	int ms;
	if (us < 0)
		return;
	ms = us / 1000;
	us = us % 1000;
	if (us > 0)
		udelay(us);
	if (ms > 0)
		msleep(ms);
}

// default callbacks
#if SI446X_DEBUG_CB == 0
#define si446x_timeout_cb(x) \
	{                    \
		(void)x;     \
	}
#define si446x_rxinvalid_cb(x, y) \
	{                         \
		(void)x;          \
	}
#define si446x_sent_cb(x) \
	{                 \
		(void)x;  \
	}
void si446x_wut_cb(struct si446x *dev)
{
	dev->wut_counter++;
}
void si446x_lowbatt_cb(struct si446x *dev)
{
	WRITE_ONCE(dev->lowbatt, true);
}
#else
void si446x_timeout_cb(struct si446x *dev)
{
	printk(KERN_DEBUG DRV_NAME ": si446x_wait_response timed out\n");
}
void si446x_rxinvalid_cb(struct si446x *dev, s16 rssi)
{
	printk(KERN_INFO DRV_NAME ": RX invalid, rssi 0x%04x\n", rssi);
}
void si446x_sent_cb(struct si446x *dev)
{
	printk(KERN_INFO DRV_NAME ": Packet sent\n");
}
void si446x_wut_cb(struct si446x *dev)
{
	dev->wut_counter++;
	printk(KERN_INFO DRV_NAME ": WUT timeout occurred\n");
}
void si446x_lowbatt_cb(struct si446x *dev)
{
	WRITE_ONCE(dev->lowbatt, true);
	printk(KERN_INFO DRV_NAME ": Low battery reported\n");
}
#endif

static unsigned char si446x_get_response(struct si446x *dev, void *buff, unsigned char len)
{
	u8 cts;
	int ret;
	u8 *dout = (u8 *)kmalloc(len + 2, GFP_NOWAIT); // 1 for cmd, 1 for cts
	u8 *din = (u8 *)kzalloc(len + 2, GFP_NOWAIT);  // no need to memset input
	struct spi_transfer xfer = {
	    .tx_buf = dout,
	    .rx_buf = din,
	    .len = len + 2,
	    .bits_per_word = 8};
	struct spi_device *spi;
	// u8 i;
	cts = 0;
	spi = dev->spibus;

	// set command
	memset(dout, 0xff, len + 2);
	dout[0] = SI446X_CMD_READ_CMD_BUFF;

	// SPI transfer, synchronous
	ret = spi_sync_transfer(spi, &xfer, 1);

	if (ret != 0)
	{
		printk(KERN_ERR DRV_NAME
		       "Error in spi transaction si446x_get_response, retcode %d\n",
		       ret);
		goto cleanup;
	}
	cts = din[1];
	if (cts)
	{
		// memcpy
		memcpy(buff, din + 2, len);
	}

cleanup:
	kfree(dout);
	kfree(din);
	return cts;
}

static u8 si446x_wait_response(struct si446x *dev, void *out, u8 outLen,
			       bool useTimeout)
{
	// With F_CPU at 8MHz and SPI at 4MHz each check takes about 7us + 10us delay
	u16 timeout = 40000;
	while (!si446x_get_response(dev, out, outLen))
	{
		microsleep(10);
		if (useTimeout && !--timeout)
		{
			si446x_timeout_cb(dev);
			return 0;
		}
	}
	return 1;
}

static void spi_write_buf(struct si446x *dev, void *out, u8 len)
{
	struct spi_device *spi;
	int ret;
	struct spi_transfer tx = {
	    .tx_buf = out,
	    .rx_buf = NULL,
	    .len = len,
	    .bits_per_word = 8};
	spi = dev->spibus;

	ret = spi_sync_transfer(spi, &tx, 1);
	if (ret != 0)
	{
		printk(KERN_ERR DRV_NAME
		       "Error in spi buffer write si446x_do_api, retcode %d\n",
		       ret);
	}
}

static void si446x_do_api(struct si446x *dev, void *data, u8 len, void *out,
			  u8 outLen)
{
	int ret;
	ret = interrupt_off(dev);
	{
		if (si446x_wait_response(dev, NULL, 0,
					 1)) // Make sure it's ok to send a command
		{
			spi_write_buf(dev, data, len);
			if (((u8 *)data)[0] ==
			    SI446X_CMD_IRCAL) // If we're doing an IRCAL then wait for its completion without a timeout since it can sometimes take a few seconds
				si446x_wait_response(dev, NULL, 0, 0);
			else if (out !=
				 NULL) // If we have an output buffer then read command response into it
				si446x_wait_response(dev, out, outLen, 1);
		}
	}
	ret = interrupt_on(dev);
}

// Configure a bunch of properties (up to 12 properties in one go)
static void si446x_set_props(struct si446x *dev, u16 prop, void *values, u8 len)
{
	// len must not be greater than 12

	u8 data[16] = {
	    SI446X_CMD_SET_PROPERTY,
	    (u8)(prop >> 8),
	    len,
	    (u8)prop};

	// Copy values into data, starting at index 4
	memcpy(data + 4, values, len);

	si446x_do_api(dev, data, len + 4, NULL, 0);
}

// Set a single property
static inline void set_property(struct si446x *dev, u16 prop, u8 value)
{
	si446x_set_props(dev, prop, &value, 1);
}

// Read a bunch of properties
static void si446x_get_props(struct si446x *dev, u16 prop, void *values, u8 len)
{
	u8 data[] = {
	    SI446X_CMD_GET_PROPERTY,
	    (u8)(prop >> 8),
	    len,
	    (u8)prop};

	si446x_do_api(dev, data, sizeof(data), values, len);
}

// Read a single property
static inline u8 get_property(struct si446x *dev, u16 prop)
{
	u8 val;
	si446x_get_props(dev, prop, &val, 1);
	return val;
}

#define IRQ_PACKET 0
#define IRQ_MODEM 1
#define IRQ_CHIP 2

// Set up callback (IRQ)
void si446x_setup_callback(struct si446x *dev, u16 callbacks, u8 state)
{

	int ret = interrupt_off(dev);
	{
		u8 data[2];
		si446x_get_props(dev, SI446X_INT_CTL_PH_ENABLE, data, sizeof(data));

		if (state)
		{
			data[0] |= callbacks >> 8;
			data[1] |= callbacks;
		}
		else
		{
			data[0] &= ~(callbacks >> 8);
			data[1] &= ~callbacks;
		}

		// always keep RXBEGIN enabled, what about RXCOMPLETE and RXINVALID?
		data[0] |= SI446X_RXBEGIN_CB >> 8;
		data[1] |= SI446X_RXBEGIN_CB;

		dev->enabledInterrupts[IRQ_PACKET] = data[0];
		dev->enabledInterrupts[IRQ_MODEM] = data[1];
		si446x_set_props(dev, SI446X_INT_CTL_PH_ENABLE, data, sizeof(data));
	}
	ret = interrupt_on(dev);
}

// Do an ADC conversion
static u16 si446x_get_adc(struct si446x *dev, u8 adc_en, u8 adc_cfg, u8 part)
{
	u8 data[6] = {
	    SI446X_CMD_GET_ADC_READING,
	    adc_en,
	    adc_cfg};
	si446x_do_api(dev, data, 3, data, 6);
	return (data[part] << 8 | data[part + 1]);
}

// Read a fast response register
static u8 si446x_get_frr(struct si446x *dev, u8 reg)
{
	u8 frr = 0;

	{
		struct spi_device *spi;
		int ret;
		u8 dout[2], din[2];
		struct spi_transfer xfer = {
		    .tx_buf = dout,
		    .rx_buf = din,
		    .len = 2,
		    .bits_per_word = 8};

		dout[0] = reg;
		dout[1] = 0xff;
		din[0] = 0;
		din[1] = 0;
		spi = dev->spibus;

		ret = spi_sync_transfer(spi, &xfer, 1);
		if (ret != 0)
		{
			printk(KERN_ERR DRV_NAME
			       "Error in spi buffer write si446x_get_frr, retcode %d\n",
			       ret);
		}
		frr = din[1];
	}
	return frr;
}

// Get device info
void si446x_get_info(struct si446x *dev, si446x_info_t *info)
{
	u8 out[1];
	u8 data[8];
	memset(data, 0x0, sizeof(data));
	out[0] = SI446X_CMD_PART_INFO;
	si446x_do_api(dev, out, 1, data, 8);

	info->chip_rev = data[0];
	info->part = (data[1] << 8) | data[2];
	info->part_build = data[3];
	info->id = (data[4] << 8) | data[5];
	info->customer = data[6];
	info->rom_id = data[7];

	out[0] = SI446X_CMD_FUNC_INFO;
	si446x_do_api(dev, out, 1, data, 6);

	info->rev_external = data[0];
	info->rev_branch = data[1];
	info->rev_internal = data[2];
	info->patch = (data[3] << 8) | data[4];
	info->func = data[5];
}

// Get the latched RSSI from the beginning of the packet
static s16 si446x_get_latched_rssi(struct si446x *dev)
{
#define rssi_dBm(val) ((val / 2) - 134)
	u8 frr;
	s16 rssi;
	frr = si446x_get_frr(dev, SI446X_CMD_READ_FRR_A);
	rssi = rssi_dBm(frr);
	return rssi;
}

// Get current RSSI
static s16 si446x_get_rssi(struct si446x *dev)
{
	s8 data[3] = {
	    SI446X_CMD_GET_MODEM_STATUS,
	    0xFF};
	s16 rssi;
	si446x_do_api(dev, data, 2, data, 3);
	rssi = rssi_dBm(data[2]);
	return rssi;
}

// Get current radio state
static si446x_state_t si446x_get_state(struct si446x *dev)
{
	u8 state;
	state = si446x_get_frr(dev, SI446X_CMD_READ_FRR_B);
	if (state == SI446X_STATE_TX_TUNE)
		state = SI446X_STATE_TX;
	else if (state == SI446X_STATE_RX_TUNE)
		state = SI446X_STATE_RX;
	else if (state == SI446X_STATE_READY2)
		state = SI446X_STATE_READY;
	return (si446x_state_t)state;
}

// Set new state
static void si446x_set_state(struct si446x *dev, si446x_state_t newState)
{
	u8 data[] = {
	    SI446X_CMD_CHANGE_STATE,
	    newState};
	si446x_do_api(dev, data, sizeof(data), NULL, 0);
}

// Clear RX and TX FIFOs
static void si446x_clear_fifo(struct si446x *dev)
{
	static const u8 clearFifo[] = {
	    SI446X_CMD_FIFO_INFO,
	    SI446X_FIFO_CLEAR_RX | SI446X_FIFO_CLEAR_TX};
	si446x_do_api(dev, (u8 *)clearFifo, sizeof(clearFifo), NULL, 0);
}

static void si446x_read_irq_regs(struct si446x *dev, void *buff)
{
	u8 data = SI446X_CMD_GET_INT_STATUS;
	si446x_do_api(dev, &data, sizeof(data), buff, 8);
}

// Similar to si446x_read_irq_regs() but with the option of not clearing certain interrupt flags
static void si446x_read_irq_noclr(struct si446x *dev, void *buff, u8 clear_ph, u8 clear_modem, u8 clear_chip)
{
	u8 data[] = {
	    SI446X_CMD_GET_INT_STATUS,
	    clear_ph,
	    clear_modem,
	    clear_chip};
	si446x_do_api(dev, data, sizeof(data), buff, 8);
}

// Reset the RF chip
static void si446x_reset_device(struct si446x *dev)
{
	if (gpio_is_valid(dev->sdn_pin))
	{
		gpio_set_value_cansleep(dev->sdn_pin, 1);
		msleep(50);
		gpio_set_value_cansleep(dev->sdn_pin, 0);
		msleep(50);
	}
	else
		printk(KERN_ERR DRV_NAME "SDN GPIO %d invalid\n", dev->sdn_pin);
}

// Apply the radio configuration
static void si446x_apply_startup_config(struct si446x *dev)
{
	u8 buff[17];
	u16 i;
	for (i = 0; i < dev->config_len; i++)
	{
		memcpy(buff, &(dev->config[i]), sizeof(buff));
		si446x_do_api(dev, &buff[1], buff[0], NULL, 0);
		i += buff[0];
	}
}

// Put radio to sleep mode
u8 si446x_sleep(struct si446x *dev)
{
	if (si446x_get_state(dev) == SI446X_STATE_TX)
		return 0;
	si446x_set_state(dev, SI446X_STATE_SLEEP);
	return 1;
}

// Read ADC GPIO pin
u16 si446x_adc_gpio(struct si446x *dev, u8 pin)
{
	u16 result = si446x_get_adc(dev, SI446X_ADC_CONV_GPIO | pin, (SI446X_ADC_SPEED << 4) | SI446X_ADC_RANGE_3P6, 0);
	return result;
}

// Read ADC battery voltage (scale by 75/32)
u16 si446x_adc_battery(struct si446x *dev)
{
	u16 result = si446x_get_adc(dev, SI446X_ADC_CONV_BATT, (SI446X_ADC_SPEED << 4), 2);
	return result;
}

/**
 * @brief Get ADC temperature
 * 
 * @param dev 
 * @return s32 Scale by (899/4096) and subtract 293
 */
s32 si446x_adc_temperature(struct si446x *dev)
{
	s32 result = si446x_get_adc(dev, SI446X_ADC_CONV_TEMP, (SI446X_ADC_SPEED << 4), 4);
	return result;
}

// Set GPIO pin
void si446x_write_gpio(struct si446x *dev, si446x_gpio_t pin, u8 value)
{
	u8 data[] = {
	    SI446X_CMD_GPIO_PIN_CFG,
	    SI446X_GPIO_MODE_DONOTHING,
	    SI446X_GPIO_MODE_DONOTHING,
	    SI446X_GPIO_MODE_DONOTHING,
	    SI446X_GPIO_MODE_DONOTHING,
	    SI446X_NIRQ_MODE_DONOTHING,
	    SI446X_SDO_MODE_DONOTHING,
	    SI446X_GPIO_DRV_HIGH};
	data[pin + 1] = value;
	si446x_do_api(dev, data, sizeof(data), NULL, 0);
}

// Read GPIO pins
u8 si446x_read_gpio(struct si446x *dev)
{
	u8 states;
	u8 data[4] = {
	    SI446X_CMD_GPIO_PIN_CFG};
	si446x_do_api(dev, data, 1, data, sizeof(data));
	states = data[0] >> 7 | (data[1] & 0x80) >> 6 | (data[2] & 0x80) >> 5 | (data[3] & 0x80) >> 4;
	return states;
}

// Set TX power
void si446x_set_tx_power(struct si446x *dev, u8 pwr)
{
	set_property(dev, SI446X_PA_PWR_LVL, pwr);
}

// Get TX power
u8 si446x_get_tx_power(struct si446x *dev)
{
	u8 pwr;
	pwr = get_property(dev, SI446X_PA_PWR_LVL);
	return pwr;
}

// Set low battery voltage (scale u16 voltage by 1/50, offset by 30)
void si446x_set_low_batt(struct si446x *dev, u8 batt)
{
	set_property(dev, SI446X_GLOBAL_LOW_BATT_THRESH, batt);
}

// Set up WUT
void si446x_setup_wut(struct si446x *dev, u8 r, u16 m, u8 ldc, u8 config)
{
	u8 do_run, do_batt, do_rx, int_chip, properties[5];
	if (!(config & (SI446X_WUT_RUN | SI446X_WUT_BATT | SI446X_WUT_RX)))
		return;
	interrupt_off(dev);
	set_property(dev, SI446X_GLOBAL_WUT_CONFIG, 0);

	do_run = !!(config & SI446X_WUT_RUN);
	do_batt = !!(config & SI446X_WUT_BATT);
	do_rx = !!(config & SI446X_WUT_RX);

	int_chip = 0;

	int_chip |= do_batt << SI446X_INT_CTL_CHIP_LOW_BATT_EN;
	int_chip |= do_run << SI446X_INT_CTL_CHIP_WUT_EN;
	dev->enabledInterrupts[IRQ_CHIP] = int_chip;
	set_property(dev, SI446X_INT_CTL_CHIP_ENABLE, int_chip);

	// set WUT clock source to internal 32 kHz RC
	if (get_property(dev, SI446X_GLOBAL_CLK_CFG) != SI446X_DIVIDED_CLK_32K_SEL_RC)
	{
		set_property(dev, SI446X_GLOBAL_CLK_CFG, SI446X_DIVIDED_CLK_32K_SEL_RC);
		microsleep(300); // Need to wait 300us for clock source to stabilize, see GLOBAL_WUT_CONFIG:WUT_EN info
	}

	// setup wdt
	properties[0] = do_rx ? SI446X_GLOBAL_WUT_CONFIG_WUT_LDC_EN_RX : 0;
	properties[0] |= do_batt << SI446X_GLOBAL_WUT_CONFIG_WUT_LBD_EN;
	properties[0] |= (1 << SI446X_GLOBAL_WUT_CONFIG_WUT_EN);
	properties[1] = m >> 8;
	properties[2] = m;
	properties[3] = r | SI446X_LDC_MAX_PERIODS_TWO | (1 << SI446X_WUT_SLEEP);
	properties[4] = ldc;

	si446x_set_props(dev, SI446X_GLOBAL_WUT_CONFIG, properties, sizeof(properties));

	interrupt_on(dev);
}

void si446x_disable_wut(struct si446x *dev)
{
	interrupt_off(dev);
	set_property(dev, SI446X_GLOBAL_WUT_CONFIG, 0);
	set_property(dev, SI446X_GLOBAL_CLK_CFG, 0);
	interrupt_on(dev);
}

// Read data buffer from radio
static void si446x_internal_read(struct si446x *dev, u8 *buf, ssize_t len)
{
	struct spi_device *spi;
	int ret, astate;
	u8 *din = (u8 *)kmalloc(len + 1, GFP_NOWAIT);
	u8 *dout = (u8 *)kmalloc(len + 1, GFP_NOWAIT);
	struct spi_transfer xfer =
	    {
		.tx_buf = dout,
		.rx_buf = din,
		.len = len + 1,
		.bits_per_word = 8};

	memset(dout, 0xff, len + 1);
	dout[0] = SI446X_CMD_READ_RX_FIFO;
	spi = dev->spibus;

	astate = READ_ONCE(dev->on_rx_state);

	ret = spi_sync_transfer(spi, &xfer, 1);
	if (ret)
	{
		printk(KERN_ERR DRV_NAME "Error in internal read\n");
	}
	memcpy(buf, din + 1, len);
	si446x_set_state(dev, astate);
	kfree(dout);
	kfree(din);
}

// Write data buffer to radio
static void si446x_internal_write(struct si446x *dev, u8 *buf, int len)
{
	u8 *dout = (u8 *)kmalloc(len + 2, GFP_NOWAIT);
	int ret;
	struct spi_device *spi;

	struct spi_transfer xfer =
	    {
		.tx_buf = dout,
		.len = len + 2,
		.bits_per_word = 8};

	memset(dout, 0xff, len + 1);
	dout[0] = SI446X_CMD_WRITE_TX_FIFO;
	dout[1] = len;

	memcpy(dout + 2, buf, len);

	spi = dev->spibus;

	ret = spi_sync_transfer(spi, &xfer, 1);

	if (ret)
	{
		printk(KERN_ERR DRV_NAME "Error in internal write\n");
	}
	kfree(dout);
}

// Transmit data packet
static int si446x_tx(struct si446x *dev, void *packet, u8 len, u8 channel, si446x_state_t onTxFinish)
{
	int retcode;
	u8 data[7];
	if (len == 0)
	{
		retcode = 0;
		goto ret;
	}
	interrupt_off(dev);

	if (si446x_get_state(dev) == SI446X_STATE_TX)
	{
		retcode = 0;
		goto cleanup;
	}

	si446x_set_state(dev, dev->SI446X_IDLE_MODE);
	si446x_clear_fifo(dev);
	si446x_read_irq_noclr(dev, NULL, 0, 0, 0xFF);

	// Load data to FIFO
	si446x_internal_write(dev, packet, len);

	// Begin transmit
	data[0] = SI446X_CMD_START_TX;
	data[1] = channel;
	data[2] = (u8)(onTxFinish << 4);
	data[3] = 0;
	data[4] = len;
	data[5] = 0;
	data[6] = 0;

	si446x_do_api(dev, data, sizeof(data), NULL, 0);
	retcode = len;
	dev->tx_pk_ctr++;
cleanup:
	interrupt_on(dev);
ret:
	return retcode;
}

// Init work handler
static void si446x_init_work_handler(struct work_struct *work)
{
	struct si446x *dev;
	dev = container_of(work, struct si446x, init_work);
	dev->data_available = false;
	dev->rxbuf->head = 0;
	dev->rxbuf->tail = 0;
	dev->isr_state = 0;
	si446x_reset_device(dev);
	printk(KERN_DEBUG DRV_NAME ": Device reset\n");
	printk(KERN_DEBUG DRV_NAME ": Applying startup config\n");
	si446x_apply_startup_config(dev); // apply startup config
	printk(KERN_DEBUG DRV_NAME ": Applied startup config\n");
	si446x_read_irq_regs(dev, NULL); // clear interrupts
	printk(KERN_DEBUG DRV_NAME ": Cleared IRQ vector\n");
	si446x_sleep(dev); // put device into sleep mode (TODO: Test if device can wake up to receive mode on radio)
	printk(KERN_DEBUG DRV_NAME ": Device in sleep mode\n");
	dev->enabledInterrupts[IRQ_PACKET] = (1 << SI446X_PACKET_RX_PEND) | (1 << SI446X_CRC_ERROR_PEND);
	si446x_setup_callback(dev, SI446X_RXBEGIN_CB, 1); // enable receive irq
	printk(KERN_DEBUG DRV_NAME ": Receive callback set\n");
	complete(&(dev->initq)); // indicate initialization is complete
	printk(KERN_DEBUG DRV_NAME ": Signalled end of init\n");
}

/*
 * If some other process is ongoing, isr_lock is locked so ISR can not execute.
 * 
 * Once the lock unlocks, ISR can lock. After lock, it updates isr_busy.
 * 
 * If another process tries to turn off interrupt just as ISR is locking,
 * it will see isr_busy as false, and proceed to obtain the lock (after incrementing
 * isr_state). However, by that time, isr_lock has been locked and the function
 * trying to prevent ISR from running will wait until ISR is executed. Any do_api
 * calls inside the ISR will however do not care since at this point isr_busy is
 * true and it will not try to lock isr_lock.
 * 
 *  
 */
// Interrupt work handler
static void si446x_irq_work_handler(struct work_struct *work)
{
	int irq_avail;
	u8 len;
	bool read_rx_fifo = false;
	struct si446x *dev;
	dev = container_of(work, struct si446x, irq_work);
	mutex_lock(&(dev->isr_lock));				 // lock down the mutex
	WRITE_ONCE(dev->isr_busy, true);			 // indicate IRQ is busy
	while ((irq_avail = gpio_get_value(dev->nirq_pin)) == 0) // IRQ low
	{
		bool sleep_on_invalid;
		u8 interrupts[8];
		si446x_read_irq_regs(dev, interrupts);
		interrupts[2] &= dev->enabledInterrupts[IRQ_PACKET];
		interrupts[4] &= dev->enabledInterrupts[IRQ_MODEM];
		interrupts[6] &= dev->enabledInterrupts[IRQ_CHIP];
		// This segment deals with invalid sync and related occassional radio lock ups
		sleep_on_invalid = READ_ONCE(dev->sleep_on_invalid);
		if (sleep_on_invalid)
		{
#define SI446X_INVALIDSYNC_CB _BV(5)
			// Valid PREAMBLE and SYNC, packet data now begins
			// Enable invalid sync callback
			if (interrupts[4] & (1 << SI446X_SYNC_DETECT_PEND))
			{
				si446x_setup_callback(dev, SI446X_INVALIDSYNC_CB, 1); // Enable INVALID_SYNC when a new packet starts, sometimes a corrupted packet will mess the radio up
			}

			// Disable INVALID_SYNC callback to service the issue
			if ((interrupts[4] & (1 << SI446X_INVALID_SYNC_PEND)) || (interrupts[2] & ((1 << SI446X_PACKET_SENT_PEND) | (1 << SI446X_CRC_ERROR_PEND))))
			{
				si446x_setup_callback(dev, SI446X_INVALIDSYNC_CB, 0); // disble INVALID_SYNC if one is detected
			}

			// INVALID_SYNC detected, sometimes the radio gets messed up in this state and requires a RX restart
			// Hence device is put in sleep mode
			if (interrupts[4] & (1 << SI446X_INVALID_SYNC_PEND))
			{
				si446x_set_state(dev, SI446X_STATE_SLEEP);
			}
#undef SI446X_INVALIDSYNC_CB
		}
		// valid packet
		if (interrupts[2] & (1 << SI446X_PACKET_RX_PEND))
		{
			len = 0;
			si446x_internal_read(dev, &len, 1);
			dev->rssi = si446x_get_latched_rssi(dev);
			if ((len != 0xff) && (len != 0))
			{
				read_rx_fifo = true;
				dev->rx_pk_ctr++;
			}
		}
		// corrupted packet
		if (interrupts[2] & (1 << SI446X_CRC_ERROR_PEND))
		{
			dev->rx_corrupt_ctr++;
			if (sleep_on_invalid && (si446x_get_state(dev) == SI446X_STATE_SPI_ACTIVE)) // si446x_get_state will cause the device to get out of sleep mode and into SPI active mode
			{
				si446x_set_state(dev, dev->SI446X_IDLE_MODE);
			}
			si446x_rxinvalid_cb(dev, si446x_get_latched_rssi(dev));
		}
		// packet sent
		if (interrupts[2] & (1 << SI446X_PACKET_SENT_PEND))
		{
			si446x_sent_cb(dev);
		}
		// low battery
		if (interrupts[6] && (1 << SI446X_LOW_BATT_PEND))
		{
			si446x_lowbatt_cb(dev);
		}
		// WUT
		if (interrupts[6] & (1 << SI446X_WUT_PEND))
		{
			si446x_wut_cb(dev);
		}
		// Read data in
		if (read_rx_fifo)
		{
			int head, tail;
			u8 buff[SI446X_MAX_PACKET_LEN];
			memset(buff, 0x0, SI446X_MAX_PACKET_LEN);
			si446x_internal_read(dev, buff, len);
			head = READ_ONCE(dev->rxbuf->head);
			tail = READ_ONCE(dev->rxbuf->tail);
			if (CIRC_SPACE(head, READ_ONCE(dev->rxbuf->tail), dev->rxbuf_len) >= len)
			{
				int remainder, seq_len;
				WRITE_ONCE(dev->rxbuf->head, (head + len) & (dev->rxbuf_len - 1));
				remainder = len % (CIRC_SPACE_TO_END(head, tail, dev->rxbuf_len) + 1);
				seq_len = len - remainder;
				/* Write the block making sure to wrap around the end of the buffer */
				memcpy(dev->rxbuf->buf + head, buff, remainder);
				memcpy(dev->rxbuf->buf, buff + remainder, seq_len);
				WRITE_ONCE(dev->data_available, true);
				wake_up_interruptible(&(dev->rxq));
			}
			else
			{
				printk(KERN_ERR DRV_NAME "Error writing data, RX buffer full");
			}
			read_rx_fifo = false;
			len = 0;
		}
	}
	WRITE_ONCE(dev->isr_busy, false); // indicate ISR not busy
	mutex_unlock(&(dev->isr_lock));
}

// Interrupt callback
static irqreturn_t si446x_irq(int irq, void *dev_id)
{
	int initd;
	bool isr_busy;
	struct si446x *dev = dev_id;
	initd = READ_ONCE(dev->initd);	     // check if device is initialized
	isr_busy = READ_ONCE(dev->isr_busy); // check if already servicing an interrupt
	if (initd && (!isr_busy))
		schedule_work(&(dev->irq_work)); // if device is initialized and not servicing an interrupt currently, queue the ISR
	return IRQ_HANDLED;			 // indicate IRQ has been handled
}

// char device read method
static ssize_t si446x_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	struct si446x *dev;
	int head, tail, byte_count, out_count, remainder, seq_len;
	bool avail;
	dev = (struct si446x *)(filp->private_data);
	if (!dev->device_avail)
	{
		return -ENODEV;
	}
	else if (!dev->initd)
	{
		return -EIO;
	}
	if ((filp->f_flags & O_ACCMODE) == O_WRONLY) // device not opened as write only
	{
		return -EACCES;
	}
	if (count == 0)
	{
		return 0;
	}
	if ((filp->f_flags & O_NONBLOCK) == O_NONBLOCK) // non-blocking call
	{
		goto nonblock;
	}
	// else
	wait_event_interruptible(dev->rxq, (dev->data_available == true)); // wait while byte count is zero
	avail = READ_ONCE(dev->data_available);
	if (avail == false) // blocking call interrupted
	{
		return -EINTR;
	}
nonblock:
	head = READ_ONCE(dev->rxbuf->head);
	tail = READ_ONCE(dev->rxbuf->tail);
	byte_count = CIRC_CNT(head, tail, dev->rxbuf_len);
	if (byte_count >= count)
	{
		out_count = count;
	}
	else if (byte_count > 0)
	{
		out_count = byte_count;
		WRITE_ONCE(dev->data_available, false);
	}
	else // should not trigger
	{
		WRITE_ONCE(dev->data_available, false);
		goto ret;
	}
	WRITE_ONCE(dev->rxbuf->tail, (tail + out_count) & (dev->rxbuf_len - 1));     // update position of head
	remainder = out_count % (CIRC_SPACE_TO_END(head, tail, dev->rxbuf_len) + 1); // number of bytes to read till end of buffer
	seq_len = out_count - remainder;					     // number of bytes to read after wrap-around (if any)
	/* Write the block making sure to wrap around the end of the buffer */
	out_count -= copy_to_user(buf, dev->rxbuf->buf + tail, remainder);    // read from tail
	out_count -= copy_to_user(buf + remainder, dev->rxbuf->buf, seq_len); // wrap around
	return out_count;
ret:
	return 0;
}

// char device write method
static ssize_t si446x_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	struct si446x *dev;
	ssize_t i, out_count, err_count;
	int astate;
	dev = (struct si446x *)(filp->private_data);
	astate = READ_ONCE(dev->on_tx_state);
	if (!dev->device_avail)
	{
		return -ENODEV;
	}
	else if (!dev->initd)
	{
		return -EIO;
	}
	if ((filp->f_flags & O_ACCMODE) == O_RDONLY)
	{
		return -EACCES;
	}
	if (count == 0)
	{
		return 0;
	}
	out_count = count;
	err_count = 0;
	for (i = 0; out_count > SI446X_MAX_PACKET_LEN; i += SI446X_MAX_PACKET_LEN, out_count -= SI446X_MAX_PACKET_LEN) // write > 128 bytes
	{
		u8 buff[SI446X_MAX_PACKET_LEN];
		err_count -= copy_from_user(buff, buf + i, SI446X_MAX_PACKET_LEN);
		while (si446x_tx(dev, buff, SI446X_MAX_PACKET_LEN, 0, astate) == 0)
			;
	}
	if (out_count > 0) // write last 128 bytes
	{
		u8 buff[SI446X_MAX_PACKET_LEN];
		err_count -= copy_from_user(buff, buf + i, out_count);
		while (si446x_tx(dev, buff, out_count, 0, astate) == 0)
			;
	}
	return (i + out_count + err_count);
}

// char device poll method for read() with timeout
static unsigned int si446x_poll(struct file *filp, struct poll_table_struct *poll_table)
{
	__poll_t mask;
	struct si446x *dev;
	bool data_avail;
	dev = (struct si446x *)filp->private_data;
	mask = 0;
	if (!dev->device_avail)
	{
		mask |= POLLERR;
		goto ret;
	}
	data_avail = READ_ONCE(dev->data_available); // check if data is already available

	if (!data_avail) // if data is not available
	{
		poll_wait(filp, &(dev->rxq), poll_table);    // wait on the rx queue
		data_avail = READ_ONCE(dev->data_available); // check if data available, otherwise indicate timeout
	}

	if (data_avail) // if data is available, set POLLIN
	{
		mask |= POLLIN | POLLRDNORM;
	}
ret:
	return mask;
}

// char device open method
static int si446x_open(struct inode *inod, struct file *filp)
{
	struct si446x *dev;
	struct cdev *tcdev;
	si446x_info_t info[1];
	int retval, open_ctr;
	tcdev = inod->i_cdev;
	dev = container_of(tcdev, struct si446x, serdev);
	filp->private_data = (void *)dev; // this is what sets this for read/write() to work
	open_ctr = READ_ONCE(dev->open_ctr);
	open_ctr++;
	WRITE_ONCE(dev->open_ctr, open_ctr);
	retval = 0;
	if (open_ctr == 1) // reset device on very first open
	{
		printk(KERN_DEBUG DRV_NAME ": First open\n");
		dev->wut_counter = 0;
		dev->lowbatt = false;
		dev->initd = false;
		si446x_reset_device(dev);
		printk(KERN_DEBUG DRV_NAME ": Reset device\n");
		si446x_get_info(dev, info);
		printk(KERN_DEBUG DRV_NAME ": Get info in open\n");
		if ((info->part & 0x4460) != 0x4460)
		{
			printk(KERN_ERR DRV_NAME ": Not Si446x device, returned 0x%x\n", info->part);
			retval = -EHOSTDOWN;	      // signal that host is down
			WRITE_ONCE(dev->open_ctr, 0); // reset open counter
		}
		else
		{
			WRITE_ONCE(dev->device_avail, true);
		}
	}
	return retval;
}

// char device close method
static int si446x_release(struct inode *inod, struct file *filp)
{
	struct si446x *dev;
	struct cdev *tcdev;
	int open_ctr;
	tcdev = inod->i_cdev;
	dev = container_of(tcdev, struct si446x, serdev);
	if (dev != filp->private_data)
		return -ESTALE;
	open_ctr = READ_ONCE(dev->open_ctr);
	open_ctr--;
	WRITE_ONCE(dev->open_ctr, open_ctr);
	if (!(open_ctr)) // reset device on very last close
	{
		dev->data_available = false;
		dev->rxbuf->head = 0;
		dev->rxbuf->tail = 0;
		interrupt_off(dev);
		si446x_reset_device(dev);
		si446x_sleep(dev);
		dev->enabledInterrupts[IRQ_PACKET] = 0;
		dev->isr_state = 0;
		dev->initd = false;
		dev->device_avail = false;
		mutex_unlock(&(dev->isr_lock)); // force unlock irq lock
	}
	return 0;
}

// char device ioctl method
static long si446x_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct si446x *dev;
	long retval;
	void *ptr;
	dev = (struct si446x *)filp->private_data;
	ptr = (void __user *)arg;
	if (!dev->device_avail)
	{
		return -ENODEV;
	}
	else if ((!dev->initd) && (cmd < SI446X_INIT))
	{
		return -EIO;
	}
	switch (cmd)
	{
	case SI446X_SET_STATE:
	{
		int st;
		if ((st < 0) || (st > SI446X_STATE_RX))
		{
			retval = -EINVAL;
			break;
		}
		retval = -copy_from_user_nofault(&st, ptr, sizeof(int));
		if (!retval)
			si446x_set_state(dev, st);
		printk(KERN_DEBUG DRV_NAME "ioctl set state %d\n", st);
		break;
	}
	case SI446X_SET_ONTX_STATE:
	{
		int st;
		retval = -copy_from_user_nofault(&st, ptr, sizeof(int));
		if ((st < 0) || (st > SI446X_STATE_RX))
		{
			retval = -EINVAL;
			break;
		}
		WRITE_ONCE(dev->on_tx_state, st);
		break;
	}
	case SI446X_SET_ONRX_STATE:
	{
		int st;
		if ((st < 0) || (st > SI446X_STATE_RX))
		{
			retval = -EINVAL;
			break;
		}
		WRITE_ONCE(dev->on_rx_state, st);
		break;
	}
	case SI446X_GET_ONTX_STATE:
	{
		retval = dev->on_tx_state;
		break;
	}
	case SI446X_GET_ONRX_STATE:
	{
		retval = dev->on_rx_state;
		break;
	}
	case SI446X_GET_STATE:
	{
		int st;
		st = si446x_get_state(dev);
		retval = -copy_to_user_nofault(ptr, &st, sizeof(int));
		printk(KERN_DEBUG DRV_NAME "ioctl get state %d\n", st);
		break;
	}
	case SI446X_GET_LATCHED_RSSI:
	{
		retval = -copy_to_user_nofault(ptr, &(dev->rssi), sizeof(s16));
		break;
	}
	case SI446X_GET_RSSI:
	{
		s16 rssi;
		rssi = si446x_get_rssi(dev);
		retval = -copy_to_user_nofault(ptr, &rssi, sizeof(s16));
		break;
	}
	case SI446X_GET_INFO:
	{
		si446x_info_t info[1];
		si446x_get_info(dev, info);
		retval = -copy_to_user_nofault(ptr, info, sizeof(si446x_info_t));
		break;
	}
	case SI446X_DISABLE_WUT:
	{
		si446x_disable_wut(dev);
		retval = 1;
		break;
	}
	case SI446X_SETUP_WUT:
	{
		struct SI446X_WUT_CONFIG conf[1];
		retval = -copy_from_user_nofault(conf, ptr, sizeof(struct SI446X_WUT_CONFIG));
		if (!retval)
			si446x_setup_wut(dev, conf->r, conf->m, conf->ldc, conf->config);
		break;
	}
	case SI446X_GET_TX_PWR:
	{
		u8 pwr;
		pwr = si446x_get_tx_power(dev);
		retval = -copy_to_user_nofault(ptr, &pwr, sizeof(u8));
		break;
	}
	case SI446X_SET_TX_PWR:
	{
		u8 pwr;
		retval = -copy_from_user_nofault(&pwr, ptr, sizeof(u8));
		si446x_set_tx_power(dev, pwr);
		break;
	}
	case SI446X_GET_TEMP:
	{
		s32 temp;
		temp = si446x_adc_temperature(dev);
		retval = -copy_to_user_nofault(ptr, &temp, sizeof(s32));
		break;
	}
	case SI446X_RD_GPIO:
	{
		u8 st;
		st = si446x_read_gpio(dev);
		retval = -copy_to_user_nofault(ptr, &st, sizeof(s8));
		break;
	}
	case SI446X_WR_GPIO:
	{
		struct SI446X_GPIO_CONFIG conf[1];
		retval = -copy_from_user_nofault(conf, ptr, sizeof(struct SI446X_GPIO_CONFIG));
		si446x_write_gpio(dev, conf->pin, conf->val);
		break;
	}
	case SI446X_SET_LOW_BATT:
	{
		u8 volt;
		retval = -copy_from_user_nofault(&volt, ptr, sizeof(u8));
		si446x_set_low_batt(dev, volt);
		break;
	}
	case SI446X_ADC_BATT:
	{
		u16 batt;
		batt = si446x_adc_battery(dev);
		retval = -copy_to_user_nofault(ptr, &batt, sizeof(u16));
		break;
	}
	case SI446X_ADC_GPIO:
	{
		struct SI446X_ADC_GPIO_MEM gp[1];
		retval = -copy_from_user_nofault(gp, ptr, sizeof(struct SI446X_ADC_GPIO_MEM));
		if (retval)
		{
			break;
		}
		gp->val = si446x_adc_gpio(dev, gp->pin);
		retval = -copy_to_user_nofault(ptr, gp, sizeof(struct SI446X_ADC_GPIO_MEM));
		break;
	}
	case SI446X_SLEEP:
	{
		retval = si446x_sleep(dev);
		if (!retval)
		{
			printk(KERN_DEBUG DRV_NAME "Can not sleep, TX going\n");
		}
		break;
	}
	case SI446X_ADC_CONF:
	{
		struct SI446X_ADC_CONFIG conf[1];
		retval = -copy_from_user_nofault(conf, ptr, sizeof(struct SI446X_ADC_CONFIG));
		if (retval)
		{
			break;
		}
		conf->ret = si446x_get_adc(dev, conf->en, conf->cfg, conf->part);
		retval = -copy_to_user_nofault(ptr, conf, sizeof(struct SI446X_ADC_CONFIG));
		break;
	}
	case SI446X_RD_WUT_COUNTER:
	{
		retval = dev->wut_counter;
		break;
	}
	case SI446X_RD_LOWBATT:
	{
		retval = READ_ONCE(dev->lowbatt);
		WRITE_ONCE(dev->lowbatt, false); // clear
		break;
	}
	case SI446X_RD_RX_BUF_SZ:
	{
		retval = -copy_to_user_nofault(ptr, &(dev->rxbuf_len), sizeof(int));
		break;
	}
	case SI446X_INIT:
	{
		struct SI446X_INIT_PROPS init_props[1];
#ifndef SI446X_DRIVER_GPL2
		u8 config[] = RADIO_CONFIGURATION_DATA_ARRAY;
#endif
		si446x_reset_device(dev);
		memset(init_props, 0x0, sizeof(struct SI446X_INIT_PROPS));
#ifndef SI446X_DRIVER_GPL2
		if (arg == 0)
		{
			printk(KERN_DEBUG DRV_NAME ": Using default config\n");
			init_props->len = sizeof(config);
			retval = 0;
		}
		else
#else
		if (arg == 0)
		{
			printk(KERN_ERR DRV_NAME ": No configuration provided, can not initialize radio\n");
			retval = -EINVAL;
			break;
		}
#endif
		{
			retval = 0;
			retval = copy_from_user(init_props, (void *__user)arg, sizeof(struct SI446X_INIT_PROPS));
		}
		if (!retval) // copy over length
		{
			dev->config_len = init_props->len;
		}
		else
		{
			retval = -EINVAL;
			break; // failed
		}
		dev->config = (u8 *)kmalloc(dev->config_len, GFP_NOWAIT); // allocate memory for config buffer
		if (dev->config == NULL)
		{
			printk(KERN_ERR DRV_NAME ": Error allocating memory for init configuration\n");
			retval = -ENOMEM;
			break;
		}
#ifndef SI446X_DRIVER_GPL2
		if (!arg)
		{
			memcpy(dev->config, config, dev->config_len);
		}
		else
#endif
		// at this point, arg != NULL if not SI446X_DRIVER_GPL2
		{
			retval = copy_from_user(dev->config, (void *__user)init_props->config, dev->config_len);
		}

		if (retval)
		{
			retval = -ENOMEM;
			break;
		}
		// now we have config, we can go with init
		init_completion(&(dev->initq));
		schedule_work(&(dev->init_work));		       // schedule initialization work
		if (!wait_for_completion_interruptible(&(dev->initq))) // can sleep
		{
			dev->initd = true;
			retval = 0;
		}
		else
		{
			dev->initd = false;
			retval = -ETIMEDOUT;
		}
		kfree(dev->config);
		break;
	}
	case SI446X_DEBUG_TX_PACKETS:
	{
		retval = dev->tx_pk_ctr;
		break;
	}
	case SI446X_DEBUG_RX_PACKETS:
	{
		retval = dev->rx_pk_ctr;
		break;
	}
	case SI446X_DEBUG_RX_CORRUPT_PACKETS:
	{
		retval = dev->rx_corrupt_ctr;
		break;
	}
	default:
		printk(KERN_ERR DRV_NAME "%d invalid ioctl command\n", cmd);
		retval = -ENOIOCTLCMD;
		break;
	}
	return retval;
}

// initialize file_operations
static const struct file_operations si446x_fops = {.owner = THIS_MODULE,
						   .open = si446x_open,
						   .release = si446x_release,
						   .unlocked_ioctl =
						       si446x_ioctl,
						   .read = si446x_read,
						   .write = si446x_write,
						   .poll = si446x_poll};

// Device probe
static int si446x_probe(struct spi_device *spi)
{
	struct si446x *dev;
	struct device *pdev;
	int sdn_pin, nirq_pin;
	int ret;
	int ma, mi;
	dev_t this_dev;
	ret = 0;

	if (minor_ct >= SI446X_MAX_DEVICES)
	{
		printk(KERN_ERR DRV_NAME ": Could not allocate more than %d devices. %u devices allocated\n", SI446X_MAX_DEVICES, minor_ct);
		return -ENOMEM;
	}

	if (!spi)
	{
		printk(KERN_ERR DRV_NAME ": spi is NULL, FATAL ERROR\n");
		return -ENOMEM;
	}

	spi->mode = SPI_MODE_0;

	dev = kmalloc(sizeof(struct si446x), GFP_KERNEL);

	if ((!dev))
	{
		ret = -ENOMEM;
		printk(KERN_ERR DRV_NAME ": Error allocating memory\n");
		goto err_alloc_main;
	}

	pdev = &(spi->dev); // get platform device

	dev->spibus = spi; // assign spi bus

	spi_set_drvdata(spi, dev); // set pointer to si446x
	// read pin numbers from device tree
	if (of_property_read_s32(pdev->of_node, "sdn_pin", &(sdn_pin)))
	{
		printk(KERN_ERR DRV_NAME ": Error reading SDN pin number, read %d\n", sdn_pin);
		ret = -ENODATA;
		goto err_main;
	}
	if (of_property_read_s32(pdev->of_node, "irq_pin", &(nirq_pin)))
	{
		printk(KERN_ERR DRV_NAME ": Error reading IRQ pin number, read %d\n", nirq_pin);
		ret = -ENODATA;
		goto err_main;
	}
	// request gpio pin
	ret = gpio_request(sdn_pin, DRV_NAME "_gpio_sdn");
	if (ret)
	{
		printk(KERN_ERR DRV_NAME ": Error requesting gpio pin %d, status %d\n", sdn_pin, ret);
		ret = -ENODEV;
		goto err_main;
	}
	// check if pin is valid
	if (!gpio_is_valid(sdn_pin))
	{
		printk(KERN_ERR DRV_NAME ": %d not valid GPIO pin\n", sdn_pin);
		ret = -ENODEV;
		goto err_main;
	}
	// set direction of SDN pin
	ret = gpio_direction_output(sdn_pin, 0);
	if (ret)
	{
		printk(KERN_ERR DRV_NAME ": Error %d setting GPIO output direction\n", ret);
		ret = -ENODEV;
		goto err_main;
	}
	// request interrupt on NIRQ pin
	ret = gpio_request(nirq_pin, DRV_NAME "_gpio_nirq");
	if (ret)
	{
		printk(KERN_ERR DRV_NAME ": Error requesting gpio pin %d, status %d\n", nirq_pin, ret);
		ret = -ENODEV;
		goto err_main;
	}
	// check if NIRQ pin is a valid GPIO, this pin is read in the interrupt handler task
	if (!gpio_is_valid(nirq_pin))
	{
		printk(KERN_ERR DRV_NAME ": %d not valid GPIO pin\n", nirq_pin);
		ret = -ENODEV;
		goto err_main;
	}
	// store the pin IDs
	dev->sdn_pin = sdn_pin;
	dev->nirq_pin = nirq_pin;
	if (!minor_ct) // only for the first probe create class
		si446x_class = class_create(THIS_MODULE, DRV_NAME "_class");
	if (!si446x_class) // could not allocate class
	{
		ret = -ENOMEM;
		printk(KERN_ERR DRV_NAME ": Error allocating memory for class\n");
		goto err_main;
	}
	dev->rxbuf = kmalloc(sizeof(struct circ_buf), GFP_KERNEL); // allocate memory
	if (!(dev->rxbuf))
	{
		ret = -ENOMEM;
		printk(KERN_ERR DRV_NAME ": Error allocating memory for receiver buffer\n");
		goto err_main;
	}
	if (!(si446x_buffer_len & (si446x_buffer_len - 1))) // length is not power of 2
	{
		// round up to nearest power of 2 (up to 256)
		si446x_buffer_len--;
		si446x_buffer_len |= si446x_buffer_len >> 1;
		si446x_buffer_len |= si446x_buffer_len >> 2;
		si446x_buffer_len |= si446x_buffer_len >> 4;
		si446x_buffer_len++;
	}
	if ((si446x_buffer_len < 0x10))				    // minimum 16 packet buffer (2 KiB)
		si446x_buffer_len = 0x10;			    // minimum
	if (si446x_buffer_len > 0x80)				    // max 128 packet buffer (16 KiB)
		si446x_buffer_len = 0x80;			    // maximum
	dev->rxbuf_len = si446x_buffer_len * SI446X_MAX_PACKET_LEN; // determine size
	dev->rxbuf->buf = kzalloc(dev->rxbuf_len, GFP_KERNEL);	    // allocate memory
	if (!dev->rxbuf->buf)					    // allocation failed for buffer
	{
		ret = -ENOMEM;
		printk(KERN_ERR DRV_NAME ": Error allocating memory for receiver buffer\n");
		goto err_alloc_buf;
	}
	ret = 0; // making sure ret = 0 at this point
	if (!minor_ct)
		ret = alloc_chrdev_region(&device_num, 0, SI446X_MAX_DEVICES, DRV_NAME "_" DEVICE_NAME);
	if ((!ret) || (minor_ct > 0)) // chardev region allocation successful
	{
		ma = MAJOR(device_num);
		mi = minor_ct++;
		this_dev = MKDEV(ma, mi);		     // make new device
		cdev_init(&(dev->serdev), &si446x_fops);     // init chardev
		ret = cdev_add(&(dev->serdev), this_dev, 1); // add chardev
		if (ret)				     // error initializing
		{
			printk(KERN_ERR DRV_NAME ": Error adding serial device interface for major %d minor %d\n", ma, mi);
			minor_ct--;
			goto err_init_serial;
		}
		// else
		dev->this_dev = this_dev;
		if (device_create(si446x_class, &(spi->dev), this_dev, dev, DEVICE_NAME "%d", mi) == NULL)
		{
			printk(KERN_ERR DRV_NAME ": Error creating device fs handle " DEVICE_NAME "%d", mi);
			cdev_del(&(dev->serdev));
			minor_ct--;
		}
	}
	else
	{
		printk(KERN_ERR DRV_NAME ": chardev region not allocated\n");
		goto err_init_serial;
	}

	mutex_init(&(dev->isr_lock));
	init_waitqueue_head(&(dev->rxq));
	dev->isr_state = 0;
	dev->enabledInterrupts[0] = 0;
	dev->enabledInterrupts[1] = 0;
	dev->enabledInterrupts[2] = 0;
	INIT_WORK(&(dev->irq_work), si446x_irq_work_handler);
	INIT_WORK(&(dev->init_work), si446x_init_work_handler);
	spi_set_drvdata(spi, dev);

	ret = request_irq(spi->irq, si446x_irq, 0, DRV_NAME, dev);
	if (ret < 0)
	{
		printk(KERN_ERR DRV_NAME ": Error requesting IRQ, return %d\n", ret);
		goto err_init_serial;
	}
	dev->open_ctr = 0;			    // init counter 0
	dev->device_avail = false;		    // device not available by default
	dev->initd = false;			    // device not initiated by default
	dev->isr_busy = false;			    // ISR not busy by default
	dev->tx_pk_ctr = 0;			    // initialize tx packet counter to zero
	dev->rx_pk_ctr = 0;			    // initialize rx packet counter to zero
	dev->rx_corrupt_ctr = 0;		    // initialize rx corrupt packet counter to zero
	dev->on_tx_state = SI446X_FINISH_STATE;	    // default: sleep on finish
	dev->on_rx_state = SI446X_FINISH_STATE;	    // default: sleep on finish
	dev->SI446X_IDLE_MODE = SI446X_STATE_READY; // default: Ready
	printk(KERN_INFO DRV_NAME ": Registered device at spi%d.%d, with SDN %d and IRQ %d at " DEVICE_NAME "%d\n", spi->controller->bus_num, spi->chip_select, dev->sdn_pin, dev->nirq_pin, mi);
	return 0;
err_init_serial:
	kfree(dev->rxbuf->buf);
err_alloc_buf:
	kfree(dev->rxbuf);
err_main:
	kfree(dev);
err_alloc_main:
	gpio_free(sdn_pin);
	gpio_free(nirq_pin);
	if (!minor_ct)
	{
		class_destroy(si446x_class); // destroy class when the last device is deleted
		unregister_chrdev_region(device_num, SI446X_MAX_DEVICES);
	}
	printk(KERN_ERR DRV_NAME ": Failed to register device at spi%d.%d, with SDN %d and IRQ %d\n", spi->controller->bus_num, spi->chip_select, dev->sdn_pin, dev->nirq_pin);
	return ret;
}

// Device remove
static int si446x_remove(struct spi_device *spi)
{
	struct si446x *dev;
	int i;
	dev = spi_get_drvdata(spi);
	if (dev->rxbuf->buf)
		kfree(dev->rxbuf->buf);
	dev->rxbuf->head = 0;
	dev->rxbuf->tail = 0;
	for (i = 0; i < 3; i++)
		dev->enabledInterrupts[i] = 0;
	cdev_del(&(dev->serdev));
	device_destroy(si446x_class, dev->this_dev);
	minor_ct--;
	if (minor_ct == 0)
	{
		class_destroy(si446x_class); // destroy class when the last device is deleted
		unregister_chrdev_region(device_num, SI446X_MAX_DEVICES);
	}
	gpio_free(dev->sdn_pin);
	gpio_free(dev->nirq_pin);
	free_irq(spi->irq, dev);
	kfree(dev->rxbuf);
	kfree(dev);
	printk(KERN_INFO DRV_NAME ": Si446x device unregistered.\n");
	return 0;
}

// Device table ID
static const struct of_device_id si446x_dt_ids[] = {
    {.compatible = "silabs,si446x"},
    {/* sentinel */}};
MODULE_DEVICE_TABLE(of, si446x_dt_ids);

/**
 * 
 * Example Device Tree Blob Overlay
 

// Overlay for the SiLabs Si446X Controller - SPI0
// Interrupt pin: 11
// SDN Pin: 13

/dts-v1/;
/plugin/;

   / {
    compatible = "brcm,bcm2708";

    fragment@0 {
        target = <&spi0>;
        __overlay__ {
            // needed to avoid dtc warning
            #address-cells = <1>;
            #size-cells = <0>;

            status = "okay";

            uhf0: si446x@0{
                compatible = "silabs,si446x";
                reg = <0>; // CE0
                pinctrl-names = "default";
                pinctrl-0 = <&uhf0_pins>;
                interrupt-parent = <&gpio>;
                interrupts = <17 0x2>; // falling edge
                spi-max-frequency = <4000000>;
                sdn_pin = <27>;
                irq_pin = <17>;
                status = "okay";
            };
        };
    };

    fragment@1 {
        target = <&gpio>;
        __overlay__ {
            uhf0_pins: uhf0_pins {
                brcm,pins = <17 27>; // gpio 11, gpio 13
                brcm,function = <0 1>; // in, out
                brcm,pull = <2 0>; // high, none
            };
        };
    };

    __overrides__ {
        int_pin = <&uhf0>, "interrupts:0",
                  <&uhf0_pins>, "brcm,pins:0";
        speed   = <&uhf0>, "spi-max-frequency:0";
    };
};

 */

static struct spi_driver si446x_driver = {
    .driver = {
	.name = DRV_NAME,
	.of_match_table = si446x_dt_ids,
    },
    .probe = si446x_probe,
    .remove = si446x_remove,
};
module_spi_driver(si446x_driver);

MODULE_DESCRIPTION(DRV_NAME " Si446x UHF Transceiver Driver");
MODULE_AUTHOR("Sunip K. Mukherjee <sunipkmukherjee@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:" DRV_NAME);