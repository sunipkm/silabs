#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

#include "si446x_kern.h"
#include "si446x_config.h"
#include "si446x_defs.h"
#include "radio_config.h"
#include "si446x.h"

#define DRV_NAME "si446x"
#define DRV_VERSION "0.1"

#define MAX_DEV 1

#define SPI_OPLEN 1

#define SPI_XFER_LEN 1 // doAPI does spi_transfer and spi_transfer_nr

// initialize file_operations
static const struct file_operations si446x_fops = {.owner = THIS_MODULE,
												   .open = si446x_open,
												   .release = si446x_release,
												   .unlocked_ioctl =
													   si446x_ioctl,
												   .read = si446x_read,
												   .write = si446x_write};

struct si446x_dev
{
	struct spi_device *spi;		 // spi bus
	struct mutex lock;			 // mutex lock
	struct cdev *ser;			 // serial device
	struct work_struct irq_work; // IRQ handler
	int sdn_pin;				 // sdn pin
	int nirq_pin;				 // IRQ pin
	int cs_pin;					 // chip select gpio
	u32 tx_pk_ctr;				 // stat for tx packets
	u32 rx_pk_ctr;				 // stat for rx packets
	unsigned int isr_state;		 // ISR lock state
	struct mutex isr_lock;		 // mutex lock to prevent ISR from running
	u8 *config;					 // initialize as RADIO_CONFIGURATION_DATA_ARRAY
	u32 config_len;				 // length of RADIO_CONFIGURATION_DATA_ARRAY
	u8 enabledInterrupts[3];	 // enabled interrupts
	u8 *rx_data;				 // pointer to received data
	u16 rx_data_sz;				 // in read, have a wait_event_interruptible on this variable, and wake it up from the rx irq work
	struct mutex rx_lock;		 // mutex lock for received data
};

static inline int
interrupt_off(struct si446x_dev *dev)
{
	mutex_lock(&(dev->isr_lock)); // prevent ISR from running
	dev->isr_state++;
	return 1;
}

static inline int interrupt_on(struct si446x_dev *dev)
{
	if (dev->isr_state > 0)
		dev->isr_state--;
	if (dev->isr_state == 0)
		mutex_unlock(&(dev->isr_lock)); // Allow ISR to run
	return 0;
}

void delay_us(int us)
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

#define delay_ms(ms) msleep(ms)

static void __empty_callback0(void)
{
	return;
}
static void __empty_callback1(s16 param1)
{
	(void)(param1);
}

void SI446X_CB_CMDTIMEOUT(void)
{
	printk(KERN_ERR DRV_NAME "waitForResponse timed out\n");
}
void __attribute__((weak, alias("__empty_callback1")))
SI446X_CB_RXBEGIN(s16 rssi);
void __attribute__((weak)) SI446X_CB_RXCOMPLETE(u8 length, s16 rssi)
{
	(void)(length);
	(void)(rssi);
}
void __attribute__((weak, alias("__empty_callback1")))
SI446X_CB_RXINVALID(s16 rssi);
void __attribute__((weak, alias("__empty_callback0"))) SI446X_CB_SENT(void);
void __attribute__((weak, alias("__empty_callback0"))) SI446X_CB_WUT(void);
void __attribute__((weak, alias("__empty_callback0"))) SI446X_CB_LOWBATT(void);

static unsigned char getResponse(struct si446x_dev *dev, void *buff, unsigned char len)
{
	u8 cts = 0;
	u8 *dout = (u8 *)kmalloc(len + 2, GFP_NOWAIT); // 1 for cmd, 1 for cts
	u8 *din = (u8 *)kzalloc(len + 2, GFP_NOWAIT);  // no need to memset input
	struct spi_transfer tx = {
		.tx_buf = dout,
		.len = len + 2,
	};
	struct spi_transfer rx = {
		.rx_buf = din,
		.len = len + 2,
	};
	struct spi_message msg;
	int ret;
	struct spi_device *spi = dev->spi;
	// set command
	dout[0] = SI446X_CMD_READ_CMD_BUFF;
	memset(dout + 1, 0xff, len + 1);

	spi_message_init(&msg);
	spi_message_add_tail(&tx, &msg);
	spi_message_add_tail(&rx, &msg);

	mutex_lock(&(dev->lock));
	ret = spi_sync(spi, &msg); // blocking
	mutex_unlock(&(dev->lock));
	if (ret != 0)
	{
		printk(KERN_ERR DRV_NAME
			   "Error in spi transaction getResponse, retcode %d\n",
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

static u8 waitForResponse(struct si446x_dev *dev, void *out, u8 outLen,
						  bool useTimeout)
{
	// With F_CPU at 8MHz and SPI at 4MHz each check takes about 7us + 10us delay
	u16 timeout = 40000;
	while (!getResponse(dev, out, outLen))
	{
		delay_us(10);
		if (useTimeout && !--timeout)
		{
			SI446X_CB_CMDTIMEOUT();
			return 0;
		}
	}
	return 1;
}

static void spi_write_buf(struct si446x_dev *dev, void *out, u8 len)
{
	struct spi_device *spi = dev->spi;
	int ret;
	struct spi_transfer tx = {
		.tx_buf = out,
		.len = len,
	};
	struct spi_transfer rx = {
		.rx_buf = NULL,
		.len = 0,
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&tx, &msg);
	spi_message_add_tail(&rx, &msg);

	mutex_lock(&(dev->lock));
	ret = spi_sync(spi, &msg);
	mutex_unlock(&(dev->lock));
	if (ret != 0)
	{
		printk(KERN_ERR DRV_NAME
			   "Error in spi buffer write doAPI, retcode %d\n",
			   ret);
	}
}

static void doAPI(struct si446x_dev *dev, void *data, u8 len, void *out,
				  u8 outLen)
{
	int ret = interrupt_off(dev);
	{
		if (waitForResponse(dev, NULL, 0,
							1)) // Make sure it's ok to send a command
		{
			spi_write_buf(dev, data, len);
			if (((u8 *)data)[0] ==
				SI446X_CMD_IRCAL) // If we're doing an IRCAL then wait for its completion without a timeout since it can sometimes take a few seconds
				waitForResponse(dev, NULL, 0, 0);
			else if (out !=
					 NULL) // If we have an output buffer then read command response into it
				waitForResponse(dev, out, outLen, 1);
		}
	}
	ret = interrupt_on(dev);
}

// Configure a bunch of properties (up to 12 properties in one go)
static void setProperties(struct si446x_dev *dev, u16 prop, void *values, u8 len)
{
	// len must not be greater than 12

	u8 data[16] = {
		SI446X_CMD_SET_PROPERTY,
		(u8)(prop >> 8),
		len,
		(u8)prop};

	// Copy values into data, starting at index 4
	memcpy(data + 4, values, len);

	doAPI(dev, data, len + 4, NULL, 0);
}

// Set a single property
static inline void setProperty(struct si446x_dev *dev, u16 prop, u8 value)
{
	setProperties(dev, prop, &value, 1);
}

// Read a bunch of properties
static void getProperties(struct si446x_dev *dev, u16 prop, void *values, u8 len)
{
	u8 data[] = {
		SI446X_CMD_GET_PROPERTY,
		(u8)(prop >> 8),
		len,
		(u8)prop};

	doAPI(dev, data, sizeof(data), values, len);
}

// Read a single property
static inline u8 getProperty(struct si446x_dev *dev, u16 prop)
{
	u8 val;
	getProperties(dev, prop, &val, 1);
	return val;
}

#define IRQ_PACKET 0
#define IRQ_MODEM 1
#define IRQ_CHIP 2

void si446x_setupCallback(struct si446x_dev *dev, u16 callbacks, u8 state)
{

	int ret = interrupt_off(dev);
	{
		uint8_t data[2];
		getProperties(dev, SI446X_INT_CTL_PH_ENABLE, data, sizeof(data));

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

		// TODO
		// make sure RXCOMPELTE, RXINVALID and RXBEGIN? are always enabled

		dev->enabledInterrupts[IRQ_PACKET] = data[0];
		dev->enabledInterrupts[IRQ_MODEM] = data[1];
		setProperties(dev, SI446X_INT_CTL_PH_ENABLE, data, sizeof(data));
	}
	ret = interrupt_on(dev);
}

// Do an ADC conversion
static u16 getADC(struct si446x_dev *dev, u8 adc_en, u8 adc_cfg, u8 part)
{
	u8 data[6] = {
		SI446X_CMD_GET_ADC_READING,
		adc_en,
		adc_cfg};
	doAPI(dev, data, 3, data, 6);
	return (data[part] << 8 | data[part + 1]);
}

// Read a fast response register
static u8 getFRR(struct si446x_dev *dev, u8 reg)
{
	u8 frr = 0;

	{
		struct spi_device *spi;
		int ret;
		uint8_t dout[2], din[2];
		dout[0] = reg;
		dout[1] = 0xff;
		din[0] = 0;
		din[1] = 0;
		spi = dev->spi;
		struct spi_transfer tx = {
			.tx_buf = dout,
			.len = 2,
		};
		struct spi_transfer rx = {
			.rx_buf = din,
			.len = 2,
		};
		struct spi_message msg;

		spi_message_init(&msg);
		spi_message_add_tail(&tx, &msg);
		spi_message_add_tail(&rx, &msg);

		mutex_lock(&(dev->lock));
		ret = spi_sync(spi, &msg);
		mutex_unlock(&(dev->lock));
		if (ret != 0)
		{
			printk(KERN_ERR DRV_NAME
				   "Error in spi buffer write getFRR, retcode %d\n",
				   ret);
		}
		frr = din[1];
	}
	return frr;
}

// Ge the patched RSSI from the beginning of the packet
static s16 getLatchedRSSI(struct si446x_dev *dev)
{
#define rssi_dBm(val) ((val / 2) - 134)
	u8 frr;
	s16 rssi;
	frr = getFRR(dev, SI446X_CMD_READ_FRR_A);
	rssi = rssi_dBm(frr);
	return rssi;
}

// Get current radio state
static si446x_state_t getState(struct si446x_dev *dev)
{
	u8 state;
	state = getFRR(dev, SI446X_CMD_READ_FRR_B);
	if (state == SI446X_STATE_TX_TUNE)
		state = SI446X_STATE_TX;
	else if (state == SI446X_STATE_RX_TUNE)
		state = SI446X_STATE_RX;
	else if (state == SI446X_STATE_READY2)
		state = SI446X_STATE_READY;
	return (si446x_state_t)state;
}

// Set new state
static void setState(struct si446x_dev *dev, si446x_state_t newState)
{
	u8 data[] = {
		SI446X_CMD_CHANGE_STATE,
		newState};
	doAPI(dev, data, sizeof(data), NULL, 0);
}

// Clear RX and TX FIFOs
static void clearFIFO(struct si446x_dev *dev)
{
	static const u8 clearFifo[] = {
		SI446X_CMD_FIFO_INFO,
		SI446X_FIFO_CLEAR_RX | SI446X_FIFO_CLEAR_TX};
	doAPI(dev, (u8 *)clearFifo, sizeof(clearFifo), NULL, 0);
}

static void interrupt(struct si446x_dev *dev, void *buff)
{
	u8 data = SI446X_CMD_GET_INT_STATUS;
	doAPI(dev, &data, sizeof(data), buff, 8);
}

// Similar to interrupt() but with the option of not clearing certain interrupt flags
static void interrupt2(struct si446x_dev *dev, void *buff, u8 clearPH, u8 clearMODEM, u8 clearCHIP)
{
	u8 data[] = {
		SI446X_CMD_GET_INT_STATUS,
		clearPH,
		clearMODEM,
		clearCHIP};
	doAPI(dev, data, sizeof(data), buff, 8);
}

// Reset the RF chip
static void resetDevice(struct si446x_dev *dev)
{
	if (gpio_is_valid(dev->sdn_pin))
	{
		gpio_set_value_cansleep(dev->sdn_pin, 1);
		delay_ms(50);
		gpio_set_value_cansleep(dev->sdn_pin, 0);
		delay_ms(50);
	}
	else
		printk(KERN_ERR DRV_NAME "SDN GPIO %d invalid\n", dev->sdn);
}

// Apply the radio configuration
static void applyStartupConfig(struct si446x_dev *dev)
{
	u8 buff[17];
	u16 i;
	for (i = 0; i < dev->config_len; i++)
	{
		memcpy(buff, &(dev->config[i]), sizeof(buff));
		doAPI(dev, &buff[1], buff[0], NULL, 0);
		i += buff[0];
	}
}

static irqreturn_t si446x_irq(int irq, void *dev_id)
{
	struct si446x_dev *dev = dev_id;
	schedule_work(&(dev->irq_work));
	return IRQ_HANDLED;
}

static void si446x_internal_read(struct si446x_dev *dev, uint8_t *buf, ssize_t len)
{
	struct spi_device *spi;
	int ret;
	u8 *din = (uint8_t *)kmalloc(len + 1, GFP_NOWAIT);
	u8 *dout = (uint8_t *)kmalloc(len + 1, GFP_NOWAIT);
	memset(dout, 0xff, len + 1);
	dout[0] = SI446X_CMD_READ_RX_FIFO;
	spi = dev->spi;
	struct spi_transfer tx = {
		.tx_buf = dout,
		.len = 2,
	};
	struct spi_transfer rx = {
		.rx_buf = din,
		.len = 2,
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&tx, &msg);
	spi_message_add_tail(&rx, &msg);
	mutex_lock(&(dev->lock));
	ret = spi_sync(spi, &msg); // assuming negative on error, zero on success
	mutex_unlock(&(dev->lock));
	if (ret)
	{
		printk(KERN_ERR DRV_NAME "Error in internal read\n");
	}
	memcpy(buf, din + 1, len);
	setState(SI446X_STATE_RX);
	free(dout);
	free(din);
}

static void si446x_irq_work_handler(struct work_struct *work)
{
	struct si446x_dev *dev = container_of(work, struct si446x_dev, irq_work);
	mutex_lock(&(dev->lock));
	int irq_avail;
	while ((irq_avail = gpio_get_value(dev->irq_pin)) == 0) // IRQ low
	{
		u8 interrupts[8];
		interrupt(dev, interrupts); // read in IRQ vectors
		interrupts[2] &= enabledInterrupts[IRQ_PACKET];
		interrupts[4] &= enabledInterrupts[IRQ_MODEM];
		interrupts[6] &= enabledInterrupts[IRQ_CHIP];
	}
	if (interrupts[2] & (1 << SI446X_PACKET_RX_PEND))
	{
		len = 0;
		si446x_internal_read(dev, &len, 1);
		if (!read_rssi)
			_rssi = getLatchedRSSI(dev);
		// eprintf("RX packet pending: RSSI %d, length: %u", rssi, len);
		SI446X_CB_RXCOMPLETE(len, _rssi);
		(data->rssi) = _rssi;
		if (len != 0xff)
			read_rx_fifo = true;
	}
	mutex_unlock(&(dev->lock));
}

/*
 * To read a reg property: 
 * property = <0xa>; // in device tree
 * 
 * void *ptr; int ret;
 * ptr = of_get_property(op->dev.of_node, "property", &ret); // ret stores the length of the property
 * 
 * int value;
 * value = be32_to_cpup(ptr); // get CPU endian value 
 * 
 */

/*
 * To do GPIO stuff:
 * 0. gpio_request()
 * 1. gpio_is_valid(int gpio)
 * 2. Get gpio desc (gpio_to_desc(gpio))
 * 3. Set dir: gpio_direction_output(gpio, value) // set this as high
 * 4. To write: gpio_set_value_cansleep(gpio, value)
 *  
 */

// static int of_si446x_get_gpio_number(struct )