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
static const struct file_operations si446x_fops = { .owner = THIS_MODULE,
						    .open = si446x_open,
						    .release = si446x_release,
						    .unlocked_ioctl =
							    si446x_ioctl,
						    .read = si446x_read,
						    .write = si446x_write };

struct si446x_dev {
	struct spi_device *spi; // spi bus
	struct mutex lock; // mutex lock
	struct cdev *ser; // serial device
	int sdn_pin; // sdn pin
	int nirq_pin; // IRQ pin
	int cs_pin; // chip select gpio
	u32 tx_pk_ctr; // stat for tx packets
	u32 rx_pk_ctr; // stat for rx packets
	u8 spi_tx_buf[SPI_XFER_LEN];
	u8 spi_rx_buf[SPI_XFER_LEN];
	s32 isr_state;
	s32 isr_busy;
	struct mutex isr_lock;
	struct mutex isr_var_lock;
}

static inline int
interrupt_off(struct si446x_dev *dev)
{
	mutex_lock(&(dev->isr_var_lock)); // lock access to ISR var
	if (!dev->isr_busy) {
		mutex_lock(&(dev->isr_lock)); // prevent ISR from running
		dev->isr_state++;
	}
	mutex_unlock(&(dev->isr_var_lock));
	return 1;
}

static inline int interrupt_on(struct si446x_dev *dev)
{
	mutex_lock(&(dev->isr_var_lock)); // lock access to ISR var
	if (!dev->isr_busy) {
		if (dev->isr_state > 0)
			dev->isr_state--;
		if (dev->isr_state == 0)
			mutex_unlock(&(dev->isr_lock)); // Allow ISR to run
	}
	mutex_unlock(&(dev->isr_var_lock));
	return 0;
}

void delay_us(int us)
{
	if (us < 0)
		return;
	int ms = us / 1000;
	us = us % 1000;
	if (us > 0)
		udelay(us);
	if (ms > 0)
		msleep(ms);
}

static void __empty_callback0(void)
{
}
static void __empty_callback1(int16_t param1)
{
	(void)(param1);
}

void __attribute__((weak, alias("__empty_callback0")))
SI446X_CB_CMDTIMEOUT(void);
void __attribute__((weak, alias("__empty_callback1")))
SI446X_CB_RXBEGIN(int16_t rssi);
void __attribute__((weak)) SI446X_CB_RXCOMPLETE(uint8_t length, int16_t rssi)
{
	(void)(length);
	(void)(rssi);
}
void __attribute__((weak, alias("__empty_callback1")))
SI446X_CB_RXINVALID(int16_t rssi);
void __attribute__((weak, alias("__empty_callback0"))) SI446X_CB_SENT(void);
void __attribute__((weak, alias("__empty_callback0"))) SI446X_CB_WUT(void);
void __attribute__((weak, alias("__empty_callback0"))) SI446X_CB_LOWBATT(void);

static u8 getResponse(struct spi_device *spi, void *buff, uint8_t len)
{
	u8 cts = 0;
	u8 *dout = (u8 *)kmalloc(len + 2); // 1 for cmd, 1 for cts
	memset(dout, 0xff, len + 2); // memset the whole output array
	u8 *din = (u8 *)kmalloc(len + 2); // no need to memset input
	SI446X_ATOMIC()
	{
		CHIPSELECT()
		{
			// set command
			dout[0] = SI446X_CMD_READ_CMD_BUFF;
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

			spi_message_init(&msg);
			spi_message_add_tail(&tx, &msg);
			spi_message_add_tail(&rx, &msg);

			ret = spi_sync(spi, &msg);
			if (ret != 0) {
				printk(KERN_ERR DRV_NAME
				       "Error in spi transaction getResponse, retcode %d\n",
				       ret);
				goto cleanup;
			}
			cts = din[1];
			if (cts) {
				// memcpy
				memcpy(buff, din + 2, len);
			}
		}
	}
cleanup:
	kfree(dout);
	kfree(din);
	return cts;
}

static u8 waitForResponse(struct spi_device *spi, void *out, u8 outLen,
			  bool useTimeout)
{
	// With F_CPU at 8MHz and SPI at 4MHz each check takes about 7us + 10us delay
	u16 timeout = 40000;
	while (!getResponse(spi, out, outLen)) {
		delay_us(10);
		if (useTimeout && !--timeout) {
			SI446X_CB_CMDTIMEOUT();
			return 0;
		}
	}
	return 1;
}

static void spi_write_buf(struct spi_device *spi, void *out, u8 len)
{
	int ret;
	u8 *dout = kmalloc(len);
	memset(dout, 0xff, len);
	struct spi_transfer tx = {
		.tx_buf = dout,
		.len = len + 2,
	};
	struct spi_transfer rx = {
		.rx_buf = din,
		.len = len + 2,
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&tx, &msg);
	spi_message_add_tail(&rx, &msg);

	ret = spi_sync(spi, &msg);
	if (ret != 0) {
		printk(KERN_ERR DRV_NAME
		       "Error in spi buffer read doAPI, retcode %d\n",
		       ret);
	}
	kfree(dout);
}

static void doAPI(struct spi_device *spi, void *data, u8 len, void *out,
		  u8 outLen)
{
	SI446X_NO_INTERRUPT()
	{
		if (waitForResponse(NULL, 0,
				    1)) // Make sure it's ok to send a command
		{
			SI446X_ATOMIC()
			{
				CHIPSELECT()
				{
					spi_write_buf(spi, data, len);
				}
			}

			if (((uint8_t *)data)[0] ==
			    SI446X_CMD_IRCAL) // If we're doing an IRCAL then wait for its completion without a timeout since it can sometimes take a few seconds
				waitForResponse(NULL, 0, 0);
			else if (out !=
				 NULL) // If we have an output buffer then read command response into it
				waitForResponse(out, outLen, 1);
		}
	}
}

/**
 * @brief IRQ handler
 * 
 * @param irq 
 * @param dev_id 
 * @return irqreturn_t 
 */
static irqreturn_t si446x_irq(int irq, void *dev_id)
{
	struct si446x_dev *dev = dev_id;
	mutex_lock(&(dev->isr_lock));
	mutex_lock(&(dev->isr_var_lock)); // lock access to ISR var
	dev->isr_busy = 1;
	mutex_unlock(&(dev->isr_var_lock));
	// handle ISR
	if (!gpio_is_valid(dev->nirq_pin)) {
		printk(KERN_ERR "GPIO %d invalid", dev->nirq_pin);
		goto end;
	}

end:
	mutex_lock(&(dev->isr_var_lock)); // lock access to ISR var
	dev->isr_busy = 0;
	mutex_unlock(&(dev->isr_var_lock));
	mutex_unlock(&(dev->isr_lock));
	return IRQ_HANDLED;
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