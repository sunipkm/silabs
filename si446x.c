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
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/circ_buf.h>
#include <linux/semaphore.h>

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

struct si446x
{
    struct cdev *cdev;
    struct spi_device *spi;      // spi bus
    struct mutex lock;           // mutex lock
    struct work_struct irq_work; // IRQ handler
    int sdn_pin;                 // sdn pin
    int nirq_pin;                // IRQ pin
    int cs_pin;                  // chip select gpio
    u32 tx_pk_ctr;               // stat for tx packets
    u32 rx_pk_ctr;               // stat for rx packets
    u32 rx_corrupt_ctr;          // stat for corrupt rx packets
    unsigned int isr_state;      // ISR lock state
    struct mutex isr_lock;       // mutex lock to prevent ISR from running
    u8 *config;                  // initialize as RADIO_CONFIGURATION_DATA_ARRAY
    u32 config_len;              // length of RADIO_CONFIGURATION_DATA_ARRAY
    u8 enabledInterrupts[3];     // enabled interrupts
    s16 rssi;                    // RSSI value returned by ioctl
    struct circ_buf *rxbuf;      // RX data buffer
    int rxbuf_len;               // rx buffer length
    bool data_available;         // indicate RX
};

static DECLARE_WAIT_QUEUE_HEAD(rxq);

static inline int interrupt_off(struct si446x *dev)
{
    mutex_lock(&(dev->isr_lock)); // prevent ISR from running
    dev->isr_state++;
    return 1;
}

static inline int interrupt_on(struct si446x *dev)
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
    printk(KERN_ERR DRV_NAME "wait_for_response timed out\n");
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

static unsigned char get_response(struct si446x *dev, void *buff, unsigned char len)
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
               "Error in spi transaction get_response, retcode %d\n",
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

static u8 wait_for_response(struct si446x *dev, void *out, u8 outLen,
                            bool useTimeout)
{
    // With F_CPU at 8MHz and SPI at 4MHz each check takes about 7us + 10us delay
    u16 timeout = 40000;
    while (!get_response(dev, out, outLen))
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

static void spi_write_buf(struct si446x *dev, void *out, u8 len)
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
               "Error in spi buffer write si446x_do_api, retcode %d\n",
               ret);
    }
}

static void si446x_do_api(struct si446x *dev, void *data, u8 len, void *out,
                          u8 outLen)
{
    int ret = interrupt_off(dev);
    {
        if (wait_for_response(dev, NULL, 0,
                              1)) // Make sure it's ok to send a command
        {
            spi_write_buf(dev, data, len);
            if (((u8 *)data)[0] ==
                SI446X_CMD_IRCAL) // If we're doing an IRCAL then wait for its completion without a timeout since it can sometimes take a few seconds
                wait_for_response(dev, NULL, 0, 0);
            else if (out !=
                     NULL) // If we have an output buffer then read command response into it
                wait_for_response(dev, out, outLen, 1);
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

void si446x_setup_callback(struct si446x *dev, u16 callbacks, u8 state)
{

    int ret = interrupt_off(dev);
    {
        uint8_t data[2];
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

        // TODO
        // make sure RXCOMPELTE, RXINVALID and RXBEGIN? are always enabled

        dev->enabledInterrupts[IRQ_PACKET] = data[0];
        dev->enabledInterrupts[IRQ_MODEM] = data[1];
        si446x_set_props(dev, SI446X_INT_CTL_PH_ENABLE, data, sizeof(data));
    }
    ret = interrupt_on(dev);
}

// Do an ADC conversion
static u16 get_adc(struct si446x *dev, u8 adc_en, u8 adc_cfg, u8 part)
{
    u8 data[6] = {
        SI446X_CMD_GET_ADC_READING,
        adc_en,
        adc_cfg};
    si446x_do_api(dev, data, 3, data, 6);
    return (data[part] << 8 | data[part + 1]);
}

// Read a fast response register
static u8 get_frr(struct si446x *dev, u8 reg)
{
    u8 frr = 0;

    {
        struct spi_device *spi;
        int ret;
        uint8_t dout[2], din[2];
        struct spi_transfer tx = {
            .tx_buf = dout,
            .len = 2,
        };
        struct spi_transfer rx = {
            .rx_buf = din,
            .len = 2,
        };
        struct spi_message msg;

        dout[0] = reg;
        dout[1] = 0xff;
        din[0] = 0;
        din[1] = 0;
        spi = dev->spi;

        spi_message_init(&msg);
        spi_message_add_tail(&tx, &msg);
        spi_message_add_tail(&rx, &msg);

        mutex_lock(&(dev->lock));
        ret = spi_sync(spi, &msg);
        mutex_unlock(&(dev->lock));
        if (ret != 0)
        {
            printk(KERN_ERR DRV_NAME
                   "Error in spi buffer write get_frr, retcode %d\n",
                   ret);
        }
        frr = din[1];
    }
    return frr;
}

// Ge the patched RSSI from the beginning of the packet
static s16 get_latched_rssi(struct si446x *dev)
{
#define rssi_dBm(val) ((val / 2) - 134)
    u8 frr;
    s16 rssi;
    frr = get_frr(dev, SI446X_CMD_READ_FRR_A);
    rssi = rssi_dBm(frr);
    return rssi;
}

// Get current radio state
static si446x_state_t get_state(struct si446x *dev)
{
    u8 state;
    state = get_frr(dev, SI446X_CMD_READ_FRR_B);
    if (state == SI446X_STATE_TX_TUNE)
        state = SI446X_STATE_TX;
    else if (state == SI446X_STATE_RX_TUNE)
        state = SI446X_STATE_RX;
    else if (state == SI446X_STATE_READY2)
        state = SI446X_STATE_READY;
    return (si446x_state_t)state;
}

// Set new state
static void set_state(struct si446x *dev, si446x_state_t newState)
{
    u8 data[] = {
        SI446X_CMD_CHANGE_STATE,
        newState};
    si446x_do_api(dev, data, sizeof(data), NULL, 0);
}

// Clear RX and TX FIFOs
static void clear_fifo(struct si446x *dev)
{
    static const u8 clearFifo[] = {
        SI446X_CMD_FIFO_INFO,
        SI446X_FIFO_CLEAR_RX | SI446X_FIFO_CLEAR_TX};
    si446x_do_api(dev, (u8 *)clearFifo, sizeof(clearFifo), NULL, 0);
}

static void interrupt(struct si446x *dev, void *buff)
{
    u8 data = SI446X_CMD_GET_INT_STATUS;
    si446x_do_api(dev, &data, sizeof(data), buff, 8);
}

// Similar to interrupt() but with the option of not clearing certain interrupt flags
static void interrupt2(struct si446x *dev, void *buff, u8 clearPH, u8 clearMODEM, u8 clearCHIP)
{
    u8 data[] = {
        SI446X_CMD_GET_INT_STATUS,
        clearPH,
        clearMODEM,
        clearCHIP};
    si446x_do_api(dev, data, sizeof(data), buff, 8);
}

// Reset the RF chip
static void reset_device(struct si446x *dev)
{
    if (gpio_is_valid(dev->sdn_pin))
    {
        gpio_set_value_cansleep(dev->sdn_pin, 1);
        delay_ms(50);
        gpio_set_value_cansleep(dev->sdn_pin, 0);
        delay_ms(50);
    }
    else
        printk(KERN_ERR DRV_NAME "SDN GPIO %d invalid\n", dev->sdn_pin);
}

// Apply the radio configuration
static void apply_startup_config(struct si446x *dev)
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

u8 si446x_sleep(struct si446x *dev)
{
    if (get_state(dev) == SI446X_STATE_TX)
        return 0;
    set_state(dev, SI446X_STATE_SLEEP);
    return 1;
}

u16 si446x_adc_gpio(struct si446x *dev, u8 pin)
{
    u16 result = get_adc(dev, SI446X_ADC_CONV_GPIO | pin, (SI446X_ADC_SPEED << 4) | SI446X_ADC_RANGE_3P6, 0);
    return result;
}

u16 si446x_adc_battery(struct si446x *dev)
{
    u16 result = get_adc(dev, SI446X_ADC_CONV_BATT, (SI446X_ADC_SPEED << 4), 2);
    result = ((u32)result * 75) / 32; // result * 2.34375;
    return result;
}

/**
 * @brief Get ADC temperature
 * 
 * @param dev 
 * @return s32 Scale by (899/4096) after subtraction with 293
 */
s32 si446x_adc_temperature(struct si446x *dev)
{
    s32 result = get_adc(dev, SI446X_ADC_CONV_TEMP, (SI446X_ADC_SPEED << 4), 4);
    return result;
}

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

u8 si446x_read_gpio(struct si446x *dev)
{
    u8 states;
    u8 data[4] = {
        SI446X_CMD_GPIO_PIN_CFG};
    si446x_do_api(dev, data, 1, data, sizeof(data));
    states = data[0] >> 7 | (data[1] & 0x80) >> 6 | (data[2] & 0x80) >> 5 | (data[3] & 0x80) >> 4;
    return states;
}

u8 si446x_dump(struct si446x *dev, void *buff, uint8_t group)
{
    static const u8 groupSizes[] = {
        SI446X_PROP_GROUP_GLOBAL, 0x0A,
        SI446X_PROP_GROUP_INT, 0x04,
        SI446X_PROP_GROUP_FRR, 0x04,
        SI446X_PROP_GROUP_PREAMBLE, 0x0E,
        SI446X_PROP_GROUP_SYNC, 0x06,
        SI446X_PROP_GROUP_PKT, 0x40,
        SI446X_PROP_GROUP_MODEM, 0x60,
        SI446X_PROP_GROUP_MODEM_CHFLT, 0x24,
        SI446X_PROP_GROUP_PA, 0x07,
        SI446X_PROP_GROUP_SYNTH, 0x08,
        SI446X_PROP_GROUP_MATCH, 0x0C,
        SI446X_PROP_GROUP_FREQ_CONTROL, 0x08,
        SI446X_PROP_GROUP_RX_HOP, 0x42,
        SI446X_PROP_GROUP_PTI, 0x04};

    u8 length, i;
    length = 0;
    for (i = 0; i < sizeof(groupSizes); i += 2)
    {
        u8 buff[2];
        memcpy(buff, &groupSizes[i], sizeof(buff));

        if (buff[0] == group)
        {
            length = buff[1];
            break;
        }
    }

    if (buff == NULL)
        return length;

    for (i = 0; i < length; i += 16)
    {
        u8 count = length - i;
        if (count > 16)
            count = 16;
        si446x_get_props(dev, (group << 8) | i, ((u8 *)buff) + i, count);
    }

    return length;
}

void si446x_set_tx_power(struct si446x *dev, u8 pwr)
{
    set_property(dev, SI446X_PA_PWR_LVL, pwr);
}

void si446x_set_low_batt(struct si446x *dev, u16 voltage)
{
    u8 batt;
    batt = (voltage / 50) - 30;
    set_property(dev, SI446X_GLOBAL_LOW_BATT_THRESH, batt);
}

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
        delay_us(300); // Need to wait 300us for clock source to stabilize, see GLOBAL_WUT_CONFIG:WUT_EN info
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

static void si446x_internal_read(struct si446x *dev, uint8_t *buf, ssize_t len)
{
    struct spi_device *spi;
    int ret;
    u8 *din = (uint8_t *)kmalloc(len + 1, GFP_NOWAIT);
    u8 *dout = (uint8_t *)kmalloc(len + 1, GFP_NOWAIT);
    struct spi_transfer tx = {
        .tx_buf = dout,
        .len = 2,
    };
    struct spi_transfer rx = {
        .rx_buf = din,
        .len = 2,
    };
    struct spi_message msg;
    memset(dout, 0xff, len + 1);
    dout[0] = SI446X_CMD_READ_RX_FIFO;
    spi = dev->spi;

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
    set_state(dev, SI446X_STATE_RX);
    kfree(dout);
    kfree(din);
}

static void si446x_internal_write(struct si446x *dev, u8 *buf, int len)
{
    u8 *dout;
    int ret;
    struct spi_message msg;
    struct spi_device *spi;
    dout = (u8 *)kmalloc(len + 2, GFP_NOWAIT);
    struct spi_transfer tx = {
        .tx_buf = dout,
        .len = len + 2,
    };
    struct spi_transfer rx = {
        .rx_buf = NULL,
        .len = 0,
    };
    memset(dout, 0xff, len + 1);
    dout[0] = SI446X_CMD_WRITE_TX_FIFO;
    dout[1] = len;
    memcpy(dout + 2, buf, len);
    spi_message_init(&msg);
    spi_message_add_tail(&tx, &msg);
    spi_message_add_tail(&rx, &msg);
    mutex_lock(&(dev->lock));
    ret = spi_sync(spi, &msg); // assuming negative on error, zero on success
    mutex_unlock(&(dev->lock));
    if (ret)
    {
        printk(KERN_ERR DRV_NAME "Error in internal write\n");
    }
}

static int si446x_tx(struct si446x *dev, void *packet, u8 len, u8 channel, si446x_state_t onTxFinish)
{
    int retcode;
    if (len == 0)
    {
        retcode = 0;
        goto ret;
    }
    interrupt_off(dev);

    if (get_state(dev) == SI446X_STATE_TX)
    {
        retcode = 0;
        goto cleanup;
    }

    set_state(dev, SI446X_IDLE_MODE);
    clear_fifo(dev);
    interrupt2(dev, NULL, 0, 0, 0xFF);

    // Load data to FIFO
    si446x_internal_write(dev, packet, len);

    // Begin transmit
    u8 data[] = {
        SI446X_CMD_START_TX,
        channel,
        (u8)(onTxFinish << 4),
        0,
        len,
        0,
        0};
    si446x_do_api(dev, data, sizeof(data), NULL, 0);
    retcode = len;
cleanup:
    interrupt_on(dev);
ret:
    return retcode;
}

static void si446x_irq_work_handler(struct work_struct *work)
{
    int irq_avail;
    u8 len;
    bool read_rx_fifo = false;
    struct si446x *dev = container_of(work, struct si446x, irq_work);
    mutex_lock(&(dev->isr_lock));
    while ((irq_avail = gpio_get_value(dev->nirq_pin)) == 0) // IRQ low
    {
        u8 interrupts[8];
        interrupt(dev, interrupts);
        interrupts[2] &= dev->enabledInterrupts[IRQ_PACKET];
        interrupts[4] &= dev->enabledInterrupts[IRQ_MODEM];
        interrupts[6] &= dev->enabledInterrupts[IRQ_CHIP];
        // valid packet
        if (interrupts[2] & (1 << SI446X_PACKET_RX_PEND))
        {
            len = 0;
            si446x_internal_read(dev, &len, 1);
            dev->rssi = get_latched_rssi(dev);
            SI446X_CB_RXCOMPLETE(len, dev->rssi);
            if ((len != 0xff) && (len != 0))
                read_rx_fifo = true;
        }
        // corrupted packet
        if (interrupts[2] & (1 << SI446X_CRC_ERROR_PEND))
        {
            dev->rx_corrupt_ctr++;
            SI446X_CB_RXINVALID(get_latched_rssi(dev));
        }
        // packet sent
        if (interrupts[2] & (1 << SI446X_PACKET_SENT_PEND))
        {
            dev->tx_pk_ctr++;
            SI446X_CB_SENT();
        }
        // low battery
        if (interrupts[6] && (1 << SI446X_LOW_BATT_PEND))
        {
            SI446X_CB_LOWBATT();
        }
        // WUT
        if (interrupts[6] & (1 << SI446X_WUT_PEND))
        {
            SI446X_CB_WUT();
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
                int remainder = len % CIRC_SPACE_TO_END(head, tail, dev->rxbuf_len);
                int seq_len = len - remainder;
                /* Write the block making sure to wrap around the end of the buffer */
                memcpy(dev->rxbuf->buf + head, buff, seq_len);
                memcpy(dev->rxbuf->buf, buff + seq_len, remainder);
                WRITE_ONCE(dev->rxbuf->head, (head + len) & (dev->rxbuf_len - 1));
                dev->data_available = true;
                wake_up_interruptible(&rxq);
            }
            else
            {
                printk(KERN_ERR DRV_NAME "Error writing data, RX buffer full");
            }
            // TODO: wake up read
            read_rx_fifo = false;
            len = 0;
        }
    }
    mutex_unlock(&(dev->isr_lock));
}

static irqreturn_t si446x_irq(int irq, void *dev_id)
{
    struct si446x *dev = dev_id;
    schedule_work(&(dev->irq_work));
    return IRQ_HANDLED;
}

static ssize_t si446x_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
    struct si446x *dev;
    int head, tail, byte_count, out_count, remainder, seq_len;
    dev = (struct si446x *)(filp->private_data);
    if (count == 0)
        return 0;
    wait_event_interruptible(rxq, (dev->data_available == false)); // wait while byte count is zero
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
        dev->data_available = false;
    }
    else // should not trigger
    {
        dev->data_available = false;
        goto ret;
    }
    remainder = out_count % CIRC_SPACE_TO_END(head, tail, dev->rxbuf_len);
    seq_len = out_count - remainder;
    /* Write the block making sure to wrap around the end of the buffer */
    copy_to_user(buf, dev->rxbuf->buf + tail, seq_len);
    copy_to_user(buf + seq_len, dev->rxbuf->buf, remainder);
    WRITE_ONCE(dev->rxbuf->tail, (tail + out_count) & (dev->rxbuf_len - 1));
    return out_count;
ret:
    return 0;
}

static ssize_t si446x_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
    struct si446x *dev;
    ssize_t i, out_count;
    dev = (struct si446x *)(filp->private_data);
    if (count == 0)
    {
        return 0;
    }
    out_count = count;
    for (i = 0; out_count > SI446X_MAX_PACKET_LEN; i += SI446X_MAX_PACKET_LEN, out_count -= SI446X_MAX_PACKET_LEN) // write > 128 bytes
    {
        u8 buff[SI446X_MAX_PACKET_LEN];
        copy_from_user(buff, buf + i, SI446X_MAX_PACKET_LEN);
        while (si446x_tx(dev, buff, SI446X_MAX_PACKET_LEN, 0, SI446X_STATE_RX) == 0)
            ;
    }
    if (out_count > 0) // write last 128 bytes
    {
        u8 buff[SI446X_MAX_PACKET_LEN];
        copy_from_user(buff, buf + i, out_count);
        while (si446x_tx(dev, buff, out_count, 0, SI446X_STATE_RX) == 0)
            ;
    }
    return (i + out_count);
}

/**
 * static int my_open(struct inode *inode, struct file *file)
{
    struct my_device_data *my_data;

    my_data = container_of(inode->i_cdev, struct my_device_data, cdev);

    file->private_data = my_data;
    //...
}

static int my_read(struct file *file, char __user *user_buffer, size_t size, loff_t *offset)
{
    struct my_device_data *my_data;

    my_data = (struct my_device_data *) file->private_data;

    //...
}
 * 
 */

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