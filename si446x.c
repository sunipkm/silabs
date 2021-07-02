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

#include "si446x_kern.h"
#include "si446x_config.h"
#include "si446x_defs.h"
#include "si446x.h"

#define DRV_NAME "si446x"
#define DRV_VERSION "1.0a"
#define DEVICE_NAME "ttyUHF"

#ifndef SI446X_MAX_DEVICES
#define SI446X_MAX_DEVICES 5
#endif

#if SI446X_MAX_DEVICES <= 0
#undef SI446X_MAX_DEVICES
#define SI446X_MAX_DEVICES 1
#endif

#ifndef SI446X_FINISH_STATE
#define SI446X_FINISH_STATE SI446X_STATE_SLEEP
#endif

#define SI446X_CB_DEBUG 0

static dev_t device_num = 0; // major number
static dev_t minor_ct = 0;   // minor count

static struct class *si446x_class;

struct si446x
{
    struct cdev serdev;
    struct spi_device *spibus;    // spi bus
    struct mutex lock;            // mutex lock
    struct work_struct irq_work;  // IRQ handler
    struct work_struct init_work; // INIT handler
    int sdn_pin;                  // sdn pin
    int nirq_pin;                 // IRQ pin
    int cs_pin;                   // chip select gpio
    u32 tx_pk_ctr;                // stat for tx packets
    u32 rx_pk_ctr;                // stat for rx packets
    u32 rx_corrupt_ctr;           // stat for corrupt rx packets
    bool device_avail;            // Device available flag
    unsigned int isr_state;       // ISR lock state
    struct mutex isr_lock;        // mutex lock to prevent ISR from running
    u8 *config;                   // initialize as RADIO_CONFIGURATION_DATA_ARRAY
    u32 config_len;               // length of RADIO_CONFIGURATION_DATA_ARRAY
    u8 enabledInterrupts[3];      // enabled interrupts
    s16 rssi;                     // RSSI value returned by ioctl
    struct circ_buf *rxbuf;       // RX data buffer
    int rxbuf_len;                // rx buffer length
    bool data_available;          // indicate RX
    int open_ctr;                 // module use counter
    bool initd;                   // indicate initialization
    dev_t this_dev;               // this device
    struct completion initq;      // init queue
    struct wait_queue_head rxq;   // RX queue
    si446x_state_t on_tx_state;   // State after TX
    si446x_state_t on_rx_state;   // State after RX
};

static inline int interrupt_off(struct si446x *dev)
{
    int isr_state;
    isr_state = READ_ONCE(dev->isr_state);
    if (isr_state == 0)
        mutex_trylock(&(dev->isr_lock)); // prevent ISR from running
    WRITE_ONCE(dev->isr_state, isr_state + 1);
    return 1;
}

static inline int interrupt_on(struct si446x *dev)
{
    int isr_state;
    isr_state = READ_ONCE(dev->isr_state);
    if (READ_ONCE(dev->isr_state) > 0)
        WRITE_ONCE(dev->isr_state, isr_state - 1);
    isr_state = READ_ONCE(dev->isr_state);
    if (READ_ONCE(dev->isr_state) == 0)
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

#if SI446X_CB_DEBUG == 0
#define SI446X_CB_CMDTIMEOUT()
#else
void SI446X_CB_CMDTIMEOUT(void)
{
    printk(KERN_DEBUG DRV_NAME ": wait_for_response timed out\n");
}
#endif

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

    mutex_lock(&(dev->lock));
    ret = spi_sync_transfer(spi, &xfer, 1);
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
    struct spi_device *spi;
    int ret;
    struct spi_transfer tx = {
        .tx_buf = out,
        .rx_buf = NULL,
        .len = len,
        .bits_per_word = 8};
    spi = dev->spibus;

    mutex_lock(&(dev->lock));
    ret = spi_sync_transfer(spi, &tx, 1);
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
    int ret;
    ret = interrupt_off(dev);
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

        mutex_lock(&(dev->lock));
        ret = spi_sync_transfer(spi, &xfer, 1);
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

void si446x_get_info(struct si446x *dev, si446x_info_t *info)
{
    u8 out[1];
    u8 data[8];
    memset(data, 0x0, sizeof(data));
    out[0] = SI446X_CMD_PART_INFO;
    si446x_do_api(dev, out, 1, data, 8);

    info->chipRev = data[0];
    info->part = (data[1] << 8) | data[2];
    info->partBuild = data[3];
    info->id = (data[4] << 8) | data[5];
    info->customer = data[6];
    info->romId = data[7];

    out[0] = SI446X_CMD_FUNC_INFO;
    si446x_do_api(dev, out, 1, data, 6);

    info->revExternal = data[0];
    info->revBranch = data[1];
    info->revInternal = data[2];
    info->patch = (data[3] << 8) | data[4];
    info->func = data[5];
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

static s16 get_rssi(struct si446x *dev)
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
 * @return s32 Scale by (899/4096) and subtract 293
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

/*
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
*/

void si446x_set_tx_power(struct si446x *dev, u8 pwr)
{
    set_property(dev, SI446X_PA_PWR_LVL, pwr);
}

u8 si446x_get_tx_power(struct si446x *dev)
{
    u8 pwr;
    pwr = get_property(dev, SI446X_PA_PWR_LVL);
    return pwr;
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
    int ret, astate;
    u8 *din = (uint8_t *)kmalloc(len + 1, GFP_NOWAIT);
    u8 *dout = (uint8_t *)kmalloc(len + 1, GFP_NOWAIT);
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

    mutex_lock(&(dev->lock));
    ret = spi_sync_transfer(spi, &xfer, 1); // assuming negative on error, zero on success
    mutex_unlock(&(dev->lock));
    if (ret)
    {
        printk(KERN_ERR DRV_NAME "Error in internal read\n");
    }
    memcpy(buf, din + 1, len);
    set_state(dev, astate);
    kfree(dout);
    kfree(din);
}

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

    mutex_lock(&(dev->lock));
    ret = spi_sync_transfer(spi, &xfer, 1); // assuming negative on error, zero on success
    mutex_unlock(&(dev->lock));

    if (ret)
    {
        printk(KERN_ERR DRV_NAME "Error in internal write\n");
    }
    kfree(dout);
}

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
    data[0] = SI446X_CMD_START_TX;
    data[1] = channel;
    data[2] = (u8)(onTxFinish << 4);
    data[3] = 0;
    data[4] = len;
    data[5] = 0;
    data[6] = 0;

    si446x_do_api(dev, data, sizeof(data), NULL, 0);
    retcode = len;
cleanup:
    interrupt_on(dev);
ret:
    return retcode;
}

static void si446x_init_work_handler(struct work_struct *work)
{
    struct si446x *dev;
    dev = container_of(work, struct si446x, init_work);
    dev->data_available = false;
    dev->rxbuf->head = 0;
    dev->rxbuf->tail = 0;
    dev->isr_state = 0;
    reset_device(dev);
    printk(KERN_DEBUG DRV_NAME ": Device reset\n");
    printk(KERN_DEBUG DRV_NAME ": Applying startup config\n");
    apply_startup_config(dev);
    printk(KERN_DEBUG DRV_NAME ": Applied startup config\n");
    interrupt(dev, NULL);
    printk(KERN_DEBUG DRV_NAME ": Cleared IRQ vector\n");
    si446x_sleep(dev);
    printk(KERN_DEBUG DRV_NAME ": Device in sleep mode\n");
    dev->enabledInterrupts[IRQ_PACKET] = (1 << SI446X_PACKET_RX_PEND) | (1 << SI446X_CRC_ERROR_PEND);
    si446x_setup_callback(dev, SI446X_CBS_RXBEGIN, 1); // enable receive irq
    printk(KERN_DEBUG DRV_NAME ": Receive callback set\n");
    complete(&(dev->initq));
    printk(KERN_DEBUG DRV_NAME ": Signalled end of init\n");
}

static void si446x_irq_work_handler(struct work_struct *work)
{
    int irq_avail;
    u8 len;
    bool read_rx_fifo = false;
    struct si446x *dev;
    dev = container_of(work, struct si446x, irq_work);
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
            {
                read_rx_fifo = true;
                dev->rx_pk_ctr++;
            }
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
                int remainder = len % (CIRC_SPACE_TO_END(head, tail, dev->rxbuf_len) + 1);
                int seq_len = len - remainder;
                /* Write the block making sure to wrap around the end of the buffer */
                memcpy(dev->rxbuf->buf + head, buff, remainder);
                memcpy(dev->rxbuf->buf, buff + remainder, seq_len);
                WRITE_ONCE(dev->rxbuf->head, (head + len) & (dev->rxbuf_len - 1));
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
    mutex_unlock(&(dev->isr_lock));
}

static irqreturn_t si446x_irq(int irq, void *dev_id)
{
    struct si446x *dev = dev_id;
    if (dev->initd)
        schedule_work(&(dev->irq_work));
    return IRQ_HANDLED;
}

static ssize_t si446x_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
    struct si446x *dev;
    int head, tail, byte_count, out_count, remainder, seq_len;
    bool avail;
    dev = (struct si446x *)(filp->private_data);
    if (!dev->device_avail)
    {
        return -EHOSTDOWN;
    }
    if (!dev->initd)
    {
        return -EINVAL;
    }
    if ((filp->f_flags & O_ACCMODE) == O_WRONLY)
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
    remainder = out_count % (CIRC_SPACE_TO_END(head, tail, dev->rxbuf_len) + 1);
    seq_len = out_count - remainder;
    /* Write the block making sure to wrap around the end of the buffer */
    out_count -= copy_to_user(buf, dev->rxbuf->buf + tail, remainder);
    out_count -= copy_to_user(buf + remainder, dev->rxbuf->buf, seq_len);
    WRITE_ONCE(dev->rxbuf->tail, (tail + out_count) & (dev->rxbuf_len - 1));
    return out_count;
ret:
    return 0;
}

static ssize_t si446x_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
    struct si446x *dev;
    ssize_t i, out_count, err_count;
    int astate;
    dev = (struct si446x *)(filp->private_data);
    astate = READ_ONCE(dev->on_tx_state);
    if (!dev->device_avail)
    {
        return -EHOSTDOWN;
    }
    if (!dev->initd)
    {
        return -EINVAL;
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

static int si446x_open(struct inode *inod, struct file *filp)
{
    struct si446x *dev;
    struct cdev *tcdev;
    si446x_info_t info[1];
    int retval, i;
    int open_ctr;
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
        dev->initd = false;
        reset_device(dev);
        printk(KERN_DEBUG DRV_NAME ": Reset device\n");
        si446x_get_info(dev, info);
        printk(KERN_DEBUG DRV_NAME ": Get info in open\n");
        if ((info->part & 0x4460) != 0x4460)
        {
            printk(KERN_ERR DRV_NAME ": Not Si446x device, returned 0x%x\n", info->part);
            retval = -EHOSTDOWN;          // signal that host is down
            WRITE_ONCE(dev->open_ctr, 0); // reset open counter
        }
        else
        {
            WRITE_ONCE(dev->device_avail, true);
        }
    }
    return retval;
}

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
        reset_device(dev);
        si446x_sleep(dev);
        dev->enabledInterrupts[IRQ_PACKET] = 0;
        dev->isr_state = 0;
        dev->initd = false;
        dev->device_avail = false;
        mutex_unlock(&(dev->isr_lock)); // force unlock irq lock
    }
    return 0;
}

static long si446x_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct si446x *dev;
    long retval;
    void *ptr;
    dev = (struct si446x *)filp->private_data;
    ptr = (void __user *)arg;
    if (!dev->device_avail)
    {
        return -EHOSTDOWN;
    }
    switch (cmd)
    {
    case SI446X_SET_STATE:
    {
        int st;
        if (!dev->initd)
        {
            retval = -EACCES;
            break;
        }
        if ((st < 0) || (st > SI446X_STATE_RX))
        {
            retval = -EINVAL;
            break;
        }
        retval = -copy_from_user_nofault(&st, ptr, sizeof(int));
        if (!retval)
            set_state(dev, st);
        printk(KERN_DEBUG DRV_NAME "ioctl set state %d\n", st);
        break;
    }
    case SI446X_SET_ONTX_STATE:
    {
        int st;
        if (!dev->initd)
        {
            retval = -EACCES;
            break;
        }
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
        if (!dev->initd)
        {
            retval = -EACCES;
            break;
        }
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
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        st = get_state(dev);
        retval = -copy_to_user_nofault(ptr, &st, sizeof(int));
        printk(KERN_DEBUG DRV_NAME "ioctl get state %d\n", st);
        break;
    }
    case SI446X_GET_LATCHED_RSSI:
    {
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        retval = -copy_to_user_nofault(ptr, &(dev->rssi), sizeof(s16));
        break;
    }
    case SI446X_GET_RSSI:
    {
        s16 rssi;
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        rssi = get_rssi(dev);
        retval = -copy_to_user_nofault(ptr, &rssi, sizeof(s16));
        break;
    }
    case SI446X_GET_INFO:
    {
        si446x_info_t info[1];
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        si446x_get_info(dev, info);
        retval = -copy_to_user_nofault(ptr, info, sizeof(si446x_info_t));
        break;
    }
    case SI446X_DISABLE_WUT:
    {
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        si446x_disable_wut(dev);
        retval = 1;
        break;
    }
    case SI446X_SETUP_WUT:
    {
        struct SI446X_WUT_CONFIG conf[1];
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        retval = -copy_from_user_nofault(conf, ptr, sizeof(struct SI446X_WUT_CONFIG));
        if (!retval)
            si446x_setup_wut(dev, conf->r, conf->m, conf->ldc, conf->config);
        break;
    }
    case SI446X_GET_TX_PWR:
    {
        u8 pwr;
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        pwr = si446x_get_tx_power(dev);
        retval = -copy_to_user_nofault(ptr, &pwr, sizeof(u8));
        break;
    }
    case SI446X_SET_TX_PWR:
    {
        u8 pwr;
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
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
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        st = si446x_read_gpio(dev);
        retval = -copy_to_user_nofault(ptr, &st, sizeof(s8));
        break;
    }
    case SI446X_WR_GPIO:
    {
        struct SI446X_GPIO_CONFIG conf[1];
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        retval = -copy_from_user_nofault(conf, ptr, sizeof(struct SI446X_GPIO_CONFIG));
        si446x_write_gpio(dev, conf->pin, conf->val);
        break;
    }
    case SI446X_SET_LOW_BATT:
    {
        u16 volt;
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        retval = -copy_from_user_nofault(&volt, ptr, sizeof(u16));
        si446x_set_low_batt(dev, volt);
        break;
    }
    case SI446X_ADC_BATT:
    {
        u16 batt;
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        batt = si446x_adc_battery(dev);
        retval = -copy_to_user_nofault(ptr, &batt, sizeof(u16));
        break;
    }
    case SI446X_ADC_GPIO:
    {
        struct SI446X_ADC_GPIO_MEM gp[1];
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
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
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
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
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        retval = -copy_from_user_nofault(conf, ptr, sizeof(struct SI446X_ADC_CONFIG));
        if (retval)
        {
            break;
        }
        conf->ret = get_adc(dev, conf->en, conf->cfg, conf->part);
        retval = -copy_to_user_nofault(ptr, conf, sizeof(struct SI446X_ADC_CONFIG));
        break;
    }
    case SI446X_RD_RX_BUF_SZ:
    {
        if (!dev->initd)
        {
            retval = -EINVAL;
            break;
        }
        retval = -copy_to_user_nofault(ptr, &(dev->rxbuf_len), sizeof(int));
        break;
    }
    case SI446X_INIT:
    {
        struct SI446X_INIT_PROPS init_props[1];
        u8 config[] = RADIO_CONFIGURATION_DATA_ARRAY;
        reset_device(dev);
        memset(init_props, 0x0, sizeof(struct SI446X_INIT_PROPS));
        if (arg == 0)
        {
            printk(KERN_DEBUG DRV_NAME ": Using default config\n");
            init_props->len = sizeof(config);
            retval = 0;
        }
        else
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
        if (arg)
        {
            retval = copy_from_user(dev->config, (void *__user)init_props->config, dev->config_len);
        }
        else
        {
            memcpy(dev->config, config, dev->config_len);
        }
        if (retval)
        {
            retval = -ENOMEM;
            break;
        }
        // now we have config, we can go with init
        init_completion(&(dev->initq));
        schedule_work(&(dev->init_work));
        if (!wait_for_completion_interruptible(&(dev->initq)))
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

    pdev = &(spi->dev);

    dev->spibus = spi;

    spi_set_drvdata(spi, dev);

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
    ret = gpio_request(sdn_pin, DRV_NAME "_gpio_sdn");
    if (ret)
    {
        printk(KERN_ERR DRV_NAME ": Error requesting gpio pin %d, status %d\n", sdn_pin, ret);
        ret = -ENODEV;
        goto err_main;
    }
    if (!gpio_is_valid(sdn_pin))
    {
        printk(KERN_ERR DRV_NAME ": %d not valid GPIO pin\n", sdn_pin);
        ret = -ENODEV;
        goto err_main;
    }
    ret = gpio_direction_output(sdn_pin, 0);
    if (ret)
    {
        printk(KERN_ERR DRV_NAME ": Error %d setting GPIO output direction\n", ret);
        ret = -ENODEV;
        goto err_main;
    }
    ret = gpio_request(nirq_pin, DRV_NAME "_gpio_nirq");
    if (ret)
    {
        printk(KERN_ERR DRV_NAME ": Error requesting gpio pin %d, status %d\n", nirq_pin, ret);
        ret = -ENODEV;
        goto err_main;
    }
    if (!gpio_is_valid(nirq_pin))
    {
        printk(KERN_ERR DRV_NAME ": %d not valid GPIO pin\n", nirq_pin);
        ret = -ENODEV;
        goto err_main;
    }

    dev->sdn_pin = sdn_pin;
    dev->nirq_pin = nirq_pin;
    if (!minor_ct) // only for the first probe create class
        si446x_class = class_create(THIS_MODULE, DRV_NAME "_class");
    if (!si446x_class)
    {
        ret = -ENOMEM;
        printk(KERN_ERR DRV_NAME ": Error allocating memory for class\n");
        goto err_main;
    }

    dev->rxbuf = kmalloc(sizeof(struct circ_buf), GFP_KERNEL);
    if (!(dev->rxbuf))
    {
        ret = -ENOMEM;
        printk(KERN_ERR DRV_NAME ": Error allocating memory for receiver buffer\n");
        goto err_main;
    }
    if ((dev->rxbuf_len < SI446X_MAX_PACKET_LEN) || (dev->rxbuf_len > 128 * SI446X_MAX_PACKET_LEN)) // malloc
        dev->rxbuf_len = 16 * SI446X_MAX_PACKET_LEN;
    dev->rxbuf->buf = kzalloc(dev->rxbuf_len, GFP_KERNEL);
    if (!dev->rxbuf->buf)
    {
        ret = -ENOMEM;
        printk(KERN_ERR DRV_NAME ": Error allocating memory for receiver buffer\n");
        goto err_alloc_buf;
    }
    ret = 0; // making sure ret = 0 at this point
    if (!minor_ct)
        ret = alloc_chrdev_region(&device_num, 0, SI446X_MAX_DEVICES, DRV_NAME "_" DEVICE_NAME);
    if ((!ret) || (minor_ct > 0))
    {
        ma = MAJOR(device_num);
        mi = minor_ct++;
        this_dev = MKDEV(ma, mi);
        cdev_init(&(dev->serdev), &si446x_fops);
        ret = cdev_add(&(dev->serdev), this_dev, 1);
        if (ret)
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

    mutex_init(&(dev->lock));
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
    dev->open_ctr = 0;                      // init counter 0
    dev->device_avail = false;              // device not available by default
    dev->initd = false;                     // device not initiated by default
    dev->tx_pk_ctr = 0;                     // initialize tx packet counter to zero
    dev->rx_pk_ctr = 0;                     // initialize rx packet counter to zero
    dev->rx_corrupt_ctr = 0;                // initialize rx corrupt packet counter to zero
    dev->on_tx_state = SI446X_FINISH_STATE; // default: sleep on finish
    dev->on_rx_state = SI446X_FINISH_STATE; // default: sleep on finish
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

static const struct of_device_id si446x_dt_ids[] = {
    {.compatible = "silabs,si446x"},
    {/* sentinel */}};
MODULE_DEVICE_TABLE(of, si446x_dt_ids);

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