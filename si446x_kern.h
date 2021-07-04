/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * @file si446x_kern.h
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Si446x Kernel Methods
 * @version 1.0a
 * @date 2021-06-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _SI446X_KERN_H
#define _SI446X_KERN_H
#include <linux/types.h>
/**
 * @brief Opens an Si446x device. Open is MT safe.
 * 
 */
static int si446x_open(struct inode *inod, struct file *filp);
/**
 * @brief Closes an Si446x device. Close is MT safe.
 * 
 */
static int si446x_release(struct inode *inod, struct file *filp);
/**
 * @brief Performs ioctl on an Si446x device. ioctl is MT safe.
 * 
 */
static long si446x_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
/**
 * @brief Reads bytes from RX buffer of an Si446x device. This call is MT safe,
 * but MT behavior sanity is not guaranteed. Read should be performed only from
 * one process.
 * 
 */
static ssize_t si446x_read(struct file *filp, char __user *buf, size_t count, loff_t *offset);
/**
 * @brief Transmits bytes over RF. This call is MT safe, however packet ordering
 * is not guaranteed. Write should be performed only from one process at a time.
 * 
 */
static ssize_t si446x_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset);
/**
 * @brief Polls for data availability on RX buffer. This call is MT safe.
 * 
 */
static unsigned int si446x_poll(struct file *filp, struct poll_table_struct *poll_table);
#endif