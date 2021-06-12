#include <linux/types.h>

static int si446x_open(struct inode *inod, struct file *filp);
static int si446x_release(struct inode *inod, struct file *filp);
static long si446x_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static ssize_t si446x_read(struct file *filp, char __user *buf, size_t count, loff_t *offset);
static ssize_t si446x_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset);