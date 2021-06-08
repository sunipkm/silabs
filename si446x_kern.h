#include <linux/types.h>

static int si446x_open(struct inode *inode, struct file *file);
static int si446x_release(struct inode *inode, struct file *file);
static long si446x_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t si446x_read(struct file *file, char __user *buf, size_t count, loff_t *offset);
static ssize_t si446x_write(struct file *file, const char __user *buf, size_t count, loff_t *offset);