/*
 * linux_virtfifo.c - Virtual FIFO Ring-Buffer Character Device Driver
 *
 * Demonstrates character device registration, dynamic ring-buffer management,
 * wait queues, spinlocks, non-blocking I/O, epoll/select polling, and sysfs.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/sysfs.h>
#include <linux/errno.h>

#define DRIVER_NAME       "virtfifo"
#define DEFAULT_BUF_SIZE  4096
#define MAX_DEVICES       4

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lohitaksha");
MODULE_DESCRIPTION("Production-style Linux character driver for virtual FIFO buffer");
MODULE_VERSION("1.0.0");

static int num_devices = 1;
module_param(num_devices, int, 0444);
MODULE_PARM_DESC(num_devices, "Number of FIFO devices to create (1 to 4)");

static int default_buf_size = DEFAULT_BUF_SIZE;
module_param(default_buf_size, int, 0644);
MODULE_PARM_DESC(default_buf_size, "Default ring-buffer capacity in bytes");

/* Per-device state structure */
struct virtfifo_dev {
	dev_t devno;
	struct cdev cdev;
	struct device *device;
	
	spinlock_t lock;
	wait_queue_head_t read_queue;
	wait_queue_head_t write_queue;

	char *buffer;
	size_t capacity;
	size_t head;
	size_t tail;
	size_t count;

	size_t total_reads;
	size_t total_writes;
};

static dev_t virtfifo_devno_base;
static struct class *virtfifo_class;
static struct virtfifo_dev *virtfifo_devices;

/* FIFO helper utilities (must be called with dev->lock held) */
static inline size_t fifo_space_available(const struct virtfifo_dev *dev)
{
	return dev->capacity - dev->count;
}

static inline size_t fifo_data_available(const struct virtfifo_dev *dev)
{
	return dev->count;
}

static void fifo_reset_nolock(struct virtfifo_dev *dev)
{
	dev->head = 0;
	dev->tail = 0;
	dev->count = 0;
}

/* File Operations Implementation */

static int virtfifo_open(struct inode *inode, struct file *filp)
{
	struct virtfifo_dev *dev;

	dev = container_of(inode->i_cdev, struct virtfifo_dev, cdev);
	filp->private_data = dev;

	return 0;
}

static int virtfifo_release(struct inode *inode, struct file *filp)
{
	/* No special teardown needed on close */
	return 0;
}

static ssize_t virtfifo_read(struct file *filp, char __user *buf,
			     size_t count, loff_t *f_pos)
{
	struct virtfifo_dev *dev = filp->private_data;
	size_t bytes_to_read;
	size_t first_chunk;
	size_t second_chunk;
	unsigned long flags;

	if (unlikely(!buf || count == 0))
		return 0;

	spin_lock_irqsave(&dev->lock, flags);

	while (fifo_data_available(dev) == 0) {
		spin_unlock_irqrestore(&dev->lock, flags);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(dev->read_queue,
					     fifo_data_available(dev) > 0)) {
			return -ERESTARTSYS;
		}

		spin_lock_irqsave(&dev->lock, flags);
	}

	bytes_to_read = min(count, fifo_data_available(dev));

	/* Calculate ring buffer boundary split */
	first_chunk = min(bytes_to_read, dev->capacity - dev->head);
	second_chunk = bytes_to_read - first_chunk;

	spin_unlock_irqrestore(&dev->lock, flags);

	/* Copy payload outside spinlock to avoid sleep-in-atomic issues */
	if (copy_to_user(buf, dev->buffer + dev->head, first_chunk))
		return -EFAULT;

	if (second_chunk > 0) {
		if (copy_to_user(buf + first_chunk, dev->buffer, second_chunk))
			return -EFAULT;
	}

	spin_lock_irqsave(&dev->lock, flags);

	dev->head = (dev->head + bytes_to_read) % dev->capacity;
	dev->count -= bytes_to_read;
	dev->total_reads += bytes_to_read;

	spin_unlock_irqrestore(&dev->lock, flags);

	wake_up_interruptible(&dev->write_queue);

	return bytes_to_read;
}

static ssize_t virtfifo_write(struct file *filp, const char __user *buf,
			      size_t count, loff_t *f_pos)
{
	struct virtfifo_dev *dev = filp->private_data;
	size_t bytes_to_write;
	size_t first_chunk;
	size_t second_chunk;
	unsigned long flags;

	if (unlikely(!buf || count == 0))
		return 0;

	spin_lock_irqsave(&dev->lock, flags);

	while (fifo_space_available(dev) == 0) {
		spin_unlock_irqrestore(&dev->lock, flags);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(dev->write_queue,
					     fifo_space_available(dev) > 0)) {
			return -ERESTARTSYS;
		}

		spin_lock_irqsave(&dev->lock, flags);
	}

	bytes_to_write = min(count, fifo_space_available(dev));

	first_chunk = min(bytes_to_write, dev->capacity - dev->tail);
	second_chunk = bytes_to_write - first_chunk;

	spin_unlock_irqrestore(&dev->lock, flags);

	if (copy_from_user(dev->buffer + dev->tail, buf, first_chunk))
		return -EFAULT;

	if (second_chunk > 0) {
		if (copy_from_user(dev->buffer, buf + first_chunk, second_chunk))
			return -EFAULT;
	}

	spin_lock_irqsave(&dev->lock, flags);

	dev->tail = (dev->tail + bytes_to_write) % dev->capacity;
	dev->count += bytes_to_write;
	dev->total_writes += bytes_to_write;

	spin_unlock_irqrestore(&dev->lock, flags);

	wake_up_interruptible(&dev->read_queue);

	return bytes_to_write;
}

static __poll_t virtfifo_poll(struct file *filp, poll_table *wait)
{
	struct virtfifo_dev *dev = filp->private_data;
	__poll_t mask = 0;
	unsigned long flags;

	poll_wait(filp, &dev->read_queue, wait);
	poll_wait(filp, &dev->write_queue, wait);

	spin_lock_irqsave(&dev->lock, flags);

	if (fifo_data_available(dev) > 0)
		mask |= EPOLLIN | EPOLLRDNORM;

	if (fifo_space_available(dev) > 0)
		mask |= EPOLLOUT | EPOLLWRNORM;

	spin_unlock_irqrestore(&dev->lock, flags);

	return mask;
}

static const struct file_operations virtfifo_fops = {
	.owner          = THIS_MODULE,
	.open           = virtfifo_open,
	.release        = virtfifo_release,
	.read           = virtfifo_read,
	.write          = virtfifo_write,
	.poll           = virtfifo_poll,
	.llseek         = no_llseek,
};

/* Sysfs Attributes */

static ssize_t stats_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct virtfifo_dev *vdev = dev_get_drvdata(dev);
	unsigned long flags;
	size_t cap, count, reads, writes;

	spin_lock_irqsave(&vdev->lock, flags);
	cap    = vdev->capacity;
	count  = vdev->count;
	reads  = vdev->total_reads;
	writes = vdev->total_writes;
	spin_unlock_irqrestore(&vdev->lock, flags);

	return sysfs_emit(buf,
		"capacity: %zu\n"
		"used:     %zu\n"
		"free:     %zu\n"
		"total_r:  %zu\n"
		"total_w:  %zu\n",
		cap, count, cap - count, reads, writes);
}

static ssize_t clear_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct virtfifo_dev *vdev = dev_get_drvdata(dev);
	unsigned long flags;

	if (count > 0 && buf[0] == '1') {
		spin_lock_irqsave(&vdev->lock, flags);
		fifo_reset_nolock(vdev);
		spin_unlock_irqrestore(&vdev->lock, flags);

		wake_up_interruptible(&vdev->write_queue);
		pr_info("FIFO buffer reset via sysfs on dev minor %d\n", MINOR(vdev->devno));
	}

	return count;
}

static DEVICE_ATTR_RO(stats);
static DEVICE_ATTR_WO(clear);

static struct attribute *virtfifo_attrs[] = {
	&dev_attr_stats.attr,
	&dev_attr_clear.attr,
	NULL,
};
ATTRIBUTE_GROUPS(virtfifo);

/* Module Init and Exit */

static int __init virtfifo_init(void)
{
	int ret;
	int i;

	if (num_devices < 1 || num_devices > MAX_DEVICES) {
		pr_err("Invalid num_devices=%d (must be 1..%d)\n", num_devices, MAX_DEVICES);
		return -EINVAL;
	}

	ret = alloc_chrdev_region(&virtfifo_devno_base, 0, num_devices, DRIVER_NAME);
	if (ret < 0) {
		pr_err("Failed to allocate chrdev region: %d\n", ret);
		return ret;
	}

	virtfifo_class = class_create(DRIVER_NAME);
	if (IS_ERR(virtfifo_class)) {
		ret = PTR_ERR(virtfifo_class);
		pr_err("Failed to create sysfs class: %d\n", ret);
		goto unregister_chrdev;
	}

	virtfifo_devices = kcalloc(num_devices, sizeof(struct virtfifo_dev), GFP_KERNEL);
	if (!virtfifo_devices) {
		ret = -ENOMEM;
		goto destroy_class;
	}

	for (i = 0; i < num_devices; i++) {
		struct virtfifo_dev *vdev = &virtfifo_devices[i];

		vdev->devno = MKDEV(MAJOR(virtfifo_devno_base), i);
		vdev->capacity = default_buf_size;

		vdev->buffer = kzalloc(vdev->capacity, GFP_KERNEL);
		if (!vdev->buffer) {
			ret = -ENOMEM;
			goto rollback_devices;
		}

		spin_lock_init(&vdev->lock);
		init_waitqueue_head(&vdev->read_queue);
		init_waitqueue_head(&vdev->write_queue);

		cdev_init(&vdev->cdev, &virtfifo_fops);
		vdev->cdev.owner = THIS_MODULE;

		ret = cdev_add(&vdev->cdev, vdev->devno, 1);
		if (ret < 0) {
			pr_err("Failed to add cdev for minor %d: %d\n", i, ret);
			kfree(vdev->buffer);
			goto rollback_devices;
		}

		vdev->device = device_create_with_groups(virtfifo_class, NULL,
							 vdev->devno, vdev,
							 virtfifo_groups,
							 "virtfifo%d", i);
		if (IS_ERR(vdev->device)) {
			ret = PTR_ERR(vdev->device);
			pr_err("Failed to create device node virtfifo%d: %d\n", i, ret);
			cdev_del(&vdev->cdev);
			kfree(vdev->buffer);
			goto rollback_devices;
		}
	}

	pr_info("Driver loaded successfully with %d device(s)\n", num_devices);
	return 0;

rollback_devices:
	while (--i >= 0) {
		device_destroy(virtfifo_class, virtfifo_devices[i].devno);
		cdev_del(&virtfifo_devices[i].cdev);
		kfree(virtfifo_devices[i].buffer);
	}
	kfree(virtfifo_devices);

destroy_class:
	class_destroy(virtfifo_class);

unregister_chrdev:
	unregister_chrdev_region(virtfifo_devno_base, num_devices);
	return ret;
}

static void __exit virtfifo_exit(void)
{
	int i;

	for (i = 0; i < num_devices; i++) {
		device_destroy(virtfifo_class, virtfifo_devices[i].devno);
		cdev_del(&virtfifo_devices[i].cdev);
		kfree(virtfifo_devices[i].buffer);
	}

	kfree(virtfifo_devices);
	class_destroy(virtfifo_class);
	unregister_chrdev_region(virtfifo_devno_base, num_devices);

	pr_info("Driver unloaded\n");
}

module_init(virtfifo_init);
module_exit(virtfifo_exit);
