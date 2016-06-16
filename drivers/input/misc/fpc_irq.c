#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/sort.h>
#include <linux/signal.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <net/sock.h>
#include <net/genetlink.h>
#include <linux/semaphore.h>

#define FPC1020_IRQ_NAME "fpc1020_irq"

extern struct semaphore sem_fp_int;

//static struct semaphore fpc_irq_mutex;

static struct class *fpc_irq_class;
static dev_t fpc_irq_device_no;
static struct cdev fpc_irq_cdev;
static struct device *fpc_irq_dev;

static int fpc_irq_open(struct inode *inode, struct file *file);
static ssize_t fpc_irq_write(struct file *file, const char *buff, size_t count, loff_t *ppos);
static ssize_t fpc_irq_read(struct file *file, char *buff, size_t count, loff_t *ppos);
static int fpc_irq_release(struct inode *inode, struct file *file);
static unsigned int fpc_irq_poll(struct file *file, poll_table *wait);
/* -------------------------------------------------------------------- */
static int fpc_irq_open(struct inode *inode, struct file *file)

{
	printk(KERN_INFO "%s\n", __func__);
#if 0
	if (down_interruptible(&fpc_irq_mutex))
		return -ERESTARTSYS;

	up(&fpc_irq_mutex);
#endif
	return 0;
}
/* -------------------------------------------------------------------- */
static ssize_t fpc_irq_write(struct file *file, const char *buff,size_t count, loff_t *ppos)
{
	printk(KERN_INFO "%s\n", __func__);

	return -ENOTTY;
}

static char fpc_irq[8] = "FPC_IRQ";
/* -------------------------------------------------------------------- */
static ssize_t fpc_irq_read(struct file *file, char *buff, size_t count, loff_t *ppos)
{
	int error = -ENOENT;

	if(down_interruptible(&sem_fp_int))
		return -ERESTARTSYS;

	error = copy_to_user(buff, fpc_irq, 8);
	
	if(error != 0)
		return error;

	error = 8;

#if 0
	printk(KERN_INFO "%s\n", __func__);
	if (down_interruptible(&fpc_irq_mutex))
		return -ERESTARTSYS;


	error = copy_to_user(buff, fpc_irq, 8);
	up(&fpc_irq_mutex);
#endif
	return error;
}


/* -------------------------------------------------------------------- */
static int fpc_irq_release(struct inode *inode, struct file *file)
{
	int status = 0;

	printk(KERN_INFO "%s\n", __func__);

	return status;
}

/* -------------------------------------------------------------------- */
static unsigned int fpc_irq_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;
#if 0
	fpc1020_data_t *fpc1020 = file->private_data;
	fpc1020_capture_mode_t mode = fpc1020->setup.capture_mode;
	bool blocking_op;

	if (down_interruptible(&fpc1020->mutex))
		return -ERESTARTSYS;

	if (fpc1020->capture.available_bytes > 0)
		ret |= (POLLIN | POLLRDNORM);
	else if (fpc1020->capture.read_pending_eof)
		ret |= POLLHUP;
	else { /* available_bytes == 0 && !pending_eof */

		blocking_op =
			(mode == FPC1020_MODE_WAIT_AND_CAPTURE) ? true : false;

		switch (fpc1020->capture.state) {
			case FPC1020_CAPTURE_STATE_IDLE:
				if (!blocking_op)
					ret |= POLLIN;
				break;

			case FPC1020_CAPTURE_STATE_STARTED:
			case FPC1020_CAPTURE_STATE_PENDING:
			case FPC1020_CAPTURE_STATE_WRITE_SETTINGS:
			case FPC1020_CAPTURE_STATE_WAIT_FOR_FINGER_DOWN:
			case FPC1020_CAPTURE_STATE_ACQUIRE:
			case FPC1020_CAPTURE_STATE_FETCH:
			case FPC1020_CAPTURE_STATE_WAIT_FOR_FINGER_UP:
			case FPC1020_CAPTURE_STATE_COMPLETED:
				ret |= POLLIN;

				poll_wait(file, &fpc1020->capture.wq_data_avail, wait);

				if (fpc1020->capture.available_bytes > 0)
					ret |= POLLRDNORM;
				else if (blocking_op)
					ret = 0;

				break;

			case FPC1020_CAPTURE_STATE_FAILED:
				if (!blocking_op)
					ret |= POLLIN;
				break;

			default:
				dev_err(&fpc1020->spi->dev,
						"%s unknown state\n", __func__);
				break;
		}
	}

	up(&fpc1020->mutex);
#endif
	return ret;
}



static const struct file_operations fpc_irq_fops = {
	.owner          = THIS_MODULE,
	.open           = fpc_irq_open,
	.write          = fpc_irq_write,
	.read           = fpc_irq_read,
	.release        = fpc_irq_release,
	.poll           = fpc_irq_poll,
};

static int fpc_irq_init(void)
{
	int rc;

	printk(KERN_ALERT "%s start create dev file\n",__func__);

	rc = alloc_chrdev_region(&fpc_irq_device_no, 0, 1, FPC1020_IRQ_NAME);
	if (rc < 0) {
		pr_err("alloc_chrdev_region failed %d\n", rc);
		return rc;
	}

	fpc_irq_class = class_create(THIS_MODULE, FPC1020_IRQ_NAME);
	if (IS_ERR(fpc_irq_class)) {
		rc = -ENOMEM;
		pr_err("class_create failed %d\n", rc);
		goto exit_unreg_chrdev_region;
	}

	fpc_irq_dev = device_create(fpc_irq_class, NULL, fpc_irq_device_no, NULL,FPC1020_IRQ_NAME);
	if (!fpc_irq_dev) {
		pr_err("class_device_create failed %d\n", rc);
		rc = -ENOMEM;
		goto exit_destroy_class;
	}

	cdev_init(&fpc_irq_cdev, &fpc_irq_fops);
	fpc_irq_cdev.owner = THIS_MODULE;

	rc = cdev_add(&fpc_irq_cdev, fpc_irq_device_no, 1);
	if(rc) {
		pr_err("cdev_add failed.\n");
		goto exit_destroy_device;
	}
	return 0;

exit_destroy_device:
	device_destroy(fpc_irq_class, fpc_irq_device_no);
exit_destroy_class:
	class_destroy(fpc_irq_class);
exit_unreg_chrdev_region:
	unregister_chrdev_region(fpc_irq_device_no,1);
	return rc;
}

static void fpc_irq_exit(void)
{
	cdev_del(&fpc_irq_cdev);
	device_destroy(fpc_irq_class, fpc_irq_device_no);
	class_destroy(fpc_irq_class);
	unregister_chrdev_region(fpc_irq_device_no, 1);
}

module_init(fpc_irq_init);
module_exit(fpc_irq_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fingerprint Cards AB <tech@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 touch sensor driver.");



