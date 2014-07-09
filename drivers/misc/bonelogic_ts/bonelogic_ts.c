/*
 *  BoneLogic TS Driver, adapted from lis3lv02d accelerometer driver

    Notes:
    -Basic polling support using polled joystick event reports
    -Framework for devicetree/platform data is here, but not used
    -Framework for interrupt operation is here but some assembly is required with the hardware layer to get it all working

 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/input-polldev.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/freezer.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/pm_runtime.h>
#include <linux/atomic.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>

#include "bonelogic_ts.h"

#define DRIVER_NAME     "bonelogic_ts"

/* joystick device poll interval in milliseconds */
#define MDPS_POLL_INTERVAL 50
#define MDPS_POLL_MIN	   0
#define MDPS_POLL_MAX	   2000

#define BLTS_SYSFS_POWERDOWN_DELAY 2500 /* In milliseconds */

#define SELFTEST_OK	       0
#define SELFTEST_FAIL	       -1
#define SELFTEST_IRQ	       -2

#define BLTS_PWRON_DELAY_WAI	(2500)

struct bonelogic_ts blts_dev = {
	.misc_wait   = __WAIT_QUEUE_HEAD_INITIALIZER(blts_dev.misc_wait),
};
EXPORT_SYMBOL_GPL(blts_dev);

/* just like param_set_int() but does sanity-check so that it won't point
 * over the axis array size
 */
static int param_set_axis(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);
	if (!ret) {
		int val = *(int *)kp->arg;
		if (val < 0)
			val = -val;
		if (!val || val > 3)
			return -EINVAL;
	}
	return ret;
}

static struct kernel_param_ops param_ops_axis = {
	.set = param_set_axis,
	.get = param_get_int,
};

//module_param_array_named(axes, blts_dev.ac.as_array, axis, NULL, 0644);
MODULE_PARM_DESC(axes, "was axis mapping");

static u8 bonelogic_ts_read_id(struct bonelogic_ts *dev)
{
	u8 devID;
	dev->read(dev, ID, &devID);

	return devID;
}

/**
 * Note that 40Hz input device can eat up about 10% CPU at 800MHZ
 */

 #define REG_01 0x01

static void bonelogic_ts_get(struct bonelogic_ts *pBlts, int *x, int *y, int *z)
{
	int position[3];
	int i;

	u16 data[3];
	pBlts->blkread(pBlts, REG_01,6, (u8 *)data);

	//todo return values
}

static s16 bonelogic_ts_read(struct bonelogic_ts *blts, int reg)
{
	s8 lo;
	if (blts->read(blts, reg, &lo) < 0)
		return 0;

	return lo;
}


static inline int bonelogic_ts_get_axis(s8 axis, int hw_values[3])
{
	if (axis > 0)
		return hw_values[axis - 1];
	else
		return -hw_values[-axis - 1];
}

static int bonelogic_ts_get_pwron_wait(struct bonelogic_ts *blts)
{
	msleep(1500);
	return 0;
}


static int bonelogic_ts_selftest(struct bonelogic_ts *dev, s16 results[3])
{
	mutex_lock(&dev->mutex);
	atomic_dec(&dev->wake_thread);
	mutex_unlock(&dev->mutex);
	return SELFTEST_OK;
}

void bonelogic_ts_poweroff(struct bonelogic_ts *dev)
{
}
EXPORT_SYMBOL_GPL(bonelogic_ts_poweroff);

int bonelogic_ts_poweron(struct bonelogic_ts *dev)
{
	int err;
	u8 reg;

	dev->init(dev);

	err = bonelogic_ts_get_pwron_wait(dev);
	if (err)
		return err;

	return 0;
}
EXPORT_SYMBOL_GPL(bonelogic_ts_poweron);

static void bonelogic_ts_joystick_poll(struct input_polled_dev *pidev)
{
	struct bonelogic_ts *dev = pidev->private;
	int x, y, z;

	mutex_lock(&dev->mutex);
	bonelogic_ts_get(dev, &x, &y, &z);
	input_report_abs(pidev->input, ABS_X, x);
	input_report_abs(pidev->input, ABS_Y, y);
	input_report_abs(pidev->input, ABS_Z, z);
	input_sync(pidev->input);
	mutex_unlock(&dev->mutex);
}

static void bonelogic_ts_joystick_open(struct input_polled_dev *pidev)
{
	struct bonelogic_ts *blts = pidev->private;

	if (blts->pm_dev)
		pm_runtime_get_sync(blts->pm_dev);

	if (blts->pdata && blts->idev)
		atomic_set(&blts->wake_thread, 1);
	/*
	 * Update coordinates for the case where poll interval is 0 and
	 * the chip in running purely under interrupt control
	 */
	bonelogic_ts_joystick_poll(pidev);
}

static void bonelogic_ts_joystick_close(struct input_polled_dev *pidev)
{
	struct bonelogic_ts *dev = pidev->private;

	atomic_set(&dev->wake_thread, 0);
	if (dev->pm_dev)
		pm_runtime_put(dev->pm_dev);
}

static irqreturn_t bonelogic_ts_interrupt(int irq, void *data)
{
	struct bonelogic_ts *dev = data;

	if (!test_bit(0, &dev->misc_opened))
		goto out;

	/*
	 * Be careful: on some HP laptops the bios force DD when on battery and
	 * the lid is closed. This leads to interrupts as soon as a little move
	 * is done.
	 */
	atomic_inc(&dev->count);

	wake_up_interruptible(&dev->misc_wait);
	kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
out:
	if (atomic_read(&dev->wake_thread))
		return IRQ_WAKE_THREAD;
	return IRQ_HANDLED;
}

static void bonelogic_ts_interrupt_handle_click(struct bonelogic_ts *blts)
{
	struct input_dev *dev = blts->idev->input;
	u8 click_src;

	mutex_lock(&blts->mutex);

/*	dev->read(dev, CLICK_SRC, &click_src);

	if (click_src & CLICK_SINGLE_X) {
		input_report_key(dev, dev->mapped_btns[0], 1);
		input_report_key(dev, dev->mapped_btns[0], 0);
	}

	if (click_src & CLICK_SINGLE_Y) {
		input_report_key(dev, dev->mapped_btns[1], 1);
		input_report_key(dev, dev->mapped_btns[1], 0);
	}

	if (click_src & CLICK_SINGLE_Z) {
		input_report_key(dev, dev->mapped_btns[2], 1);
		input_report_key(dev, dev->mapped_btns[2], 0);
	}
	*/
	input_sync(dev);
	mutex_unlock(&blts->mutex);
}

static inline void bonelogic_ts_data_ready(struct bonelogic_ts *dev, int index)
{
	int dummy;

	/* Dummy read to ack interrupt */
	bonelogic_ts_get(dev, &dummy, &dummy, &dummy);
	dev->data_ready_count[index]++;
}

static irqreturn_t bonelogic_ts_interrupt_thread(int irq, void *data)
{
	
	struct bonelogic_ts *dev = data;
/*	u8 irq_cfg = dev->irq_cfg & BLTS_IRQ1_MASK;

	if (irq_cfg == dev_IRQ1_CLICK)
		bonelogic_ts_interrupt_handle_click(dev);
	else if (unlikely(irq_cfg == dev_IRQ1_DATA_READY))
		bonelogic_ts_data_ready(dev, IRQ_LINE0);
	else
	*/
		bonelogic_ts_joystick_poll(dev->idev);

	return IRQ_HANDLED;
}

static int bonelogic_ts_misc_open(struct inode *inode, struct file *file)
{
	struct bonelogic_ts *dev = container_of(file->private_data,struct bonelogic_ts, miscdev);

	if (test_and_set_bit(0, &dev->misc_opened))
		return -EBUSY; /* already open */

	if (dev->pm_dev)
		pm_runtime_get_sync(dev->pm_dev);

	atomic_set(&dev->count, 0);
	return 0;
}

static int bonelogic_ts_misc_release(struct inode *inode, struct file *file)
{
	struct bonelogic_ts *dev = container_of(file->private_data,struct bonelogic_ts, miscdev);

	clear_bit(0, &dev->misc_opened); /* release the device */
	if (dev->pm_dev)
		pm_runtime_put(dev->pm_dev);
	return 0;
}

static ssize_t bonelogic_ts_misc_read(struct file *file, char __user *buf,
				size_t count, loff_t *pos)
{
	struct bonelogic_ts *dev = container_of(file->private_data,struct bonelogic_ts, miscdev);

	DECLARE_WAITQUEUE(wait, current);
	u32 data;
	unsigned char byte_data;
	ssize_t retval = 1;

	if (count < 1)
		return -EINVAL;

	add_wait_queue(&dev->misc_wait, &wait);
	while (true) {
		set_current_state(TASK_INTERRUPTIBLE);
		data = atomic_xchg(&dev->count, 0);
		if (data)
			break;

		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			goto out;
		}

		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			goto out;
		}

		schedule();
	}

	if (data < 255)
		byte_data = data;
	else
		byte_data = 255;

	/* make sure we are not going into copy_to_user() with
	 * TASK_INTERRUPTIBLE state */
	set_current_state(TASK_RUNNING);
	if (copy_to_user(buf, &byte_data, sizeof(byte_data)))
		retval = -EFAULT;

out:
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&dev->misc_wait, &wait);

	return retval;
}

static unsigned int bonelogic_ts_misc_poll(struct file *file, poll_table *wait)
{
	struct bonelogic_ts *dev = container_of(file->private_data,struct bonelogic_ts, miscdev);

	poll_wait(file, &dev->misc_wait, wait);
	if (atomic_read(&dev->count))
		return POLLIN | POLLRDNORM;
	return 0;
}

static int bonelogic_ts_misc_fasync(int fd, struct file *file, int on)
{
	struct bonelogic_ts *dev = container_of(file->private_data,
					      struct bonelogic_ts, miscdev);

	return fasync_helper(fd, file, on, &dev->async_queue);
}

static const struct file_operations bonelogic_ts_misc_fops = {
	.owner   = THIS_MODULE,
	.llseek  = no_llseek,
	.read    = bonelogic_ts_misc_read,
	.open    = bonelogic_ts_misc_open,
	.release = bonelogic_ts_misc_release,
	.poll    = bonelogic_ts_misc_poll,
	.fasync  = bonelogic_ts_misc_fasync,
};

int bonelogic_ts_joystick_enable(struct bonelogic_ts *dev)
{
	struct input_dev *input_dev;
	int err;
	int btns[] = {BTN_X, BTN_Y, BTN_Z};

	if (dev->idev)
		return -EINVAL;

	dev->idev = input_allocate_polled_device();
	if (!dev->idev)
		return -ENOMEM;

	dev->idev->poll = bonelogic_ts_joystick_poll;
	dev->idev->open = bonelogic_ts_joystick_open;
	dev->idev->close = bonelogic_ts_joystick_close;
	dev->idev->poll_interval = MDPS_POLL_INTERVAL;
	dev->idev->poll_interval_min = MDPS_POLL_MIN;
	dev->idev->poll_interval_max = MDPS_POLL_MAX;
	dev->idev->private = dev;
	input_dev = dev->idev->input;

	input_dev->name       = "BoneLogic Touchscreen";
	input_dev->phys       = DRIVER_NAME "/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor  = 0;
	input_dev->dev.parent = &dev->pdev->dev;

	set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_X, 0, 400, /*fuzz*/ 5, /*flat*/ 1);
	input_set_abs_params(input_dev, ABS_Y, 0, 240, 5, 1);
	input_set_abs_params(input_dev, ABS_Z, 0, 100, 5, 1);

	dev->mapped_btns[0] = bonelogic_ts_get_axis(abs(dev->ac.x), btns);
	dev->mapped_btns[1] = bonelogic_ts_get_axis(abs(dev->ac.y), btns);
	dev->mapped_btns[2] = bonelogic_ts_get_axis(abs(dev->ac.z), btns);

	err = input_register_polled_device(dev->idev);
	if (err) {
		input_free_polled_device(dev->idev);
		dev->idev = NULL;
	}

	return err;
}
EXPORT_SYMBOL_GPL(bonelogic_ts_joystick_enable);

void bonelogic_ts_joystick_disable(struct bonelogic_ts *dev)
{
	if (dev->irq)
		free_irq(dev->irq, dev);
	//if (dev->pdata && dev->pdata->irq2)
	//	free_irq(dev->pdata->irq2, dev);

	if (!dev->idev)
		return;

	if (dev->irq)
		misc_deregister(&dev->miscdev);
	input_unregister_polled_device(dev->idev);
	input_free_polled_device(dev->idev);
	dev->idev = NULL;
}
EXPORT_SYMBOL_GPL(bonelogic_ts_joystick_disable);

/* Sysfs stuff */
static void bonelogic_ts_sysfs_poweron(struct bonelogic_ts *dev)
{
	/*
	 * SYSFS functions are fast visitors so put-call
	 * immediately after the get-call. However, keep
	 * chip running for a while and schedule delayed
	 * suspend. This way periodic sysfs calls doesn't
	 * suffer from relatively long power up time.
	 */

	if (dev->pm_dev) {
		pm_runtime_get_sync(dev->pm_dev);
		pm_runtime_put_noidle(dev->pm_dev);
		pm_schedule_suspend(dev->pm_dev, BLTS_SYSFS_POWERDOWN_DELAY);
	}
}

static ssize_t bonelogic_ts_selftest_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct bonelogic_ts *blts = dev_get_drvdata(dev);
	s16 values[3];

	static const char ok[] = "OK";
	static const char fail[] = "FAIL";
	static const char irq[] = "FAIL_IRQ";
	const char *res;

	bonelogic_ts_sysfs_poweron(blts);
	switch (bonelogic_ts_selftest(blts, values)) {
	case SELFTEST_FAIL:
		res = fail;
		break;
	case SELFTEST_IRQ:
		res = irq;
		break;
	case SELFTEST_OK:
	default:
		res = ok;
		break;
	}
	return sprintf(buf, "%s %d %d %d\n", res,values[0], values[1], values[2]);
}

static ssize_t bonelogic_ts_position_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct bonelogic_ts *blts = dev_get_drvdata(dev);
	int x, y, z;

	bonelogic_ts_sysfs_poweron(blts);
	mutex_lock(&blts->mutex);
	bonelogic_ts_get(blts, &x, &y, &z);
	mutex_unlock(&blts->mutex);
	return sprintf(buf, "(%d,%d,%d)\n", x, y, z);
}

static ssize_t bonelogic_ts_get_something(struct device *dev,struct device_attribute *attr, char *buf)
{

	struct bonelogic_ts *blts = dev_get_drvdata(dev);

	bonelogic_ts_sysfs_poweron(blts);
	return sprintf(buf, "something\n");//%d\n", bonelogic_ts_get_odr(dev));
}

static ssize_t bonelogic_ts_set_something(struct device *dev,struct device_attribute *attr, const char *buf,size_t count)
{
	struct bonelogic_ts *blts = dev_get_drvdata(dev);
	unsigned long value_to_set;
	int ret;

	ret = kstrtoul(buf, 0, &value_to_set);
	if (ret)
		return ret;

	bonelogic_ts_sysfs_poweron(blts);
	//if (bonelogic_ts_set_odr(dev, rate))
	//	return -EINVAL;

	return count;
}

static DEVICE_ATTR(selftest, S_IRUSR, bonelogic_ts_selftest_show, NULL);
static DEVICE_ATTR(position, S_IRUGO, bonelogic_ts_position_show, NULL);
static DEVICE_ATTR(something, S_IRUGO | S_IWUSR, bonelogic_ts_get_something, bonelogic_ts_set_something);

static struct attribute *bonelogic_ts_attributes[] = {
	&dev_attr_selftest.attr,
	&dev_attr_position.attr,
	&dev_attr_something.attr,
	NULL
};

static struct attribute_group bonelogic_ts_attribute_group = {
	.attrs = bonelogic_ts_attributes
};


static int bonelogic_ts_add_fs(struct bonelogic_ts *dev )
{
	dev->pdev = platform_device_register_simple(DRIVER_NAME, -1, NULL, 0);
	if (IS_ERR(dev->pdev))
		return PTR_ERR(dev->pdev);

	platform_set_drvdata(dev->pdev, dev);
	return sysfs_create_group(&dev->pdev->dev.kobj, &bonelogic_ts_attribute_group);
}

int bonelogic_ts_remove_fs(struct bonelogic_ts *dev)
{
	sysfs_remove_group(&dev->pdev->dev.kobj, &bonelogic_ts_attribute_group);
	platform_device_unregister(dev->pdev);
	if (dev->pm_dev) {
		/* Barrier after the sysfs remove */
		pm_runtime_barrier(dev->pm_dev);

		/* SYSFS may have left chip running. Turn off if necessary */
		if (!pm_runtime_suspended(dev->pm_dev))
			bonelogic_ts_poweroff(dev);

		pm_runtime_disable(dev->pm_dev);
		pm_runtime_set_suspended(dev->pm_dev);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(bonelogic_ts_remove_fs);

static void bonelogic_ts_configure(struct bonelogic_ts *dev,	struct bonelogic_ts_platform_data *p)
{
	int err;

	//if (p->click_flags) //example on how to pull something from platform data
	
	/*
	if (p->irq2) {
		err = request_threaded_irq(p->irq2,
					NULL,
					bonelogic_ts_interrupt_thread2_8b,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT |
					(p->irq_flags2 & IRQF_TRIGGER_MASK),
					DRIVER_NAME, dev);
		if (err < 0)
			pr_err("No second IRQ. Limited functionality\n");
	}
	*/
}

#ifdef CONFIG_OF
int bonelogic_ts_init_dt(struct bonelogic_ts *dev)
{

	return 0;
}
#else
int bonelogic_ts_init_dt(struct bonelogic_ts *dev)
{
	return 0;
}
#endif
EXPORT_SYMBOL_GPL(bonelogic_ts_init_dt);

/*
 * Initialise the touchscreen driver and the various subsystems.
 * Should be rather independent of the bus system.
 */
int bonelogic_ts_init_device(struct bonelogic_ts *dev)
{
	int err;
	irq_handler_t thread_fn;
	int irq_flags = 0;

	dev->whoami = bonelogic_ts_read_id(dev);

	//do something with ID...

	pr_debug(DRIVER_NAME ":%s @ %d, ok\n",__FILE__,__LINE__);


	pr_debug("yay.\n");
	dev->read_data = bonelogic_ts_read;
	dev->mdps_max_val = 2048;
	dev->pwron_delay = 1000;

//	dev->reg_cache = kzalloc(max(sizeof(dev_wai8_regs),
//				     sizeof(dev_wai12_regs)), GFP_KERNEL);
//
//	if (dev->reg_cache == NULL) {
//		pr_debug(KERN_ERR DRIVER_NAME "out of memory\n");
//		return -ENOMEM;
//	}

	mutex_init(&dev->mutex);
	atomic_set(&dev->wake_thread, 0);

	bonelogic_ts_add_fs(dev);
	err = bonelogic_ts_poweron(dev);
	if (err) {
		pr_debug(DRIVER_NAME ":%s @ %d, ERROR\n",__FILE__,__LINE__);
		bonelogic_ts_remove_fs(dev);
		return err;
	}
	pr_debug(DRIVER_NAME ":%s @ %d, ok\n",__FILE__,__LINE__);


	if (dev->pm_dev) {
		pm_runtime_set_active(dev->pm_dev);
		pm_runtime_enable(dev->pm_dev);
	}

	if (bonelogic_ts_joystick_enable(dev))
		pr_err("joystick initialization failed\n");

	pr_debug(DRIVER_NAME ":%s @ %d, ok\n",__FILE__,__LINE__);


	//if here, idev is proper

	//add sme input capabilities
	if (dev->idev) {
		struct input_dev *input_dev = dev->idev->input;
		input_set_capability(input_dev, EV_KEY, BTN_X);
		input_set_capability(input_dev, EV_KEY, BTN_Y);
		input_set_capability(input_dev, EV_KEY, BTN_Z);
	}
	else
		pr_debug(DRIVER_NAME ":%s @ %d, ERROR\n",__FILE__,__LINE__);

	/* passing in platform specific data is purely optional */
	if (dev->pdata) 
	{
		struct bonelogic_ts_platform_data *p = dev->pdata;
		bonelogic_ts_configure(dev, p);
	}
	
	/* bail if we did not get an IRQ from the bus layer */
	if (!dev->irq) {
		pr_debug("No IRQ. Disabling /dev/blts_intr interrupt.\n");
		pr_debug(DRIVER_NAME ":%s @ %d, ERROR\n",__FILE__,__LINE__);
		goto out;
	}

	/*
	 * IRQF_TRIGGER_RISING seems pointless on HP laptops because the
	 * io-apic is not configurable (and generates a warning) but I keep it
	 * in case of support for other hardware.
	 */
	thread_fn = bonelogic_ts_interrupt_thread;

	err = request_threaded_irq(dev->irq, bonelogic_ts_interrupt,
				thread_fn,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT |
				irq_flags,
				DRIVER_NAME, dev);

	if (err < 0) {
		pr_err("Cannot get IRQ\n");
		goto out;
	}
	pr_debug(DRIVER_NAME ":%s @ %d, ok\n",__FILE__,__LINE__);


	dev->miscdev.minor	= MISC_DYNAMIC_MINOR;
	dev->miscdev.name	= "blts_intr";
	dev->miscdev.fops	= &bonelogic_ts_misc_fops;

	if (misc_register(&dev->miscdev))
		pr_err("misc_register failed\n");
	else
		pr_debug(DRIVER_NAME ":%s @ %d, ok\n",__FILE__,__LINE__);

out:
	return 0;
}
EXPORT_SYMBOL_GPL(bonelogic_ts_init_device);

static inline s32 blts_i2c_write(struct bonelogic_ts *dev, int reg, u8 value)
{
	struct i2c_client *c = dev->bus_priv;
	return i2c_smbus_write_byte_data(c, reg, value);
}

static inline s32 blts_i2c_read(struct bonelogic_ts *dev, int reg, u8 *v)
{
	struct i2c_client *c = dev->bus_priv;
	*v = i2c_smbus_read_byte_data(c, reg);
	return 0;
}

static inline s32 blts_i2c_blockread(struct bonelogic_ts *dev, int reg, int len,u8 *v)
{
	struct i2c_client *c = dev->bus_priv;
	reg |= (1 << 7); /* 7th bit enables address auto incrementation */
	return i2c_smbus_read_i2c_block_data(c, reg, len, v);
}

static int blts_i2c_init(struct bonelogic_ts *dev)
{
	u8 reg;
	int ret;

	//blts_reg_ctrl(lis3, LIS3_REG_ON);

	dev->read(dev, ID, &reg);
	if (reg != dev->whoami)
		pr_debug(KERN_ERR "bonelogic_ts: ID mismatch\n");

	return 0;
}

/* Default axis mapping but it can be overwritten by platform data */
static union axis_conversion bonelogic_ts_axis_map =
	{ .as_array = { 0, 1, 2 } };

#ifdef CONFIG_OF
static struct of_device_id bonelogic_ts_i2c_dt_ids[] = {
	{ .compatible = "blts,bonelogic_ts" },
	{}
};
MODULE_DEVICE_TABLE(of, bonelogic_ts_i2c_dt_ids);
#endif

static int bonelogic_ts_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int ret = 0;
	struct bonelogic_ts_platform_data *pdata = client->dev.platform_data;

	pr_debug(DRIVER_NAME ":%s @ %d, OK\n",__FILE__,__LINE__);

#ifdef CONFIG_OF
	pr_debug(DRIVER_NAME ":%s @ %d, OK [CONFIG_OF is def]\n",__FILE__,__LINE__);

	if (of_match_device(bonelogic_ts_i2c_dt_ids, &client->dev)) {
		blts_dev.of_node = client->dev.of_node;
		ret = bonelogic_ts_init_dt(&blts_dev);
		if (ret)
			return ret;
		pdata = blts_dev.pdata;
	}
#endif

	if (i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_I2C_BLOCK))
	{
		blts_dev.blkread  = blts_i2c_blockread;
		pr_debug(DRIVER_NAME ":%s @ %d, OK, block read supported\n",__FILE__,__LINE__);
	}
	else
		pr_debug(DRIVER_NAME ":%s @ %d, ERROR, no block read\n",__FILE__,__LINE__);

	pr_debug(DRIVER_NAME ":%s @ %d, OK\n",__FILE__,__LINE__);

	blts_dev.pdata	  = pdata;
	blts_dev.bus_priv = client;
	blts_dev.init	  = blts_i2c_init;
	blts_dev.read	  = blts_i2c_read;
	blts_dev.write	  = blts_i2c_write;
	blts_dev.irq	  = client->irq;
	blts_dev.ac	  	  = bonelogic_ts_axis_map;
	blts_dev.pm_dev	  = &client->dev;

	i2c_set_clientdata(client, &blts_dev);

	pr_debug(DRIVER_NAME ":%s @ %d, Calling init device\n",__FILE__,__LINE__);
	ret = bonelogic_ts_init_device(&blts_dev);
	pr_debug(DRIVER_NAME ":%s @ %d, init device done\n",__FILE__,__LINE__);

	if (ret)
		goto fail;
	return 0;

fail:

	//if (pdata && pdata->release_resources)
	//	pdata->release_resources();
	return ret;
}

static int bonelogic_ts_i2c_remove(struct i2c_client *client)
{
	struct bonelogic_ts *blts = i2c_get_clientdata(client);
	struct bonelogic_ts_platform_data *pdata = client->dev.platform_data;

	//if (pdata && pdata->release_resources)
	//	pdata->release_resources();

	bonelogic_ts_joystick_disable(blts);
	bonelogic_ts_remove_fs(&blts_dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bonelogic_ts_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct bonelogic_ts *blts = i2c_get_clientdata(client);

	//if (!dev->pdata || !dev->pdata->wakeup_flags)
	//	bonelogic_ts_poweroff(blts);
	return 0;
}

static int bonelogic_ts_i2c_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct bonelogic_ts *blts = i2c_get_clientdata(client);

	/*
	 * pm_runtime documentation says that devices should always
	 * be powered on at resume. Pm_runtime turns them off after system
	 * wide resume is complete.
	 */
	//if (!dev->pdata || !dev->pdata->wakeup_flags ||
	//	pm_runtime_suspended(dev))
	//	bonelogic_ts_poweron(blts);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM_RUNTIME
static int blts_i2c_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct bonelogic_ts *blts = i2c_get_clientdata(client);

	bonelogic_ts_poweroff(blts);
	return 0;
}

static int blts_i2c_runtime_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct bonelogic_ts *blts = i2c_get_clientdata(client);

	bonelogic_ts_poweron(blts);
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static const struct i2c_device_id bonelogic_ts_id[] = {
	{"bonelogic_ts", 0x32},
	{}
};

MODULE_DEVICE_TABLE(i2c, bonelogic_ts_id);

static const struct dev_pm_ops blts_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bonelogic_ts_i2c_suspend,
				bonelogic_ts_i2c_resume)
	SET_RUNTIME_PM_OPS(blts_i2c_runtime_suspend,
			   blts_i2c_runtime_resume,
			   NULL)
};

static struct i2c_driver bonelogic_ts_i2c_driver = {
	.driver	 = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
		.pm     = &blts_pm_ops,
		.of_match_table = of_match_ptr(bonelogic_ts_i2c_dt_ids),
	},
	.probe	= bonelogic_ts_i2c_probe,
	.remove	= bonelogic_ts_i2c_remove,
	.id_table = bonelogic_ts_id,
};

module_i2c_driver(bonelogic_ts_i2c_driver);

MODULE_AUTHOR("Chris Vondrachek");
MODULE_DESCRIPTION("BoneLogic Touchscreen");
MODULE_LICENSE("GPL");

