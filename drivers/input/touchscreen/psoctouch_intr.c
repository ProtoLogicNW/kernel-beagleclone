/*
 * drivers/input/touchscreen/psoctouch.c
 *
 * Copyright (c) 2014 ProtoLogic, LLC
 *	Chris Vondrachek <chris@protoligicnw.com>
 */


#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

struct ts_event {
	u16	x;
	u16	y;
};

struct psoctouch {
	struct input_dev	*input;
	char			phys[32];

	struct i2c_client	*client;

	unsigned long		poll_period;
	int			fuzzx;
	int			fuzzy;
	int			fuzzz;

	unsigned		gpio;
	int			irq;

	wait_queue_head_t	wait;
	bool			stopped;

	int			(*get_pendown_state)(struct device *);
	void			(*clear_penirq)(void);
};

static inline int psoctouch_xfer(struct psoctouch *pst)
{
	s32 data;
	u16 val;

	data = i2c_smbus_read_word_data(pst->client, 0);
	if (data < 0) {
		dev_err(&pst->client->dev, "i2c io error: %d\n", data);
		return data;
	}

	//not sure about the format... yet.  Just print!
	printk("psoc data: 0x%x\n", data);

	return 0;
}

static void psoctouch_read_values(struct psoctouch *pst, struct ts_event *tc)
{
	u16 val = psoctouch_xfer(pst);
	tc->x = val;
	tc->y = val; 
	//todo: do something a little more interesting here
}	

static void psoctouch_poll(struct input_polled_dev *dev)
{
	struct psoctouch *pst = dev->private;
        struct input_dev *input = dev->input;

	struct ts_event tc;
	psoctouch_read_values(pst,&tc);	
}

static bool psoctouch_is_pen_down(struct psoctouch *pst)
{
	if (!pst->get_pendown_state)
		return true;

	return pst->get_pendown_state(&pst->client->dev);
}

/* software irq, aka the "work thread"
	triggered by hardware irq returning IRQ_WAKE_THREAD
*/
static irqreturn_t psoctouch_soft_irq(int irq, void *handle)
{
	struct psoctouch *pst = handle;
	struct input_dev *input = pst->input;
/*
	input_report_key(input, BTN_TOUCH, 1);
	input_report_abs(input, ABS_X, tc.x);
	input_report_abs(input, ABS_Y, tc.y);
	input_report_abs(input, ABS_PRESSURE, rt);
	input_sync(input);
*/
	//loop pacing!
	wait_event_timeout(pst->wait, pst->stopped,msecs_to_jiffies(pst->poll_period));

	//dev_dbg(&ts->client->dev, "UP\n");
	//input_report_key(input, KEY_LEFT, 0);
	//input_report_key(input, KEY_RIGHT, 0);
	//input_report_key(input, BTN_TOUCH, 0);
	//input_report_abs(input, ABS_PRESSURE, 0);
	//input_sync(input);

	if (pst->clear_penirq)
		pst->clear_penirq();

	return IRQ_HANDLED;
}

/*harware interrupt, triggered on falling-edge of gpio irq
	Checks if gpio is in-fact low and if it is, wakes-up the 
	thread that handles reading data from the PSoC

	Reason for this "split" is simple; this code executes with 
	all other interrupts blocked so we need to get in & get out ASAP.
	The work of reading/writing to I/O can occur in the "soft" thread 
	which doesn't need to block everything else.

	Not used... yet.
*/
static irqreturn_t psoctouch_hard_irq(int irq, void *handle)
{
/*
	struct psoctouch *pst = handle;

	if (psoctouch_is_pen_down(pst))
		return IRQ_WAKE_THREAD;

	if (pst->clear_penirq)
		pst->clear_penirq();
*/
	return IRQ_HANDLED;
}

static void psoctouch_stop(struct psoctouch *pst)
{
	pst->stopped = true;
	mb();
	wake_up(&pst->wait);
	disable_irq(pst->irq);
}

static int psoctouch_open(struct input_dev *input_dev)
{
	struct psoctouch *pst = input_get_drvdata(input_dev);
	int err;
	pst->stopped = false;
	mb();
	enable_irq(pst->irq);
	return 0;
}

static void psoctouch_close(struct input_dev *input_dev)
{
	struct psoctouch *pst = input_get_drvdata(input_dev);
	psoctouch_stop(pst);
}

//DEVICETREE
#ifdef CONFIG_OF
static int psoctouch_get_pendown_state_gpio(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct psoctouch *pst = i2c_get_clientdata(client);
	return !gpio_get_value(pst->gpio);
}

static int psoctouch_probe_dt(struct i2c_client *client, struct psoctouch *pst)
{
	struct device_node *np = client->dev.of_node;
	u32 val32;
	u64 val64;

/*	if (!np) {
		dev_err(&client->dev, "missing device tree data\n");
		return -EINVAL;
	}

	if (!of_property_read_u64(np, "pst,poll-period", &val64))
		pst->poll_period = val64;
	else
		pst->poll_period = 1;

	pst->gpio = of_get_gpio(np, 0);
	if (gpio_is_valid(pst->gpio))
	 	pst->get_pendown_state = psoctouch_get_pendown_state_gpio;
	else
		dev_warn(&client->dev,
			 "GPIO not specified in DT (of_get_gpio returned %d)\n",pst->gpio);
*/
	dev_warn(&client->dev,"Someone should actually implement device-tree for me... please?\n");
	return 0;
}
#else //OLD, NON DT METHOD
static int psoctouch_probe_dt(struct i2c_client *client, struct psoctouch *pst)
{
	dev_err(&client->dev, "platform data is required!\n");
	return -EINVAL;
}
#endif

#if 0

static int psoctouch_probe_pdev(struct i2c_client *client, struct psoctouch *pst,
			      const struct psoctouch_platform_data *pdata,
			      const struct i2c_device_id *id)
{
	pst->poll_period = pdata->poll_period ? : 1000;

	return 0;
}

static void psoctouch_call_exit_platform_hw(void *data)
{
	struct device *dev = data;
	const struct psoctouch_platform_data *pdata = dev_get_platdata(dev);

	pdata->exit_platform_hw();
}
#endif

#define PSOC_POLL

//DT SUPPORT ONLY!
static int psoctouch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//const struct psoctouch_platform_data *pdata = dev_get_platdata(&client->dev);
	struct psoctouch *pst;
	struct input_dev *input_dev;
#ifdef PSOC_POLL
	struct input_polled_dev *poll_dev;
#endif
	int err;

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	pst = devm_kzalloc(&client->dev, sizeof(struct psoctouch), GFP_KERNEL);
	if (!pst)
		return -ENOMEM;

	//if (pdata)
	//	err = psoctouch_probe_pdev(client, pst, pdata, id);
	//else
	err = psoctouch_probe_dt(client, pst);
	if (err)
		return err;

#ifdef PSOC_POLL
        poll_dev = input_allocate_polled_device();
        if (!poll_dev) {
                dev_err(poll_dev, "no memory for polled device\n");
                return -ENOMEM;
        }
        poll_dev->private = pst;
        poll_dev->poll = psoctouch_poll;
        poll_dev->poll_interval = 1000;
	input_dev = poll_dev->input;
#else
	input_dev = devm_input_allocate_device(&client->dev);
#endif
	if (!input_dev)
		return -ENOMEM;

	i2c_set_clientdata(client, pst);

	pst->client = client;
	pst->irq = client->irq;
	pst->input = input_dev;
	init_waitqueue_head(&pst->wait);

	snprintf(pst->phys, sizeof(pst->phys),"%s/input0", dev_name(&client->dev));
	
	input_dev->name = "ProtoLogic PSoC Touchscreen";
	input_dev->phys = pst->phys;
	input_dev->id.bustype = BUS_I2C;

#ifdef PSOC_POLL
	poll_dev->open = NULL;
	poll_dev->close = NULL;
#else
	input_dev->open = psoctouch_open;
	input_dev->close = psoctouch_close;
#endif

	input_set_drvdata(input_dev, pst);

	/* hints:
		/usr/include/linux/input.h 
		https://www.kernel.org/doc/Documentation/input/input-programming.txt
		https://www.kernel.org/doc/Documentation/input/event-codes.txt
	*/

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_dev->keybit[BIT_WORD(KEY_LEFT)] = BIT_MASK(KEY_LEFT);
	input_dev->keybit[BIT_WORD(KEY_RIGHT)] = BIT_MASK(KEY_RIGHT);

	input_set_abs_params(input_dev, ABS_X, 0, 4096, pst->fuzzx, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 4096, pst->fuzzy, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 4096,pst->fuzzz, 0);

#ifndef PSOC_POLL
	err = devm_request_threaded_irq(&client->dev, pst->irq,
					psoctouch_hard_irq, psoctouch_soft_irq,
					IRQF_ONESHOT,
					client->dev.driver->name, pst);
	if (err) {
		dev_err(&client->dev, "Failed to request irq %d: %d\n",
			pst->irq, err);
		return err;
	}
	psoctouch_stop(ts);
#endif

#ifdef PSOC_POLL
	err = input_register_polled_device(poll_dev);
#else
	err = input_register_device(input_dev);
#endif
	if (err) {
		dev_err(&client->dev,"Failed to register input device: %d\n", err);
		return err;
	}

	return 0;
}

static const struct i2c_device_id psoctouch_idtable[] = {
	{ "psoctouch", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, psoctouch_idtable);

#ifdef CONFIG_OF
static const struct of_device_id psoctouch_of_match[] = {
	{ .compatible = "psoctouch" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, psoctouch_of_match);
#endif

static struct i2c_driver psoctouch_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "psoctouch",
		.of_match_table = of_match_ptr(psoctouch_of_match),
	},
	.id_table	= psoctouch_idtable,
	.probe		= psoctouch_probe,
};

module_i2c_driver(psoctouch_driver);

MODULE_AUTHOR("Chris Vondrachek <chris@protologicnw.com>");
MODULE_DESCRIPTION("PSoC TouchScreen Driver");
MODULE_LICENSE("GPL");
