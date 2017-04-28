/*
 * ======================================================================
 *
 *       Filename:  seco-cec.c
 *
 *    Description:  CEC driver for SECO X86 Boards
 *
 *        Version:  1.0
 *        Created:  04/03/2017 05:04:26 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ettore Chimenti (ek5.chimenti@gmail.com),
 *   Organization:  SECO Srl.
 *
 * ======================================================================
 */

#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>

// CEC Framework
#include <media/cec.h>

#define SECOCEC_IRQ_GPIO 42	// TODO change this
#define SECOCEC_MAX_ADDRS 1
#define SECOCEC_DEV_NAME "secocec"

struct secocec_data {
	struct device *dev;
	struct i2c_client *i2c_cec;
	struct cec_adapter *cec_adap;
	int irq;
	int irq_gpio;

	struct mutex read_lock;	//config lock, not used for now
	struct mutex write_lock;

	u8 cec_addr[SECOCEC_MAX_ADDRS];
	u8 cec_valid_addrs;
	bool cec_enabled_adap;
};

static struct secocec_data *secocec_data_init(struct i2c_client *client)
{
	struct secocec_data *drvdata;

	drvdata = devm_kzalloc(&client->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return NULL;

	drvdata->i2c_cec = client;
	drvdata->dev = &client->dev;

	mutex_init(&drvdata->read_lock);
	mutex_init(&drvdata->write_lock);

	return drvdata;
}

//TODO cec implementation
static int secocec_adap_enable(struct cec_adapter *adap, bool enable)
{
	return 0;
}

static int secocec_adap_monitor_all_enable(struct cec_adapter *adap,
					   bool enable)
{
	return 0;
}

static int secocec_adap_log_addr(struct cec_adapter *adap, u8 logical_addr)
{
	return 0;
}

static int secocec_adap_transmit(struct cec_adapter *adap, u8 attempts,
				 u32 signal_free_time, struct cec_msg *msg)
{
	return 0;
}

static void secocec_adap_status(struct cec_adapter *adap, struct seq_file *file)
{
}

static int secocec_received(struct cec_adapter *adap, struct cec_msg *msg)
{
	return 0;
}

struct cec_adap_ops secocec_cec_adap_ops = {
	/* Low-level callbacks */
	.adap_enable = secocec_adap_enable,
	.adap_monitor_all_enable = secocec_adap_monitor_all_enable,
	.adap_log_addr = secocec_adap_log_addr,
	.adap_transmit = secocec_adap_transmit,
	.adap_status = secocec_adap_status,

	/* High-level callbacks */
	.received = secocec_received,
};

static irq_handler_t secocec_irq_handler(unsigned int irq, void *dev_id,
					 struct pt_regs *regs)
{
	//TODO irq handler
	return 0;

}

static s32 seco_smbus_read_byte_data_check(struct i2c_client *client,
					   u8 command, bool check)
{
	union i2c_smbus_data data;

	if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			    I2C_SMBUS_READ, command,
			    I2C_SMBUS_BYTE_DATA, &data))
		return data.byte;
	if (check)
		dev_err(&client->dev, "error reading %02x, %02x\n",
			client->addr, command);
	return -1;
}

static int secocec_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct secocec_data *secocec = secocec_data_init(client);
	int gpio = SECOCEC_IRQ_GPIO;
	unsigned long IRQflags;
	u16 rev;
	int ret;
	u8 opts;
	struct pinctrl *pinctrl;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	dev_dbg(dev, "detecting secocec client on address 0x%x\n",
		client->addr << 1);

	/* TODO i2c access to secocec? */
	rev = seco_smbus_read_byte_data_check(client, 0xea, false) << 8 |
	    seco_smbus_read_byte_data_check(client, 0xeb, false);
	if (rev != 0x2012) {
		dev_dbg(dev, "got rev=0x%04x on first read attempt\n", rev);
		rev =
		    seco_smbus_read_byte_data_check(client, 0xea,
						    false) << 8 |
		    seco_smbus_read_byte_data_check(client, 0xeb, false);
	}
	if (rev != 0x2012) {
		dev_dbg(dev, "not an secocec on address 0x%x (rev=0x%04x)\n",
			client->addr << 1, rev);
		return -ENODEV;
	}
	//request GPIO IRQ
	ret = gpio_is_valid(gpio);
	if (ret < 0) {
		dev_err(dev, "gpio %d is invalid", gpio);
		return -EINVAL;
	}
	//TODO check if this is ok
	pinctrl = devm_pinctrl_get_select_default(dev);
	ret = devm_gpio_request_one(dev, gpio, GPIOF_IN, "secocec_gpio");
	if (ret < 0) {
		dev_err(dev, "Cannot request gpio %d", gpio);
		return -ENODEV;
	}

	secocec->irq_gpio = gpio;

	secocec->irq = gpio_to_irq(gpio);
	if (!secocec->irq) {
		dev_err(dev, "Cannot request gpio for IRQ %d", gpio);
		ret = -ENODEV;
		goto err_free_gpio;
	}

	IRQflags = IRQF_SHARED;
	ret = request_irq(secocec->irq,	// The interrupt number requested
			  // The pointer to the handler function below
			  (irq_handler_t) secocec_irq_handler,
			  // Use the custom kernel param to set interrupt type
			  IRQflags,
			  // Used in /proc/interrupts to identify the owner
			  dev_name(&client->dev),
			  // The *dev_id for shared interrupt lines
			  dev);

	if (!ret) {
		dev_err(dev, "Cannot request IRQ %d", secocec->irq);
		ret = -EIO;
		goto err_free_gpio;
	}
	//allocate cec
	opts = CEC_CAP_TRANSMIT |
	    CEC_CAP_LOG_ADDRS | CEC_CAP_PASSTHROUGH | CEC_CAP_RC;

	secocec->cec_adap = cec_allocate_adapter(&secocec_cec_adap_ops,
						 secocec,
						 dev_name(dev),
						 opts,
						 SECOCEC_MAX_ADDRS);
	ret = PTR_ERR_OR_ZERO(secocec->cec_adap);
	if (ret)
		goto err_free_irq;

	ret = cec_register_adapter(secocec->cec_adap, dev);
	if (ret)
		goto err_delete_adapter;

	dev_dbg(dev, "%s found @ 0x%x (%s)\n", client->name,
		client->addr << 1, client->adapter->name);

	return ret;

err_delete_adapter:
	cec_delete_adapter(secocec->cec_adap);
err_free_irq:
	free_irq(secocec->irq, &client->dev);
err_free_gpio:
	gpio_free(secocec->irq_gpio);

	return ret;
}

/* ----------------------------------------------------------------------- */

static int secocec_remove(struct i2c_client *client)
{
	struct secocec_data *secocec = i2c_get_clientdata(client);

	//free gpio
	gpio_free(secocec->irq_gpio);

	//free irq
	free_irq(secocec->irq, &client->dev);

	//release cec
	cec_delete_adapter(secocec->cec_adap);

	return 0;
}

/* ----------------------------------------------------------------------- */

static struct i2c_device_id secocec_id[] = {
	{SECOCEC_DEV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, secocec_id);

static struct i2c_driver secocec_driver = {
	.driver = {
		   .name = SECOCEC_DEV_NAME,
		   },
	.probe = secocec_probe,
	.remove = secocec_remove,
	.id_table = secocec_id,
};

module_i2c_driver(secocec_driver);

MODULE_DESCRIPTION("SECO CEC X86 Driver");
MODULE_AUTHOR("Ettore Chimenti <ek5.chimenti@gmail.com>");
MODULE_LICENSE("GPL");
