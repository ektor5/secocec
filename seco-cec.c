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
#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>

// CEC Framework
#include <media/cec.h>

#define SECOCEC_MAX_ADDRS 1
#define SECOCEC_DEV_NAME "secocec"
#define GPIO_I2C6_SCL 51
#define GPIOCHIP_SOUTHWEST 456

struct secocec_data {
	struct device *dev;
	struct platform_device *pdev;
	struct cec_adapter *cec_adap;
	int irq;

	struct mutex read_lock;	//config lock, not used for now
	struct mutex write_lock;

	u8 cec_addr[SECOCEC_MAX_ADDRS];
	u8 cec_valid_addrs;
	bool cec_enabled_adap;
};

static struct secocec_data *secocec_data_init(struct platform_device *pdev)
{
	struct secocec_data *drvdata;
	struct device *dev = &pdev->dev;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return NULL;

	dev_set_drvdata(dev, drvdata);

	drvdata->pdev = pdev;
	drvdata->dev = dev;

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
	
	printk("SECO CEC Interrupt Handled");

	return (irq_handler_t) IRQ_HANDLED;

}
static irq_handler_t secocec_irq_handler_quick(unsigned int irq, void *dev_id,
					 struct pt_regs *regs)
{
	//TODO irq handler
	
	return (irq_handler_t) IRQ_WAKE_THREAD;

}

/*
 *static s32 seco_smbus_read_byte_data_check(struct platform_driver *drv
 *                                           u8 command, bool check)
 *{
 *        union i2c_smbus_data data;
 *
 *        if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
 *                            I2C_SMBUS_READ, command,
 *                            I2C_SMBUS_BYTE_DATA, &data))
 *                return data.byte;
 *        if (check)
 *                dev_err(&client->dev, "error reading %02x, %02x\n",
 *                        client->addr, command);
 *        return -1;
 *}
 */

static const struct acpi_gpio_params irq_gpios = { 0, 0, false };	// crs_entry_index, line_index, active_low

static const struct acpi_gpio_mapping secocec_acpi_gpios[] = {
	{"irq-gpios", &irq_gpios, 1},
	{},
};

static int secocec_acpi_probe(struct secocec_data *sdev)
{
	struct device *dev = sdev->dev;
	struct platform_device *pdev = sdev->pdev;
	LIST_HEAD(resources);
	const struct acpi_gpio_mapping *gpio_mapping = secocec_acpi_gpios;
	const struct acpi_device_id *id;
	struct gpio_desc *gpio;
	int ret;
	int irq;

	/* Retrieve GPIO data from ACPI, if _DSD is present */
	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (id) {
		dev_dbg(dev, "_DSD Found, using ACPI package");
		gpio_mapping =
		    (const struct acpi_gpio_mapping *)id->driver_data;
	} else {
		dev_dbg(dev, "no _DSD Found, using package");
	}

	ret = acpi_dev_add_driver_gpios(ACPI_COMPANION(dev), gpio_mapping);
	if (ret) {
		dev_dbg(dev, "Cannot add gpio irq data to the driver");
	}

	gpio = devm_gpiod_get(dev, "irq-gpios", GPIOF_IN);
	if (IS_ERR(gpio)) {
		dev_err(dev, "Cannot request interrupt gpio");
		return PTR_ERR(gpio);
	}

	ret = gpiod_to_irq(gpio);
	if (ret < 0) {
		dev_err(dev, "gpio is not binded to IRQ");
	} else { 
		irq = ret;
	}

	dev_dbg(dev,"irq-gpio is binded to IRQ %d", irq);
 
	if (irq != gpiod_to_irq(gpio)){
		dev_warn(dev, "IRQ %d is not GPIO %d (%d)\n",
			 irq, desc_to_gpio(gpio),
			 gpiod_to_irq(gpio));
	}

	sdev->irq = irq;

	acpi_dev_free_resource_list(&resources);

	return 0;

}

static int secocec_noacpi_probe(struct secocec_data *sdev)
{
	struct device *dev = sdev->dev;
	int gpio_irq = GPIOCHIP_SOUTHWEST + GPIO_I2C6_SCL;
	int irq;
	int ret;

	ret = devm_gpio_request_one(dev, gpio_irq, GPIOF_IN, "irq");
	if (ret){
		dev_err(dev, "cannot request gpio %d\n",
			 gpio_irq);
		return ret;
	}
	dev_dbg(dev, "requested gpio %d", gpio_irq);

	irq = gpio_to_irq(gpio_irq);
	if (irq <= 0){
		dev_err(dev, "IRQ associated to gpio %d is not valid (%d)\n",
			 gpio_irq, irq);
		return irq;
	}
	dev_dbg(dev, "assigned IRQ %d", irq);

	sdev->irq = irq;

	return 0;

}

static int secocec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct secocec_data *secocec = secocec_data_init(pdev);
	// u16 rev;
	int ret;
	u8 opts;

	/* Check if the adapter supports the needed features */
	/*
	 *if (!i2c_check_functionality(pdev->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
	 *        ret = -EIO;
	 *        goto err;
	 *}
	 */

	/* TODO i2c access to secocec? */
	/*
	 *rev = seco_smbus_read_byte_data_check(client, 0xea, false) << 8 |
	 *    seco_smbus_read_byte_data_check(client, 0xeb, false);
	 *if (rev != 0x2012) {
	 *        dev_dbg(dev, "got rev=0x%04x on first read attempt\n", rev);
	 *        rev =
	 *            seco_smbus_read_byte_data_check(client, 0xea,
	 *                                            false) << 8 |
	 *            seco_smbus_read_byte_data_check(client, 0xeb, false);
	 *}
	 *if (rev != 0x2012) {
	 *        dev_err(dev, "not an secocec on address 0x%x (rev=0x%04x)\n",
	 *                client->addr, rev);
	 *        ret = -ENODEV;
	 *        goto err;
	 *}
	 */

	//request GPIO IRQ via acpi
	//TODO check s is ok

	if (has_acpi_companion(dev)){
		dev_dbg(dev, "ACPI companion found");
		ret = secocec_acpi_probe(secocec);
	}
	else {
		dev_dbg(dev, "Cannot find any ACPI companion");
		ret = secocec_noacpi_probe(secocec);
	}

	if (ret) {
		dev_err(dev, "Cannot assign gpio to IRQ");
		ret = -ENODEV;
		goto err;
	}


	if (!ACPI_HANDLE(dev))
		dev_dbg(dev, "Cannot find any ACPI handle");

	if (secocec->irq <= 0) {
		dev_err(dev, "Cannot find irq (%d)", secocec->irq);
		ret = -ENODEV;
		goto err;
	}
	dev_dbg(dev, "IRQ detected at %d", secocec->irq);

	ret = devm_request_threaded_irq(dev,
			       // The interrupt number requested
			       secocec->irq,
			       // The pointer to the handler function below
			       (irq_handler_t) secocec_irq_handler,
			       (irq_handler_t) secocec_irq_handler_quick,
			       // Use the custom kernel param to set interrupt type
			       IRQF_TRIGGER_RISING | IRQF_SHARED,
			       // Used in /proc/interrupts to identify the owner
			       dev_name(&pdev->dev),
			       // The *dev_id for shared interrupt lines
			       dev);

	if (ret < 0) {
		dev_err(dev, "Cannot request IRQ %d", secocec->irq);
		ret = -EIO;
		goto err;
	}

	//allocate cec
	opts = CEC_CAP_TRANSMIT |
	    CEC_CAP_LOG_ADDRS | CEC_CAP_PASSTHROUGH | CEC_CAP_RC;

	secocec->cec_adap = cec_allocate_adapter(&secocec_cec_adap_ops,
						 secocec,
						 dev_name(dev),
						 opts, SECOCEC_MAX_ADDRS);
	ret = PTR_ERR_OR_ZERO(secocec->cec_adap);
	if (ret)
		goto err_delete_adapter;

	ret = cec_register_adapter(secocec->cec_adap, dev);
	if (ret)
		goto err_delete_adapter;

	platform_set_drvdata(pdev, secocec);

	dev_dbg(dev, "Device registered");

	return ret;

err_delete_adapter:
	cec_delete_adapter(secocec->cec_adap);
err:
	dev_err(dev, "%s device probe failed.\n", dev_name(dev));

	return ret;
}

/* ----------------------------------------------------------------------- */

static int secocec_remove(struct platform_device *pdev)
{
	struct secocec_data *secocec = platform_get_drvdata(pdev);

	//release cec
	cec_unregister_adapter(secocec->cec_adap);

	return 0;
}

/* ----------------------------------------------------------------------- */

#ifdef CONFIG_ACPI
static const struct acpi_device_id secocec_acpi_match[] = {
	{"CEC00001", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, secocec_acpi_match);
#endif

static struct platform_driver secocec_driver = {
	.driver = {
		   .name = SECOCEC_DEV_NAME,
		   .acpi_match_table = ACPI_PTR(secocec_acpi_match),
		   },
	.probe = secocec_probe,
	.remove = secocec_remove,
};
module_platform_driver(secocec_driver);

MODULE_DESCRIPTION("SECO CEC X86 Driver");
MODULE_AUTHOR("Ettore Chimenti <ek5.chimenti@gmail.com>");
MODULE_LICENSE("GPL");
