/*
 *
 * CEC driver for SECO X86 Boards
 *
 * Author:  Ettore Chimenti <ek5.chimenti@gmail.com>
 * Copyright (C) 2017, SECO Srl.
 * Copyright (C) 2017, Aidilab Srl.
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Released under the terms of 3-clause BSD License
 * Released under the terms of GNU General Public License Version 2.0
 *
 */

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

/* CEC Framework */
#include <media/cec.h>

#include "seco-cec.h"

struct secocec_data {
	struct device *dev;
	struct platform_device *pdev;
	struct cec_adapter *cec_adap;
	int irq;
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

	return drvdata;
}

#define smb_wr16(cmd, data) smb_word_op(CMD_WORD_DATA, MICRO_ADDRESS, \
					     cmd, data, SMBUS_WRITE, NULL)
#define smb_rd16(cmd, res) smb_word_op(CMD_WORD_DATA, MICRO_ADDRESS, cmd, 0, \
				       SMBUS_READ, res)

static int smb_word_op(short data_format,
		       unsigned short slave_addr,
		       unsigned char cmd,
		       unsigned short data,
		       unsigned char operation, unsigned short *result)
{
	unsigned int count;
	short _data_format;
	int ret, status = 0;

	switch (data_format) {
	case CMD_BYTE_DATA:
		_data_format = BRA_SMB_CMD_BYTE_DATA;
		break;
	case CMD_WORD_DATA:
		_data_format = BRA_SMB_CMD_WORD_DATA;
		break;
	default:
		return -EINVAL;
	}

	/* Request SMBus regions */
	if (!request_muxed_region(BRA_SMB_BASE_ADDR, 7, "CEC00001")) {
		pr_debug("request_region BRA_SMB_BASE_ADDR fail\n");
		return -ENXIO;
	}

	/* Active wait until ready */
	for (count = 0; (count <= SMBTIMEOUT) && (inb(HSTS) & BRA_INUSE_STS);
	     ++count) {
		udelay(SMB_POLL_UDELAY);
	}

	if (count > SMBTIMEOUT) {
		/* Reset the lock instead of failing */
		outb(0xFF, HSTS);
		pr_warn("smb_word_op SMBTIMEOUT\n");
	}

	outb(0x00, HCNT);
	outb((unsigned char)(slave_addr & 0xFE) | operation, XMIT_SLVA);
	outb(cmd, HCMD);
	inb(HCNT);

	if (operation == SMBUS_WRITE) {
		outb((unsigned char)data, HDAT0);
		outb((unsigned char)(data >> 8), HDAT1);
		pr_debug("smb_word_op WRITE (0x%02x - count %05d): 0x%04x\n",
			 cmd, count, data);
	}

	outb(BRA_START + _data_format, HCNT);

	for (count = 0; (count <= SMBTIMEOUT) && ((inb(HSTS) & BRA_HOST_BUSY));
	     count++) {
		udelay(SMB_POLL_UDELAY);
	}

	if (count > SMBTIMEOUT) {
		pr_debug("smb_word_op SMBTIMEOUT_1\n");
		status = -EBUSY;
		goto err;
	}

	ret = inb(HSTS);
	if (ret & BRA_HSTS_ERR_MASK) {
		pr_debug("smb_word_op HSTS(0x%02X): 0x%X\n", cmd, ret);
		status = -EIO;
		goto err;
	}

	if (operation == SMBUS_READ) {
		*result = ((inb(HDAT0) & 0xFF) + ((inb(HDAT1) & 0xFF) << 8));
		pr_debug("smb_word_op READ (0x%02x - count %05d): 0x%04x\n",
			 cmd, count, *result);
	}

err:
	outb(0xFF, HSTS);
	release_region(BRA_SMB_BASE_ADDR, 7);

	return status;
}

static int secocec_adap_enable(struct cec_adapter *adap, bool enable)
{
	struct secocec_data *cec = adap->priv;
	struct device *dev = cec->dev;
	int status;
	unsigned short val = 0;

	if (enable) {
		/* Clear the status register */
		status = smb_rd16(STATUS_REGISTER_1, &val);
		if (status)
			goto err;

		status = smb_wr16(STATUS_REGISTER_1, val);
		if (status)
			goto err;

		/* Enable the interrupts */
		status = smb_rd16(ENABLE_REGISTER_1, &val);
		if (status)
			goto err;

		status = smb_wr16(ENABLE_REGISTER_1,
				  val | ENABLE_REGISTER_1_CEC);
		if (status)
			goto err;

		dev_dbg(dev, "Device enabled");

	} else {
		/* Clear the status register */
		status = smb_rd16(STATUS_REGISTER_1, &val);
		if (status)
			goto err;

		status = smb_wr16(STATUS_REGISTER_1, val);
		if (status)
			goto err;

		/* Disable the interrupts */
		status = smb_rd16(ENABLE_REGISTER_1, &val);
		if (status)
			goto err;

		status = smb_wr16(ENABLE_REGISTER_1, val &
				  ~ENABLE_REGISTER_1_CEC &
				  ~ENABLE_REGISTER_1_IRDA_RC5);
		if (status)
			goto err;

		dev_dbg(dev, "Device disabled");
	}

	return 0;
err:
	dev_err(dev, "Adapter setup failed (%d)", status);
	return status;
}

static int secocec_adap_log_addr(struct cec_adapter *adap, u8 logical_addr)
{
	struct secocec_data *cec = adap->priv;
	struct device *dev = cec->dev;
	int status;
	unsigned short val, enable_val = 0;

	if (logical_addr != CEC_LOG_ADDR_INVALID) {
		val = logical_addr;
	} else {
		dev_dbg(dev, "Invalid addr, resetting address");
		val = 0xF;
	}

	status = smb_rd16(ENABLE_REGISTER_1, &enable_val);
	if (status)
		goto err;

	dev_dbg(dev, "Set logical address: Disabling device");
	status = smb_wr16(ENABLE_REGISTER_1,
			  enable_val & ~ENABLE_REGISTER_1_CEC);
	if (status)
		goto err;

	/* Write logical address */
	dev_dbg(dev, "Set logical address to %02x", val);
	status = smb_wr16(CEC_DEVICE_LA, val);
	if (status)
		goto err;

	dev_dbg(dev, "Set logical address: Re-enabling device");
	status = smb_wr16(ENABLE_REGISTER_1,
			  enable_val | ENABLE_REGISTER_1_CEC);
	if (status)
		goto err;

	return 0;

err:
	dev_err(dev, "Set logical address failed (%d)", status);
	return status;
}

static int secocec_adap_transmit(struct cec_adapter *adap, u8 attempts,
				 u32 signal_free_time, struct cec_msg *msg)
{
	struct secocec_data *cec = adap->priv;
	struct device *dev = cec->dev;
	int status;
	unsigned short val;
	unsigned short payload_len, payload_id_len, destination = 0;
	u8 i;
	u8 *payload_msg;

	dev_dbg(dev, "Sending message (len %d)", msg->len);

	for (i = 0; i < msg->len; i++)
		pr_debug("\t byte %d : 0x%02x\n", i, msg->msg[i]);

	if (msg->len > 12) {
		dev_warn(dev,
			 "Trying to send a message longer than 12 bytes, cutting");
		msg->len = 12;
	}
	/* Device msg len already accounts for header */
	payload_id_len = msg->len - 1;

	/* Send data length */
	status = smb_wr16(CEC_WRITE_DATA_LENGTH, payload_id_len);
	if (status)
		goto err;

	/* Send Operation ID if present */
	if (payload_id_len > 0) {
		status = smb_wr16(CEC_WRITE_OPERATION_ID, msg->msg[1]);
		if (status)
			goto err;
	}
	/* Send data if present */
	if (payload_id_len > 1) {
		/* Only data; */
		payload_len = msg->len - 2;
		payload_msg = &msg->msg[2];

		/* Copy message into registers */
		for (i = 0; i < payload_len / 2 + payload_len % 2; i++) {
			/* hi byte */
			val = payload_msg[(i << 1) + 1] << 8;

			/* lo byte */
			val |= payload_msg[(i << 1)];

			status = smb_wr16(CEC_WRITE_DATA_00 + i, val);
			if (status)
				goto err;
		}
	}
	/* Send msg source/destination and fire msg */
	destination = msg->msg[0];
	status = smb_wr16(CEC_WRITE_BYTE0, destination);
	if (status)
		goto err;

	return 0;

err:
	dev_err(dev, "Transmit failed (%d)", status);
	return status;
}

static int secocec_tx_done(struct cec_adapter *adap, unsigned short status_val)
{
	struct secocec_data *cec = adap->priv;
	struct device *dev = cec->dev;

	int status = 0;

	if (status_val & CEC_STATUS_TX_ERROR_MASK) {
		if (status_val & CEC_STATUS_TX_LINE_ERROR) {
			cec_transmit_done(adap, CEC_TX_STATUS_ARB_LOST, 1, 0, 0,
					  0);
			status = -EBUSY;
			dev_warn(dev, "Transmit failed (LINE_ERR)");
		} else if (status_val & CEC_STATUS_TX_NACK_ERROR) {
			cec_transmit_done(adap, CEC_TX_STATUS_NACK, 0, 1, 0, 0);
			status = -EAGAIN;
			dev_dbg(dev, "Transmit not acknowledged (NACK)");
		} else {
			cec_transmit_done(adap, CEC_TX_STATUS_ERROR, 0, 0, 0,
					  1);
			status = -EIO;
			dev_warn(dev, "Transmit failed (ERROR)");
		}

	} else {
		dev_dbg(dev, "Transmitted frame successfully");
		cec_transmit_done(adap, CEC_TX_STATUS_OK, 0, 0, 0, 0);
	}

	/* Reset status reg */
	status_val = CEC_STATUS_TX_ERROR_MASK | CEC_STATUS_MSG_SENT_MASK |
	    CEC_STATUS_TX_NACK_ERROR | CEC_STATUS_TX_LINE_ERROR;
	smb_wr16(CEC_STATUS, status_val);

	return status;
}

static int secocec_rx_done(struct cec_adapter *adap, unsigned short status_val)
{
	struct secocec_data *cec = adap->priv;
	struct device *dev = cec->dev;
	struct cec_msg msg = { };
	u8 i;
	u8 payload_len, payload_id_len = 0;
	u8 *payload_msg;

	int status;
	unsigned short val = 0;

	if (status_val & CEC_STATUS_RX_ERROR_MASK) {
		dev_warn(dev, "Message received with errors. Discarding");
		status = -EIO;
		goto rxerr;
	}

	/* Read message length */
	status = smb_rd16(CEC_READ_DATA_LENGTH, &val);
	if (status)
		goto err;

	payload_id_len = val;
	dev_dbg(dev, "Incoming message (payload len %d):", payload_id_len);

	if (payload_id_len > 11) {
		payload_id_len = 11;
		dev_warn(dev, "Message received longer than 12 bytes, cutting");
	}

	/* Device msg len already accounts for the header */
	msg.len = payload_id_len + 1;

	/* Read logical address */
	status = smb_rd16(CEC_READ_BYTE0, &val);
	if (status)
		goto err;

	/* device stores source LA and destination */
	msg.msg[0] = val;

	/* Read operation ID if present */
	if (payload_id_len > 0) {
		status = smb_rd16(CEC_READ_OPERATION_ID, &val);
		if (status)
			goto err;

		msg.msg[1] = val;
	}

	/* Read data if present */
	if (payload_id_len > 1) {
		payload_len = msg.len - 2;
		payload_msg = &msg.msg[2];

		/* device stores 2 bytes in every 16bit val */
		for (i = 0; i < payload_len / 2 + payload_len % 2; i++) {
			status = smb_rd16(CEC_READ_DATA_00 + i, &val);
			if (status)
				goto err;

			/* low byte, skipping header */
			payload_msg[(i << 1)] = val & 0x00FF;

			/* hi byte */
			payload_msg[(i << 1) + 1] = (val & 0xFF00) >> 8;
		}
	}

	for (i = 0; i < msg.len; i++)
		pr_debug("\t byte %d : 0x%02x\n", i, msg.msg[i]);

	cec_received_msg(cec->cec_adap, &msg);

	/* Reset status reg */
	status_val = CEC_STATUS_MSG_RECEIVED_MASK;
	status = smb_wr16(CEC_STATUS, status_val);
	if (status)
		goto err;

	dev_dbg(dev, "Message received successfully");

	return 0;

rxerr:
	/* Reset error reg */
	status_val = CEC_STATUS_MSG_RECEIVED_MASK | CEC_STATUS_RX_ERROR_MASK;
	smb_wr16(CEC_STATUS, status_val);

err:
	dev_err(dev, "Receive message failed (%d)", status);
	return status;
}

struct cec_adap_ops secocec_cec_adap_ops = {
	/* Low-level callbacks */
	.adap_enable = secocec_adap_enable,
	.adap_log_addr = secocec_adap_log_addr,
	.adap_transmit = secocec_adap_transmit,
};

static irqreturn_t secocec_irq_handler(int irq, void *priv)
{
	struct secocec_data *cec = priv;
	struct device *dev = cec->dev;

	int status;
	unsigned short status_val, cec_val, val = 0;

	/*  Read status register */
	status = smb_rd16(STATUS_REGISTER_1, &status_val);
	if (status)
		goto err;

	if (status_val & STATUS_REGISTER_1_CEC) {
		dev_dbg(dev, "+++++ CEC Interrupt Caught");

		/* Read CEC status register */
		status = smb_rd16(CEC_STATUS, &cec_val);
		if (status)
			goto err;

		if (cec_val & CEC_STATUS_MSG_RECEIVED_MASK)
			secocec_rx_done(cec->cec_adap, cec_val);

		if (cec_val & CEC_STATUS_MSG_SENT_MASK)
			secocec_tx_done(cec->cec_adap, cec_val);

		if ((~cec_val & CEC_STATUS_MSG_SENT_MASK) &&
		    (~cec_val & CEC_STATUS_MSG_RECEIVED_MASK))
			dev_warn(dev,
				 "Message not received or sent, but interrupt fired \\_\"._/");

		val = STATUS_REGISTER_1_CEC;
	}

	if (status_val & STATUS_REGISTER_1_IRDA_RC5) {
		dev_dbg(dev, "IRDA RC5 Interrupt Caught");
		val |= STATUS_REGISTER_1_IRDA_RC5;
		//TODO IRDA RX
	}

	/*  Reset status register */
	status = smb_wr16(STATUS_REGISTER_1, val);
	if (status)
		goto err;

	dev_dbg(dev, "----- CEC Interrupt Handled");

	return IRQ_HANDLED;

err:
	dev_err(dev, "IRQ: Read/Write SMBus operation failed (%d)", status);

	/*  Reset status register */
	val = STATUS_REGISTER_1_CEC | STATUS_REGISTER_1_IRDA_RC5;
	smb_wr16(STATUS_REGISTER_1, val);

	return IRQ_HANDLED;
}

static irqreturn_t secocec_irq_handler_quick(int irq, void *priv)
{
	return IRQ_WAKE_THREAD;
}

static int secocec_acpi_probe(struct secocec_data *sdev)
{
	struct device *dev = sdev->dev;
	struct gpio_desc *gpio;
	int irq = 0;

	gpio = devm_gpiod_get(dev, NULL, GPIOF_IN);
	if (IS_ERR(gpio)) {
		dev_err(dev, "Cannot request interrupt gpio");
		return PTR_ERR(gpio);
	}

	irq = gpiod_to_irq(gpio);
	if (irq < 0) {
		dev_err(dev, "Cannot find valid irq");
		return -ENODEV;
	}
	dev_dbg(dev, "irq-gpio is bound to IRQ %d", irq);

	sdev->irq = irq;

	return 0;
}

static int secocec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct secocec_data *secocec = secocec_data_init(pdev);
	int ret;
	u8 opts;

	if (!has_acpi_companion(dev)) {
		dev_dbg(dev, "Cannot find any ACPI companion");
		ret = -ENODEV;
		goto err;
	}

	ret = secocec_acpi_probe(secocec);
	if (ret) {
		dev_err(dev, "Cannot assign gpio to IRQ");
		ret = -ENODEV;
		goto err;
	}

	dev_dbg(dev, "IRQ detected at %d", secocec->irq);

	ret = devm_request_threaded_irq(dev,
					secocec->irq,
					secocec_irq_handler_quick,
					secocec_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev_name(&pdev->dev), secocec);

	if (ret) {
		dev_err(dev, "Cannot request IRQ %d", secocec->irq);
		ret = -EIO;
		goto err;
	}
	/* Allocate CEC adapter */
	opts = CEC_CAP_DEFAULTS | CEC_CAP_PHYS_ADDR;

	secocec->cec_adap = cec_allocate_adapter(&secocec_cec_adap_ops,
						 secocec,
						 dev_name(dev),
						 opts, SECOCEC_MAX_ADDRS);

	if (IS_ERR(secocec->cec_adap))
		ret = PTR_ERR(secocec->cec_adap);
	if (ret)
		goto err;

	ret = cec_register_adapter(secocec->cec_adap, dev);
	if (ret)
		goto err_delete_adapter;

	platform_set_drvdata(pdev, secocec);

	dev_dbg(dev, "Device registered");

	return ret;

err_delete_adapter:
	cec_delete_adapter(secocec->cec_adap);
err:
	dev_err(dev, "%s device probe failed\n", dev_name(dev));

	return ret;
}

/* ----------------------------------------------------------------------- */

static int secocec_remove(struct platform_device *pdev)
{
	struct secocec_data *secocec = platform_get_drvdata(pdev);

	cec_unregister_adapter(secocec->cec_adap);

	return 0;
}

/* ----------------------------------------------------------------------- */

#ifdef CONFIG_PM_SLEEP
static int secocec_suspend(struct device *dev)
{
	int status;
	unsigned short val;

	dev_dbg(dev, "Device going to suspend, disabling");

	/* Clear the status register */
	status = smb_rd16(STATUS_REGISTER_1, &val);
	if (status)
		goto err;

	status = smb_wr16(STATUS_REGISTER_1, val);
	if (status)
		goto err;

	/* Disable the interrupts */
	status = smb_rd16(ENABLE_REGISTER_1, &val);
	if (status)
		goto err;

	status = smb_wr16(ENABLE_REGISTER_1, val &
			  ~ENABLE_REGISTER_1_CEC & ~ENABLE_REGISTER_1_IRDA_RC5);
	if (status)
		goto err;

	return 0;

err:
	dev_err(dev, "Suspend failed (err: %d)", status);
	return status;
}

static int secocec_resume(struct device *dev)
{
	int status;
	unsigned short val;

	dev_dbg(dev, "Resuming device from suspend");

	/* Clear the status register */
	status = smb_rd16(STATUS_REGISTER_1, &val);
	if (status)
		goto err;

	status = smb_wr16(STATUS_REGISTER_1, val);
	if (status)
		goto err;

	/* Enable the interrupts */
	status = smb_rd16(ENABLE_REGISTER_1, &val);
	if (status)
		goto err;

	status = smb_wr16(ENABLE_REGISTER_1, val | ENABLE_REGISTER_1_CEC);
	if (status)
		goto err;

	dev_dbg(dev, "Device resumed from suspend");

	return 0;

err:
	dev_err(dev, "Resume failed (err: %d)", status);
	return status;
}

static SIMPLE_DEV_PM_OPS(secocec_pm_ops, secocec_suspend, secocec_resume);
#define SECOCEC_PM_OPS (&secocec_pm_ops)
#else
#define SECOCEC_PM_OPS NULL
#endif

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
		   .pm = SECOCEC_PM_OPS,
		   },
	.probe = secocec_probe,
	.remove = secocec_remove,
};

module_platform_driver(secocec_driver);

MODULE_DESCRIPTION("SECO CEC X86 Driver");
MODULE_AUTHOR("Ettore Chimenti <ek5.chimenti@gmail.com>");
MODULE_LICENSE("Dual BSD/GPL");
