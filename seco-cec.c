// SPDX-License-Identifier: GPL-2.0 AND BSD-3-Clause
/*
 *
 * CEC driver for SECO X86 Boards
 *
 * Author:  Ettore Chimenti <ek5.chimenti@gmail.com>
 * Copyright (C) 2018, SECO Srl.
 * Copyright (C) 2018, Aidilab Srl.
 *
 */

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/dmi.h>

/* CEC Framework */
#include <media/cec.h>

#include "seco-cec.h"

struct secocec_data {
	struct device *dev;
	struct platform_device *pdev;
	struct cec_adapter *cec_adap;
	struct cec_notifier *notifier;
	struct rc_dev *irda_rc;
	char irda_input_name[32];
	char irda_input_phys[32];
	int irq;
};

#define smb_wr16(cmd, data) smb_word_op(CMD_WORD_DATA, SECOCEC_MICRO_ADDRESS, \
					     cmd, data, SMBUS_WRITE, NULL)
#define smb_rd16(cmd, res) smb_word_op(CMD_WORD_DATA, SECOCEC_MICRO_ADDRESS, \
				       cmd, 0, SMBUS_READ, res)

static int smb_word_op(short data_format, u16 slave_addr, u8 cmd, u16 data,
		       u8 operation, u16 *result)
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
		pr_debug("%s: request_region BRA_SMB_BASE_ADDR fail\n",
			 __func__);
		return -ENXIO;
	}

	/* Active wait until ready */
	for (count = 0; count <= SMBTIMEOUT; ++count) {
		if (!(inb(HSTS) & BRA_INUSE_STS))
			break;
		udelay(SMB_POLL_UDELAY);
	}

	if (count > SMBTIMEOUT) {
		/* Reset the lock instead of failing */
		outb(0xff, HSTS);
		pr_warn("%s: SMBTIMEOUT\n", __func__);
	}

	outb(0x00, HCNT);
	outb((u8)(slave_addr & 0xfe) | operation, XMIT_SLVA);
	outb(cmd, HCMD);
	inb(HCNT);

	if (operation == SMBUS_WRITE) {
		outb((u8)data, HDAT0);
		outb((u8)(data >> 8), HDAT1);
		pr_debug("%s: WRITE (0x%02x - count %05d): 0x%04x\n",
			 __func__, cmd, count, data);
	}

	outb(BRA_START + _data_format, HCNT);

	for (count = 0; count <= SMBTIMEOUT; count++) {
		if (!(inb(HSTS) & BRA_HOST_BUSY))
			break;
		udelay(SMB_POLL_UDELAY);
	}

	if (count > SMBTIMEOUT) {
		pr_debug("%s: SMBTIMEOUT_1\n", __func__);
		status = -EBUSY;
		goto err;
	}

	ret = inb(HSTS);
	if (ret & BRA_HSTS_ERR_MASK) {
		pr_debug("%s: HSTS(0x%02X): 0x%X\n", __func__, cmd, ret);
		status = -EIO;
		goto err;
	}

	if (operation == SMBUS_READ) {
		*result = ((inb(HDAT0) & 0xff) + ((inb(HDAT1) & 0xff) << 8));
		pr_debug("%s: READ (0x%02x - count %05d): 0x%04x\n",
			 __func__, cmd, count, *result);
	}

err:
	outb(0xff, HSTS);
	release_region(BRA_SMB_BASE_ADDR, 7);

	return status;
}

static int secocec_adap_enable(struct cec_adapter *adap, bool enable)
{
	struct secocec_data *cec = cec_get_drvdata(adap);
	struct device *dev = cec->dev;
	u16 val = 0;
	int status;

	if (enable) {
		/* Clear the status register */
		status = smb_rd16(SECOCEC_STATUS_REG_1, &val);
		if (status)
			goto err;

		status = smb_wr16(SECOCEC_STATUS_REG_1, val);
		if (status)
			goto err;

		/* Enable the interrupts */
		status = smb_rd16(SECOCEC_ENABLE_REG_1, &val);
		if (status)
			goto err;

		status = smb_wr16(SECOCEC_ENABLE_REG_1,
				  val | SECOCEC_ENABLE_REG_1_CEC);
		if (status)
			goto err;

		dev_dbg(dev, "Device enabled");

	} else {
		/* Clear the status register */
		status = smb_rd16(SECOCEC_STATUS_REG_1, &val);
		if (status)
			goto err;

		status = smb_wr16(SECOCEC_STATUS_REG_1, val);
		if (status)
			goto err;

		/* Disable the interrupts */
		status = smb_rd16(SECOCEC_ENABLE_REG_1, &val);
		if (status)
			goto err;

		status = smb_wr16(SECOCEC_ENABLE_REG_1, val &
				  ~SECOCEC_ENABLE_REG_1_CEC &
				  ~SECOCEC_ENABLE_REG_1_IR);
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
	u16 enable_val = 0;
	int status;

	/* Disable device */
	status = smb_rd16(SECOCEC_ENABLE_REG_1, &enable_val);
	if (status)
		return status;

	status = smb_wr16(SECOCEC_ENABLE_REG_1,
			  enable_val & ~SECOCEC_ENABLE_REG_1_CEC);
	if (status)
		return status;

	/* Write logical address */
	status = smb_wr16(SECOCEC_DEVICE_LA, logical_addr);
	if (status)
		return status;

	/* Re-enable device */
	status = smb_wr16(SECOCEC_ENABLE_REG_1,
			  enable_val | SECOCEC_ENABLE_REG_1_CEC);
	if (status)
		return status;

	return 0;
}

static int secocec_adap_transmit(struct cec_adapter *adap, u8 attempts,
				 u32 signal_free_time, struct cec_msg *msg)
{
	struct secocec_data *cec = cec_get_drvdata(adap);
	struct device *dev = cec->dev;
	u16 payload_len, payload_id_len, destination, val = 0;
	u8 *payload_msg;
	int status;
	u8 i;

	/* Device msg len already accounts for header */
	payload_id_len = msg->len - 1;

	/* Send data length */
	status = smb_wr16(SECOCEC_WRITE_DATA_LENGTH, payload_id_len);
	if (status)
		goto err;

	/* Send Operation ID if present */
	if (payload_id_len > 0) {
		status = smb_wr16(SECOCEC_WRITE_OPERATION_ID, msg->msg[1]);
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

			status = smb_wr16(SECOCEC_WRITE_DATA_00 + i, val);
			if (status)
				goto err;
		}
	}
	/* Send msg source/destination and fire msg */
	destination = msg->msg[0];
	status = smb_wr16(SECOCEC_WRITE_BYTE0, destination);
	if (status)
		goto err;

	return 0;

err:
	dev_err(dev, "Transmit failed (%d)", status);
	return status;
}

static int secocec_tx_done(struct cec_adapter *adap, u16 status_val)
{
	int status = 0;

	if (status_val & SECOCEC_STATUS_TX_ERROR_MASK) {
		if (status_val & SECOCEC_STATUS_TX_NACK_ERROR) {
			cec_transmit_attempt_done(adap, CEC_TX_STATUS_NACK);
			status = -EAGAIN;
		} else {
			cec_transmit_attempt_done(adap, CEC_TX_STATUS_ERROR);
			status = -EIO;
		}
	} else {
		cec_transmit_attempt_done(adap, CEC_TX_STATUS_OK);
	}

	/* Reset status reg */
	status_val = SECOCEC_STATUS_TX_ERROR_MASK |
		SECOCEC_STATUS_MSG_SENT_MASK |
		SECOCEC_STATUS_TX_NACK_ERROR;
	smb_wr16(SECOCEC_STATUS, status_val);

	return status;
}

static int secocec_rx_done(struct cec_adapter *adap, u16 status_val)
{
	struct secocec_data *cec = cec_get_drvdata(adap);
	struct device *dev = cec->dev;
	struct cec_msg msg = { };
	bool flag_overflow = false;
	u8 payload_len, i = 0;
	u8 *payload_msg;
	u16 val = 0;
	int status;

	if (status_val & SECOCEC_STATUS_RX_OVERFLOW_MASK) {
		dev_warn(dev, "Received more than 16 bytes. Discarding");
		flag_overflow = true;
	}

	if (status_val & SECOCEC_STATUS_RX_ERROR_MASK) {
		dev_warn(dev, "Message received with errors. Discarding");
		status = -EIO;
		goto rxerr;
	}

	/* Read message length */
	status = smb_rd16(SECOCEC_READ_DATA_LENGTH, &val);
	if (status)
		goto err;

	dev_dbg(dev, "Incoming message (payload len %d):", val);

	/* Device msg len already accounts for the header */
	msg.len = min(val + 1, CEC_MAX_MSG_SIZE);

	/* Read logical address */
	status = smb_rd16(SECOCEC_READ_BYTE0, &val);
	if (status)
		goto err;

	/* device stores source LA and destination */
	msg.msg[0] = val;

	/* Read operation ID if present */
	if (msg.len > 0) {
		status = smb_rd16(SECOCEC_READ_OPERATION_ID, &val);
		if (status)
			goto err;

		msg.msg[1] = val;
	}

	/* Read data if present */
	if (msg.len > 1) {
		payload_len = msg.len - 2;
		payload_msg = &msg.msg[2];

		/* device stores 2 bytes in every 16-bit val */
		for (i = 0; i < payload_len / 2 + payload_len % 2; i++) {
			status = smb_rd16(SECOCEC_READ_DATA_00 + i, &val);
			if (status)
				goto err;

			/* low byte, skipping header */
			payload_msg[(i << 1)] = val & 0x00ff;

			/* hi byte */
			payload_msg[(i << 1) + 1] = (val & 0xff00) >> 8;
		}
	}

	cec_received_msg(cec->cec_adap, &msg);

	/* Reset status reg */
	status_val = SECOCEC_STATUS_MSG_RECEIVED_MASK;
	if (flag_overflow)
		status_val |= SECOCEC_STATUS_RX_OVERFLOW_MASK;

	status = smb_wr16(SECOCEC_STATUS, status_val);
	if (status)
		goto err;

	dev_dbg(dev, "Message received successfully");

	return 0;

rxerr:
	/* Reset error reg */
	status_val = SECOCEC_STATUS_MSG_RECEIVED_MASK |
		SECOCEC_STATUS_RX_ERROR_MASK;
	if (flag_overflow)
		status_val |= SECOCEC_STATUS_RX_OVERFLOW_MASK;
	smb_wr16(SECOCEC_STATUS, status_val);

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

#ifdef CONFIG_VIDEO_SECO_RC
static int secocec_irda_probe(void *priv)
{
	struct secocec_data *cec = priv;
	struct device *dev = cec->dev;
	int status;
	u16 val;

	/* Prepare the RC input device */
	cec->irda_rc = devm_rc_allocate_device(dev, RC_DRIVER_SCANCODE);
	if (!cec->irda_rc) {
		dev_err(dev, "Failed to allocate memory for rc_dev");
		return -ENOMEM;
	}

	snprintf(cec->irda_input_name, sizeof(cec->irda_input_name),
		 "IrDA RC for %s", dev_name(dev));
	snprintf(cec->irda_input_phys, sizeof(cec->irda_input_phys),
		 "%s/input0", dev_name(dev));

	cec->irda_rc->device_name = cec->irda_input_name;
	cec->irda_rc->input_phys = cec->irda_input_phys;
	cec->irda_rc->input_id.bustype = BUS_HOST;
	cec->irda_rc->input_id.vendor = 0;
	cec->irda_rc->input_id.product = 0;
	cec->irda_rc->input_id.version = 1;
	cec->irda_rc->driver_name = SECOCEC_DEV_NAME;
	cec->irda_rc->allowed_protocols = RC_PROTO_BIT_RC5;
	cec->irda_rc->enabled_protocols = RC_PROTO_BIT_RC5;
	cec->irda_rc->priv = cec;
	cec->irda_rc->map_name = RC_MAP_HAUPPAUGE;
	cec->irda_rc->timeout = MS_TO_NS(100);

	/* Clear the status register */
	status = smb_rd16(SECOCEC_STATUS_REG_1, &val);
	if (status != 0)
		goto err;

	status = smb_wr16(SECOCEC_STATUS_REG_1, val);
	if (status != 0)
		goto err;

	/* Enable the interrupts */
	status = smb_rd16(SECOCEC_ENABLE_REG_1, &val);
	if (status != 0)
		goto err;

	status = smb_wr16(SECOCEC_ENABLE_REG_1,
			  val | SECOCEC_ENABLE_REG_1_IR);
	if (status != 0)
		goto err;

	dev_dbg(dev, "IR enabled");

	status = devm_rc_register_device(dev, cec->irda_rc);

	if (status) {
		dev_err(dev, "Failed to prepare input device");
		cec->irda_rc = NULL;
		goto err;
	}

	return 0;

err:
	smb_rd16(SECOCEC_ENABLE_REG_1, &val);

	smb_wr16(SECOCEC_ENABLE_REG_1,
		 val & ~SECOCEC_ENABLE_REG_1_IR);

	dev_dbg(dev, "IR disabled");
	return status;
}

static int secocec_irda_rx(struct secocec_data *priv)
{
	struct secocec_data *cec = priv;
	struct device *dev = cec->dev;
	u16 val, status, key, addr, toggle;

	if (!cec->irda_rc)
		return -ENODEV;

	status = smb_rd16(SECOCEC_IR_READ_DATA, &val);
	if (status != 0)
		goto err;

	key = val & SECOCEC_IR_COMMAND_MASK;
	addr = (val & SECOCEC_IR_ADDRESS_MASK) >> SECOCEC_IR_ADDRESS_SHL;
	toggle = (val & SECOCEC_IR_TOGGLE_MASK) >> SECOCEC_IR_TOGGLE_SHL;

	rc_keydown(cec->irda_rc, RC_PROTO_RC5, key, toggle);

	dev_dbg(dev, "IR key pressed: 0x%02x addr 0x%02x toggle 0x%02x", key,
		addr, toggle);

	return 0;

err:
	dev_err(dev, "IR Receive message failed (%d)", status);
	return -EIO;
}
#else
static void secocec_irda_rx(struct secocec_data *priv)
{
}

static int secocec_irda_probe(void *priv)
{
	return 0;
}
#endif

static irqreturn_t secocec_irq_handler(int irq, void *priv)
{
	struct secocec_data *cec = priv;
	struct device *dev = cec->dev;
	u16 status_val, cec_val, val = 0;
	int status;

	/*  Read status register */
	status = smb_rd16(SECOCEC_STATUS_REG_1, &status_val);
	if (status)
		goto err;

	if (status_val & SECOCEC_STATUS_REG_1_CEC) {
		dev_dbg(dev, "+++++ CEC Interrupt Caught");

		/* Read CEC status register */
		status = smb_rd16(SECOCEC_STATUS, &cec_val);
		if (status)
			goto err;

		if (cec_val & SECOCEC_STATUS_MSG_RECEIVED_MASK)
			secocec_rx_done(cec->cec_adap, cec_val);

		if (cec_val & SECOCEC_STATUS_MSG_SENT_MASK)
			secocec_tx_done(cec->cec_adap, cec_val);

		if ((~cec_val & SECOCEC_STATUS_MSG_SENT_MASK) &&
		    (~cec_val & SECOCEC_STATUS_MSG_RECEIVED_MASK))
			dev_warn(dev,
				 "Message not received or sent, but interrupt fired \\_\"._/");

		val = SECOCEC_STATUS_REG_1_CEC;
	}

	if (status_val & SECOCEC_STATUS_REG_1_IR) {
		dev_dbg(dev, "IR RC5 Interrupt Caught");
		val |= SECOCEC_STATUS_REG_1_IR;

		secocec_irda_rx(cec);
	}

	/*  Reset status register */
	status = smb_wr16(SECOCEC_STATUS_REG_1, val);
	if (status)
		goto err;

	dev_dbg(dev, "----- CEC Interrupt Handled");

	return IRQ_HANDLED;

err:
	dev_err(dev, "IRQ: Read/Write SMBus operation failed (%d)", status);

	/*  Reset status register */
	val = SECOCEC_STATUS_REG_1_CEC | SECOCEC_STATUS_REG_1_IR;
	smb_wr16(SECOCEC_STATUS_REG_1, val);

	return IRQ_HANDLED;
}

#ifdef CONFIG_CEC_NOTIFIER
struct cec_dmi_match {
	char *sys_vendor;
	char *product_name;
	char *devname;
	char *conn;
};

static const struct cec_dmi_match secocec_dmi_match_table[] = {
	/* UDOO X86 */
	{ "SECO", "UDOO x86", "0000:00:02.0", "Port B" },
};

static int secocec_cec_get_notifier(struct cec_notifier **notify)
{
	int i;

	for (i = 0 ; i < ARRAY_SIZE(secocec_dmi_match_table) ; ++i) {
		const struct cec_dmi_match *m = &secocec_dmi_match_table[i];

		if (dmi_match(DMI_SYS_VENDOR, m->sys_vendor) &&
		    dmi_match(DMI_PRODUCT_NAME, m->product_name)) {
			struct device *d;

			/* Find the device, bail out if not yet registered */
			d = bus_find_device_by_name(&pci_bus_type, NULL,
						    m->devname);
			if (!d)
				return -EPROBE_DEFER;

			*notify = cec_notifier_get_conn(d, m->conn);
			return 0;
		}
	}

	return -EINVAL;
}
#endif

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
	struct secocec_data *secocec;
	struct device *dev = &pdev->dev;
	u8 cec_caps;
	int ret;
	u16 val;

	secocec = devm_kzalloc(dev, sizeof(*secocec), GFP_KERNEL);
	if (!secocec)
		return -ENOMEM;

	dev_set_drvdata(dev, secocec);

	secocec->pdev = pdev;
	secocec->dev = dev;

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

	/* Firmware version check */
	ret = smb_rd16(SECOCEC_VERSION, &val);
	if (ret) {
		dev_err(dev, "Cannot check fw version");
		goto err;
	}
	if (val < SECOCEC_LATEST_FW) {
		dev_err(dev, "CEC Firmware not supported (v.%04x). Use ver > v.%04x",
			val, SECOCEC_LATEST_FW);
		ret = -EINVAL;
		goto err;
	}

#ifdef CONFIG_CEC_NOTIFIER
	ret = secocec_cec_get_notifier(&secocec->notifier);
	if (ret) {
		dev_err(dev, "no CEC notifier available\n");
		goto err;
	}
#endif

	ret = devm_request_threaded_irq(dev,
					secocec->irq,
					NULL,
					secocec_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev_name(&pdev->dev), secocec);

	if (ret) {
		dev_err(dev, "Cannot request IRQ %d", secocec->irq);
		ret = -EIO;
		goto err;
	}

	/* Allocate CEC adapter */
	cec_caps = CEC_CAP_DEFAULTS;

	secocec->cec_adap = cec_allocate_adapter(&secocec_cec_adap_ops,
						 secocec,
						 dev_name(dev),
						 cec_caps, SECOCEC_MAX_ADDRS);

	if (IS_ERR(secocec->cec_adap)) {
		ret = PTR_ERR(secocec->cec_adap);
		goto err;
	}

	ret = cec_register_adapter(secocec->cec_adap, dev);
	if (ret)
		goto err_delete_adapter;

	if (secocec->notifier)
		cec_register_cec_notifier(secocec->cec_adap, secocec->notifier);

	ret = secocec_irda_probe(secocec);
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
	u16 val;

	if (secocec->irda_rc) {
		smb_rd16(SECOCEC_ENABLE_REG_1, &val);

		smb_wr16(SECOCEC_ENABLE_REG_1,
			 val & ~SECOCEC_ENABLE_REG_1_IR);

		dev_dbg(&pdev->dev, "IR disabled");
	}
	cec_unregister_adapter(secocec->cec_adap);

	if (secocec->notifier)
		cec_notifier_put(secocec->notifier);

	dev_dbg(&pdev->dev, "CEC device removed");

	return 0;
}

/* ----------------------------------------------------------------------- */

#ifdef CONFIG_PM_SLEEP
static int secocec_suspend(struct device *dev)
{
	int status;
	u16 val;

	dev_dbg(dev, "Device going to suspend, disabling");

	/* Clear the status register */
	status = smb_rd16(SECOCEC_STATUS_REG_1, &val);
	if (status)
		goto err;

	status = smb_wr16(SECOCEC_STATUS_REG_1, val);
	if (status)
		goto err;

	/* Disable the interrupts */
	status = smb_rd16(SECOCEC_ENABLE_REG_1, &val);
	if (status)
		goto err;

	status = smb_wr16(SECOCEC_ENABLE_REG_1, val &
			  ~SECOCEC_ENABLE_REG_1_CEC & ~SECOCEC_ENABLE_REG_1_IR);
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
	u16 val;

	dev_dbg(dev, "Resuming device from suspend");

	/* Clear the status register */
	status = smb_rd16(SECOCEC_STATUS_REG_1, &val);
	if (status)
		goto err;

	status = smb_wr16(SECOCEC_STATUS_REG_1, val);
	if (status)
		goto err;

	/* Enable the interrupts */
	status = smb_rd16(SECOCEC_ENABLE_REG_1, &val);
	if (status)
		goto err;

	status = smb_wr16(SECOCEC_ENABLE_REG_1, val | SECOCEC_ENABLE_REG_1_CEC);
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
