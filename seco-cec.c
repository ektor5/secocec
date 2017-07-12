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

#include "Stm32Microntroller.h"

// CEC Framework
#include <media/cec.h>

#define SECOCEC_MAX_ADDRS 1
#define SECOCEC_DEV_NAME "secocec"
#define GPIO_I2C6_SCL 51
#define GPIOCHIP_SOUTHWEST 456

#define MICRO_ADDRESS 	0x40

#define SMBUS_WRITE	0
#define SMBUS_READ	1

#define SMBTIMEOUT 0xFFFF

#define CMD_BYTE_DATA					0
#define CMD_WORD_DATA					1


// --------------------------------------------------------
// ------------ SMBus definitons for Brasswell ------------
// --------------------------------------------------------
#define BRA_DONE_STATUS			(1<<7)
#define	BRA_INUSE_STS			(1<<6)
#define BRA_FAILED_OP			(1<<4)
#define BRA_BUS_ERR			(1<<3)
#define	BRA_DEV_ERR			(1<<2)
#define BRA_INTR			(1<<1)
#define BRA_HOST_BUSY			(1<<0)
#define BRA_HSTS_ERR_MASK   (BRA_FAILED_OP | BRA_BUS_ERR | BRA_DEV_ERR)

#define BRA_PEC_EN 			(1<<7)
#define	BRA_START			(1<<6)
#define BRA_LAST__BYTE			(1<<5)
#define BRA_SMB_CMD 			(7<<2)
#define	BRA_SMB_CMD_QUICK		(0<<2)
#define BRA_SMB_CMD_BYTE		(1<<2)
#define	BRA_SMB_CMD_BYTE_DATA		(2<<2)
#define BRA_SMB_CMD_WORD_DATA		(3<<2)
#define	BRA_SMB_CMD_PROCESS_CALL	(4<<2)
#define BRA_SMB_CMD_BLOCK		(5<<2)
#define	BRA_SMB_CMD_I2CREAD		(6<<2)
#define BRA_SMB_CMD_BLOCK_PROCESS 	(7<<2)
#define BRA_INTREN			(1<<0)

/* ---------------------------------------------------------------- */

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

static int smbWordOp(
		     short DataFormat, 
		     unsigned short slaveAddr, 
		     unsigned char cmd, 
		     unsigned short data, 
		     unsigned char operation, 
		     unsigned short * result
		    )
{
	unsigned int count;
	short DataFormat_Local;
	int ret;

	unsigned long m_SMBus_Base_Address = 0x2040;
	//unsigned long ulRet;

	//PCI_SLOT_NUMBER slotNumber;

	//slotNumber.u.bits.Reserved = 0;
	//slotNumber.u.bits.DeviceNumber = 31;
	//slotNumber.u.bits.FunctionNumber = 3;

	//ulRet = HalGetBusDataByOffset(
	//	PCIConfiguration, 
	//	0, 
	//	slotNumber.u.AsULONG, 
	//	&m_SMBus_Base_Address, 
	//	0x20, 
	//	sizeof(unsigned short));

	//m_SMBus_Base_Address = m_SMBus_Base_Address & 0xFFFFFFE0;

	unsigned short HSTS = (unsigned short)m_SMBus_Base_Address + 0;
	unsigned short HCNT = (unsigned short)m_SMBus_Base_Address + 2;
	unsigned short HCMD = (unsigned short)m_SMBus_Base_Address + 3;
	unsigned short XMIT_SLVA = (unsigned short)m_SMBus_Base_Address + 4;
	unsigned short HDAT0 = (unsigned short)m_SMBus_Base_Address + 5;
	unsigned short HDAT1 = (unsigned short)m_SMBus_Base_Address + 6;
	//unsigned short BLOCK_DB	= m_SMBus_Base_Address+7;

	switch (DataFormat)
	{
	case CMD_BYTE_DATA:		DataFormat_Local = BRA_SMB_CMD_BYTE_DATA; break;
	case CMD_WORD_DATA:		DataFormat_Local = BRA_SMB_CMD_WORD_DATA; break;
	default:
					return 1; 
					break;
	}


	if(!request_muxed_region(0xEB, 1, "CEC00001"))
	{
		pr_debug("request_region 0xEB fail\n");
		return 1;
	}

	if(!request_muxed_region(m_SMBus_Base_Address, 7, "CEC00001"))
	{
		pr_debug("request_region m_SMBus_Base_Address fail\n");
		release_region(0xEB, 1);
		return 2;
	}

	for(count=0; (count <= SMBTIMEOUT)&&(inb(HSTS)&BRA_INUSE_STS); ++count);

	if (count > SMBTIMEOUT)
	{
		pr_debug("smbWordOp SMBTIMEOUT\n");
		outb(0xFF, HSTS);
		release_region(0xEB, 1);
		pr_debug("smbWordOp RELEASE 0xEB\n");
		release_region(m_SMBus_Base_Address, 7);
		pr_debug("smbWordOp RELEASE Base Addr\n");
		return 3;
	}

	//outb(~BRA_INUSE_STS, HSTS);
	outb(0x00, HCNT);
	outb((unsigned char)(slaveAddr & 0xFE) | operation, XMIT_SLVA);
	outb(cmd, HCMD);
	inb(HCNT); // per azzeramento contatori su eventuale block read
	if (operation == SMBUS_WRITE)
	{
		outb((unsigned char)data, HDAT0);
		outb((unsigned char)(data >> 8), HDAT1);
		pr_debug("smbWordOp WRITE (0x%02x - count %05d): 0x%04x\n", cmd, count, data);
	}

	outb(BRA_START + DataFormat_Local, HCNT);


	for(count = 0; (count <= SMBTIMEOUT) && ((inb(HSTS)&BRA_HOST_BUSY)); count++)
	{
		outb(0x00, 0xEB);
		outb(0x01, 0xEB);
	}

	if (count > SMBTIMEOUT)
	{
		outb(0xFF, HSTS);
		pr_debug("smbWordOp SMBTIMEOUT_1\n");
		release_region(m_SMBus_Base_Address, 7);
		release_region(0xEB, 1);
		return 4;
	}

	ret = inb(HSTS);
	if (ret & (BRA_HSTS_ERR_MASK))
	{
		outb(0xFF, HSTS);
		pr_debug("smbWordOp HSTS(0x%02X): 0x%X\n", cmd, ret);
		release_region(m_SMBus_Base_Address, 7);
		release_region(0xEB, 1);
		return 5;
	}

	if (operation == SMBUS_READ)
	{
		*result = ((inb(HDAT0) & 0xFF) + ((inb(HDAT1) & 0xFF) << 8));
		pr_debug("smbWordOp READ (0x%02x - count %05d): 0x%04x\n", cmd, count, *result);
	}

	outb(0xFF, HSTS);

	release_region(m_SMBus_Base_Address, 7);
	release_region(0xEB, 1);
	return 0;
}

//TODO cec implementation
static int secocec_adap_enable(struct cec_adapter *adap, bool enable)
{
	struct secocec_data *cec = adap->priv;
	struct device *dev = cec->dev;
	int status;
	unsigned short result, ReadReg = 0;

	if (enable) {
		/* Clear the status register */
		status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, STATUS_REGISTER_1, 0,
				   SMBUS_READ, &ReadReg);
		if (status != 0)
			goto err;

		status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, STATUS_REGISTER_1,
				   ReadReg, SMBUS_WRITE, &result);
		if (status != 0)
			goto err;

		/* Enable the interrupts */
		status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, ENABLE_REGISTER_1, 0,
				   SMBUS_READ, &ReadReg);
		if (status != 0)
			goto err;

		status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS,
				   ENABLE_REGISTER_1,
				   ReadReg | ENABLE_REGISTER_1_CEC,
				   SMBUS_WRITE, &result);
		if (status != 0)
			goto err;

		dev_dbg(dev, "Device enabled");

	} else {
		/* Clear the status register */
		status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, STATUS_REGISTER_1, 0,
				   SMBUS_READ, &ReadReg);
		if (status != 0)
			goto err;

		status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, STATUS_REGISTER_1,
				   ReadReg, SMBUS_WRITE, &result);
		if (status != 0)
			goto err;

		/* Disable the interrupts */
		status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, ENABLE_REGISTER_1, 0,
				   SMBUS_READ, &ReadReg);
		if (status != 0)
			goto err;

		status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS,
				   ENABLE_REGISTER_1, ReadReg &
				   ~ENABLE_REGISTER_1_CEC &
				   ~ENABLE_REGISTER_1_IRDA_RC5, SMBUS_WRITE,
				   &result);
		if (status != 0)
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
	unsigned short result, reg, ReadReg = 0;

	if (logical_addr != CEC_LOG_ADDR_INVALID){
		reg = logical_addr;
	} else {
		dev_dbg(dev, "Invalid addr, resetting address");
		reg = 0xF;
	}

	status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, ENABLE_REGISTER_1, 0,
			   SMBUS_READ, &ReadReg);
	if (status != 0)
		goto err;

	dev_dbg(dev, "Set logical address: Disabling device");
	status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, ENABLE_REGISTER_1,
			   ReadReg & ~ENABLE_REGISTER_1_CEC , SMBUS_WRITE,
			   &result);
	if (status != 0)
		goto err;

	// Write logical address
	dev_dbg(dev, "Set logical address to %02x", reg);
	status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, CEC_Device_LA,
			   reg, SMBUS_WRITE, &result);
	if (status != 0)
		goto err;


	dev_dbg(dev, "Set logical address: Re-enabling device");
	status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, ENABLE_REGISTER_1,
			   ReadReg | ENABLE_REGISTER_1_CEC, SMBUS_WRITE,
			   &result);
	if (status != 0)
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
	unsigned short result;
	unsigned short reg;
	unsigned short payload_len, payload_id_len, destination = 0;
	u8 i;
	u8 * payload_msg;

	dev_dbg(dev, "Sending message (len %d)", msg->len);

        for (i = 0; i < msg->len; i++) {
		pr_debug("\t byte %d : 0x%02x\n", i, msg->msg[i]);
        }

	if (msg->len > 12){
		dev_warn(dev, "Trying to send a message longer than 12 bytes, cutting");
		msg->len = 12;
	}

	// Device msg len already accounts for header
	payload_id_len = msg->len - 1;

	// Send data length
	status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, CEC_WRITE_DATA_LENGTH,
			   payload_id_len, SMBUS_WRITE, &result);
	if (status != 0)
		goto err;

	// Send Operation ID if present
	if (payload_id_len > 0) {
		status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS,
				   CEC_WRITE_OPERATION_ID, msg->msg[1],
				   SMBUS_WRITE, &result);
		if (status != 0)
			goto err;
	}

	// Send data if present
	if (payload_id_len > 1)
	{
		// Only data;
		payload_len = msg->len - 2;
		payload_msg = &msg->msg[2];

		// Copy message into registers
		for (i = 0; i < payload_len / 2 + payload_len % 2; i++) {

			// hi byte
			reg = payload_msg[ (i<<1)+1 ] << 8;

			// lo
			reg |= payload_msg[ (i<<1) ];

			status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS,
					   CEC_WRITE_DATA_00 + i, reg,
					   SMBUS_WRITE, &result);
			if (status != 0)
				goto err;
		}
	}

	// Send msg source/destination and fire msg
	destination = msg->msg[0];
	status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, CEC_WRITE_BYTE0,
			   destination, SMBUS_WRITE, &result);
	if (status != 0)
		goto err;

	return 0;

err:
	dev_err(dev, "Transmit failed (%d)", status);
	return status;
}

static int secocec_tx_done(struct cec_adapter *adap, unsigned short StatusReg)
{
	struct secocec_data *cec = adap->priv;
	struct device *dev = cec->dev;

	int status;
	unsigned short result = 0;

	if ( StatusReg & CEC_STATUS_TX_ERROR_MASK ) {

		if (StatusReg & CEC_STATUS_TX_LINE_ERROR) {
			cec_transmit_done(adap, CEC_TX_STATUS_ARB_LOST, 1, 0, 0, 0);
			status = -EBUSY;
			dev_warn(dev, "Transmit failed (LINE_ERR)");
		} else if (StatusReg & CEC_STATUS_TX_NACK_ERROR) {
			cec_transmit_done(adap, CEC_TX_STATUS_NACK, 0, 1, 0, 0);
			status = -EAGAIN;
			dev_dbg(dev, "Transmit not acknowledged (NACK)");
		} else {
			cec_transmit_done(adap, CEC_TX_STATUS_ERROR, 0, 0, 0, 1);
			status = -EIO;
			dev_warn(dev, "Transmit failed (ERROR)");
		}

	} else {

		dev_dbg(dev, "Transmitted frame successfully");
		cec_transmit_done(adap, CEC_TX_STATUS_OK, 0, 0, 0, 0);
		status = 0;
	}

	/* Reset status reg */
	StatusReg = CEC_STATUS_TX_ERROR_MASK | CEC_STATUS_MSG_SENT_MASK |
		CEC_STATUS_TX_NACK_ERROR | CEC_STATUS_TX_LINE_ERROR ;
	smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, CEC_STATUS,
			   StatusReg, SMBUS_WRITE, &result);

	return status;
}

static int secocec_rx_done(struct cec_adapter *adap, unsigned short StatusReg)
{
	struct secocec_data *cec = adap->priv;
	struct device *dev = cec->dev;
	struct cec_msg msg = {};
	u8 i;
	u8 payload_len, payload_id_len = 0;
	u8 * payload_msg;

	int status;
	unsigned short result, ReadReg = 0;

	if ( StatusReg & CEC_STATUS_RX_ERROR_MASK ) {
		dev_warn(dev, "Message received with errors. Discarding");
		status = -EIO;
		goto rxerr;
	}

	/* Read message length */
	status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, CEC_READ_DATA_LENGTH,
			   0, SMBUS_READ, &ReadReg);
	if (status != 0)
		goto err;

	payload_id_len = ReadReg;
	dev_dbg(dev, "Incoming message (payload len %d):", payload_id_len);

	if (payload_id_len > 11) {
		payload_id_len = 11;
		dev_warn(dev, "Message received longer than 12 bytes, cutting");
	}

	/* Device msg len already accounts for the header */
	msg.len = payload_id_len + 1;

	/* Read logical address */
	status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS,
			   CEC_READ_BYTE0, 0, SMBUS_READ,
			   &ReadReg);
	if (status != 0)
		goto err;

	/* device stores source LA and destination */
	msg.msg[0] = ReadReg ;

	/* Read operation ID if present */
	if (payload_id_len > 0) {
		status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS,
				   CEC_READ_OPERATION_ID, 0, SMBUS_READ,
				   &ReadReg);
		if (status != 0)
			goto err;

		msg.msg[1] = ReadReg;
	}

	/* Read data if present */
	if (payload_id_len > 1) {
		payload_len = msg.len - 2;
		payload_msg = &msg.msg[2];

		/* device stores 2 bytes in every 16bit register */
		for (i = 0 ; i < payload_len / 2 + payload_len % 2; i++){
			status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS,
					   CEC_READ_DATA_00 + i, 0, SMBUS_READ,
					   &ReadReg);
			if (status != 0)
				goto err;

			/* low byte, skipping header */
			payload_msg[ (i<<1) ] = ReadReg & 0x00FF ;

			/* hi byte */
			payload_msg[ (i<<1)+1 ] = ( ReadReg & 0xFF00 ) >> 8 ;
		}
	}

        for (i = 0; i < msg.len; i++) {
		pr_debug("\t byte %d : 0x%02x\n", i, msg.msg[i]);
        }

	cec_received_msg(cec->cec_adap, &msg);

	/* Reset status reg */
	StatusReg = CEC_STATUS_MSG_RECEIVED_MASK;
	status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, CEC_STATUS,
			   StatusReg, SMBUS_WRITE, &result);
	if (status != 0)
		goto err;

	dev_dbg(dev, "Message received successfully");

	return 0;

rxerr:
	/* Reset error reg */
	StatusReg = CEC_STATUS_MSG_RECEIVED_MASK | CEC_STATUS_RX_ERROR_MASK;
	smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, CEC_STATUS,
			   StatusReg, SMBUS_WRITE, &result);

err:
	dev_err(dev, "Receive message failed (%d)", status);
	return -EIO;
}

struct cec_adap_ops secocec_cec_adap_ops = {
	/* Low-level callbacks */
	.adap_enable = secocec_adap_enable,
//	.adap_monitor_all_enable = secocec_adap_monitor_all_enable,
	.adap_log_addr = secocec_adap_log_addr,
	.adap_transmit = secocec_adap_transmit,
//	.adap_status = secocec_adap_status,

	/* High-level callbacks */
//	.received = secocec_received,
};

static irqreturn_t secocec_irq_handler(int irq, void *priv)
{
	//TODO irq handler
	struct secocec_data *cec = priv;
	struct device *dev = cec->dev;

	int status;
	unsigned short result, ReadReg, StatusReg = 0;

	/*  Read status register */
	status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, STATUS_REGISTER_1, 0,
			   SMBUS_READ, &ReadReg);
	if (status != 0)
		goto err;

	if (ReadReg & STATUS_REGISTER_1_CEC){
		dev_dbg(dev, "+++++ CEC Interrupt Catched");

		/* Read CEC status register */
		status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, CEC_STATUS, 0,
				   SMBUS_READ, &StatusReg);
		if (status != 0)
			goto err;

		if (StatusReg & CEC_STATUS_MSG_RECEIVED_MASK)
			secocec_rx_done(cec->cec_adap, StatusReg);

		if (StatusReg & CEC_STATUS_MSG_SENT_MASK)
			secocec_tx_done(cec->cec_adap, StatusReg);

		if ( (~StatusReg & CEC_STATUS_MSG_SENT_MASK) &&
		     (~StatusReg & CEC_STATUS_MSG_RECEIVED_MASK) )
			dev_warn(dev, "Message not received or sent, but interrupt fired \\_\"._/");

	}

	if (ReadReg & STATUS_REGISTER_1_IRDA_RC5){
		dev_dbg(dev, "IRDA RC5 Interrupt Catched");
		//TODO IRDA RX
	}

	/*  Reset status register */
	status = smbWordOp(CMD_WORD_DATA, MICRO_ADDRESS, STATUS_REGISTER_1,
			   ReadReg, SMBUS_WRITE, &result);
	if (status != 0)
		goto err;

	dev_dbg(dev, "----- CEC Interrupt Handled");

	return IRQ_HANDLED;

err:
	dev_err(dev, "IRQ: Read/Write SMBus operation failed");

	return IRQ_HANDLED;
}
static irqreturn_t secocec_irq_handler_quick(int irq, void *priv)
{
	//TODO irq handler

	return IRQ_WAKE_THREAD;

}

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
	int irq = 0;

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

	if (irq != acpi_dev_gpio_irq_get(ACPI_COMPANION(dev), 0) ){
		dev_warn(dev, "IRQ %d is not GPIO %d (%d), available %d\n",
			 irq, desc_to_gpio(gpio),
			 acpi_dev_gpio_irq_get(ACPI_COMPANION(dev), 0), platform_irq_count(pdev));
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
			       secocec_irq_handler_quick,
			       secocec_irq_handler,
			       // Use the custom kernel param to set interrupt type
			       IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			       // Used in /proc/interrupts to identify the owner
			       dev_name(&pdev->dev),
			       // The *dev_id for shared interrupt lines
			       secocec);

	if (ret < 0) {
		dev_err(dev, "Cannot request IRQ %d", secocec->irq);
		ret = -EIO;
		goto err;
	}

	//allocate cec
	opts = CEC_CAP_TRANSMIT | CEC_CAP_PHYS_ADDR |
	    CEC_CAP_LOG_ADDRS | CEC_CAP_PASSTHROUGH | CEC_CAP_RC;

	secocec->cec_adap = cec_allocate_adapter(&secocec_cec_adap_ops,
						 secocec,
						 dev_name(dev),
						 opts, SECOCEC_MAX_ADDRS);
	ret = PTR_ERR_OR_ZERO(secocec->cec_adap);
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
