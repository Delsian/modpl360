
#include "modpl360.h"
#include "pl360_firmware.h"

/* ! Reset pin of transceiver */
#define PLC_RST_DELAY                (100)
/** SPI Header field when bootload is in the other side of spi*/
#define PLC_SPI_HEADER_BOOT					0x5634
/** SPI Header MASK for bootloader heade*/
#define PLC_SPI_HEADER_BOOT_MASK				0xFFFe
/** SPI Header field when atpl360 is in the other side of spi*/
#define PLC_SPI_HEADER_CORTEX					0x1022
/** Bootloader Passwords for enable writing  */
#define ATPL360_BOOT_WRITE_KEY              0x5345ACBA
/** Bootloader Address of CPUWAIT */
#define ATPL360_BOOT_CMD_ENABLE_WRITE       0xDE05
/** Bootloader Address for writing program */
#define ATPL360_BOOT_PROGRAM_ADDR           0x00000000
/** Bootloader command: Write Word (32 bits) */
#define ATPL360_BOOT_CMD_WRITE_WORD         0x0000
/** Bootloader command: Write buffer */
#define ATPL360_BOOT_CMD_WRITE_BUF          0x0001
/** Bootloader command: Disable SPI control to bootloader */
#define ATPL360_BOOT_CMD_DIS_SPI_CTRL       0xA55A

static void pl360_reset(struct pl360_local *lp) {
	int err;

    gpio_set_value(lp->gpio_ldo, 0);
	msleep(PLC_RST_DELAY);
    gpio_set_value(lp->gpio_nrst, 0);
    gpio_set_value(lp->gpio_ldo, 1);

	lp->spi_transfer.len = 1;
	err = spi_sync(lp->spi, &lp->spi_msg);
	if(err<0) {
		dev_crit(&lp->spi->dev,"SPI error %d\n", err);
	}
	msleep(10);
	gpio_set_value(lp->gpio_nrst, 1);
	msleep(50);
}






int pl360_hw_init(struct pl360_local *lp)
{
    struct spi_device *spi = lp->spi;
	dev_dbg(&spi->dev, "%s called\n", __func__);
    
	spi_message_init(&lp->spi_msg);
	lp->spi_transfer.len = 1;
	lp->spi_transfer.tx_buf = &lp->buf_tx;
	lp->spi_transfer.rx_buf = &lp->buf_rx;

	spi_message_add_tail(&lp->spi_transfer, &lp->spi_msg);

#if 0
	data->cs_ctrl.gpio_dev =
		device_get_binding(DT_INST_SPI_DEV_CS_GPIOS_LABEL(0));
	data->cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
	data->cs_ctrl.delay = 0U;
	data->spi_cfg.cs = &(data->cs_ctrl);

	data->nrst_gpio =
		device_get_binding(DT_INST_GPIO_LABEL(0, nrst_gpios));
	if (data->nrst_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for PLC reset");
		return -EPERM;
	}
	data->nrst_gpio_pin = DT_INST_GPIO_PIN(0, nrst_gpios);
	data->ldo_gpio =
		device_get_binding(DT_INST_GPIO_LABEL(0, ldo_gpios));
	if (data->ldo_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for LDO enable");
		return -EPERM;
	}
	data->ldo_gpio_pin = DT_INST_GPIO_PIN(0, ldo_gpios);

	gpio_pin_configure(data->nrst_gpio, data->nrst_gpio_pin,
			   GPIO_OUTPUT_ACTIVE |
			   DT_INST_GPIO_FLAGS(0, nrst_gpios));
	gpio_pin_configure(data->ldo_gpio, data->ldo_gpio_pin,
			   GPIO_OUTPUT_ACTIVE |
			   DT_INST_GPIO_FLAGS(0, ldo_gpios));

	pl360_reset(dev);

	// Init Interrupt pin (but not handler)
	data->int_gpio =
		device_get_binding(DT_INST_GPIO_LABEL(0, int_gpios));
	if (data->int_gpio == NULL) {
		LOG_ERR("Could not get interrupt pin for PLC");
		return -EPERM;
	}
	data->int_gpio_pin = DT_INST_GPIO_PIN(0, int_gpios);
	int err = gpio_pin_interrupt_configure(data->int_gpio,
					   data->int_gpio_pin, GPIO_INT_EDGE_TO_INACTIVE);
	if( err < 0) {
		LOG_ERR("%d: Failed to configure interrupt on pin %d\n",
			err, DT_INST_GPIO_PIN(0, int_gpios));
		return -EPERM;
	}
#endif
    return 0;
}
