
#ifndef __modpl360_H
#define __modpl360_H

#include <linux/module.h> 
#include <linux/kernel.h> 
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/ieee802154.h>
#include <net/mac802154.h>
#include <net/cfg802154.h>

/** SPI Header size. */
#define PDC_SPI_HEADER_SIZE				4
/** SPI Max Msg_Data size. */
#define PDC_SPI_MSG_DATA_SIZE			256
/** SPI Max Msg_Data size. */
#define PDC_SPI_MSG_PARAMS_SIZE			118   /* Worst case = 118: sizeof(rx_msg_t) [G3] */
/** PDC buffer us_size. */
#define PDC_PLC_BUFFER_SIZE			(PDC_SPI_HEADER_SIZE + PDC_SPI_MSG_DATA_SIZE + PDC_SPI_MSG_PARAMS_SIZE)
/** PDC buffer us_size to firmware update process */
#define PDC_SPI_FUP_BUFFER_SIZE        256
#define MAX_PLC_PKT_LEN 160

#define PLC_CMD_READ							0
#define PLC_CMD_WRITE							1

#define PLC_WR_RD_POS							15
#define PLC_LEN_MASK							0x7FFF
#define PL360_REG_CMD_WR                        (1 << 10)
#define PL360_REG_LEN_MASK                      0x1FF
#define PL360_REG_ID_MASK                       0xF000
#define PL360_REG_OFFSET_MASK                   0x0FFF

/* -------- Register Definition -------- */
#define ATPL360_MISCR                               (0x400E1800U) /**< \brief (MISC) Miscelaneous Register */
#define ATPL360_RSTR                                (0x400E1804U) /**< \brief (RSTR) Reset Register */
#define ATPL360_SR                                  (0x400E1808U) /**< \brief (SR) Status Register */

/* -------- ATPL360_MISCR : Miscelaneous Register -------- */
#define ATPL360_MISCR_CPUWAIT                       (0x1u << 0) /**< \brief (ATPL360_MISCR) Cortex M7 Hold */
#define ATPL360_MISCR_PPM_CALIB_ON                  (0x1u << 8) /**< \brief (ATPL360_MISCR) PPM Calibration On */
#define ATPL360_MISCR_PPM_CALIB_OFF                 (0x0u << 8) /**< \brief (ATPL360_MISCR) PPM Calibration Off */
#define ATPL360_MISCR_MEM_128_64_CFG                (0x0u << 16) /**< \brief (ATPL360_MISCR) Memory configuration: 128kB ITCM - 64kB DTCM */
#define ATPL360_MISCR_MEM_96_96_CFG                 (0x1u << 16) /**< \brief (ATPL360_MISCR) Memory configuration: 96kB ITCM - 96kB DTCM */
#define ATPL360_MISCR_EN_ACCESS_ERROR               (0x1u << 24) /**< \brief (ATPL360_MISCR) Access Errors from CM7 enable */
#define ATPL360_MISCR_SET_GPIO_12_ZC                (0x0u << 25) /**< \brief (ATPL360_MISCR) Change GPIO ZeroCross: ZC by GPIO_12 */
#define ATPL360_MISCR_SET_GPIO_2_ZC                 (0x1u << 25) /**< \brief (ATPL360_MISCR) Change GPIO ZeroCross: ZC by GPIO_2 */
#define ATPL360_MISCR_SIGN_FAIL                     (0x1u << 26) /**< \brief (ATPL360_MISCR) Check fail in Signature check */

struct pl360_local {
	struct spi_device *spi;
	//struct completion tx_complete;
	struct ieee802154_hw *hw;
	struct mutex bmux; /* protect SPI messages */
	struct dentry *debugfs_root;
	struct delayed_work work;
	struct workqueue_struct *wqueue;
	unsigned long flags;
	int tx_stat;
	bool promiscuous;
	s8 rssi;
	u8 max_frame_retries;
	u8 max_cca_retries;
	u8 max_be;
	u8 min_be;

	int gpio_nrst;
	int gpio_irq;
	int gpio_ldo;
	int gpio_cs;
	int irq_id;

	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;
	u8 buf_tx[PDC_PLC_BUFFER_SIZE];
	u8 buf_rx[PDC_PLC_BUFFER_SIZE];
};

// Ops
int ops_pl360_start(struct ieee802154_hw *hw);
void ops_pl360_stop(struct ieee802154_hw *hw);
int ops_pl360_ed(struct ieee802154_hw *hw, u8 *level);
int ops_pl360_set_txpower(struct ieee802154_hw *hw, int mbm);
int ops_pl360_set_channel(struct ieee802154_hw  *hw,	u8 page, u8 channel);
int ops_pl360_xmit(struct ieee802154_hw *hw, struct sk_buff *skb);
int ops_pl360_set_promiscuous_mode(struct ieee802154_hw *hw, bool on);
int ops_pl360_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm);
irqreturn_t pl360_isr(int irq, void *data);
// Hardware
int pl360_hw_init(struct pl360_local *data);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eug Krashtan <eug.krashtan@gmail.com>");
MODULE_DESCRIPTION("PL360 IEEE802.15.4 Transceiver Driver");

#endif /* __modpl360_H */