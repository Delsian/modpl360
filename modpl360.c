
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/slab.h>     /* kmalloc() */
#include "modpl360.h"

static const struct ieee802154_ops pl360_ops = {
	.owner = THIS_MODULE,
	.xmit_sync = ops_pl360_xmit,
	.ed = ops_pl360_ed,
	.set_channel = ops_pl360_set_channel,
	.set_hw_addr_filt = ops_pl360_set_hw_addr_filt,
	.start = ops_pl360_start,
	.stop = ops_pl360_stop,
	.set_csma_params = ops_pl360_set_csma_params,
	.set_frame_retries = ops_pl360_set_frame_retries,
	.set_txpower = ops_pl360_set_txpower,
	.set_promiscuous_mode = ops_pl360_set_promiscuous_mode,
	.set_cca_ed_level = ops_pl360_set_cca_ed_level,
};

static int pl360_stats_show(struct seq_file *file, void *offset)
{
	struct pl360_local *lp = spi_get_drvdata(file->private);
	//u8 stat, irq1;

	/*adf7242_status(lp, &stat);
	adf7242_read_reg(lp, REG_IRQ1_SRC1, &irq1);

	seq_printf(file, "IRQ1 = %X:\n%s%s%s%s%s%s%s%s\n", irq1,
		   irq1 & IRQ_CCA_COMPLETE ? "IRQ_CCA_COMPLETE\n" : "",
		   irq1 & IRQ_SFD_RX ? "IRQ_SFD_RX\n" : "",
		   irq1 & IRQ_SFD_TX ? "IRQ_SFD_TX\n" : "",
		   irq1 & IRQ_RX_PKT_RCVD ? "IRQ_RX_PKT_RCVD\n" : "",
		   irq1 & IRQ_TX_PKT_SENT ? "IRQ_TX_PKT_SENT\n" : "",
		   irq1 & IRQ_CSMA_CA ? "IRQ_CSMA_CA\n" : "",
		   irq1 & IRQ_FRAME_VALID ? "IRQ_FRAME_VALID\n" : "",
		   irq1 & IRQ_ADDRESS_VALID ? "IRQ_ADDRESS_VALID\n" : "");

	seq_printf(file, "STATUS = %X:\n%s\n%s\n%s\n%s\n%s%s%s%s%s\n", stat,
		   stat & STAT_SPI_READY ? "SPI_READY" : "SPI_BUSY",
		   stat & STAT_IRQ_STATUS ? "IRQ_PENDING" : "IRQ_CLEAR",
		   stat & STAT_RC_READY ? "RC_READY" : "RC_BUSY",
		   stat & STAT_CCA_RESULT ? "CHAN_IDLE" : "CHAN_BUSY",
		   (stat & 0xf) == RC_STATUS_IDLE ? "RC_STATUS_IDLE" : "",
		   (stat & 0xf) == RC_STATUS_MEAS ? "RC_STATUS_MEAS" : "",
		   (stat & 0xf) == RC_STATUS_PHY_RDY ? "RC_STATUS_PHY_RDY" : "",
		   (stat & 0xf) == RC_STATUS_RX ? "RC_STATUS_RX" : "",
		   (stat & 0xf) == RC_STATUS_TX ? "RC_STATUS_TX" : "");*/

	seq_printf(file, "RSSI = %d\n", lp->rssi);

	return 0;
}

static void pl360_debugfs_init(struct pl360_local *lp)
{
	char debugfs_dir_name[DNAME_INLINE_LEN + 1] = "pl360-";

	strncat(debugfs_dir_name, dev_name(&lp->spi->dev), DNAME_INLINE_LEN);

	lp->debugfs_root = debugfs_create_dir(debugfs_dir_name, NULL);

	debugfs_create_devm_seqfile(&lp->spi->dev, "status", lp->debugfs_root,
				    pl360_stats_show);
}

static const s32 pl360_powers[] = {
	100, 0, -100,
};
static const s32 pl360_ed_levels[] = {
	-6000, -5900, -5800, -5700, -5600, -5500, -5400, -5300, -5200, -5100,
	-5000, -4900, -4800, -4700, -4600, -4500, -4400, -4300, -4200, -4100
};

static int pl360_probe(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct pl360_local *lp;
	int ret;

	printk("Probe pl360\n");

	hw = ieee802154_alloc_hw(sizeof(*lp), &pl360_ops);
	if (!hw)
		return -ENOMEM;

	lp = hw->priv;
	lp->hw = hw;
	lp->spi = spi;

	hw->priv = lp;
	hw->parent = &spi->dev;
	hw->extra_tx_headroom = 0;

	/* ToDo - Emulate 2.4 Ghz channel */
	hw->phy->supported.channels[0] = 0x7FFF800;

	hw->flags = IEEE802154_HW_RX_OMIT_CKSUM |
		    IEEE802154_HW_FRAME_RETRIES |
		    IEEE802154_HW_PROMISCUOUS;

	hw->phy->flags = WPAN_PHY_FLAG_TXPOWER; /* |
			 WPAN_PHY_FLAG_CCA_ED_LEVEL |
			 WPAN_PHY_FLAG_CCA_MODE;*/

	hw->phy->supported.cca_modes = BIT(NL802154_CCA_ENERGY);

	hw->phy->supported.cca_ed_levels = pl360_ed_levels;
	hw->phy->supported.cca_ed_levels_size = ARRAY_SIZE(pl360_ed_levels);

	hw->phy->cca.mode = NL802154_CCA_ENERGY;

	hw->phy->supported.tx_powers = pl360_powers;
	hw->phy->supported.tx_powers_size = ARRAY_SIZE(pl360_powers);

	hw->phy->supported.min_minbe = 0;
	hw->phy->supported.max_minbe = 8;

	hw->phy->supported.min_maxbe = 3;
	hw->phy->supported.max_maxbe = 8;

	hw->phy->supported.min_frame_retries = 0;
	hw->phy->supported.max_frame_retries = 15;

	hw->phy->supported.min_csma_backoffs = 0;
	hw->phy->supported.max_csma_backoffs = 5;

	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

	mutex_init(&lp->bmux);
	mutex_init(&lp->plmux);
	//init_completion(&lp->tx_complete);

	ret = pl360_hw_init(lp);
	if (ret) {
		printk("pl360 HW err %d\n", ret);
		goto err_hw_init;
	}

	ret = ieee802154_register_hw(lp->hw);
	if (ret) {
		printk("ieee802154_register_hw err %d\n", ret);
		goto err_hw_init;
	}


	dev_set_drvdata(&spi->dev, lp);

	pl360_debugfs_init(lp);

	dev_info(&spi->dev, "mac802154 IRQ-%d registered\n", spi->irq);

	return ret;

err_hw_init:
	mutex_destroy(&lp->bmux);
	mutex_destroy(&lp->plmux);
	ieee802154_free_hw(lp->hw);

	return ret;
}

static int pl360_remove(struct spi_device *spi)
{
	struct pl360_local *lp = spi_get_drvdata(spi);

	printk("remove pl360\n");

	debugfs_remove_recursive(lp->debugfs_root);

	ieee802154_unregister_hw(lp->hw);
	mutex_destroy(&lp->bmux);
	mutex_destroy(&lp->plmux);
	ieee802154_free_hw(lp->hw);

	return 0;
}

static const struct of_device_id pl360_of_match[] = {
	{ .compatible = "atmel,pl360", },
	{ .compatible = "atmel,pl360a", },
	{ },
};
MODULE_DEVICE_TABLE(of, pl360_of_match);

static const struct spi_device_id pl360_device_id[] = {
	{ .name = "pl360", },
    { .name = "pl360a", },
	{ },
};
MODULE_DEVICE_TABLE(spi, pl360_device_id);

static struct spi_driver pl360_driver = {
	.id_table = pl360_device_id,
	.driver = {
		   .of_match_table = of_match_ptr(pl360_of_match),
		   .name = "pl360",
		   .owner = THIS_MODULE,
		   },
	.probe = pl360_probe,
	.remove = pl360_remove,
};

module_spi_driver(pl360_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eug Krashtan <eug.krashtan@gmail.com>");
MODULE_DESCRIPTION("PL360 IEEE802.15.4 Transceiver Driver");
