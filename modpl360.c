/*
Copyright (c) 2020 Eug Krashtan

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

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
	struct pl360_local *lp = file->private;

	seq_printf(file, "TX SUCCESS:\t\t%8llu\n", lp->trac.tx_success);
	seq_printf(file, "RX SUCCESS:\t\t%8llu\n", lp->trac.rx_success);
	seq_printf(file, "INVALID:\t\t%8llu\n", lp->trac.invalid);
	return 0;
}

static int pl360_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, pl360_stats_show, inode->i_private);
}

static const struct file_operations pl360_stats_fops = {
	.open		= pl360_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int pl360_debugfs_init(struct pl360_local *lp)
{
	char debugfs_dir_name[DNAME_INLINE_LEN + 1] = "pl360-";
	struct dentry *stats;

	strncat(debugfs_dir_name, dev_name(&lp->spi->dev), DNAME_INLINE_LEN);

	lp->debugfs_root = debugfs_create_dir(debugfs_dir_name, NULL);
	if (!lp->debugfs_root)
		return -ENOMEM;

	stats = debugfs_create_file("stats", 0444,
				    lp->debugfs_root, lp,
				    &pl360_stats_fops);
	if (!stats)
		return -ENOMEM;
	return 0;
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

	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM |
			IEEE802154_HW_RX_OMIT_CKSUM |
		    /*IEEE802154_HW_FRAME_RETRIES |*/
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

	spi_set_drvdata(spi, lp);
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

	pl360_debugfs_init(lp);

	dev_info(&spi->dev, "mac802154 IRQ-%d registered\n", spi->irq);

	return ret;

err_hw_init:
	mutex_destroy(&lp->bmux);
	ieee802154_free_hw(lp->hw);

	return ret;
}

static int pl360_remove(struct spi_device *spi)
{
	struct pl360_local *lp = spi_get_drvdata(spi);

	dev_info(&spi->dev,"remove pl360\n");

	debugfs_remove_recursive(lp->debugfs_root);

	ieee802154_unregister_hw(lp->hw);
	mutex_destroy(&lp->bmux);
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
