
#include "modpl360.h"

int ops_pl360_start(struct ieee802154_hw *hw)
{
	struct pl360_local *lp = hw->priv;

	//adf7242_clear_irqstat(lp);
	enable_irq(lp->spi->irq);

	return 0;
}

int ops_pl360_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct pl360_local *lp = hw->priv;

	*level = lp->rssi;

	dev_vdbg(&lp->spi->dev, "%s :Exit level=%d\n",
		 __func__, *level);

	return 0;
}

void ops_pl360_stop(struct ieee802154_hw *hw)
{
	struct pl360_local *lp = hw->priv;

	disable_irq(lp->spi->irq);
	cancel_delayed_work_sync(&lp->work);
	//adf7242_clear_irqstat(lp);
}

int ops_pl360_xmit(struct ieee802154_hw *hw, struct sk_buff *skb) {
	struct pl360_local *lp = hw->priv;

    return 0;
}

int ops_pl360_set_channel(struct ieee802154_hw  *hw,
	u8 page, u8 channel) {
    return 0;
}

int ops_pl360_set_txpower(struct ieee802154_hw *hw, int mbm) {
    return 0;
}

int ops_pl360_set_promiscuous_mode(struct ieee802154_hw *hw, bool on) {
    return 0;
}

int ops_pl360_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm) {
    return 0;
}

irqreturn_t pl360_isr(int irq, void *data) {
	//k_work_submit(&handlerx);
}
