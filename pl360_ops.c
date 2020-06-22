
#include "modpl360.h"

/* ! \name G3 Modulation types */
enum mod_types {
	MOD_TYPE_BPSK = 0,
	MOD_TYPE_QPSK = 1,
	MOD_TYPE_8PSK = 2,
	MOD_TYPE_QAM = 3,
	MOD_TYPE_BPSK_ROBO = 4,
};

/* ! \name G3 Modulation schemes */
enum mod_schemes {
	MOD_SCHEME_DIFFERENTIAL = 0,
	MOD_SCHEME_COHERENT = 1,
};

/* ! \name G3 Frame Delimiter Types */
enum delimiter_types {
	DT_SOF_NO_RESP = 0,
	DT_SOF_RESP = 1,
	DT_ACK = 2,
	DT_NACK = 3,
};

/* ! Internal Memory Map */
typedef enum atpl360_mem_id {
	ATPL360_STATUS_INFO_ID = 0,
	ATPL360_TX_PARAM_ID,
	ATPL360_TX_DATA_ID,
	ATPL360_TX_CFM_ID,
	ATPL360_RX_PARAM_ID,
	ATPL360_RX_DATA_ID,
	ATPL360_REG_INFO_ID,
	ATPL360_IDS,
} pl360_mem_id_t;

/* Defines relatives to configuration parameter (G3) */
typedef enum atpl360_reg_id {
	ATPL360_REG_PRODID = 0x4000,
	ATPL360_REG_MODEL,
	ATPL360_REG_VERSION_STR,
	ATPL360_REG_VERSION_NUM,
	ATPL360_REG_TONE_MASK,
	ATPL360_REG_TONE_MAP_RSP_DATA,
	ATPL360_REG_TX_TOTAL,
	ATPL360_REG_TX_TOTAL_BYTES,
	ATPL360_REG_TX_TOTAL_ERRORS,
	ATPL360_REG_TX_BAD_BUSY_TX,
	ATPL360_REG_TX_BAD_BUSY_CHANNEL,
	ATPL360_REG_TX_BAD_LEN,
	ATPL360_REG_TX_BAD_FORMAT,
	ATPL360_REG_TX_TIMEOUT,
	ATPL360_REG_RX_TOTAL,
	ATPL360_REG_RX_TOTAL_BYTES,
	ATPL360_REG_RX_RS_ERRORS,
	ATPL360_REG_RX_EXCEPTIONS,
	ATPL360_REG_RX_BAD_LEN,
	ATPL360_REG_RX_BAD_CRC_FCH,
	ATPL360_REG_RX_FALSE_POSITIVE,
	ATPL360_REG_RX_BAD_FORMAT,
	ATPL360_REG_ENABLE_AUTO_NOISE_CAPTURE,
	ATPL360_REG_TIME_BETWEEN_NOISE_CAPTURES,
	ATPL360_REG_DELAY_NOISE_CAPTURE_AFTER_RX,
	ATPL360_REG_RRC_NOTCH_ACTIVE,
	ATPL360_REG_RRC_NOTCH_INDEX,
	ATPL360_REG_NOISE_PEAK_POWER,
	ATPL360_REG_RSV0,
	ATPL360_REG_RSV1,
	ATPL360_REG_CFG_AUTODETECT_IMPEDANCE,
	ATPL360_REG_CFG_IMPEDANCE,
	ATPL360_REG_ZC_PERIOD,
	ATPL360_REG_FCH_SYMBOLS,
	ATPL360_REG_PAY_SYMBOLS_TX,
	ATPL360_REG_PAY_SYMBOLS_RX,
	ATPL360_REG_RRC_NOTCH_AUTODETECT,
	ATPL360_REG_MAX_RMS_TABLE_HI,
	ATPL360_REG_MAX_RMS_TABLE_VLO,
	ATPL360_REG_THRESHOLDS_TABLE_HI,
	ATPL360_REG_THRESHOLDS_TABLE_LO,
	ATPL360_REG_THRESHOLDS_TABLE_VLO,
	ATPL360_REG_PREDIST_COEF_TABLE_HI,
	ATPL360_REG_PREDIST_COEF_TABLE_LO,
	ATPL360_REG_PREDIST_COEF_TABLE_VLO,
	ATPL360_REG_GAIN_TABLE_HI,
	ATPL360_REG_GAIN_TABLE_LO,
	ATPL360_REG_GAIN_TABLE_VLO,
	ATPL360_REG_DACC_TABLE_CFG,
	ATPL360_REG_RSV2,
	ATPL360_REG_NUM_TX_LEVELS,
	ATPL360_REG_CORRECTED_RMS_CALC,
	ATPL360_REG_RRC_NOTCH_THR_ON,
	ATPL360_REG_RRC_NOTCH_THR_OFF,
	ATPL360_REG_CURRENT_GAIN,
	ATPL360_REG_ZC_CONF_INV,
	ATPL360_REG_ZC_CONF_FREQ,
	ATPL360_REG_ZC_CONF_DELAY,
	ATPL360_REG_NOISE_PER_CARRIER,
	ATPL360_REG_END_ID,
} pl360_reg_id_t;

/* ! Defines relatives to some ATPL360 registers */
#define ATPL360_REG_ADC_MASK                    0x1000
#define ATPL360_REG_DAC_MASK                    0x2000
#define ATPL360_REG_MASK                        0x4000
#define ATPL360_FUSES_MASK                      0x8000
#define ATPL360_REG_ADC_BASE                    0x40000000
#define ATPL360_REG_DAC_BASE                    0x40004000
#define ATPL360_REG_BASE                        0x80000000
#define ATPL360_FUSES_BASE                      0x400E1800

/* ! FLAG MASKs for set events */
#define ATPL360_TX_CFM_FLAG_MASK                 0x0001
#define ATPL360_RX_DATA_IND_FLAG_MASK            0x0002
#define ATPL360_CD_FLAG_MASK                     0x0004
#define ATPL360_REG_RSP_MASK                     0x0008
#define ATPL360_RX_QPAR_IND_FLAG_MASK            0x0010

/* ! Event Info MASKs */
#define ATPL360_EV_DAT_LEN_MASK                  0x0000FFFF
#define ATPL360_EV_REG_LEN_MASK                  0xFFFF0000
#define ATPL360_GET_EV_DAT_LEN_INFO(x)           ((uint32_t)x & ATPL360_EV_DAT_LEN_MASK)
#define ATPL360_GET_EV_REG_LEN_INFO(x)           (((uint32_t)x & ATPL360_EV_REG_LEN_MASK) >> 16)

#define PL360_IF_DEFAULT_TONE_MAP									{ 0x03, 0xFF, 0xFF }
#define PL360_IF_TX_POWER										0
/* ! TX Mode: Delayed transmission */
#define TX_MODE_RELATIVE             (1 << 1)
/* ! Maximum number of subbands */
#define NUM_SUBBANDS_MAX                       24
/* ! Maximum number of tone map */
#define TONE_MAP_SIZE_MAX                      3
/* ! Maximum number of protocol carriers */
#define PROTOCOL_CARRIERS_MAX                  72

/* Init packet data size */
#define INIT_PKT_DATA_SIZE (8)

/* ! \name G3 TX Result values */
enum tx_result_values {
	TX_RESULT_PROCESS = 0,                  /* Transmission result: already in process */
	TX_RESULT_SUCCESS = 1,                  /* Transmission result: end successfully */
	TX_RESULT_INV_LENGTH = 2,               /* Transmission result: invalid length error */
	TX_RESULT_BUSY_CH = 3,                  /* Transmission result: busy channel error */
	TX_RESULT_BUSY_TX = 4,                  /* Transmission result: busy in transmission error */
	TX_RESULT_BUSY_RX = 5,                  /* Transmission result: busy in reception error */
	TX_RESULT_INV_SCHEME = 6,               /* Transmission result: invalid modulation scheme error */
	TX_RESULT_TIMEOUT = 7,                  /* Transmission result: timeout error */
	TX_RESULT_INV_TONEMAP = 8,              /* Transmission result: invalid tone map error */
	TX_RESULT_INV_MODE = 9,                 /* Transmission result: invalid G3 Mode error */
	TX_RESULT_NO_TX = 255,                  /* Transmission result: No transmission ongoing */
};

#pragma pack(push,1)

typedef struct __attribute__((__packed__)) {
	uint32_t tx_time;
	uint16_t data_len;
	uint8_t tone_groups[NUM_SUBBANDS_MAX];
	uint8_t tone_map[TONE_MAP_SIZE_MAX];
	uint8_t tx_mode;                            /* Transmission Mode (absolute, relative, forced, continuous, cancel). Constants above */
	uint8_t tx_power;                           /* Power to transmit */
	uint8_t mod_type;                           /* Modulation type */
	uint8_t mod_scheme;                         /* Modulation scheme */
	uint8_t pdc;                                /* Phase Detector Counter */
	uint8_t rs_blocks;                          /* Flag to indicate whether 2 RS blocks have to be used (only used for FCC) */
	uint8_t uc_delimiter_type;                  /* DT field to be used in header */
} pl360_tx_config_t;

/* ! \name G3 Structure defining Rx message */
typedef struct rx_msg {
	uint32_t ul_rx_time;                           /* /< Instant when frame was received */
	uint32_t ul_frame_duration;                    /* /< Frame duration referred to 1ms PHY counter (FCH + Payload) */
	uint16_t us_rssi;                              /* /< Reception RSSI */
	uint16_t us_data_len;                          /* /< Length of the data buffer */
	uint8_t uc_zct_diff;                           /* /< ZCT info */
	uint8_t uc_rs_corrected_errors;                /* /< Errors corrected by RS */
	enum mod_types uc_mod_type;                    /* /< Modulation type of the last received message */
	enum mod_schemes uc_mod_scheme;                /* /< Modulation scheme of the last received message */
	uint32_t ul_agc_factor;                        /* /< Test data information */
	uint16_t us_agc_fine;                          /* /< Test data information */
	int16_t ss_agc_offset_meas;                    /* /< Test data information */
	uint8_t uc_agc_active;                         /* /< Test data information */
	uint8_t uc_agc_pga_value;                      /* /< Test data information */
	int16_t ss_snr_fch;                            /* /< Test data information */
	int16_t ss_snr_pay;                            /* /< Test data information */
	uint16_t us_payload_corrupted_carriers;        /* /< BER: Number of corrupted carriers */
	uint16_t us_payload_noised_symbols;            /* /< BER: Number of noised symbols */
	uint8_t uc_payload_snr_worst_carrier;          /* /< BER: SNR of the worst carrier */
	uint8_t uc_payload_snr_worst_symbol;           /* /< BER: SNR of the worst symbol */
	uint8_t uc_payload_snr_impulsive;              /* /< BER: SNR on impulsive noise */
	uint8_t uc_payload_snr_band;                   /* /< BER: Narrowband SNR */
	uint8_t uc_payload_snr_background;             /* /< BER: Background SNR */
	uint8_t uc_lqi;                                /* /< BER: Link Quality */
	enum delimiter_types uc_delimiter_type;        /* /< DT field coming in header */
	uint8_t uc_rsrv0;                              /* /< MAC CRC. 1: OK; 0: NOK (CRC capability can be enabled/disabled). 16 bits for allignement  */
	uint8_t puc_tone_map[TONE_MAP_SIZE_MAX];       /* /< Reception Tone Map */
	uint8_t puc_carrier_snr[PROTOCOL_CARRIERS_MAX]; /* /< SNR per carrier */
	uint8_t uc_rsrv1;                              /* /< Reserved byte */
	uint8_t *puc_data_buf;                         /* /< Pointer to data buffer */
} rx_msg_t;

/* ! \name G3 Structure defining result of a transmission */
typedef struct tx_cfm {
	uint32_t ul_rms_calc;                          /* RMS_CALC it allows to estimate tx power injected */
	uint32_t ul_tx_time;                           /* Instant when frame transmission ended referred to 1ms PHY counter */
	enum tx_result_values uc_tx_result;            /* Tx Result (see "TX Result values" above) */
} tx_cfm_t;

typedef struct __attribute__((__packed__)) {
	uint32_t time;
	uint32_t evt;
} status_t;

#pragma pack(pop)

#define ATPL360_CMF_PKT_SIZE                      sizeof(tx_cfm_t)

static plc_pkt_t* txpkt; // Packet queued to transmit
static status_t status;
static pl360_tx_config_t txconf;

typedef struct {
    uint16_t len;
    uint16_t addr;
    uint8_t buf[8];
} plc_pkt8_t;

static void pl360_update_status(struct pl360_local *lp) {
	plc_pkt8_t pkt = {
		.addr = ATPL360_STATUS_INFO_ID,
		.len = INIT_PKT_DATA_SIZE,
	};
	pl360_datapkt(lp, PLC_CMD_READ, (plc_pkt_t*)&pkt);
	memcpy(&status, pkt.buf, sizeof(status));
}

static int pl360_rx(struct pl360_local *lp, plc_pkt_t* pkt) {
	struct sk_buff *skb;

	if (!ieee802154_is_valid_psdu_len(pkt->len)) {
		dev_dbg(&lp->spi->dev,
			"corrupted frame received len %d\n", (int)pkt->len);
		pkt->len = IEEE802154_MTU;
	}
	skb = dev_alloc_skb(pkt->len + 2);
	if (!skb) {
		return -ENOMEM;
	}
	memcpy(skb_put(skb, pkt->len),pkt->buf,pkt->len);
	ieee802154_rx_irqsafe(lp->hw, skb, 1); // LQI ToDo!!!
	return 0;
}

static void pl360_handle_rx_work(struct work_struct *work)
{
	struct pl360_local *lp =
		container_of(work, struct pl360_local, rxwork);

	mutex_lock(&lp->plmux);
	do {
		pl360_update_status(lp);
		if (lp->events & ATPL360_TX_CFM_FLAG_MASK) {
			// Handle TX confirm
			plc_pkt_t* cfmpkt = kmalloc(sizeof(plc_pkt_t)
				+ ATPL360_CMF_PKT_SIZE, GFP_KERNEL);
			cfmpkt->addr = ATPL360_TX_CFM_ID;
			cfmpkt->len = ATPL360_CMF_PKT_SIZE;
			pl360_datapkt(lp, PLC_CMD_READ, cfmpkt);
			kfree(cfmpkt);
			if(txpkt) {
				kfree(txpkt);
				txpkt = NULL;
			}
		} else if (lp->events & ATPL360_REG_RSP_MASK) {
			// Handle RegResp
			plc_pkt_t* rsppkt;
			uint16_t evt_len = ATPL360_GET_EV_REG_LEN_INFO(status.evt);
			if ((evt_len == 0) || (evt_len > MAX_PLC_PKT_LEN)) {
				evt_len = 1;
			}
			rsppkt = (plc_pkt_t*)kmalloc(evt_len
				+ sizeof(plc_pkt_t), GFP_KERNEL);
			rsppkt->addr = ATPL360_REG_INFO_ID;
			rsppkt->len = evt_len;
			pl360_datapkt(lp, PLC_CMD_READ, rsppkt);
			kfree(rsppkt);
		} else if (lp->events & ATPL360_RX_QPAR_IND_FLAG_MASK) {
			plc_pkt_t* qpkt;
			// Handle RX qpar
			qpkt = (plc_pkt_t*)kmalloc(sizeof(rx_msg_t)
				- 4 + sizeof(plc_pkt_t), GFP_KERNEL);
			qpkt->addr = ATPL360_RX_PARAM_ID;
			qpkt->len = sizeof(rx_msg_t) - 4;
			pl360_datapkt(lp, PLC_CMD_READ, qpkt);
			rx_msg_t* rxq = (rx_msg_t *)qpkt->buf;
			lp->rssi = rxq->us_rssi;
			kfree(qpkt);
		} else if (lp->events & ATPL360_RX_DATA_IND_FLAG_MASK) {
			plc_pkt_t* rxpkt;
			// Handle RX data, data length (15 bits) 
			uint16_t l = status.evt & 0x7F;
			rxpkt = (plc_pkt_t*)kmalloc(l + sizeof(plc_pkt_t), 
				GFP_KERNEL);
			rxpkt->addr = ATPL360_RX_DATA_ID;
			rxpkt->len = l;
			pl360_datapkt(lp, PLC_CMD_READ, rxpkt);
			pl360_rx(lp, rxpkt);
			kfree(rxpkt);
		}
	} while (lp->events);
	mutex_unlock(&lp->plmux);
}

static void pl360_handle_tx_work(struct work_struct *work)
{
	plc_pkt_t* parampkt;
	struct pl360_local *lp =
		container_of(work, struct pl360_local, txwork);

	if(lp->events) {
		queue_work(lp->wqueue, &lp->rxwork);
		queue_work(lp->wqueue, &lp->txwork);
	} else {
		if (txpkt) {
			txconf.data_len = txpkt->len;
			// ToDo: change to static copy
			parampkt = (plc_pkt_t*)kmalloc(sizeof(pl360_tx_config_t)
				+ sizeof(plc_pkt_t), GFP_KERNEL);
			parampkt->addr = ATPL360_TX_PARAM_ID;
			parampkt->len = sizeof(pl360_tx_config_t);
			memcpy(parampkt->buf,&txconf,sizeof(pl360_tx_config_t));
			mutex_lock(&lp->plmux);
			pl360_datapkt(lp, PLC_CMD_WRITE, parampkt);
			pl360_datapkt(lp, PLC_CMD_WRITE, txpkt);
			mutex_unlock(&lp->plmux);
			kfree(parampkt);
		}
	}
}

static uint32_t access_type(uint16_t param_id)
{
	uint32_t address = 0;

	if (param_id & ATPL360_REG_ADC_MASK) {
		address = (uint32_t)ATPL360_REG_ADC_BASE;
	} else if (param_id & ATPL360_REG_DAC_MASK) {
		address = (uint32_t)ATPL360_REG_DAC_BASE;
	} else if (param_id & ATPL360_FUSES_MASK) {
		address = (uint32_t)ATPL360_FUSES_BASE;
	} else if ((param_id & ATPL360_REG_MASK) && (param_id < ATPL360_REG_END_ID)) {
		address = (uint32_t)ATPL360_REG_BASE;
	}

	return address;
}

static void pl360_set_config(plc_pkt_t* pkt, uint16_t param_id, uint16_t value) {
	uint32_t reg_addr, reg_len;

	reg_addr = (uint32_t)(param_id & PL360_REG_OFFSET_MASK) + access_type(param_id);
	reg_len = PL360_REG_CMD_WR | (1 & PL360_REG_LEN_MASK);
	pkt->buf[0] = (uint8_t)(reg_addr >> 24);
	pkt->buf[1] = (uint8_t)(reg_addr >> 16);
	pkt->buf[2] = (uint8_t)(reg_addr >> 8);
	pkt->buf[3] = (uint8_t)(reg_addr);
	pkt->buf[4] = (uint8_t)(reg_len >> 8);
	pkt->buf[5] = (uint8_t)(reg_len);
	pkt->buf[6] = value&0xFF;
	pkt->buf[7] = value>>8;

	pkt->addr = ATPL360_REG_INFO_ID;
	pkt->len = 8;
}

void pl360_conrigure(struct pl360_local *lp) {
	plc_pkt_t* pkt = kmalloc(INIT_PKT_DATA_SIZE+sizeof(plc_pkt_t), GFP_KERNEL);
	/* Read Time Ref to get SPI status and boot if necessary */
	pkt->addr = ATPL360_STATUS_INFO_ID;
	pkt->len = INIT_PKT_DATA_SIZE;
	pl360_datapkt(lp, PLC_CMD_READ, pkt);

	// Restart IRQ after boot
	pl360_datapkt(lp, PLC_CMD_READ, pkt);

	/* Disable AUTO mode and set VLO behavior by default in order to maximize signal level in anycase */
	pl360_set_config(pkt, ATPL360_REG_CFG_AUTODETECT_IMPEDANCE, 0);
	pl360_datapkt(lp, PLC_CMD_WRITE, pkt);
	msleep(1);
	pl360_set_config(pkt, ATPL360_REG_CFG_IMPEDANCE, 2);
	pl360_datapkt(lp, PLC_CMD_WRITE, pkt);
	msleep(1);
    kfree(pkt);
}

int ops_pl360_start(struct ieee802154_hw *hw)
{
	int ret = 0;
	struct pl360_local *lp = hw->priv;

	INIT_WORK(&lp->rxwork, pl360_handle_rx_work);
	INIT_WORK(&lp->txwork, pl360_handle_tx_work);
	lp->wqueue = create_singlethread_workqueue(dev_name(&lp->spi->dev));
	if (unlikely(!lp->wqueue)) {
		ret = -ENOMEM;
		goto err_ops_start;
	}

	/* Prepare default TX config */
	const uint8_t tonemap[] = PL360_IF_DEFAULT_TONE_MAP;
	txconf.tx_time = 0;
	memcpy(txconf.tone_map, tonemap, sizeof(tonemap));

	txconf.tx_mode =  TX_MODE_RELATIVE; // uc_tx_mode
	txconf.tx_time = 0x3E8;
	txconf.tx_power =  PL360_IF_TX_POWER; // uc_tx_power

	txconf.mod_type =  MOD_TYPE_BPSK; // uc_mod_type
	txconf.mod_scheme =  MOD_SCHEME_DIFFERENTIAL; // uc_mod_scheme
	txconf.uc_delimiter_type =  DT_SOF_NO_RESP; // uc_delimiter_type

	ret = request_irq(lp->irq_id, pl360_isr,
		IRQF_TRIGGER_FALLING, "pl360-irq", spi_get_drvdata(lp->spi)	);
	if(ret) {
		printk( KERN_INFO "ISR handler req fail %d\n", ret );
	}

	/* Update RX status */
	queue_work(lp->wqueue, &lp->rxwork);

err_ops_start:
	return ret;
}

int ops_pl360_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct pl360_local *lp = hw->priv;

	*level = lp->rssi;

	dev_vdbg(&lp->spi->dev, "%s level=%d\n",
		 __func__, *level);

	return 0;
}

void ops_pl360_stop(struct ieee802154_hw *hw)
{
	struct pl360_local *lp = hw->priv;

	disable_irq(lp->spi->irq);
	flush_workqueue(lp->wqueue);
	destroy_workqueue(lp->wqueue);
}

int ops_pl360_xmit(struct ieee802154_hw *hw, struct sk_buff *skb) {
	//printk("pl360 xmit %d \n", skb->len);
	plc_pkt_t* pkt;
	struct pl360_local *lp = hw->priv;
	while(txpkt) {
		queue_work(lp->wqueue, &lp->rxwork);
		msleep(5);
	}

    pkt = (plc_pkt_t*) kmalloc(sizeof(plc_pkt_t)
		+ skb->len, GFP_KERNEL);
    memcpy(pkt->buf, skb->data, skb->len);
    pkt->len = skb->len;
    pkt->addr = ATPL360_TX_DATA_ID;

    txpkt = pkt;
	queue_work(lp->wqueue, &lp->txwork);
    return 0;
}

int ops_pl360_set_frame_retries(struct ieee802154_hw *hw, s8 retries) {
	//printk("pl360 set retries %d\n", retries);
	return 0;
}

int ops_pl360_set_csma_params(struct ieee802154_hw *hw, u8 min_be,
				   u8 max_be, u8 retries) {
	//printk("pl360 set csma_params %d %d %d\n", min_be, max_be, retries);
	return 0;
}

int ops_pl360_set_hw_addr_filt(struct ieee802154_hw *hw,
				    struct ieee802154_hw_addr_filt *filt,
				    unsigned long changed) {
	return 0;
}

int ops_pl360_set_channel(struct ieee802154_hw  *hw,
	u8 page, u8 channel) {
	//printk("pl360 set ch %d\n", channel);
    return 0;
}

int ops_pl360_set_txpower(struct ieee802154_hw *hw, int mbm) {
	//printk("pl360 set txpower %d\n", mbm);
    return 0;
}

int ops_pl360_set_promiscuous_mode(struct ieee802154_hw *hw, bool on) {
	//printk("pl360 set promiscuous %d\n", on);
    return 0;
}

int ops_pl360_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm) {
	//printk("pl360 cca ed %d\n", mbm);
    return 0;
}

irqreturn_t pl360_isr(int irq, void *data) {
	struct pl360_local *lp = data;
	queue_work(lp->wqueue, &lp->rxwork);
	return IRQ_HANDLED;
}
