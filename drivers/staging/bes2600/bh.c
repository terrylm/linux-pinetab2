/*
 * Mac80211 driver for BES2600 device
 *
 * Copyright (c) 2022, Bestechnic
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <net/mac80211.h>
#include <linux/kthread.h>
#include <uapi/linux/ip.h>
#include <uapi/linux/tcp.h>
#include <uapi/linux/udp.h>

#include "bes2600.h"
#include "bh.h"
#include "hwio.h"
#include "wsm.h"
#include "sbus.h"
#include "debug.h"
#include "epta_coex.h"
#include "bes_chardev.h"
#include "txrx_opt.h"
#include "sta.h"
#include "bes_log.h"

extern void sdio_work_debug(struct sbus_priv *self);

#define DOWNLOAD_BLOCK_SIZE_WR (0x1000 - 4)
#define MAX_SZ_RD_WR_BUFFERS (DOWNLOAD_BLOCK_SIZE_WR * 2)
#define PIGGYBACK_CTRL_REG (2)
#define EFFECTIVE_BUF_SIZE (MAX_SZ_RD_WR_BUFFERS - PIGGYBACK_CTRL_REG)

#define BH_RX_CONT_LIMIT 3
#define BH_TX_CONT_LIMIT 20
#define BH_WAIT_TIMEOUT (5 * HZ)

/* Suspend state privates */
enum bes2600_bh_pm_state {
	BES2600_BH_RESUMED = 0,
	BES2600_BH_SUSPEND,
	BES2600_BH_SUSPENDED,
	BES2600_BH_RESUME,
};

/* Forward declarations */
static void bes2600_bh_work(struct work_struct *work);
static int bes2600_bh_rx(struct bes2600_common *hw_priv, int *tx);
static int bes2600_bh_tx(struct bes2600_common *hw_priv);
static int bes2600_bh_wait_event(struct bes2600_common *hw_priv, int *rx, int *tx, int *term, int *suspend);
static int bes2600_bh_handle_suspend(struct bes2600_common *hw_priv);
static int bes2600_bh_rx_helper(struct bes2600_common *hw_priv, int *tx);
static int bes2600_bh_tx_helper(struct bes2600_common *hw_priv, unsigned int *tx_burst);
static void bes2600_bh_parse_wakeup_event(struct bes2600_common *hw_priv, struct sk_buff *skb);
static int bes2600_bh(void *arg);

static inline void wsm_alloc_tx_buffer(struct bes2600_common *hw_priv)
{
	++hw_priv->hw_bufs_used;
}

int bes2600_register_bh(struct bes2600_common *hw_priv)
{
	int err = 0;
	hw_priv->bh_workqueue = alloc_workqueue("bes2600_bh",
		WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!hw_priv->bh_workqueue)
		return -ENOMEM;

	INIT_WORK(&hw_priv->bh_work, bes2600_bh_work);

	bes_devel("[BH] register.\n");

#ifdef WIFI_BT_COEXIST_EPTA_ENABLE
#ifdef WIFI_BT_COEXIST_EPTA_FDD
	coex_init_mode(hw_priv, WIFI_COEX_MODE_FDD_BIT);
#else
	coex_init_mode(hw_priv, 0);
#endif
#endif
	atomic_set(&hw_priv->bh_rx, 0);
	atomic_set(&hw_priv->bh_tx, 0);
	atomic_set(&hw_priv->bh_term, 0);
	atomic_set(&hw_priv->bh_suspend, BES2600_BH_RESUMED);
	hw_priv->buf_id_tx = 0;
	hw_priv->buf_id_rx = 0;
	timer_setup(&hw_priv->lmac_mon_timer, bes2600_bh_lmac_active_monitor, 0);
	timer_setup(&hw_priv->mcu_mon_timer, bes2600_bh_mcu_active_monitor, 0);
	init_waitqueue_head(&hw_priv->bh_wq);
	init_waitqueue_head(&hw_priv->bh_evt_wq);

	err = queue_work(hw_priv->bh_workqueue, &hw_priv->bh_work) ? 0 : -EBUSY;
	WARN_ON(err);
	return err;
}

void bes2600_unregister_bh(struct bes2600_common *hw_priv)
{
#ifdef WIFI_BT_COEXIST_EPTA_ENABLE
	coex_deinit_mode(hw_priv);
#endif
	atomic_add(1, &hw_priv->bh_term);
	wake_up(&hw_priv->bh_wq);
	flush_workqueue(hw_priv->bh_workqueue);
	destroy_workqueue(hw_priv->bh_workqueue);
	hw_priv->bh_workqueue = NULL;
	bes_devel("[BH] unregistered.\n");
}

void bes2600_irq_handler(struct bes2600_common *hw_priv)
{
	bes_devel("[BH] irq.\n");
	if (!hw_priv) {
		bes_warn("%s hw private data is null\n", __func__);
		return;
	}
	if (atomic_read(&hw_priv->bh_error)) {
		bes_err("%s bh error\n", __func__);
		return;
	}
	if (atomic_add_return(1, &hw_priv->bh_rx) == 1)
		wake_up(&hw_priv->bh_wq);
}
EXPORT_SYMBOL(bes2600_irq_handler);

void bes2600_bh_wakeup(struct bes2600_common *hw_priv)
{
	bes_devel("[BH] wakeup.\n");
	if (WARN_ON(atomic_read(&hw_priv->bh_error)))
		return;
	if (atomic_add_return(1, &hw_priv->bh_tx) == 1)
		wake_up(&hw_priv->bh_wq);
}
EXPORT_SYMBOL(bes2600_bh_wakeup);

int bes2600_bh_suspend(struct bes2600_common *hw_priv)
{
#ifdef MCAST_FWDING
	int i = 0;
	struct bes2600_vif *priv = NULL;
#endif
	bes_devel("[BH] suspend.\n");
	if (atomic_read(&hw_priv->bh_error)) {
		wiphy_warn(hw_priv->hw->wiphy, "BH error -- can't suspend\n");
		return -EINVAL;
	}
#ifdef MCAST_FWDING
	bes2600_for_each_vif(hw_priv, priv, i) {
		if (!priv)
			continue;
		if (priv->multicast_filter.enable && priv->join_status == BES2600_JOIN_STATUS_AP) {
			wsm_release_buffer_to_fw(priv, hw_priv->wsm_caps.numInpChBufs - 1);
			break;
		}
	}
#endif
	atomic_set(&hw_priv->bh_suspend, BES2600_BH_SUSPEND);
	wake_up(&hw_priv->bh_wq);
	return wait_event_timeout(hw_priv->bh_evt_wq,
		atomic_read(&hw_priv->bh_error) || (BES2600_BH_SUSPENDED == atomic_read(&hw_priv->bh_suspend)),
		1 * HZ) ? 0 : -ETIMEDOUT;
}
EXPORT_SYMBOL(bes2600_bh_suspend);

int bes2600_bh_resume(struct bes2600_common *hw_priv)
{
#ifdef MCAST_FWDING
	int i = 0;
	struct bes2600_vif *priv = NULL;
#endif
	bes_devel("[BH] resume.\n");
	if (atomic_read(&hw_priv->bh_error)) {
		wiphy_warn(hw_priv->hw->wiphy, "BH error -- can't resume\n");
		return -EINVAL;
	}
	atomic_set(&hw_priv->bh_suspend, BES2600_BH_RESUME);
	wake_up(&hw_priv->bh_wq);
	int ret = wait_event_timeout(hw_priv->bh_evt_wq,
		atomic_read(&hw_priv->bh_error) || (BES2600_BH_RESUMED == atomic_read(&hw_priv->bh_suspend)),
		1 * HZ) ? 0 : -ETIMEDOUT;
#ifdef MCAST_FWDING
	bes2600_for_each_vif(hw_priv, priv, i) {
		if (!priv)
			continue;
		if (priv->join_status == BES2600_JOIN_STATUS_AP && priv->multicast_filter.enable) {
			u8 count = 0;
			WARN_ON(wsm_request_buffer_request(priv, &count));
			bes_devel("[BH] BH resume. Reclaim Buff %d\n", count);
			break;
		}
	}
#endif
	return ret;
}
EXPORT_SYMBOL(bes2600_bh_resume);

static void bes2600_bh_work(struct work_struct *work)
{
	struct bes2600_common *hw_priv = container_of(work, struct bes2600_common, bh_work);
	bes2600_bh(hw_priv);
}

static int bes2600_bh_wait_event(struct bes2600_common *hw_priv, int *rx, int *tx, int *term, int *suspend)
{
	long status;
	unsigned long timeout = hw_priv->hw_bufs_used ? BH_WAIT_TIMEOUT :
		(!bes2600_pwr_device_is_idle(hw_priv) && !atomic_read(&hw_priv->recent_scan) &&
		 bes2600_chrdev_is_signal_mode()) ? BH_WAIT_TIMEOUT : MAX_SCHEDULE_TIMEOUT;

	status = wait_event_interruptible_timeout(hw_priv->bh_wq, ({
		*rx = atomic_xchg(&hw_priv->bh_rx, 0);
		*tx = atomic_xchg(&hw_priv->bh_tx, 0);
		*term = atomic_xchg(&hw_priv->bh_term, 0);
		*suspend = *tx ? 0 : atomic_read(&hw_priv->bh_suspend);
		(*rx || *tx || *term || *suspend || atomic_read(&hw_priv->bh_error));
	}), timeout);

	if (status < 0 && status != -ERESTARTSYS)
		return status;
	if (*term || atomic_read(&hw_priv->bh_error))
		return -EINVAL;
	if (!status && hw_priv->hw_bufs_used) {
		bes_err("Missed interrupt? (%d frames outstanding)\n", hw_priv->hw_bufs_used);
		sdio_work_debug(hw_priv->sbus_priv);
		bes2600_chrdev_wifi_force_close(hw_priv, false);
		return -ETIMEDOUT;
	}
	return 0;
}

static int bes2600_bh_handle_suspend(struct bes2600_common *hw_priv)
{
	bes_devel("[BH] Device suspend.\n");
	atomic_set(&hw_priv->bh_suspend, BES2600_BH_SUSPENDED);
	wake_up(&hw_priv->bh_evt_wq);
	long status = wait_event_interruptible(hw_priv->bh_wq,
		BES2600_BH_RESUME == atomic_read(&hw_priv->bh_suspend));
	if (status < 0) {
		wiphy_err(hw_priv->hw->wiphy, "Failed to wait for resume: %ld\n", status);
		return status;
	}
	bes_devel("[BH] Device resume.\n");
	atomic_set(&hw_priv->bh_suspend, BES2600_BH_RESUMED);
	wake_up(&hw_priv->bh_evt_wq);
	return 0;
}

static int bes2600_bh_rx(struct bes2600_common *hw_priv, int *tx)
{
	int ret, rx_cont = 0;
	while (rx_cont < BH_RX_CONT_LIMIT) {
		ret = bes2600_bh_rx_helper(hw_priv, tx);
		if (ret < 0) {
			bes_err("RX helper failed\n");
			sdio_work_debug(hw_priv->sbus_priv);
			bes2600_chrdev_wifi_force_close(hw_priv, false);
			return ret;
		}
		if (ret == 0)
			break;
		rx_cont++;
	}
	return 0;
}

static int bes2600_bh_tx(struct bes2600_common *hw_priv)
{
	int ret, tx_cont = 0;
	unsigned int tx_burst;
	while (tx_cont < BH_TX_CONT_LIMIT) {
		if (hw_priv->hw_bufs_used >= hw_priv->wsm_caps.numInpChBufs) {
			bes_devel("[BH] TX not allowed: buffers full\n");
			return 0;
		}
		ret = bes2600_bh_tx_helper(hw_priv, &tx_burst);
		if (ret < 0) {
			bes_err("TX helper failed\n");
			sdio_work_debug(hw_priv->sbus_priv);
			return ret;
		}
		if (ret == 0)
			break;
		tx_cont++;
	}
	return 0;
}

static int bes2600_bh(void *arg)
{
	struct bes2600_common *hw_priv = arg;
	int rx, tx, term, suspend, ret;

	for (;;) {
		ret = bes2600_bh_wait_event(hw_priv, &rx, &tx, &term, &suspend);
		if (ret) {
			bes_err("[BH] Wait event failed: %d\n", ret);
			break;
		}
		if (suspend) {
			ret = bes2600_bh_handle_suspend(hw_priv);
			if (ret) {
				bes_err("[BH] Suspend handling failed: %d\n", ret);
				break;
			}
			rx = 1; // Trigger RX after resume
			continue;
		}
		if (rx) {
			ret = bes2600_bh_rx(hw_priv, &tx);
			if (ret) {
				bes_err("[BH] RX failed: %d\n", ret);
				break;
			}
		}
		if (tx) {
			ret = bes2600_bh_tx(hw_priv);
			if (ret) {
				bes_err("[BH] TX failed: %d\n", ret);
				break;
			}
		}
	}
	hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
	__bes2600_irq_enable(0);
	hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);
	bes_err("[BH] Fatal error, exiting.\n");
	sdio_work_debug(hw_priv->sbus_priv);
	atomic_set(&hw_priv->bh_error, 1);
	return ret;
}

static int bes2600_bh_rx_helper(struct bes2600_common *hw_priv, int *tx)
{
    int ret = 0;
    struct sk_buff *skb = NULL;
    struct wsm_hdr *wsm;
    size_t wsm_len;
    u16 wsm_id;
    u8 wsm_seq;
    u32 confirm_label = 0;
#if defined(BES_SDIO_RX_MULTIPLE_ENABLE)
    skb = hw_priv->sbus_ops->pipe_read(hw_priv->sbus_priv);
    if (!skb)
        return 0;
#else
    u32 ctrl_reg = 0;
    size_t read_len = 0, alloc_len;
    u8 *data;
    if (bes2600_bh_read_ctrl_reg(hw_priv, &ctrl_reg)) {
        ret = -EIO;
        goto err;
    }
    read_len = ctrl_reg & BES_TX_NEXT_LEN_MASK;
    if (!read_len)
        return 0;
    if (WARN_ON(read_len < sizeof(struct wsm_hdr) || read_len > EFFECTIVE_BUF_SIZE)) {
        bes_err("Invalid read len: %zu (%04x)\n", read_len, ctrl_reg);
        ret = -EINVAL;
        goto err;
    }
    alloc_len = hw_priv->sbus_ops->align_size(hw_priv->sbus_priv, read_len);
    if (WARN_ON(alloc_len > EFFECTIVE_BUF_SIZE))
        bes_devel("Read aligned len: %zu\n", alloc_len);
    skb = dev_alloc_skb(alloc_len);
    if (!skb) {
        ret = -ENOMEM;
        goto err;
    }
    skb_put(skb, read_len);
    data = skb->data;
    if (bes2600_data_read(hw_priv, data, alloc_len)) {
        bes_err("RX read failed, len %zu\n", alloc_len);
        ret = -EIO;
        goto err;
    }
#endif
    wsm = (struct wsm_hdr *)skb->data;
    wsm_len = __le16_to_cpu(wsm->len);
    if (WARN_ON(wsm_len > skb->len)) {
        bes_err("wsm_len err %zu %u\n", wsm_len, skb->len);
        ret = -EINVAL;
        goto err;
    }
    if (hw_priv->wsm_enable_wsm_dumps)
        print_hex_dump(KERN_DEBUG, "<-- ", DUMP_PREFIX_NONE, 16, 1, skb->data, wsm_len, false);
    wsm_id = __le16_to_cpu(wsm->id) & 0xFFF;
    wsm_seq = (__le16_to_cpu(wsm->id) >> 13) & 7;
    bes_devel("[RX] wsm_id:0x%04x seq:%d\n", wsm_id, wsm_seq);
    skb_trim(skb, wsm_len);
    if (wsm_id == 0x0800) {
        wsm_handle_exception(hw_priv, &skb->data[sizeof(*wsm)], wsm_len - sizeof(*wsm));
        bes_err("WSM exception\n");
        ret = -EINVAL;
        goto err;
    }
    if (wsm_seq != hw_priv->wsm_rx_seq[WSM_TXRX_SEQ_IDX(wsm_id)]) {
        bes_err("Seq error: %u != %u, id:0x%x\n", wsm_seq, hw_priv->wsm_rx_seq[WSM_TXRX_SEQ_IDX(wsm_id)], wsm_id);
        ret = -EINVAL;
        goto err;
    }
    bes2600_bh_parse_wakeup_event(hw_priv, skb);
    hw_priv->wsm_rx_seq[WSM_TXRX_SEQ_IDX(wsm_id)] = (wsm_seq + 1) & 7;
    if (IS_DRIVER_TO_MCU_CMD(wsm_id))
        confirm_label = __le32_to_cpu(((struct wsm_mcu_hdr *)wsm)->handle_label);
    if (WSM_CONFIRM_CONDITION(wsm_id, confirm_label)) {
        int rc = wsm_release_tx_buffer(hw_priv, 1);
        bes2600_bh_dec_pending_count(hw_priv, WSM_TXRX_SEQ_IDX(wsm_id));
        if (rc < 0) {
            WARN_ON(rc);
            ret = rc;
            goto err;
        }
        if (rc > 0)
            *tx = 1;
    }
    ret = wsm_handle_rx(hw_priv, wsm_id, wsm, &skb);
    if (ret) {
        bes_err("wsm_handle_rx failed, id:0x%.4X\n", wsm_id);
        goto skip;
    }
    if (skb)
        dev_kfree_skb(skb);
#if defined(BES_SDIO_RX_MULTIPLE_ENABLE)
    return 1;
#else
    return 0;
#endif
skip:
    if (skb)
        dev_kfree_skb(skb);
    return 0;
err:
    if (skb)
        dev_kfree_skb(skb);
    bes_err("[BH] RX failed: %d\n", ret);
    atomic_set(&hw_priv->bh_error, 1);
    wake_up(&hw_priv->bh_evt_wq);
    return ret;
}

static int bes2600_bh_tx_helper(struct bes2600_common *hw_priv, unsigned int *tx_burst)
{
	size_t tx_len;
	u8 *data;
	int ret, vif_selected;
	struct wsm_hdr *wsm;

	wsm_alloc_tx_buffer(hw_priv);
	ret = wsm_get_tx(hw_priv, &data, &tx_len, tx_burst, &vif_selected);
	if (ret <= 0) {
		wsm_release_tx_buffer(hw_priv, 1);
		if (ret < 0) {
			bes_err("wsm_get_tx failed: %d\n", ret);
			return ret;
		}
		return 0;
	}
	wsm = (struct wsm_hdr *)data;
	BUG_ON(tx_len < sizeof(*wsm));
	BUG_ON(__le16_to_cpu(wsm->len) != tx_len);
#ifdef BES2600_HOST_TIMESTAMP_DEBUG
	tx_len += 4;
#endif
	atomic_add(1, &hw_priv->bh_tx);
	tx_len = hw_priv->sbus_ops->align_size(hw_priv->sbus_priv, tx_len);
	if (WARN_ON(tx_len > EFFECTIVE_BUF_SIZE))
		bes_err("Write aligned len: %zu\n", tx_len);
	wsm->id &= __cpu_to_le16(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
	wsm->id |= __cpu_to_le16(WSM_TX_SEQ(hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)]));
	bes_devel("[TX] id:0x%04x seq:%d\n", wsm->id, hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)]);
#ifndef BES_SDIO_TX_MULTIPLE_ENABLE
	if (bes2600_data_write(hw_priv, data, tx_len)) {
#else
	if (hw_priv->sbus_ops->pipe_send(hw_priv->sbus_priv, 1, tx_len, data)) {
#endif
		bes_err("TX write failed, len %zu\n", tx_len);
		wsm_release_tx_buffer(hw_priv, 1);
		return -EIO;
	}
	if (vif_selected != -1)
		hw_priv->hw_bufs_used_vif[vif_selected]++;
	if (hw_priv->wsm_enable_wsm_dumps)
		print_hex_dump(KERN_DEBUG, "--> ", DUMP_PREFIX_NONE, 16, 1, data, __le16_to_cpu(wsm->len), false);
	wsm_txed(hw_priv, data);
	hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)] = (hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)] + 1) & WSM_TX_SEQ_MAX;
	bes2600_bh_inc_pending_count(hw_priv, WSM_TXRX_SEQ_IDX(wsm->id));
	return *tx_burst > 1 ? 1 : 0;
}

static void bes2600_bh_parse_ipv4_data(struct iphdr *ip)
{
	u8 *tmp_ptr = (u8 *)ip;
	bes_info("IP Addr src:0x%08x dst:0x%08x\n", __be32_to_cpu(ip->saddr), __be32_to_cpu(ip->daddr));
	if (ip->protocol == IPPROTO_TCP) {
		struct tcphdr *tcp = (struct tcphdr *)(tmp_ptr + ip->ihl * 4);
		bes_info("TCP Port src:%d dst:%d\n", __be16_to_cpu(tcp->source), __be16_to_cpu(tcp->dest));
	} else if (ip->protocol == IPPROTO_UDP) {
		struct udphdr *udp = (struct udphdr *)(tmp_ptr + ip->ihl * 4);
		bes_info("UDP Port src:%d dst:%d\n", __be16_to_cpu(udp->source), __be16_to_cpu(udp->dest));
	}
}

static void bes2600_bh_parse_data_pkt(struct bes2600_common *hw_priv, struct sk_buff *skb)
{
	struct wsm_hdr *wsm = (struct wsm_hdr *)skb->data;
	u16 wsm_id = __le16_to_cpu(wsm->id) & 0xFFF;
	int if_id = (wsm_id >> 6) & 0x0F;
	u8 *data_ptr = (u8 *)&wsm[1];
	struct ieee80211_hdr *i80211_ptr = (struct ieee80211_hdr *)(data_ptr + 28);
	__le16 fctl = *(__le16 *)i80211_ptr;
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(hw_priv->vif_list[if_id]);
	u32 encry_hdr_len = bes2600_bh_get_encry_hdr_len(priv ? priv->cipherType : 0);
	u32 i80211_len = ieee80211_hdrlen(fctl);
	u8 *tmp_ptr = (u8 *)i80211_ptr;
	u16 *eth_type_ptr = (u16 *)(tmp_ptr + i80211_len + encry_hdr_len + ETH_ALEN);
	u16 eth_type = __be16_to_cpu(*eth_type_ptr);
	bes_devel("Host was waked by data:\nRA:%pM\nETH_TYPE:0x%04x\n", ieee80211_get_DA(i80211_ptr), eth_type);
	if (eth_type == ETH_P_IP) {
		struct iphdr *ip = (struct iphdr *)&eth_type_ptr[1];
		bes_info("IP version: %d\nIP proto: %d\n", ip->version, ip->protocol);
		if (ip->version == 4)
			bes2600_bh_parse_ipv4_data(ip);
	}
}

static void bes2600_bh_parse_wakeup_event(struct bes2600_common *hw_priv, struct sk_buff *skb)
{
	struct wsm_hdr *wsm = (struct wsm_hdr *)skb->data;
	u16 wsm_id = __le16_to_cpu(wsm->id) & 0xFFF;
	bool set_wakeup_reason_later = false;
	if (!hw_priv->sbus_ops->wakeup_source || !hw_priv->sbus_ops->wakeup_source(hw_priv->sbus_priv))
		return;
	if (wsm_id == 0x0804) {
		u8 *data_ptr = (u8 *)&wsm[1];
		u8 *i80211_ptr = data_ptr + 28;
		__le16 fctl = *(__le16 *)i80211_ptr;
		if (ieee80211_is_mgmt(fctl)) {
			u16 type = (fctl & cpu_to_le16(IEEE80211_FCTL_FTYPE)) >> 2;
			u16 stype = (fctl & cpu_to_le16(IEEE80211_FCTL_STYPE)) >> 4;
			if (ieee80211_is_deauth(fctl) || ieee80211_is_disassoc(fctl)) {
				bes2600_chrdev_wakeup_by_event_set(WAKEUP_EVENT_PEER_DETACH);
				set_wakeup_reason_later = true;
				bes_devel("Host was waked by mgmt(deauth or disassoc)\n");
			}
			bes_devel("Host was waked by mgmt, type:%d(%d)\n", type, stype);
		} else if (ieee80211_is_data(fctl)) {
			bes2600_bh_parse_data_pkt(hw_priv, skb);
		} else {
			bes_devel("Host was waked by unexpected frame, fctl:0x%04x\n", fctl);
		}
	} else if (wsm_id == 0x0C31) {
		bes_devel("Host was waked by BT:0x%04x\n", wsm_id);
		bes2600_chrdev_wifi_update_wakeup_reason(WAKEUP_REASON_BT_PLAY, 0);
	} else {
		if (wsm_id == 0x0805) {
			bes2600_chrdev_wakeup_by_event_set(WAKEUP_EVENT_WSME);
			set_wakeup_reason_later = true;
		}
		bes_devel("Host was waked by event:0x%04x\n", wsm_id);
	}
	if (!set_wakeup_reason_later)
		bes2600_chrdev_wakeup_by_event_set(WAKEUP_EVENT_NONE);
}

void bes2600_enable_powersave(struct bes2600_vif *priv, bool enable)
{
	bes_devel("[BH] Powersave %s\n", enable ? "enabled" : "disabled");
	priv->powersave_enabled = enable;
}

int wsm_release_tx_buffer(struct bes2600_common *hw_priv, int count)
{
	int ret = 0;
	int hw_bufs_used = hw_priv->hw_bufs_used;
	hw_priv->hw_bufs_used -= count;
	if (WARN_ON(hw_priv->hw_bufs_used < 0))
		ret = -1;
	else if (hw_bufs_used >= (hw_priv->wsm_caps.numInpChBufs - 1))
		ret = 1;
	if (!hw_priv->hw_bufs_used) {
		bes2600_pwr_clear_busy_event(hw_priv, BES_PWR_LOCK_ON_LMAC_RSP);
		wake_up(&hw_priv->bh_evt_wq);
	}
	return ret;
}
EXPORT_SYMBOL(wsm_release_tx_buffer);

int wsm_release_vif_tx_buffer(struct bes2600_common *hw_priv, int if_id, int count)
{
	int ret = 0;
	hw_priv->hw_bufs_used_vif[if_id] -= count;
	if (!hw_priv->hw_bufs_used_vif[if_id])
		wake_up(&hw_priv->bh_evt_wq);
	if (WARN_ON(hw_priv->hw_bufs_used_vif[if_id] < 0))
		ret = -1;
	return ret;
}

#ifdef MCAST_FWDING
int wsm_release_buffer_to_fw(struct bes2600_vif *priv, int count)
{
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	int i;
	u8 flags;
	struct wsm_buf *buf;
	size_t buf_len;
	struct wsm_hdr *wsm;
	if (priv->join_status != BES2600_JOIN_STATUS_AP)
		return 0;
	bes_devel("Rel buffer to FW %d, %d\n", count, hw_priv->hw_bufs_used);
	for (i = 0; i < count; i++) {
		if ((hw_priv->hw_bufs_used + 1) < hw_priv->wsm_caps.numInpChBufs) {
			flags = i ? 0 : 0x1;
			wsm_alloc_tx_buffer(hw_priv);
			buf = &hw_priv->wsm_release_buf[i];
			buf_len = buf->data - buf->begin;
			wsm = (struct wsm_hdr *)buf->begin;
			BUG_ON(buf_len < sizeof(*wsm));
			wsm->id &= __cpu_to_le32(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
			wsm->id |= cpu_to_le32(WSM_TX_SEQ(hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)]));
			if (bes2600_data_write(hw_priv, buf->begin, buf_len))
				break;
			hw_priv->buf_released = 1;
			hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)] =
				(hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)] + 1) & WSM_TX_SEQ_MAX;
		} else
			break;
	}
	if (i == count)
		return 0;
	bes_devel("[BH] Less HW buf %d,%d\n", hw_priv->hw_bufs_used, hw_priv->wsm_caps.numInpChBufs);
	WARN_ON(1);
	return -1;
}
#endif

#ifdef KEY_FRAME_SW_RETRY
static inline bool ieee80211_is_tcp_pkt(struct sk_buff *skb)
{
	if (!skb)
		return false;
	if (skb->protocol == cpu_to_be16(ETH_P_IP)) {
		struct iphdr *iph = (struct iphdr *)skb_network_header(skb);
		if (iph->protocol == IPPROTO_TCP)
			return true;
	}
	return false;
}

static int bes2600_need_retry_type(struct sk_buff *skb, int status)
{
	if (!skb)
		return -1;
	if (skb->protocol == cpu_to_be16(ETH_P_IP) && ieee80211_is_tcp_pkt(skb))
		return status != WSM_STATUS_RETRY_EXCEEDED ? 0 : 1;
	return 0;
}

int bes2600_bh_sw_process(struct bes2600_common *hw_priv, struct wsm_tx_confirm *tx_confirm)
{
	struct bes2600_txpriv *txpriv;
	struct sk_buff *skb = NULL;
	unsigned long timestamp = 0;
	struct bes2600_queue *queue;
	u8 queue_id, queue_gen;
	long delta_time;
	if (!tx_confirm) {
		bes_err("%s tx_confirm is NULL\n", __func__);
		return 0;
	}
	queue_id = bes2600_queue_get_queue_id(tx_confirm->packetID);
	queue = &hw_priv->tx_queue[queue_id];
	if (!queue) {
		bes_err("%s queue is NULL\n", __func__);
		return 0;
	}
	queue_gen = bes2600_queue_get_generation(tx_confirm->packetID);
	if (queue_gen != queue->generation)
		return -1;
	bes2600_queue_get_skb_and_timestamp(queue, tx_confirm->packetID, &skb, &txpriv, &timestamp);
	if (!skb) {
		bes_err("%s skb is NULL\n", __func__);
		return -1;
	}
	delta_time = (timestamp > jiffies) ? jiffies + (ULONG_MAX - timestamp) : jiffies - timestamp;
	bes2600_add_tx_delta_time(delta_time);
	bes2600_add_tx_ac_delta_time(queue_id, delta_time);
	if (bes2600_need_retry_type(skb, tx_confirm->status) == 0)
		return -1;
	if (delta_time > 1000)
		return -1;
	if (txpriv->retry_count < CW1200_MAX_SW_RETRY_CNT) {
		struct bes2600_vif *priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, txpriv->if_id);
		txpriv->retry_count++;
		bes2600_tx_status(priv, skb);
		bes2600_pwr_set_busy_event_with_timeout_async(hw_priv, BES_PWR_LOCK_ON_TX, BES_PWR_EVENT_TX_TIMEOUT);
		bes2600_sw_retry_requeue(hw_priv, queue, tx_confirm->packetID, true);
		return 0;
	}
	txpriv->retry_count = 0;
	return -1;
}
#endif

void bes2600_bh_inc_pending_count(struct bes2600_common *hw_priv, int idx)
{
	struct timer_list *timer = (idx == 0) ? &hw_priv->lmac_mon_timer : &hw_priv->mcu_mon_timer;
	if (hw_priv->wsm_tx_pending[idx]++ == 0) {
		bes_devel("start timer in tx, idx:%d\n", idx);
		mod_timer(timer, jiffies + 3 * HZ);
	}
}

void bes2600_bh_dec_pending_count(struct bes2600_common *hw_priv, int idx)
{
	struct timer_list *timer = (idx == 0) ? &hw_priv->lmac_mon_timer : &hw_priv->mcu_mon_timer;
	if (hw_priv->wsm_tx_pending[idx] == 0) {
		bes_err("tx pending count error, idx:%d\n", idx);
		return;
	}
	if (--hw_priv->wsm_tx_pending[idx] == 0)
		timer_delete_sync(timer);
	else
		mod_timer(timer, jiffies + 3 * HZ);
}

void bes2600_bh_mcu_active_monitor(struct timer_list *t)
{
	struct bes2600_common *hw_priv = from_timer(hw_priv, t, mcu_mon_timer);
	bes_err("link break between mcu and host, hw_buf_used:%d pending:%d\n",
		hw_priv->hw_bufs_used, hw_priv->wsm_tx_pending[1]);
	bes2600_chrdev_wifi_force_close(hw_priv, true);
}

void bes2600_bh_lmac_active_monitor(struct timer_list *t)
{
	struct bes2600_common *hw_priv = from_timer(hw_priv, t, lmac_mon_timer);
	bes_err("link break between lmac and host, hw_buf_used:%d pending:%d\n",
		hw_priv->hw_bufs_used, hw_priv->wsm_tx_pending[0]);
	bes2600_chrdev_wifi_force_close(hw_priv, true);
}
