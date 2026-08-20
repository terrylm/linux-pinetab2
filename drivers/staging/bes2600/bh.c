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

static int bes2600_bh(struct bes2600_common *hw_priv);

extern void sdio_work_debug(struct sbus_priv *self);

/* TODO: Verify these numbers with WSM specification. */
#define DOWNLOAD_BLOCK_SIZE_WR	(0x1000 - 4)
/* an SPI message cannot be bigger than (2"12-1)*2 bytes
 * "*2" to cvt to bytes */
#define MAX_SZ_RD_WR_BUFFERS	(DOWNLOAD_BLOCK_SIZE_WR * 2)
#define PIGGYBACK_CTRL_REG	(2)
#define EFFECTIVE_BUF_SIZE	(MAX_SZ_RD_WR_BUFFERS - PIGGYBACK_CTRL_REG)

/* Suspend state privates */
enum bes2600_bh_pm_state {
	BES2600_BH_RESUMED = 0,
	BES2600_BH_SUSPEND,
	BES2600_BH_SUSPENDED,
	BES2600_BH_RESUME,
};

typedef int (*bes2600_wsm_handler)(struct bes2600_common *hw_priv,
	u8 *data, size_t size);

#ifdef MCAST_FWDING
int wsm_release_buffer_to_fw(struct bes2600_vif *priv, int count);
#endif

/*
 * BH used to run as a work_struct that never returned.  That is fragile:
 * if the work ever exits, nothing restarts it, and wake_up on bh_wq is a
 * no-op while bh_tx climbs forever (exactly the join failure mode).
 * A dedicated kthread is the correct model for a permanent wait loop.
 */
static int bes2600_bh_thread(void *arg)
{
	struct bes2600_common *hw_priv = arg;

	bes2600_bh(hw_priv);
	return 0;
}

int bes2600_register_bh(struct bes2600_common *hw_priv)
{
	bes_devel("[BH] register.\n");

#ifdef CONFIG_WIFI_BT_COEXIST_EPTA_FDD
	coex_init_mode(hw_priv, WIFI_COEX_MODE_FDD_BIT);
#else
	coex_init_mode(hw_priv, 0);
#endif
	atomic_set(&hw_priv->bh_rx, 0);
	atomic_set(&hw_priv->bh_tx, 0);
	atomic_set(&hw_priv->bh_term, 0);
	atomic_set(&hw_priv->bh_suspend, BES2600_BH_RESUMED);
	atomic_set(&hw_priv->bh_error, 0);
	hw_priv->buf_id_tx = 0;
	hw_priv->buf_id_rx = 0;
	hw_priv->bh_workqueue = NULL;
	init_waitqueue_head(&hw_priv->bh_wq);
	init_waitqueue_head(&hw_priv->bh_evt_wq);

	hw_priv->bh_thread = kthread_run(bes2600_bh_thread, hw_priv, "bes2600_bh");
	if (IS_ERR(hw_priv->bh_thread)) {
		int err = PTR_ERR(hw_priv->bh_thread);

		hw_priv->bh_thread = NULL;
		bes_err("[BH] kthread_run failed: %d\n", err);
		return err;
	}

	bes_devel("[BH] kthread started\n");
	return 0;
}

void bes2600_unregister_bh(struct bes2600_common *hw_priv)
{
	coex_deinit_mode(hw_priv);
	atomic_add(1, &hw_priv->bh_term);
	wake_up(&hw_priv->bh_wq);
	if (hw_priv->bh_thread) {
		kthread_stop(hw_priv->bh_thread);
		hw_priv->bh_thread = NULL;
	}
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

/* Wake up the Bottom Half thread when there is work to do */
int bes2600_bh_wakeup(struct bes2600_common *hw_priv)
{
	if (!hw_priv) {
		bes_err("%s: hw_priv is NULL\n", __func__);
		return -EINVAL;
	}

	bes_devel("[BH] wakeup.\n");

	/* If the BH thread has already hit a fatal error, we cannot wake it */
	if (atomic_read(&hw_priv->bh_error) != 0) {
		bes_err("%s: BH thread is in error state - cannot wake\n", __func__);
		return -EIO;
	}

	/*
	 * Always wake_up.  Only waking on 0→1 misses the case where BH is
	 * mid-loop with a stale bh_tx count and a pending wsm_cmd (join
	 * then sits until full WSM_CMD_JOIN_TIMEOUT with no "BH handing").
	 */
	atomic_inc(&hw_priv->bh_tx);
	wake_up(&hw_priv->bh_wq);

	return 0;
}
EXPORT_SYMBOL(bes2600_bh_wakeup);

static inline void wsm_alloc_tx_buffer(struct bes2600_common *hw_priv)
{
	++hw_priv->hw_bufs_used; /* Protected by tx_lock in bes2600_tx */
}

static bool bes2600_bh_wsm_cmd_in_flight(struct bes2600_common *hw_priv);

void bes2600_bh_prepare_for_join(struct bes2600_common *hw_priv)
{
	int susp;
	int used;

	if (!hw_priv)
		return;

	/*
	 * If BH is parked in suspend wait, request RESUME and wait for it to
	 * ack.  Do not force RESUMED ourselves — that races and can leave BH
	 * waiting for RESUME forever while state already reads RESUMED.
	 */
	susp = atomic_read(&hw_priv->bh_suspend);
	if (susp == BES2600_BH_SUSPEND || susp == BES2600_BH_SUSPENDED) {
		bes_devel("%s: BH suspend state=%d — requesting RESUME\n",
			 __func__, susp);
		atomic_set(&hw_priv->bh_suspend, BES2600_BH_RESUME);
		wake_up(&hw_priv->bh_wq);
		wait_event_timeout(hw_priv->bh_evt_wq,
				   atomic_read(&hw_priv->bh_suspend) ==
					BES2600_BH_RESUMED ||
				   atomic_read(&hw_priv->bh_error),
				   HZ);
	}

	/*
	 * Stale host TX slots after scan leave join with no free FW input
	 * buffer (or a desynced count).  Wait briefly for a natural confirm,
	 * then drop host-side accounting so join can be sent and confirmed.
	 */
	used = hw_priv->hw_bufs_used;
	if (used > 0) {
		bes_warn("%s: hw_bufs_used=%d before join — waiting for drain\n",
			 __func__, used);
		atomic_inc(&hw_priv->bh_rx);
		bes2600_bh_wakeup(hw_priv);
		wait_event_timeout(hw_priv->bh_evt_wq,
				   !hw_priv->hw_bufs_used, HZ / 4);
		if (hw_priv->hw_bufs_used > 0) {
			bes_warn("%s: still bufs=%d — soft-clear host count\n",
				 __func__, hw_priv->hw_bufs_used);
			timer_delete(&hw_priv->lmac_mon_timer);
			timer_delete(&hw_priv->mcu_mon_timer);
			hw_priv->wsm_tx_pending[0] = 0;
			hw_priv->wsm_tx_pending[1] = 0;
			wsm_release_tx_buffer(hw_priv, hw_priv->hw_bufs_used);
			if (hw_priv->hw_bufs_used < 0)
				hw_priv->hw_bufs_used = 0;
		}
	}

	bes2600_bh_wakeup(hw_priv);

	bes_devel("%s: bh_thread=%s susp=%d bufs=%d bh_tx=%d\n",
		 __func__,
		 hw_priv->bh_thread ? "yes" : "NO",
		 atomic_read(&hw_priv->bh_suspend),
		 hw_priv->hw_bufs_used,
		 atomic_read(&hw_priv->bh_tx));
}
EXPORT_SYMBOL(bes2600_bh_prepare_for_join);

void bes2600_bh_tx_fail_work(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, tx_fail_work);
	int i;

	/*
	 * Process context: safe to complete skbs to mac80211.  Clears the
	 * REPORTS_TX_ACK_STATUS hang after mon/BH soft-clear of buf counts.
	 */
	bes_warn("%s: completing stuck TX frames to mac80211\n", __func__);
	for (i = 0; i < 4; i++)
		bes2600_queue_clear(&hw_priv->tx_queue[i], CW12XX_ALL_IFS);
}

void bes2600_bh_mark_bus_stale(struct bes2600_common *hw_priv)
{
	if (!hw_priv)
		return;
	if (!hw_priv->bus_stale)
		bes_warn("%s: bus marked stale (no WSM sleep / mon thrash)\n",
			 __func__);
	hw_priv->bus_stale = true;
	/*
	 * Wake join/WSM waiters immediately.  Do not complete wsm_cmd
	 * under this lock from a timer (deadlock vs BH confirm).
	 * wsm_cmd_send treats bus_stale && !done as a timeout.
	 */
	wake_up(&hw_priv->wsm_cmd_wq);
	bes2600_bh_abort_pending_tx(hw_priv);
}
EXPORT_SYMBOL(bes2600_bh_mark_bus_stale);

void bes2600_bh_clear_bus_stale(struct bes2600_common *hw_priv)
{
	if (!hw_priv || !hw_priv->bus_stale)
		return;
	hw_priv->bus_stale = false;
	bes_info("%s: bus recovered\n", __func__);
}
EXPORT_SYMBOL(bes2600_bh_clear_bus_stale);

bool bes2600_bh_bus_quiet(struct bes2600_common *hw_priv)
{
	if (!hw_priv)
		return true;
	if (hw_priv->bus_stale)
		return true;
	return time_is_before_jiffies(hw_priv->rx_timestamp + 2 * HZ);
}
EXPORT_SYMBOL(bes2600_bh_bus_quiet);

void bes2600_bh_abort_pending_tx(struct bes2600_common *hw_priv)
{
	int used;

	if (!hw_priv)
		return;

	timer_delete(&hw_priv->lmac_mon_timer);
	timer_delete(&hw_priv->mcu_mon_timer);
	hw_priv->wsm_tx_pending[0] = 0;
	hw_priv->wsm_tx_pending[1] = 0;

	used = hw_priv->hw_bufs_used;
	if (used > 0) {
		bes_warn("%s: drop host hw_bufs_used=%d after cmd timeout\n",
			 __func__, used);
		wsm_release_tx_buffer(hw_priv, used);
		if (hw_priv->hw_bufs_used < 0)
			hw_priv->hw_bufs_used = 0;
	}

	/*
	 * Never queue_clear/tx_status from softirq (mon timer) — that has
	 * hard-locked after usedbuf.  Always hand skb completion to WQ.
	 *
	 * Skip only while join (0x000B) still owns the auth skb.  A
	 * post-join MIB in flight must not block completing data/auth.
	 */
	if (bes2600_bh_wsm_cmd_in_flight(hw_priv)) {
		u16 cmd;

		spin_lock(&hw_priv->wsm_cmd.lock);
		cmd = hw_priv->wsm_cmd.cmd;
		spin_unlock(&hw_priv->wsm_cmd.lock);
		if ((cmd & 0x0fff) == 0x000B) {
			bes_info("%s: skip queue_clear — join in flight\n",
				 __func__);
			return;
		}
	}
	if (hw_priv->workqueue)
		queue_work(hw_priv->workqueue, &hw_priv->tx_fail_work);
}
EXPORT_SYMBOL(bes2600_bh_abort_pending_tx);

/*
 * Direct SDIO join TX was a diagnostic workaround.  It left pending host
 * buffers and never fixed the real issue (BH not running).  Kept as a
 * no-op so call sites can be removed cleanly.
 */
int bes2600_bh_flush_wsm_cmd(struct bes2600_common *hw_priv)
{
	if (!hw_priv)
		return -EINVAL;
	/* Prefer waking BH over racing it on the bus */
	bes2600_bh_wakeup(hw_priv);
	return 0;
}
EXPORT_SYMBOL(bes2600_bh_flush_wsm_cmd);

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
		if (priv->multicast_filter.enable
			&& priv->join_status == BES2600_JOIN_STATUS_AP) {
			wsm_release_buffer_to_fw(priv, hw_priv->wsm_caps.numInpChBufs - 1);
			break;
		}
	}
#endif

	atomic_set(&hw_priv->bh_suspend, BES2600_BH_SUSPEND);
	wake_up(&hw_priv->bh_wq);
	return wait_event_timeout(hw_priv->bh_evt_wq,
		atomic_read(&hw_priv->bh_error) ||
		(BES2600_BH_SUSPENDED == atomic_read(&hw_priv->bh_suspend)),
		1 * HZ) ? 0 : -ETIMEDOUT;
}
EXPORT_SYMBOL(bes2600_bh_suspend);

int bes2600_bh_resume(struct bes2600_common *hw_priv)
{
	int ret;

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
	ret = wait_event_timeout(hw_priv->bh_evt_wq, atomic_read(&hw_priv->bh_error) ||
		(BES2600_BH_RESUMED == atomic_read(&hw_priv->bh_suspend)),
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

int wsm_release_tx_buffer(struct bes2600_common *hw_priv, int count)
{
	int ret = 0;
	int hw_bufs_used = hw_priv->hw_bufs_used;

	hw_priv->hw_bufs_used -= count;

	if (WARN_ON(hw_priv->hw_bufs_used < 0)) {
		hw_priv->hw_bufs_used = 0;
		ret = -1;
	}
	/* Tx data patch stops when all but one hw buffers are used.
	   So, re-start tx path in case we find hw_bufs_used equals
	   numInputChBufs - 1.
	 */
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
	int i;
	u8 flags;
	struct wsm_buf *buf;
	size_t buf_len;
	struct wsm_hdr *wsm;
	struct bes2600_common *hw_priv = priv->hw_priv;

	if (priv->join_status != BES2600_JOIN_STATUS_AP)
		return 0;

	bes_devel("Rel buffer to FW %d, %d\n", count, hw_priv->hw_bufs_used);

	for (i = 0; i < count; i++) {
		if ((hw_priv->hw_bufs_used + 1) < hw_priv->wsm_caps.numInpChBufs) {
			flags = i ? 0: 0x1;

			wsm_alloc_tx_buffer(hw_priv);

			buf = &hw_priv->wsm_release_buf[i];
			buf_len = buf->data - buf->begin;

			/* Add sequence number */
			wsm = (struct wsm_hdr *)buf->begin;
			BUG_ON(buf_len < sizeof(*wsm));

			wsm->id &= __cpu_to_le32(~WSM_TX_SEQ(WSM_TX_SEQ_MAX));
			wsm->id |= cpu_to_le32(WSM_TX_SEQ(hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)]));

			bes_devel("REL %d\n", hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)]);
			if (WARN_ON(bes2600_data_write(hw_priv, buf->begin, buf_len)))
				break;

			hw_priv->buf_released = 1;
			hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)] =
				(hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)] + 1) & WSM_TX_SEQ_MAX;
		} else
			break;
	}

	if (i == count)
		return 0;

	/* Should not be here */
	bes_devel("[BH] Less HW buf %d,%d.\n", hw_priv->hw_bufs_used,
			hw_priv->wsm_caps.numInpChBufs);
	WARN_ON(1);

	return -1;
}
#endif


/* Must be called from BH thraed. */
void bes2600_enable_powersave(struct bes2600_vif *priv, bool enable)
{
	bes_devel("[BH] Powersave is %s.\n", enable ? "enabled" : "disabled");
	priv->powersave_enabled = enable;
}

extern int bes2600_bh_read_ctrl_reg(struct bes2600_common *priv, u32 *ctrl_reg);

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
	struct ieee80211_hdr *i80211_ptr = (struct ieee80211_hdr *)(data_ptr + 28 /* radio header */);
	__le16 fctl = *(__le16 *)i80211_ptr;
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(hw_priv->vif_list[if_id]);
	u32 encry_hdr_len = bes2600_bh_get_encry_hdr_len(priv->cipherType);
	u32 i80211_len = ieee80211_hdrlen(fctl);
	u8 *tmp_ptr = (u8 *)i80211_ptr;
	u16 *eth_type_ptr = (u16 *)(tmp_ptr + i80211_len + encry_hdr_len + ETH_ALEN);
	u16 eth_type = __be16_to_cpu(*eth_type_ptr);

	bes_devel("Host was waked by data:\nRA:%pM\nETH_TYPE:0x%04x\n", ieee80211_get_DA(i80211_ptr), eth_type);

	if (eth_type == ETH_P_IP) {
		struct iphdr *ip = (struct iphdr *)&eth_type_ptr[1];
		bes_info("IP version: %d\nIP proto: %d", ip->version, ip->protocol);

		if (ip->version == 4) {
			bes2600_bh_parse_ipv4_data(ip);
		}
	}
}

static void bes2600_bh_parse_wakeup_event(struct bes2600_common *hw_priv, struct sk_buff *skb)
{
	struct wsm_hdr *wsm = (struct wsm_hdr *)skb->data;
	u16 wsm_id = __le16_to_cpu(wsm->id) & 0xFFF;
	bool set_wakeup_reason_later = false;

	if (hw_priv->sbus_ops->wakeup_source &&
		hw_priv->sbus_ops->wakeup_source(hw_priv->sbus_priv)) {
		if (wsm_id == 0x0804) {
			u8 *data_ptr = (u8 *)&wsm[1];
			u8 *i80211_ptr = data_ptr + 28/* radio header */;
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
			} else if (ieee80211_is_data(fctl)){
				bes2600_bh_parse_data_pkt(hw_priv, skb);
			} else {
				bes_devel("Host was waked by unexpected frame, fctl:0x%04x\n", fctl);
			}
		} else if (wsm_id == 0x0C31) {
			bes_devel("Host was waked by BT:0x%04x.\n", wsm_id);
			bes2600_chrdev_wifi_update_wakeup_reason(WAKEUP_REASON_BT_PLAY, 0);
		} else {
			if (wsm_id == 0x0805) {
				bes2600_chrdev_wakeup_by_event_set(WAKEUP_EVENT_WSME);
				set_wakeup_reason_later = true;
			}
			bes_devel("Host was waked by event:0x%04x.\n", wsm_id);
		}
		if (!set_wakeup_reason_later)
			bes2600_chrdev_wakeup_by_event_set(WAKEUP_EVENT_NONE);
	}
}

static int bes2600_bh_rx_helper(struct bes2600_common *priv, int *tx)
{
	struct sk_buff *skb = NULL;
	struct wsm_hdr *wsm;
	size_t wsm_len;
	u16 wsm_id;
	u8 wsm_seq;
	int rx = 0;
	u32 confirm_label = 0x0; /* wsm to mcu cmd cnfirm label */

#if defined(CONFIG_BES_SDIO_RX_MULTIPLE_ENABLE)
	skb = (struct sk_buff *)priv->sbus_ops->pipe_read(priv->sbus_priv);
	if (!skb)
		return 0;
	rx = 1; // always consider rx pipe not empty
#else
	u32 ctrl_reg = 0;
	size_t read_len = 0;
//	int rx_resync = 1;
	size_t alloc_len;
	u8 *data;

	bes2600_bh_read_ctrl_reg(priv, &ctrl_reg);

	read_len = (ctrl_reg & BES_TX_NEXT_LEN_MASK);

	if (!read_len)
		return 0; /* No more work */

	if (WARN_ON((read_len < sizeof(struct wsm_hdr)) ||
			(read_len > EFFECTIVE_BUF_SIZE))) {
		bes_err("Invalid read len: %zu (%04x)\n", read_len, ctrl_reg);
		goto err;
	}

	/* more 2 byte is not needed ? */
#if 0
	/* Add SIZE of PIGGYBACK reg (CONTROL Reg)
	 * to the NEXT Message length + 2 Bytes for SKB
	 */
	read_len = read_len + 2;
#endif

	alloc_len = priv->sbus_ops->align_size(
		priv->sbus_priv, read_len);

	/* Check if not exceeding BES2600 capabilities */
	if (WARN_ON_ONCE(alloc_len > EFFECTIVE_BUF_SIZE))
		bes_devel("Read aligned len: %zu\n", alloc_len);

	skb = dev_alloc_skb(alloc_len);
	if (WARN_ON(!skb))
		goto err;

	skb_trim(skb, 0);
	skb_put(skb, read_len);
	data = skb->data;
	if (WARN_ON(!data))
		goto err;

	if (WARN_ON(bes2600_data_read(priv, data, alloc_len))) {
		bes_err("rx blew up, len %zu\n", alloc_len);
		goto err;
	}

	/* piggyback is not implemented,
	 * and only recieve data once
	 */
#if 0
	/* Piggyback */
	ctrl_reg = __le16_to_cpu(
		((__le16 *)data)[alloc_len / 2 - 1]);

	/* check if more data need to recv */
	if (ctrl_reg & ST90TDS_CONT_NEXT_LEN_MASK)
		rx = 1;
#else
	rx = 0;
#endif

#endif

	wsm = (struct wsm_hdr *)skb->data;
	wsm_len = __le16_to_cpu(wsm->len);
	if (WARN_ON(wsm_len > skb->len)) {
		bes_err("wsm_len err %d %d\n", (int)wsm_len, (int)skb->len);
		goto err;
	}

	if (priv->wsm_enable_wsm_dumps)
		print_hex_dump(KERN_DEBUG, "<-- ", DUMP_PREFIX_NONE, 16, 1, skb->data, wsm_len, false);

	wsm_id	= __le16_to_cpu(wsm->id) & 0xFFF;
	wsm_seq = (__le16_to_cpu(wsm->id) >> 13) & 7;
	bes_devel("bes2600_bh_rx_helper wsm_id:0x%04x seq:%d\n", wsm_id, wsm_seq);

	skb_trim(skb, wsm_len);

	if (wsm_id == 0x0800) {
		wsm_handle_exception(priv,
					 &skb->data[sizeof(*wsm)],
					 wsm_len - sizeof(*wsm));
		bes_err("wsm exception\n");
		goto err;
	} else if ((wsm_seq != priv->wsm_rx_seq[WSM_TXRX_SEQ_IDX(wsm_id)])) {
		bes_err("seq error! %u. %u. 0x%x.", wsm_seq, priv->wsm_rx_seq[WSM_TXRX_SEQ_IDX(wsm_id)], wsm_id);
		goto err;
	}

	bes2600_bh_parse_wakeup_event(priv, skb);

	priv->wsm_rx_seq[WSM_TXRX_SEQ_IDX(wsm_id)] = (wsm_seq + 1) & 7;

	/* Any good SDIO RX means the pipe is alive again */
	priv->rx_timestamp = jiffies;
	bes2600_bh_clear_bus_stale(priv);

	if (IS_DRIVER_TO_MCU_CMD(wsm_id))
		confirm_label = __le32_to_cpu(((struct wsm_mcu_hdr *)wsm)->handle_label);

	if (WSM_CONFIRM_CONDITION(wsm_id, confirm_label)) {
		int rc = wsm_release_tx_buffer(priv, 1);
		bes2600_bh_dec_pending_count(priv, WSM_TXRX_SEQ_IDX(wsm->id));

		if (WARN_ON(rc < 0))
			return rc;
		else if (rc > 0)
			*tx = 1;
	}

	/* bes2600_wsm_rx takes care on SKB livetime */
	//if (WARN_ON(wsm_handle_rx(priv, wsm_id, wsm, &skb)))
	if ((wsm_handle_rx(priv, wsm_id, wsm, &skb))) {
		bes_err("wsm_handle_rx fail\n");
		goto err;
	}

	if (skb) {
		dev_kfree_skb(skb);
		skb = NULL;
	}
	return rx;

err:
	bes_err("bes2600_bh_rx_helper err\n");
	if (skb) {
		dev_kfree_skb(skb);
		skb = NULL;
	}
	return -1;
}

static int bes2600_bh_tx_helper(struct bes2600_common *hw_priv,
				   int *pending_tx,
				   int *tx_burst)
{
	size_t tx_len;
	u8 *data;
	int ret;
	struct wsm_hdr *wsm;
	int vif_selected;

	wsm_alloc_tx_buffer(hw_priv);
	ret = wsm_get_tx(hw_priv, &data, &tx_len, tx_burst, &vif_selected);
	if (ret <= 0) {
		wsm_release_tx_buffer(hw_priv, 1);
		if (WARN_ON(ret < 0)) {
			bes_err("bh get tx failed.\n");
			return ret; /* Error */
		}
		return 0; /* No work */
	}

	wsm = (struct wsm_hdr *)data;
	BUG_ON(tx_len < sizeof(*wsm));
	BUG_ON(__le16_to_cpu(wsm->len) != tx_len);
	tx_len += 4;

	atomic_add(1, &hw_priv->bh_tx);

	tx_len = hw_priv->sbus_ops->align_size(
		hw_priv->sbus_priv, tx_len);

	/* Check if not exceeding BES2600 capabilities */
	if (WARN_ON_ONCE(tx_len > EFFECTIVE_BUF_SIZE))
		bes_err("Write aligned len: %zu\n", tx_len);

	wsm->id &= __cpu_to_le16(0xffff ^ WSM_TX_SEQ(WSM_TX_SEQ_MAX));
	wsm->id |= __cpu_to_le16(WSM_TX_SEQ(hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)]));

	bes_devel("%s id:0x%04x seq:%d\n", __func__, wsm->id, hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)]);

#ifndef CONFIG_BES_SDIO_TX_MULTIPLE_ENABLE
	if (WARN_ON(bes2600_data_write(data, tx_len))) {
#else
	if (WARN_ON(hw_priv->sbus_ops->pipe_send(hw_priv->sbus_priv, 1, tx_len, data))) {
#endif
		bes_err("tx blew up, len %zu\n", tx_len);
		wsm_release_tx_buffer(hw_priv, 1);
		return -1; /* Error */
	}

	if (vif_selected != -1)
		hw_priv->hw_bufs_used_vif[vif_selected] ++;

	if (hw_priv->wsm_enable_wsm_dumps)
		print_hex_dump_bytes("--> ",
					 DUMP_PREFIX_NONE,
					 data,
					 __le16_to_cpu(wsm->len));

	wsm_txed(hw_priv, data);

	hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)] =
		(hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)] + 1) & WSM_TX_SEQ_MAX;
	bes2600_bh_inc_pending_count(hw_priv, WSM_TXRX_SEQ_IDX(wsm->id));

	/* Prove join actually left the host over SDIO */
	if ((__le16_to_cpu(wsm->id) & 0x0fff) == 0x000B)
		bes_pin("join 0x000B TX'd bufs=%d\n", hw_priv->hw_bufs_used);
	else if ((__le16_to_cpu(wsm->id) & 0x0fff) == 0x0004)
		bes_info("%s: data/mgmt 0x0004 TX'd len=%zu bufs=%d\n",
			 __func__, tx_len, hw_priv->hw_bufs_used);

	if (*tx_burst > 1) {
		bes2600_debug_tx_burst(hw_priv);
		return 1; /* Work remains */
	}

	return 0;
}

#ifdef KEY_FRAME_SW_RETRY

static inline bool
ieee80211_is_tcp_pkt(struct sk_buff *skb)
{

	if (!skb) {
		return false;
	}

	if (skb->protocol == cpu_to_be16(ETH_P_IP)) {
		struct iphdr *iph = (struct iphdr *)skb_network_header(skb);
		if (iph->protocol == IPPROTO_TCP) { // TCP
			bes_devel("################ %s line =%d.\n", __func__, __LINE__);
			return true;
		}
	}
	return false;
}


static int bes2600_need_retry_type(struct sk_buff *skb, int status)
{
	int ret = 0;
	if (!skb) {
		bes_devel("################ %s line =%d.\n", __func__, __LINE__);
		return -1;
	}

	if (skb->protocol == cpu_to_be16(ETH_P_IP)) {
		if (ieee80211_is_tcp_pkt(skb)) {
			ret = 1;
		}
	}
	if (status !=  WSM_STATUS_RETRY_EXCEEDED)
		ret = 0;
	return ret;
}

int bes2600_bh_sw_process(struct bes2600_common *hw_priv,
			 struct wsm_tx_confirm *tx_confirm)
{
	struct bes2600_txpriv *txpriv;
	struct sk_buff *skb = NULL;
	unsigned long timestamp = 0;
	struct bes2600_queue *queue;
	u8 queue_id, queue_gen;

#ifdef KEY_FRAME_SW_RETRY
	long delta_time;
#endif
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

	/* don't retry if the connection is already disconnected */
	queue_gen = bes2600_queue_get_generation(tx_confirm->packetID);
	if(queue_gen != queue->generation)
		return -1;

	bes2600_queue_get_skb_and_timestamp(queue, tx_confirm->packetID,
						&skb, &txpriv, &timestamp);
	if (skb == NULL) {
		bes_err("%s skb is NULL\n", __func__);
		return -1;
	}
	if (timestamp > jiffies)
		delta_time = jiffies + ((unsigned long)0xffffffff - timestamp);
	else
		delta_time =  jiffies - timestamp;
	bes2600_add_tx_delta_time(delta_time);
	bes2600_add_tx_ac_delta_time(queue_id, delta_time);

	if (bes2600_need_retry_type(skb, tx_confirm->status) == 0)
		return -1;

	if (delta_time > 1000)
		return -1;

	if (txpriv->retry_count < CW1200_MAX_SW_RETRY_CNT ) {
		struct bes2600_vif *priv =
		__cw12xx_hwpriv_to_vifpriv(hw_priv, txpriv->if_id);
		txpriv->retry_count++;

		bes2600_tx_status(priv,skb);

		bes2600_pwr_set_busy_event_with_timeout_async(
			hw_priv, BES_PWR_LOCK_ON_TX, BES_PWR_EVENT_TX_TIMEOUT);

		bes2600_sw_retry_requeue(hw_priv, queue, tx_confirm->packetID, true);
		return 0;
	} else {
		txpriv->retry_count = 0;
	}

	return -1;
}
#endif

void bes2600_bh_inc_pending_count(struct bes2600_common *hw_priv, int idx)
{
	struct timer_list *timer = (idx == 0) ? &hw_priv->lmac_mon_timer
										  : &hw_priv->mcu_mon_timer;

	if (hw_priv->wsm_tx_pending[idx]++ == 0) {
		bes_devel("start timer in tx, idx:%d\n", idx);
		mod_timer(timer, jiffies + 3 * HZ);
	}
}

void bes2600_bh_dec_pending_count(struct bes2600_common *hw_priv, int idx)
{
	struct timer_list *timer = (idx == 0) ? &hw_priv->lmac_mon_timer
										  : &hw_priv->mcu_mon_timer;

	if (hw_priv->wsm_tx_pending[idx] == 0) {
		bes_err("tx pending count error, idx:%d\n", idx);
		return;
	}

	if (--hw_priv->wsm_tx_pending[idx] == 0)
		/* Non-sync: timer handler must not deadlock on this path */
		timer_delete(timer);
	else
		mod_timer(timer, jiffies + 3 * HZ);
}

static bool bes2600_bh_wsm_cmd_in_flight(struct bes2600_common *hw_priv)
{
	u16 cmd;

	spin_lock(&hw_priv->wsm_cmd.lock);
	cmd = hw_priv->wsm_cmd.cmd;
	spin_unlock(&hw_priv->wsm_cmd.lock);
	/* 0xFFFF means idle after confirm path; 0 means never used */
	return cmd != 0 && cmd != 0xFFFF && !hw_priv->wsm_cmd.done;
}

void bes2600_bh_mcu_active_monitor(struct timer_list* t)
{
	struct bes2600_common *hw_priv = from_timer(hw_priv, t, mcu_mon_timer);

	/*
	 * Soft only from timer: never force_close.  While a WSM command is
	 * waiting for confirm (esp. join), do not kick BH into SDIO — that
	 * has hard-locked the tablet when the FW is silent.
	 */
	bes_err("link break between mcu and host, hw_buf_used:%d pending:%d (soft)\n",
		hw_priv->hw_bufs_used, hw_priv->wsm_tx_pending[1]);
	hw_priv->wsm_tx_pending[1] = 0;
	if (hw_priv->bus_stale || bes2600_bh_wsm_cmd_in_flight(hw_priv))
		return;
	atomic_inc(&hw_priv->bh_rx);
	wake_up(&hw_priv->bh_wq);
}

void bes2600_bh_lmac_active_monitor(struct timer_list* t)
{
	struct bes2600_common *hw_priv = from_timer(hw_priv, t, lmac_mon_timer);

	bes_err("link break between lmac and host, hw_buf_used:%d pending:%d (soft)\n",
		hw_priv->hw_bufs_used, hw_priv->wsm_tx_pending[0]);
	hw_priv->wsm_tx_pending[0] = 0;
	/*
	 * Join occupies hw_bufs_used=1 until 0x040B.  The old test marked
	 * stale whenever bufs>0, which killed join at 3s (LMAC timer) —
	 * log: 0x000B TX'd @71.281, link-break stale @74.343, -110.
	 * Leave in-flight WSM to wsm_cmd_send's own timeout.
	 */
	if (hw_priv->bus_stale || bes2600_bh_wsm_cmd_in_flight(hw_priv))
		return;
	if (hw_priv->hw_bufs_used > 0) {
		bes2600_bh_mark_bus_stale(hw_priv);
		return;
	}
	atomic_inc(&hw_priv->bh_rx);
	wake_up(&hw_priv->bh_wq);
}

#define BH_RX_CONT_LIMIT	3
#define BH_TX_CONT_LIMIT	20
static int bes2600_bh(struct bes2600_common *hw_priv)
{
	int rx, tx, term, suspend;
	int tx_allowed;
	int pending_tx = 0;
	int tx_burst;
	long status;
	int ret;

	int tx_cont = 0;
	int rx_cont = 0;

	bes_devel("BH: Starting RX processing\n");

	for (;;) {
		rx_cont = 0;
		tx_cont = 0;

		if (!hw_priv->hw_bufs_used &&
			!bes2600_pwr_device_is_idle(hw_priv) &&
			!atomic_read(&hw_priv->recent_scan) &&
			bes2600_chrdev_is_signal_mode()) {
			status = 5 * HZ;
		} else if (hw_priv->hw_bufs_used > 0) {
			/* Interrupt loss detection */
			status = 5 * HZ;
		} else {
			status = MAX_SCHEDULE_TIMEOUT;
		}

		status = wait_event_interruptible_timeout(hw_priv->bh_wq, ({
				rx = atomic_xchg(&hw_priv->bh_rx, 0);
				tx = atomic_xchg(&hw_priv->bh_tx, 0);
				term = atomic_xchg(&hw_priv->bh_term, 0);
				/*
				 * Only SUSPEND (not RESUME/RESUMED) enters the
				 * suspend path.  Any non-zero used to wake and
				 * incorrectly park BH in suspend wait.
				 */
				suspend = (!pending_tx &&
					   atomic_read(&hw_priv->bh_suspend) ==
						BES2600_BH_SUSPEND) ? 1 : 0;
				(rx || tx || term || suspend ||
				 atomic_read(&hw_priv->bh_error) ||
				 kthread_should_stop());
			}), status);

		/* Did an error occur? */
		if ((status < 0 && status != -ERESTARTSYS) ||
			term || atomic_read(&hw_priv->bh_error) ||
			kthread_should_stop()) {
			break;
		}
		if (!status) {	/* wait_event timed out */
			/*
			 * Outstanding host TX with no BH wake: do NOT poke SDIO
			 * RX (dwmmc hard LOCKUP).  Always soft-clear and idle.
			 * Log proved: usedbuf after join → thrash → freeze.
			 */
			if (hw_priv->hw_bufs_used > 0) {
				unsigned long since_rx =
					jiffies - hw_priv->rx_timestamp;

				bes_err("usedbuf:%u. rx:%u. tx:%u. since_rx=%u ms — soft clear, no RX poke\n",
					hw_priv->hw_bufs_used, rx, tx,
					jiffies_to_msecs(since_rx));
				sdio_work_debug(hw_priv->sbus_priv);
				bes2600_bh_mark_bus_stale(hw_priv);
				continue;
			}
			continue;
		} else if (suspend) {
			bes_devel("[BH] Device suspend.\n");

			atomic_set(&hw_priv->bh_suspend, BES2600_BH_SUSPENDED);
			wake_up(&hw_priv->bh_evt_wq);
			/*
			 * Exit on RESUME *or* RESUMED.  If a caller races and
			 * sets RESUMED without RESUME, waiting only for RESUME
			 * parks BH forever while bh_tx climbs (join hangs).
			 */
			status = wait_event_interruptible(hw_priv->bh_wq, ({
					int s = atomic_read(&hw_priv->bh_suspend);
					(s == BES2600_BH_RESUME ||
					 s == BES2600_BH_RESUMED ||
					 atomic_read(&hw_priv->bh_error) ||
					 kthread_should_stop());
				}));
			if (status < 0 || kthread_should_stop() ||
			    atomic_read(&hw_priv->bh_error)) {
				wiphy_err(hw_priv->hw->wiphy,
					  "Failed to wait for resume: %ld.\n",
					  status);
				break;
			}
			bes_devel("[BH] Device resume.\n");
			atomic_set(&hw_priv->bh_suspend, BES2600_BH_RESUMED);
			wake_up(&hw_priv->bh_evt_wq);
			atomic_add(1, &hw_priv->bh_rx);
			goto done;
		}

	rx:
		tx += pending_tx;
		pending_tx = 0;
#ifdef CONFIG_BES2600_WLAN_SPI
		if (rx) {
#endif
		ret = bes2600_bh_rx_helper(hw_priv, &tx);
		if (ret < 0) {
			bes_err("bes2600_bh_rx_helper fail (no force_close)\n");
			sdio_work_debug(hw_priv->sbus_priv);
			/* Keep BH alive; force_close has hard-locked the tablet */
		} else if (ret == 1) {
			rx = 1; // continue rx
			rx_cont++;
		}
		else
			rx = 0; // wait for a new rx event
		if (rx && (rx_cont < BH_RX_CONT_LIMIT))
			goto rx;
		rx_cont = 0;
#ifdef CONFIG_BES2600_WLAN_SPI
		}
#endif
	tx:
		if (1) {
			if (WARN_ON(hw_priv->hw_bufs_used >
				    hw_priv->wsm_caps.numInpChBufs)) {
				/* Stale accounting must not BUG the tablet */
				hw_priv->hw_bufs_used =
					hw_priv->wsm_caps.numInpChBufs;
			}
			tx_burst = hw_priv->wsm_caps.numInpChBufs - hw_priv->hw_bufs_used;
			tx_allowed = tx_burst > 0;

			if (!tx_allowed) {
				/*
				 * Buffers full: must not drop the wakeup (old
				 * code set pending_tx=0 after zeroing tx).
				 * Retry TX after next RX frees a slot.
				 */
				bes_devel("bh tx not allowed (used=%d num=%u).\n",
					  hw_priv->hw_bufs_used,
					  hw_priv->wsm_caps.numInpChBufs);
				pending_tx = 1;
				tx = 0;
				goto done_rx;
			}
			tx = 0;
			ret = bes2600_bh_tx_helper(hw_priv, &pending_tx, &tx_burst);
			if (ret < 0) {
				bes_err("bes2600_bh_tx_helper fail\n");
				sdio_work_debug(hw_priv->sbus_priv);
				break;
			}
			if (ret > 0) {
				/* More to transmit */
				tx_cont++;
				tx = ret;
			}

			if (tx && tx_cont < BH_TX_CONT_LIMIT)
				goto tx;
			tx_cont = 0;
#if 0
			/* Re-read ctrl reg */
			if (bes2600_bh_read_ctrl_reg(priv, &ctrl_reg))
				break;
#endif
		}

	done_rx:
		if (atomic_read(&hw_priv->bh_error))
			break;
		//if (ctrl_reg & ST90TDS_CONT_NEXT_LEN_MASK)
		if (rx)
			goto rx;
		if (tx)
			goto tx;

	done:
		/* Re-enable device interrupts */
		//hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
		//__bes2600_irq_enable(1);
		//hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);
		asm volatile ("nop");
	}

	/* Explicitly disable device interrupts */
	hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
	__bes2600_irq_enable(0);
	hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);

	if (!term) {
		bes_err("[BH] Fatal error, exiting.\n");
		sdio_work_debug(hw_priv->sbus_priv);
		atomic_set(&hw_priv->bh_error, 1);
		/* TODO: schedule_work(recovery) */
	}
	else
		bes_info("%s: At bottom, fail?\n", __func__);

	return 0;
}
