/*
 * Scan implementation for BES2600 mac80211 drivers
 *
 * Copyright (c) 2022, Bestechnic
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include "bes2600.h"
#include "scan.h"
#include "sta.h"
#include "pm.h"
#include "epta_request.h"
#include "bes_pwr.h"
#include "bes_chardev.h"

static void bes2600_scan_restart_delayed(struct bes2600_vif *priv);

static bool bes2600_scan_bus_unusable(struct bes2600_common *hw_priv)
{
	return hw_priv->bus_stale || bes2600_chrdev_is_bus_error();
}

#ifdef CONFIG_BES2600_TESTMODE
static int bes2600_advance_scan_start(struct bes2600_common *hw_priv)
{
	int tmo = 0;
	tmo += hw_priv->advanceScanElems.duration;
	bes2600_pwr_set_busy_event_with_timeout(hw_priv, BES_PWR_LOCK_ON_ADV_SCAN, tmo);
	/* Invoke Advance Scan Duration Timeout Handler */
	queue_delayed_work(hw_priv->workqueue,
		&hw_priv->advance_scan_timeout, tmo * HZ / 1000);
	return 0;
}
#endif

static void bes2600_remove_wps_p2p_ie(struct wsm_template_frame *frame)
{
	u8 *ies;
	u32 ies_len;
	u32 ie_len;
	u32 p2p_ie_len = 0;
	u32 wps_ie_len = 0;

	ies = &frame->skb->data[sizeof(struct ieee80211_hdr_3addr)];
	ies_len = frame->skb->len - sizeof(struct ieee80211_hdr_3addr);

	while (ies_len >= 6) {
		ie_len = ies[1] + 2;
		if ((ies[0] == WLAN_EID_VENDOR_SPECIFIC)
			&& (ies[2] == 0x00 && ies[3] == 0x50 && ies[4] == 0xf2 && ies[5] == 0x04)) {
			wps_ie_len = ie_len;
			memmove(ies, ies + ie_len, ies_len);
			ies_len -= ie_len;

		}
		else if ((ies[0] == WLAN_EID_VENDOR_SPECIFIC) &&
			(ies[2] == 0x50 && ies[3] == 0x6f && ies[4] == 0x9a && ies[5] == 0x09)) {
			p2p_ie_len = ie_len;
			memmove(ies, ies + ie_len, ies_len);
			ies_len -= ie_len;
		} else {
			ies += ie_len;
			ies_len -= ie_len;
		}
	}

	if (p2p_ie_len || wps_ie_len) {
		skb_trim(frame->skb, frame->skb->len - (p2p_ie_len + wps_ie_len));
	}
}

#ifdef CONFIG_BES2600_TESTMODE
static int bes2600_disable_filtering(struct bes2600_vif *priv)
{
	int ret = 0;
	bool bssid_filtering = 0;
	struct wsm_rx_filter rx_filter;
	struct wsm_beacon_filter_control bf_control;
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);

	/* RX Filter Disable */
	rx_filter.promiscuous = 0;
	rx_filter.bssid = 0;
	rx_filter.fcs = 0;
	rx_filter.probeResponder = 0;
	rx_filter.keepalive = 0;
	ret = wsm_set_rx_filter(hw_priv, &rx_filter,
			priv->if_id);

	/* Beacon Filter Disable */
	bf_control.enabled = __cpu_to_le32(0);
	bf_control.bcn_count = __cpu_to_le32(1);
	if (!ret)
		ret = wsm_beacon_filter_control(hw_priv, &bf_control,
					priv->if_id);

	/* BSSID Filter Disable */
	if (!ret)
		ret = wsm_set_bssid_filtering(hw_priv, bssid_filtering,
					 priv->if_id);

	return ret;
}
#endif

static int bes2600_scan_get_first_active_if(struct bes2600_common *hw_priv)
{
	int i = 0;
	struct bes2600_vif *vif;

	bes2600_for_each_vif(hw_priv, vif, i) {
		if (vif->join_status > BES2600_JOIN_STATUS_PASSIVE)
			return i;
	}

	return -1;
}

static int bes2600_scan_start(struct bes2600_vif *priv, struct wsm_scan *scan)
{
	int ret, i;
	int tmo = 5000;
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);

	if (hw_priv->scan_switch_if_id == -1 &&
		hw_priv->ht_info.channel_type > NL80211_CHAN_HT20 &&
		priv->if_id >= 0) {
		hw_priv->scan_switch_if_id = bes2600_scan_get_first_active_if(hw_priv);
		if(hw_priv->scan_switch_if_id >= 0) {
			struct wsm_switch_channel channel;
			channel.channelMode = 0 << 4;
			channel.channelSwitchCount = 0;
			channel.newChannelNumber = hw_priv->channel->hw_value;
			wsm_switch_channel(hw_priv, &channel, hw_priv->scan_switch_if_id);
			bes_devel("%s: scan start channel type %d num %d\n", __func__, hw_priv->ht_info.channel_type, channel.newChannelNumber);
		}
	}

	for (i = 0; i < scan->numOfChannels; ++i)
		tmo += scan->ch[i].maxChannelTime + 10;

	atomic_set(&hw_priv->scan.in_progress, 1);
	atomic_set(&hw_priv->recent_scan, 1);
	queue_delayed_work(hw_priv->workqueue, &hw_priv->scan.timeout, tmo * HZ / 1000);

	bes_info("%s: 0x0007 type=%u flags=0x%x n_ch=%u tmo=%d "
		 "join=%d assoc=%d home=%u\n",
		 __func__, scan->scanType, scan->scanFlags,
		 scan->numOfChannels, tmo, priv->join_status,
		 priv->vif && priv->vif->cfg.assoc,
		 hw_priv->channel ? hw_priv->channel->center_freq : 0);

	ret = wsm_scan(hw_priv, scan, 0);
	/*
	 * Idle: FW status 13 = scan refused while LMAC still joined.
	 * Reset+retry is the recovery.  Associated: never wsm_reset —
	 * that drops the join.  Let this scan fail and stay connected.
	 */
	if (ret && !hw_priv->bus_stale) {
		bool associated = priv->join_status == BES2600_JOIN_STATUS_STA &&
				  priv->vif && priv->vif->cfg.assoc;

		if (associated) {
			bes_warn("%s: scan 0x0007 failed %d while associated "
				 "— no reset\n", __func__, ret);
		} else {
			struct wsm_reset reset = {
				.reset_statistics = true,
			};

			bes_warn("%s: scan 0x0007 failed %d — reset and retry\n",
				 __func__, ret);
			if (wsm_reset(hw_priv, &reset, priv->if_id))
				bes_warn("%s: recovery reset failed\n", __func__);
			else
				ret = wsm_scan(hw_priv, scan, 0);
		}
	}
	if (unlikely(ret)) {
		atomic_set(&hw_priv->scan.in_progress, 0);
		cancel_delayed_work_sync(&hw_priv->scan.timeout);
		bes2600_scan_restart_delayed(priv);
	}
	return ret;
}

int bes2600_hw_scan(struct ieee80211_hw *hw,
		   struct ieee80211_vif *vif,
		   struct ieee80211_scan_request *hw_req)
{
	if (!hw_req) {
		bes_info("%s %d hw_req is NULL!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (!vif) {
		bes_err("NULL vif/bss_conf in hw_scan\n");
		dump_stack();
		return -EINVAL;
	}

	struct bes2600_common *hw_priv = hw->priv;
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(vif);
	if (!priv) {  // Or whatever indicates bad state
		bes_devel("Scan skipped: Invalid state (priv=%p, channel=%p)\n", priv, hw_priv->channel);
		//dump_stack();
		return -EBUSY;  // Fail scan request
	}

	bes_devel("Scan state: (priv=%p, channel=%p)\n", priv, hw_priv->channel);

	struct cfg80211_scan_request *req = &hw_req->req;
	struct wsm_template_frame frame = {
		.frame_type = WSM_FRAME_TYPE_PROBE_REQUEST,
	};
	int i;

	bes_devel("%s %d if_id:%d,num_channel:%d, n_ssids=%u.\n", __func__, __LINE__,
		  priv->if_id, req->n_channels, req->n_ssids);
/*
	for (size_t i = 0; i < req->n_channels; i++)
		bes_info("[SCAN] Channel %zu: %u MHz\n", i, req->channels[i]->center_freq);
*/
	/* Scan when P2P_GO corrupt firmware MiniAP mode */
	if (priv->join_status == BES2600_JOIN_STATUS_AP)
		return -EOPNOTSUPP;

	/*
	 * Associated scans use BACKGROUND + FORCE_BACKGROUND (set in
	 * scan_configure_channels).  Foreground 0x0007 while joined
	 * used to time out; do not wsm_reset on failure (see scan_start).
	 */

	if (bes2600_scan_bus_unusable(hw_priv)) {
		bes_warn("%s: skip scan (bus unusable)\n", __func__);
		return -EBUSY;
	}

	/* NM pre-connect scan: n_ssids=1 with empty SSID (wildcard).  We zero
	 * n_ssids for FW below, but must still hold JOIN awake until auth.
	 * Sticky: a later full-band passive scan must not clear the hold.
	 */
	if (req->n_ssids > 0 || req->n_channels == 1)
		hw_priv->scan.hold_join_awake = true;

	if (req->n_ssids == 1 && !req->ssids[0].ssid_len)
		req->n_ssids = 0;

	wiphy_dbg(hw->wiphy, "%s: Scan request for %d SSIDs.\n",
		  __func__, req->n_ssids);

	if (req->n_ssids > hw->wiphy->max_scan_ssids)
		return -EINVAL;

	bes2600_pwr_set_busy_event(hw_priv, BES_PWR_LOCK_ON_SCAN);

	if (hw_priv->scan.hold_join_awake)
		bes2600_pwr_request_awake(hw_priv, BES_PWR_LOCK_ON_JOIN);

	frame.skb = ieee80211_probereq_get(hw, priv->vif->addr, NULL, 0,
		req->ie_len);
	if (!frame.skb)
		return -ENOMEM;

	if (req->ie_len)
		skb_put_data(frame.skb, req->ie, req->ie_len);

	/* will be unlocked in bes2600_scan_work() */
	down(&hw_priv->scan.lock);
	down(&hw_priv->conf_lock);

	if (frame.skb) {
		int ret=0; /* Initialize ret */
		//if (priv->if_id == 0)
		//	bes2600_remove_wps_p2p_ie(&frame);
		ret = wsm_set_template_frame(hw_priv, &frame, 0);
		if (ret) {
			up(&hw_priv->conf_lock);
			up(&hw_priv->scan.lock);
			dev_kfree_skb(frame.skb);
			return ret;
		}
	}

	wsm_vif_lock_tx(priv);

	if (hw_priv->scan.req) {
		bes_err("%s: scan already in progress\n", __func__);
		wsm_unlock_tx(hw_priv);
		up(&hw_priv->conf_lock);
		up(&hw_priv->scan.lock);
		dev_kfree_skb(frame.skb);
		return -EBUSY;
	}
	hw_priv->scan.req = req;
	hw_priv->scan.n_ssids = 0;
	hw_priv->scan.status = 0;
	hw_priv->scan.begin = &req->channels[0];
	hw_priv->scan.curr = hw_priv->scan.begin;
	hw_priv->scan.end = &req->channels[req->n_channels];
	hw_priv->scan.output_power = hw_priv->output_power;
	hw_priv->scan.if_id = priv->if_id;
	/* TODO:COMBO: Populate BIT4 in scanflags to decide on which MAC
	 * address the SCAN request will be sent */
	bes_devel("%s %d if_id:%d,num_channel:%d.\n", __func__, __LINE__, priv->if_id, req->n_channels);

	for (i = 0; i < req->n_ssids; ++i) {
		struct wsm_ssid *dst =
			&hw_priv->scan.ssids[hw_priv->scan.n_ssids];
		if (req->ssids[i].ssid_len > sizeof(dst->ssid)) {
			bes_err("%s: SSID too long (%u)\n", __func__,
				req->ssids[i].ssid_len);
			continue;
		}
		memcpy(&dst->ssid[0], req->ssids[i].ssid,
			sizeof(dst->ssid));
		dst->length = req->ssids[i].ssid_len;
		++hw_priv->scan.n_ssids;
	}

	up(&hw_priv->conf_lock);

	if (frame.skb)
		dev_kfree_skb(frame.skb);

	bwifi_change_current_status(hw_priv, BWIFI_STATUS_SCANNING);
	queue_work(hw_priv->workqueue, &hw_priv->scan.work);

	return 0;
}

/*                            GROK 3                                  */

/* Initialize scan parameters */
static void bes2600_scan_init(struct bes2600_common *hw_priv, struct bes2600_vif *priv,
			     struct wsm_scan *scan)
{
    scan->scanType = WSM_SCAN_TYPE_FOREGROUND;
    scan->scanFlags = 0; /* TODO:COMBO */
    // scan.scanFlags = WSM_SCAN_FLAG_SPLIT_METHOD; /* TODO:COMBO */

    if (priv->if_id)
	scan->scanFlags |= WSM_FLAG_MAC_INSTANCE_1;
    else
	scan->scanFlags &= ~WSM_FLAG_MAC_INSTANCE_1;
}

/* Update scan flags based on VIF status */
static void bes2600_scan_update_vif_flags(struct bes2600_common *hw_priv, struct wsm_scan *scan)
{
    struct bes2600_vif *vif;
    int i;

    bes2600_for_each_vif(hw_priv, vif, i) {
		if (i == (CW12XX_MAX_VIFS - 1) || !vif)
	    	continue;
		if (vif->bss_loss_status > BES2600_BSS_LOSS_NONE)
	    	scan->scanFlags |= WSM_SCAN_FLAG_FORCE_BACKGROUND;
    }
}

/* Handle initial scan setup */
static bool bes2600_scan_setup(struct bes2600_common *hw_priv, struct bes2600_vif *priv,
			      bool first_run)
{
	/*
	 * Only toggle PS when we are a STA.  Idle/disconnect scans were
	 * sending 0x0010, then unjoin raced ("cmd in flight" ret 13)
	 * and scan timed out.
	 */
	if (priv->join_status == BES2600_JOIN_STATUS_STA &&
	    priv->vif && priv->vif->cfg.assoc) {
		/*
		 * Wake from PS so the scan can run.  Already ACTIVE
		 * (typical after join): skip the no-op 0x0010.
		 * Do not save/restore powersave_mode (0x81 FAST_PS) —
		 * that is the *desired* mac80211 mode, not the firmware
		 * mode, and 0x0010 FAST_PS after scan hangs confirm.
		 */
		if (priv->firmware_ps_mode.pmMode != WSM_PSM_ACTIVE) {
			struct wsm_set_pm pm = priv->powersave_mode;

			pm.pmMode = WSM_PSM_ACTIVE;
			bes2600_set_pm(priv, &pm);
		}
	}

    if (first_run) {
#ifdef CONFIG_BES2600_TESTMODE
	u16 advance_scan_req_channel = hw_priv->scan.begin[0]->hw_value;
	if (hw_priv->enable_advance_scan &&
	    (hw_priv->advanceScanElems.scanMode == BES2600_SCAN_MEASUREMENT_PASSIVE) &&
	    (priv->join_status == BES2600_JOIN_STATUS_STA) &&
	    (hw_priv->channel->hw_value == advance_scan_req_channel)) {
	    if (priv->powersave_mode.pmMode & WSM_PSM_PS) {
		struct wsm_set_pm pm = priv->powersave_mode;
		pm.pmMode = WSM_PSM_ACTIVE;
		wsm_set_pm(hw_priv, &pm, priv->if_id);
	    }
	    int ret = bes2600_disable_filtering(priv);
	    if (ret)
			wiphy_err(hw_priv->hw->wiphy,
			  "%s: Disable BSSID or Beacon filtering failed: %d.\n", __func__, ret);
	} else if (hw_priv->enable_advance_scan &&
		   (hw_priv->advanceScanElems.scanMode == BES2600_SCAN_MEASUREMENT_PASSIVE) &&
		   (priv->join_status == BES2600_JOIN_STATUS_STA)) {
	    if (!(priv->powersave_mode.pmMode & WSM_PSM_PS)) {
			struct wsm_set_pm pm = priv->powersave_mode;
			pm.pmMode = WSM_PSM_PS;
			bes2600_set_pm(priv, &pm);
	    }
	} else {
#endif
	    if (priv->join_status == BES2600_JOIN_STATUS_MONITOR) {
		/* Problematic: FW bug requires restarting p2p-dev mode after scan */
		bes2600_disable_listening(priv);
	    }
#ifdef CONFIG_BES2600_TESTMODE
	}
#endif
    }
    return true;
}

/* Complete scan and clean up */
static void bes2600_scan_finish(struct bes2600_common *hw_priv, struct bes2600_vif *priv,
			       bool aborted)
{
    struct cfg80211_scan_info info = { .aborted = aborted };

#ifdef CONFIG_BES2600_TESTMODE
    u16 advance_scan_req_channel = hw_priv->scan.begin[0]->hw_value;

    if (hw_priv->enable_advance_scan &&
		(hw_priv->advanceScanElems.scanMode == BES2600_SCAN_MEASUREMENT_PASSIVE) &&
		(priv->join_status == BES2600_JOIN_STATUS_STA) &&
		(hw_priv->channel->hw_value == advance_scan_req_channel)) {
		wsm_vif_lock_tx(priv);
		if (priv->powersave_mode.pmMode & WSM_PSM_PS)
		    wsm_set_pm(hw_priv, &priv->powersave_mode, priv->if_id);
		bes2600_update_filtering(priv);
    } else {
		if (!hw_priv->enable_advance_scan) {
#endif
	    if (hw_priv->scan.status >= 0 &&
		!bes2600_scan_bus_unusable(hw_priv) &&
		hw_priv->scan.output_power != hw_priv->output_power)
		wsm_set_output_power(hw_priv, hw_priv->output_power * 10,
				     priv->if_id ? 0 : 0);
#ifdef CONFIG_BES2600_TESTMODE
		}
    }
#endif

    if (hw_priv->scan.status < 0)
		wiphy_dbg(priv->hw->wiphy, "[SCAN] Scan failed (%d).\n", hw_priv->scan.status);
    else if (hw_priv->scan.req)
		wiphy_dbg(priv->hw->wiphy, "[SCAN] Scan completed.\n");
    else
		wiphy_dbg(priv->hw->wiphy, "[SCAN] Scan canceled.\n");

    /*
     * Failed/aborted scans already timed out on 0x0006.  Do not pile
     * 0x0029 / switch-channel on a sick bus (that WARN-looped into a
     * hard lockup).
     */
    if (hw_priv->scan.status >= 0 &&
	!bes2600_scan_bus_unusable(hw_priv)) {
	    if (priv->join_status == BES2600_JOIN_STATUS_STA) {
		if (hw_priv->channel->band != NL80211_BAND_2GHZ)
	    	bwifi_change_current_status(hw_priv, BWIFI_STATUS_GOT_IP_5G);
		else
	    	bwifi_change_current_status(hw_priv, BWIFI_STATUS_GOT_IP);
	    } else {
		bwifi_change_current_status(hw_priv, BWIFI_STATUS_IDLE);
	    }

	    bes_devel("%s %d %d.", __func__, __LINE__, hw_priv->ht_info.channel_type);
	    if (hw_priv->scan_switch_if_id >= 0) {
		struct wsm_switch_channel channel;
		channel.channelMode = hw_priv->ht_info.channel_type << 4;
		channel.channelSwitchCount = 0;
		channel.newChannelNumber = hw_priv->channel->hw_value;
		wsm_switch_channel(hw_priv, &channel, hw_priv->scan_switch_if_id);
		hw_priv->scan_switch_if_id = -1;
		bes_devel("scan done channel type %d num %d\n", hw_priv->ht_info.channel_type,
		  channel.newChannelNumber);
	    }
    } else {
	    hw_priv->scan_switch_if_id = -1;
    }

    hw_priv->scan.req = NULL;
    bes2600_scan_restart_delayed(priv);
#ifdef CONFIG_BES2600_TESTMODE
    hw_priv->enable_advance_scan = false;
#endif
    /* Refresh JOIN hold through scan completion → auth */
    if (hw_priv->scan.hold_join_awake)
        bes2600_pwr_request_awake(hw_priv, BES_PWR_LOCK_ON_JOIN);

    wsm_unlock_tx(hw_priv);
    bes2600_pwr_clear_busy_event(hw_priv, BES_PWR_LOCK_ON_SCAN);
    ieee80211_scan_completed(hw_priv->hw, &info);
    up(&hw_priv->scan.lock);
}

/* Configure scan channels and parameters */
static int bes2600_scan_configure_channels(struct bes2600_common *hw_priv, struct bes2600_vif *priv,
					  struct wsm_scan *scan)
{
    struct ieee80211_channel **it;
    int i;
    const u32 ProbeRequestTime = 2;
    const u32 ChannelRemainTime = 15;
    u32 maxChannelTime;
    u32 minChannelTime;

    struct ieee80211_channel *first = *hw_priv->scan.curr;
    for (it = hw_priv->scan.curr + 1, i = 1; it != hw_priv->scan.end &&
	 i < WSM_SCAN_MAX_NUM_OF_CHANNELS; ++it, ++i) {
	if ((*it)->band != first->band)
	    break;
    }
    scan->band = first->band;

    if (hw_priv->scan.req->no_cck)
	scan->maxTransmitRate = WSM_TRANSMIT_RATE_6;
    else
	scan->maxTransmitRate = WSM_TRANSMIT_RATE_1;

#ifdef CONFIG_BES2600_TESTMODE
    if (hw_priv->enable_advance_scan) {
	if (hw_priv->advanceScanElems.scanMode == BES2600_SCAN_MEASUREMENT_PASSIVE)
	    scan->numOfProbeRequests = 0;
	else
	    scan->numOfProbeRequests = 2;
    } else {
#endif
	scan->numOfProbeRequests = (first->flags & IEEE80211_CHAN_NO_IR) ? 0 : 2;
#ifdef CONFIG_BES2600_TESTMODE
    }
#endif
    scan->numOfSSIDs = hw_priv->scan.n_ssids;
    scan->ssids = &hw_priv->scan.ssids[0];
    scan->numOfChannels = it - hw_priv->scan.curr;
    scan->probeDelay = 100;

    if (priv->join_status == BES2600_JOIN_STATUS_STA) {
	scan->scanType = WSM_SCAN_TYPE_BACKGROUND;
	scan->scanFlags |= WSM_SCAN_FLAG_FORCE_BACKGROUND;
    }

    scan->ch = kzalloc((it - hw_priv->scan.curr) * sizeof(struct wsm_scan_ch), GFP_KERNEL);
    if (!scan->ch) {
	hw_priv->scan.status = -ENOMEM;
	return -ENOMEM;
    }

    maxChannelTime = (scan->numOfSSIDs * scan->numOfProbeRequests * ProbeRequestTime) +
		     ChannelRemainTime;
    maxChannelTime = (maxChannelTime < 35) ? 35 : maxChannelTime;

    if (scan->band == NL80211_BAND_2GHZ) {
	coex_calc_wifi_scan_time(&minChannelTime, &maxChannelTime);
    } else {
	minChannelTime = 100;
	maxChannelTime = 100;
    }

    for (i = 0; i < scan->numOfChannels; ++i) {
	scan->ch[i].number = hw_priv->scan.curr[i]->hw_value;
#ifdef CONFIG_BES2600_TESTMODE
	if (hw_priv->enable_advance_scan) {
	    scan->ch[i].minChannelTime = hw_priv->advanceScanElems.duration;
	    scan->ch[i].maxChannelTime = hw_priv->advanceScanElems.duration;
	} else {
#endif
	    scan->ch[i].minChannelTime = minChannelTime;
	    scan->ch[i].maxChannelTime = maxChannelTime;
#ifdef CONFIG_BES2600_TESTMODE
	}
#endif
    }

#ifdef CONFIG_BES2600_TESTMODE
    if (!hw_priv->enable_advance_scan) {
#endif
	if (!(first->flags & IEEE80211_CHAN_NO_IR) &&
	    hw_priv->scan.output_power != first->max_power) {
	    int pret;

	    hw_priv->scan.output_power = first->max_power;
	    pret = wsm_set_output_power(hw_priv,
					hw_priv->scan.output_power * 10,
					priv->if_id ? 0 : 0);
	    if (pret) {
		bes_warn("%s: set_output_power failed %d\n",
			 __func__, pret);
		kfree(scan->ch);
		scan->ch = NULL;
		hw_priv->scan.status = pret;
		return pret;
	    }
	}
#ifdef CONFIG_BES2600_TESTMODE
    }
#endif

    return 0;
}

/* Execute the scan */
static void bes2600_scan_execute(struct bes2600_common *hw_priv, struct bes2600_vif *priv,
				struct wsm_scan *scan)
{
#ifdef CONFIG_BES2600_TESTMODE
    u16 advance_scan_req_channel = hw_priv->scan.begin[0]->hw_value;
    if (hw_priv->enable_advance_scan &&
	(hw_priv->advanceScanElems.scanMode == BES2600_SCAN_MEASUREMENT_PASSIVE) &&
	(priv->join_status == BES2600_JOIN_STATUS_STA) &&
	(hw_priv->channel->hw_value == advance_scan_req_channel)) {
	hw_priv->scan.status = bes2600_advance_scan_start(hw_priv);
	wsm_unlock_tx(hw_priv);
    } else
#endif
    {
	hw_priv->scan.status = bes2600_scan_start(priv, scan);
    }

    kfree(scan->ch);

    if (hw_priv->scan.status) {
	bes_warn("%s: scan_start failed status=%d\n",
		 __func__, hw_priv->scan.status);
	hw_priv->scan.curr = hw_priv->scan.end;
	queue_work(hw_priv->workqueue, &hw_priv->scan.work);
	return;
    }

    hw_priv->scan.curr = hw_priv->scan.curr + scan->numOfChannels;
}

/* Main scan work function */
void bes2600_scan_work(struct work_struct *work)
{
    struct bes2600_common *hw_priv = container_of(work, struct bes2600_common, scan.work);
    struct bes2600_vif *priv;
    struct wsm_scan scan = {0};
    bool first_run;

    priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, hw_priv->scan.if_id);
    /* Problematic: Potential race if vif is removed, needs locking */
    if (!priv) {
		wiphy_warn(hw_priv->hw->wiphy, "[SCAN] interface removed, ignoring scan work\n");
		return;
    }

    down(&hw_priv->conf_lock);

    if (bes2600_scan_bus_unusable(hw_priv)) {
	hw_priv->scan.status = -EIO;
	bes2600_scan_finish(hw_priv, priv, true);
	up(&hw_priv->conf_lock);
	return;
    }

    bes2600_scan_init(hw_priv, priv, &scan);
    bes2600_scan_update_vif_flags(hw_priv, &scan);

    first_run = hw_priv->scan.begin == hw_priv->scan.curr &&
		hw_priv->scan.begin != hw_priv->scan.end;

    if (first_run) {
	/* Problematic: Firmware sensitive to scan during unassociated STA state */
		if (cancel_delayed_work_sync(&priv->join_timeout) > 0) {
	    	bes2600_join_timeout(&priv->join_timeout.work);
		}
    }

    if (!bes2600_scan_setup(hw_priv, priv, first_run)) {
		up(&hw_priv->conf_lock);
		return;
    }

    if (!hw_priv->scan.req || (hw_priv->scan.curr == hw_priv->scan.end)) {
		bes2600_scan_finish(hw_priv, priv, hw_priv->scan.status ? 1 : 0);
		up(&hw_priv->conf_lock);
		return;
    }

    if (bes2600_scan_configure_channels(hw_priv, priv, &scan)) {
		if (!hw_priv->scan.status)
			hw_priv->scan.status = -EIO;
		bes2600_scan_finish(hw_priv, priv, true);
		up(&hw_priv->conf_lock);
		return;
    }

    bes2600_scan_execute(hw_priv, priv, &scan);
    up(&hw_priv->conf_lock);
}

/*                            GROK 3                                  */

static void bes2600_scan_restart_delayed(struct bes2600_vif *priv)
{
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);

	if (priv->delayed_link_loss) {
		int tmo = priv->cqm_beacon_loss_count;

		if (hw_priv->scan.direct_probe)
			tmo = 0;

		priv->delayed_link_loss = 0;
		/* Restart beacon loss timer and requeue
		   BSS loss work. */
		wiphy_dbg(priv->hw->wiphy,
				"[CQM] Requeue BSS loss in %d "
				"beacons.\n", tmo);
		spin_lock(&priv->bss_loss_lock);
		priv->bss_loss_status = BES2600_BSS_LOSS_NONE;
		spin_unlock(&priv->bss_loss_lock);
		cancel_delayed_work_sync(&priv->bss_loss_work);
		queue_delayed_work(hw_priv->workqueue,
				&priv->bss_loss_work,
				tmo * HZ / 10);
	}

	/* FW bug: driver has to restart p2p-dev mode after scan. */
	if (priv->join_status == BES2600_JOIN_STATUS_MONITOR) {
		/*bes2600_enable_listening(priv);*/
		// bes_err("%s: unexpected path\n", __func__);
		bes_devel("scan complete join_status is monitor");
		bes2600_update_filtering(priv);
	}

	if (priv->delayed_unjoin) {
		priv->delayed_unjoin = false;
		if (queue_work(hw_priv->workqueue, &priv->unjoin_work) <= 0)
			wsm_unlock_tx(hw_priv);
	}
}

static void bes2600_scan_complete(struct bes2600_common *hw_priv, int if_id)
{
	struct bes2600_vif *priv;
	atomic_xchg(&hw_priv->recent_scan, 0);

	if (hw_priv->scan.direct_probe) {
		down(&hw_priv->conf_lock);
		priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, if_id);
		if (priv) {
			wiphy_dbg(priv->hw->wiphy, "[SCAN] Direct probe complete.\n");
			bes2600_scan_restart_delayed(priv);
		} else {
			wiphy_dbg(priv->hw->wiphy,
				"[SCAN] Direct probe complete without interface!\n");
		}
		up(&hw_priv->conf_lock);
		hw_priv->scan.direct_probe = 0;
		up(&hw_priv->scan.lock);
		wsm_unlock_tx(hw_priv);
	} else {
		bes2600_scan_work(&hw_priv->scan.work);
	}
}

void bes2600_scan_complete_cb(struct bes2600_common *hw_priv,
			struct wsm_scan_complete *arg)
{
	static int empty_scans = 0;
	struct bes2600_vif *priv = cw12xx_hwpriv_to_vifpriv(hw_priv,
					hw_priv->scan.if_id);

	if (unlikely(!priv)) {
		wiphy_err(hw_priv->hw->wiphy, "Scan complete ignored: NULL priv\n");
		return;
	}

	if (unlikely(priv->mode == NL80211_IFTYPE_UNSPECIFIED)) {
		/* STA is stopped. */
		spin_unlock(&priv->vif_lock);
		return;
	}
	spin_unlock(&priv->vif_lock);

	bes_info("%s: FW scan complete status=%d channels=%d join=%d\n",
		 __func__, arg->status, arg->numChannels,
		 priv->join_status);

	if (arg->status == 0 && arg->numChannels > 0) {
		empty_scans = 0;
	} else if (priv->join_status == BES2600_JOIN_STATUS_STA) {
		bes_warn("%s: associated scan complete status=%d channels=%d "
			 "(no reset)\n",
			 __func__, arg->status, arg->numChannels);
	} else {
		empty_scans++;
		if (empty_scans > 3) {
			bes_warn("%s: Too many empty scans - soft reset\n", __func__);
			struct wsm_reset reset = { .reset_statistics = true, .link_id = -1 };
			wsm_reset(hw_priv, &reset, -1);
			msleep(50);
			empty_scans = 0;
		}
	}

	wake_up(&hw_priv->scan.wq);

	/*
	 * Never wsm_set_pm from this BH/RX path.  Restoring FAST_PS
	 * (saved powersave_mode 0x81) blocked waiting for 0x0010
	 * confirm, so the scan-complete work never ran: host timed
	 * out ~6s later while ping still worked.  Stay ACTIVE.
	 */

	if (hw_priv->scan.status == -ETIMEDOUT)
		wiphy_warn(hw_priv->hw->wiphy,
			   "Scan timeout already occurred. Don't cancel work");
	if (hw_priv->scan.status != -ETIMEDOUT) {
		/*
		 * Never cancel_delayed_work_sync() from BH/RX: it can wait on
		 * scan.timeout while that work waits for conf_lock/BH → freeze.
		 * Non-blocking cancel + queue is enough; if work is already
		 * running it will see status=1 and finish cleanly.
		 */
		cancel_delayed_work(&hw_priv->scan.timeout);
		hw_priv->scan.status = 1;
		if (atomic_read(&hw_priv->scan.in_progress))
			queue_delayed_work(hw_priv->workqueue,
					   &hw_priv->scan.timeout, 0);
	}
}

void bes2600_scan_timeout(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, scan.timeout.work);
	if (likely(atomic_xchg(&hw_priv->scan.in_progress, 0))) {
		if (hw_priv->scan.status > 0)
			hw_priv->scan.status = 0;
		else if (!hw_priv->scan.status) {
			wiphy_warn(hw_priv->hw->wiphy,
				"Timeout waiting for scan "
				"complete notification.\n");
			hw_priv->scan.status = -ETIMEDOUT;
			hw_priv->scan.curr = hw_priv->scan.end;
			if (!bes2600_scan_bus_unusable(hw_priv))
				wsm_stop_scan(hw_priv,
					      hw_priv->scan.if_id ? 1 : 0);
		}
		bes2600_scan_complete(hw_priv, hw_priv->scan.if_id);
	}
}

#ifdef CONFIG_BES2600_TESTMODE
void bes2600_advance_scan_timeout(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, advance_scan_timeout.work);

	struct bes2600_vif *priv = cw12xx_hwpriv_to_vifpriv(hw_priv,
					hw_priv->scan.if_id);
	if (WARN_ON(!priv))
		return;
	spin_unlock(&priv->vif_lock);

	hw_priv->scan.status = 0;
	if (hw_priv->advanceScanElems.scanMode ==
		BES2600_SCAN_MEASUREMENT_PASSIVE) {
		/* Passive Scan on Serving Channel
		 * Timer Expire */
		bes2600_scan_complete(hw_priv, hw_priv->scan.if_id);
	} else {
		struct cfg80211_scan_info info = {
			.aborted = hw_priv->scan.status ? 1 : 0,
		};
		/* Active Scan on Serving Channel
		 * Timer Expire */
		down(&hw_priv->conf_lock);
		//wsm_lock_tx(priv);
		wsm_vif_lock_tx(priv);
		/* Once Duration is Over, enable filtering
		 * and Revert Back Power Save */
		if ((priv->powersave_mode.pmMode & WSM_PSM_PS))
			wsm_set_pm(hw_priv, &priv->powersave_mode,
				priv->if_id);
		hw_priv->scan.req = NULL;
		bes2600_update_filtering(priv);
		hw_priv->enable_advance_scan = false;
		wsm_unlock_tx(hw_priv);
		up(&hw_priv->conf_lock);
		if (hw_priv->scan.req) /* Check req before completion */
			ieee80211_scan_completed(hw_priv->hw, &info);
		up(&hw_priv->scan.lock);
	}
}
#endif

void bes2600_cancel_hw_scan(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(vif);
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);

	if(hw_priv->scan.if_id == priv->if_id) {
		bes_devel("cancel hw_scan on intf:%d\n", priv->if_id);

		down(&hw_priv->conf_lock);
		hw_priv->scan.req = NULL;
		up(&hw_priv->conf_lock);

		/* cancel scan operation */
		wsm_stop_scan(hw_priv, priv->if_id);

		/* wait scan operation end */
		down(&hw_priv->scan.lock);
		up(&hw_priv->scan.lock);
	}
}

void bes2600_probe_work(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, scan.probe_work.work);
	struct bes2600_vif *priv, *vif;
	u8 queueId = bes2600_queue_get_queue_id(hw_priv->pending_frame_id);
	struct bes2600_queue *queue = &hw_priv->tx_queue[queueId];
	const struct bes2600_txpriv *txpriv;
	struct wsm_tx *wsm;
	struct wsm_template_frame frame = {
		.frame_type = WSM_FRAME_TYPE_PROBE_REQUEST,
	};
	struct wsm_ssid ssids[1] = {{
		.length = 0,
	} };
	struct wsm_scan_ch ch[1] = {{
		.minChannelTime = 0,
		.maxChannelTime = 10,
	} };
	struct wsm_scan scan = {
		.scanType = WSM_SCAN_TYPE_FOREGROUND,
		.numOfProbeRequests = 2,
		.probeDelay = 0,
		.numOfChannels = 1,
		.ssids = ssids,
		.ch = ch,
	};
	u8 *ies;
	size_t ies_len;
	int ret = 1;
	int i;
	wiphy_info(hw_priv->hw->wiphy, "[SCAN] Direct probe work.\n");

	if (WARN_ON(queueId >= 4) || WARN_ON(!hw_priv->channel))
		return;

	down(&hw_priv->conf_lock);
	if (unlikely(down_trylock(&hw_priv->scan.lock))) {
		/* Scan is already in progress. Requeue self. */
		schedule();
		queue_delayed_work(hw_priv->workqueue,
					&hw_priv->scan.probe_work, HZ / 10);
		up(&hw_priv->conf_lock);
		return;
	}

	if (bes2600_queue_get_skb(queue, hw_priv->pending_frame_id,
			&frame.skb, &txpriv)) {
		up(&hw_priv->scan.lock);
		up(&hw_priv->conf_lock);
		wsm_unlock_tx(hw_priv);
		return;
	}
	priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, txpriv->if_id);
	if (!priv) {
		up(&hw_priv->scan.lock);
		up(&hw_priv->conf_lock);
		return;
	}
	wsm = (struct wsm_tx *)frame.skb->data;
	scan.maxTransmitRate = wsm->maxTxRate;
	scan.band = (hw_priv->channel->band == NL80211_BAND_5GHZ) ?
		WSM_PHY_BAND_5G : WSM_PHY_BAND_2_4G;
	if (priv->join_status == BES2600_JOIN_STATUS_STA) {
		scan.scanType = WSM_SCAN_TYPE_BACKGROUND;
		scan.scanFlags = WSM_SCAN_FLAG_FORCE_BACKGROUND;
		if (priv->if_id)
			scan.scanFlags |= WSM_FLAG_MAC_INSTANCE_1;
		else
			scan.scanFlags &= ~WSM_FLAG_MAC_INSTANCE_1;
	}
	bes2600_for_each_vif(hw_priv, vif, i) {
		if (!vif)
			continue;
		if (vif->bss_loss_status > BES2600_BSS_LOSS_NONE)
			scan.scanFlags |= WSM_SCAN_FLAG_FORCE_BACKGROUND;
	}
	ch[0].number = hw_priv->channel->hw_value;

	skb_pull(frame.skb, txpriv->offset);

	ies = &frame.skb->data[sizeof(struct ieee80211_hdr_3addr)];
	ies_len = frame.skb->len - sizeof(struct ieee80211_hdr_3addr);

	if (ies_len) {
		u8 *ssidie =
			(u8 *)cfg80211_find_ie(WLAN_EID_SSID, ies, ies_len);
		if (ssidie && ssidie[1] && ssidie[1] <= sizeof(ssids[0].ssid)) {
			u8 *nextie = &ssidie[2 + ssidie[1]];
			/* Remove SSID from the IE list. It has to be provided
			 * as a separate argument in bes2600_scan_start call */

			/* Store SSID locally */
			ssids[0].length = ssidie[1];
			memcpy(ssids[0].ssid, &ssidie[2], ssids[0].length);
			scan.numOfSSIDs = 1;

			/* Remove SSID from IE list */
			ssidie[1] = 0;
			memmove(&ssidie[2], nextie, &ies[ies_len] - nextie);
			skb_trim(frame.skb, frame.skb->len - ssids[0].length);
		}
	}

	if (priv->if_id == 0)
		bes2600_remove_wps_p2p_ie(&frame);

	/* FW bug: driver has to restart p2p-dev mode after scan */
	if (priv->join_status == BES2600_JOIN_STATUS_MONITOR) {
		bes_err("%s: unexpected path\n", __func__);
		/*bes2600_disable_listening(priv);*/
	}
	ret = bes_fail(wsm_set_template_frame(hw_priv, &frame,
				priv->if_id), "wsm_set_template_frame");

	hw_priv->scan.direct_probe = 1;
	hw_priv->scan.if_id = priv->if_id;
	if (!ret) {
		wsm_flush_tx(hw_priv);
		ret = bes_fail(bes2600_scan_start(priv, &scan), "bes2600_scan_start");
	}
	up(&hw_priv->conf_lock);

	skb_push(frame.skb, txpriv->offset);
	if (!ret)
		IEEE80211_SKB_CB(frame.skb)->flags |= IEEE80211_TX_STAT_ACK;
#ifdef CONFIG_BES2600_TESTMODE
	bes_fail(bes2600_queue_remove(hw_priv, queue,
			hw_priv->pending_frame_id), "bes2600_queue_remove");
#else
	bes_fail(bes2600_queue_remove(queue, hw_priv->pending_frame_id), "bes2600_queue_remove");
#endif

	if (ret) {
		hw_priv->scan.direct_probe = 0;
		up(&hw_priv->scan.lock);
		wsm_unlock_tx(hw_priv);
	}

	return;
}
