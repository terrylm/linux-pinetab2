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
#include <linux/list.h>
#include <linux/pm.h>
#include "bes2600.h"
#include "sbus.h"
#include "bes_pwr.h"
#include "sta.h"
#include "bes_chardev.h"
#include "bes_log.h"

static int bes2600_add_power_delay_event(struct bes2600_pwr_t *bes_pwr, u32 event, u32 timeout);
static int bes2600_add_async_timeout_power_delay_event(struct bes2600_pwr_t *bes_pwr,
						       u32 event, u32 timeout);
static bool bes2600_pwr_needs_hw_wake(struct bes2600_pwr_t *bes_pwr);

int bes2600_pwr_set_busy_event(struct bes2600_common *hw_priv, u32 event);
int bes2600_pwr_set_busy_event_async(struct bes2600_common *hw_priv, u32 event);
int bes2600_pwr_clear_busy_event(struct bes2600_common *hw_priv, u32 event);

static void bes2600_dump_power_busy_event(struct bes2600_pwr_t *bes_pwr, char *location)
{
	struct bes2600_pwr_event_t *item;
	char *dump_str;
	int i;
	const int buffer_len = 4096;
	struct {
		struct list_head *list;
		const char *prefix;
	} dump_lists[3] = {
		{ &bes_pwr->async_timeout_list, "async" },
		{ &bes_pwr->pending_event_list, "pending" },
		{ &bes_pwr->free_event_list, "free" },
	};

	bes_devel("power busy event dump at %s\n", location);
	if (!bes_pwr)
		return;

	dump_str = kzalloc(buffer_len, GFP_ATOMIC);
	if (!dump_str) {
		bes_err("%s alloc buffer failed\n", __func__);
		return;
	}

	for (i = 0; i < 3; i++) {
		int used_len = snprintf(dump_str, buffer_len, "%s list[",
					dump_lists[i].prefix);
		bool have = false;
		bool dump_ok = true;

		if (!list_empty(dump_lists[i].list)) {
			list_for_each_entry(item, dump_lists[i].list, link) {
				size_t remaining = buffer_len - used_len;
				int ret = snprintf(dump_str + used_len, remaining,
						   "%d(%d) ",
						   BES_PWR_EVENT_NUMBER(item->event),
						   (item->event >> 31));

				if (ret < 0 || (size_t)ret >= remaining) {
					dump_ok = false;
					break;
				}

				used_len += ret;
				have = true;
			}
		}

		if (dump_ok) {
			size_t remaining = buffer_len - used_len;

			if (remaining >= 3) {
				if (have)
					used_len--;
				snprintf(dump_str + used_len, buffer_len - used_len, "]\n");
			} else {
				dump_ok = false;
			}
		}

		if (!dump_ok)
			bes_err("buffer is not enough for dumping %s event\n",
				dump_lists[i].prefix);
		else
			bes_devel("%s", dump_str);
	}

	kfree(dump_str);
}

static char *bes2600_get_ps_mode_str(u8 mode)
{
	char *ps_mode_str = NULL;

	ps_mode_str = (mode == WSM_PSM_ACTIVE ? "WSM_PSM_ACTIVE" :
		mode == WSM_PSM_PS ? "WSM_PSM_PS" :
		mode == WSM_PSM_FAST_PS ? "WSM_PSM_FAST_PS" :
		"UNKNOWN");

	return ps_mode_str;
}

static char *bes2600_get_mac_str(char *buffer, u32 ip)
{
	sprintf(buffer, "%d.%d.%d.%d",
			(ip & 0xff),((ip >> 8) & 0xff),
			((ip >> 16) & 0xff), ((ip >> 24) & 0xff));

	return buffer;
}

static char *bes2600_get_pwr_busy_event_name(struct bes2600_pwr_event_t *item)
{
	char *name = NULL;

	switch(BES_PWR_EVENT_NUMBER(item->event)) {
		case BES_PWR_LOCK_ON_SCAN:      name = "SCAN";          break;
		case BES_PWR_LOCK_ON_JOIN:      name = "JOIN";          break;
		case BES_PWR_LOCK_ON_TX:        name = "TX";            break;
		case BES_PWR_LOCK_ON_RX:        name = "RX";            break;
		case BES_PWR_LOCK_ON_FLUSH:     name = "FLUSH";         break;
		case BES_PWR_LOCK_ON_ROC:       name = "ROC";           break;
		case BES_PWR_LOCK_ON_WSM_TX:    name = "WSM_TX";        break;
		case BES_PWR_LOCK_ON_WSM_OPER:  name = "WSM_OPER";      break;
		case BES_PWR_LOCK_ON_BSS_LOST:  name = "BSS_LOST";      break;
		case BES_PWR_LOCK_ON_GET_IP:    name = "GET_IP";        break;
		case BES_PWR_LOCK_ON_PS_ACTIVE: name = "PS_ACTIVE";     break;
		case BES_PWR_LOCK_ON_LMAC_RSP:  name = "LMAC_RSP";      break;
		case BES_PWR_LOCK_ON_AP:        name = "AP";            break;
		case BES_PWR_LOCK_ON_TEST_CMD:  name = "TEST_CMD";      break;
		case BES_PWR_LOCK_ON_MUL_REQ:   name = "MUL_REQ";       break;
		case BES_PWR_LOCK_ON_ADV_SCAN:  name = "ADV_SCAN";      break;
		case BES_PWR_LOCK_ON_DISCON:    name = "DISCON";        break;
		case BES_PWR_LOCK_ON_QUEUE_GC:  name = "QUEUE_GC";      break;
		case BES_PWR_LOCK_ON_AP_LP_BAD: name = "AP_LP_BAD";     break;
		default:                        name = "UNKNOW";        break;
	}

	return name;
}

static unsigned long bes2600_get_pwr_busy_event_timeout(struct bes2600_pwr_event_t *item)
{
	unsigned long timeout = 0;

	if(BES_PWR_IS_CONSTANT_EVENT(item->event))
		return 0;

	if(time_after(jiffies, item->timeout))
		return 0;

	timeout = (item->timeout >= jiffies) ?
		(item->timeout - jiffies) : (ULONG_MAX - jiffies + item->timeout);

	return timeout;
}

static bool bes2600_update_power_delay_events(struct bes2600_pwr_t *bes_pwr, unsigned long *timeout)
{
	struct bes2600_pwr_event_t *item = NULL, *temp = NULL;
	unsigned long max_timeout = 0;
	bool constant_event_exist = false;

	/* move event from async_timeout_list to pending_event_list */
	if(!list_empty(&bes_pwr->async_timeout_list)) {
		list_for_each_entry_safe(item, temp, &bes_pwr->async_timeout_list, link) {
			bes2600_add_power_delay_event(bes_pwr, item->event, item->delay);
			list_move_tail(&item->link, &bes_pwr->free_event_list);
		}
	}

	/* age power event and get max timeout */
	list_for_each_entry_safe(item, temp, &bes_pwr->pending_event_list, link) {
		if(BES_PWR_IS_CONSTANT_EVENT(item->event)) {
		       constant_event_exist = true;
		       continue;
		}
		if(time_after(jiffies, item->timeout)) {
			bes_devel("power busy event:0x%08x timeout\n", item->event);
			list_move_tail(&item->link, &bes_pwr->free_event_list);
		} else {
			if(max_timeout == 0) {
				max_timeout = item->timeout;
			} else {
				max_timeout = time_after(item->timeout, max_timeout)
						? item->timeout : max_timeout;
			}
		}
	}

	bes2600_dump_power_busy_event(bes_pwr, "refresh");

	if(timeout) {
		*timeout = max_timeout;
	}

	return constant_event_exist;
}

static int bes2600_add_async_timeout_power_delay_event(struct bes2600_pwr_t *bes_pwr,
						       u32 event, u32 timeout)
{
	struct bes2600_pwr_event_t *item = NULL;
	unsigned long max_timeout = 0;
	bool match = false;

	if (!bes_pwr)
		return -EINVAL;

	/* check if the event is already in async list */
	if (!list_empty(&bes_pwr->async_timeout_list)) {
		list_for_each_entry(item, &bes_pwr->async_timeout_list, link) {
			if (item->event == event) {
				match = true;
				break;
			}
		}
	}

	if (match && item) {
		item->delay = timeout;
		return 0;
	}

	if (list_empty(&bes_pwr->free_event_list)) {
		bes_devel("%s: free list empty, cleaning expired events\n", __func__);
		bes2600_update_power_delay_events(bes_pwr, &max_timeout);
	}

	if (list_empty(&bes_pwr->free_event_list)) {
		bes_err("%s: no free power event slots\n", __func__);
		return -EBUSY;
	}

	bes_devel("%s: add async event:%d(%d) timeout:%d\n",
		  __func__, BES_PWR_EVENT_NUMBER(event), event >> 31, timeout);
	item = list_first_entry(&bes_pwr->free_event_list,
				struct bes2600_pwr_event_t, link);
	list_move_tail(&item->link, &bes_pwr->async_timeout_list);
	item->event = event;
	item->delay = timeout;

	return 0;
}

/* Add (or update) a power delay event. Returns 0 on success, negative on failure. */
static int bes2600_add_power_delay_event(struct bes2600_pwr_t *bes_pwr, u32 event, u32 timeout)
{
	struct bes2600_pwr_event_t *item = NULL;
	unsigned long max_timeout = 0;
	bool match = false;

	if(!bes_pwr)
		return -EINVAL;

	/* Check if the event is already in the pending list */
	if(!list_empty(&bes_pwr->pending_event_list)) {
		list_for_each_entry(item, &bes_pwr->pending_event_list, link) {
			if(item->event == event) {
				match = true;
				break;
			}
		}
	}

	/* Update existing event or add a new one */
	if(match && item != NULL) {
		/* Duplicate event - just refresh timeout */
		item->timeout = jiffies + (timeout * HZ + HZ * BES2600_POWER_DOWN_DELAY) / 1000;
		bes_devel("%s: updated event:%d(%d) timeout:%d\n",
			  __func__, BES_PWR_EVENT_NUMBER(event), event >> 31, timeout);
		return 0;
	}

	/* No match - need a new slot */
	if (list_empty(&bes_pwr->free_event_list)) {
		bes_devel("%s: free list empty, cleaning expired events\n", __func__);
		bes2600_update_power_delay_events(bes_pwr, &max_timeout);
	}

	/* Still no free slot? This is a real error now */
	if (list_empty(&bes_pwr->free_event_list)) {
		bes_err("%s: no free power event slots (list empty after cleanup)\n", __func__);
		return -EBUSY;          /* or -ENOMEM, whichever fits your convention */
	}

	/* Take a free item and move it to pending */
	item = list_first_entry(&bes_pwr->free_event_list, struct bes2600_pwr_event_t, link);
	list_move_tail(&item->link, &bes_pwr->pending_event_list);

	item->event = event;
	item->timeout = jiffies + (timeout * HZ + HZ * BES2600_POWER_DOWN_DELAY) / 1000;

	bes_devel("%s: added event:%d(%d) timeout:%d\n",
		  __func__, BES_PWR_EVENT_NUMBER(event), event >> 31, timeout);
	bes2600_dump_power_busy_event(bes_pwr, "add new event");

	return 0;
}

static bool bes2600_del_pending_event(struct bes2600_pwr_t *bes_pwr, u32 event)
{
	struct bes2600_pwr_event_t *item = NULL, *temp = NULL;
	bool matched = false;

	list_for_each_entry_safe(item, temp, &bes_pwr->pending_event_list, link) {
		if(event == item->event) {
			matched = true;
			list_move_tail(&item->link, &bes_pwr->free_event_list);
			bes_devel("delete pending event:%d(%d)\n",
				BES_PWR_EVENT_NUMBER(event), event >> 31);
			break;
		}
	}

	bes2600_dump_power_busy_event(bes_pwr, "delete");

	return matched;
}

static void bes2600_flush_pending_events(struct bes2600_pwr_t *bes_pwr)
{
	struct bes2600_pwr_event_t *item = NULL, *temp = NULL;

	list_for_each_entry_safe(item, temp, &bes_pwr->pending_event_list, link) {
		list_move_tail(&item->link, &bes_pwr->free_event_list);
		bes_devel("flush pending event:%d(%d)\n",
				BES_PWR_EVENT_NUMBER(item->event), item->event >> 31);
	}
	bes2600_dump_power_busy_event(bes_pwr, "flush");
}

static void bes2600_trigger_power_delay_down(struct bes2600_pwr_t *bes_pwr, unsigned long max_timeout)
{
	/* get the gap of the two timestamp */
	max_timeout = (max_timeout >= jiffies) ?
		(max_timeout - jiffies) : (ULONG_MAX - jiffies + max_timeout);

	bes_devel("restart delayed work, timeout:%lu\n", max_timeout);
	queue_delayed_work(bes_pwr->hw_priv->workqueue, &bes_pwr->power_down_work, max_timeout);
}

static void bes2600_pwr_lock_tx(struct bes2600_common *hw_priv)
{
	unsigned long flags;

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	if(!hw_priv->bes_power.pending_lock) {
		hw_priv->bes_power.pending_lock = true;
		bes_devel("bes pwr lock tx\n");
		wsm_lock_tx_async(hw_priv);
	}
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
}

/* Unlock TX path from power management context */
static int bes2600_pwr_unlock_tx(struct bes2600_common *hw_priv)
{
	unsigned long flags;
	int ret = 0;

	if (!hw_priv) {
		bes_err("%s: hw_priv is NULL\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);

	if(hw_priv->bes_power.pending_lock) {
		hw_priv->bes_power.pending_lock = false;
		bes_devel("bes pwr unlock tx\n");

		/* Now that wsm_unlock_tx returns an error code, we check it */
		ret = wsm_unlock_tx(hw_priv);
		if (ret) {
			bes_err("%s: wsm_unlock_tx failed (ret=%d)\n", __func__, ret);
		}
	}

	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	return ret;   /* 0 = success, negative = error */
}

static void bes2600_pwr_call_enter_lp_cb(struct bes2600_common *hw_priv)
{
	struct bes2600_pwr_enter_cb_item *item = NULL;

	mutex_lock(&hw_priv->bes_power.pwr_cb_mutex);
	if (!list_empty(&hw_priv->bes_power.enter_cb_list)) {
		list_for_each_entry(item, &hw_priv->bes_power.enter_cb_list, link) {
			if(item->cb != NULL)
				item->cb(hw_priv);
		}
	}
	mutex_unlock(&hw_priv->bes_power.pwr_cb_mutex);
}

static void bes2600_pwr_call_exit_lp_cb(struct bes2600_common *hw_priv)
{
	struct bes2600_pwr_exit_cb_item *item = NULL;

	mutex_lock(&hw_priv->bes_power.pwr_cb_mutex);
	if (!list_empty(&hw_priv->bes_power.exit_cb_list)) {
		list_for_each_entry(item, &hw_priv->bes_power.exit_cb_list, link) {
			if(item->cb != NULL)
				item->cb(hw_priv);
		}
	}
	mutex_unlock(&hw_priv->bes_power.pwr_cb_mutex);
}

static void bes2600_pwr_delete_all_cb(struct bes2600_common *hw_priv)
{
	struct bes2600_pwr_enter_cb_item *item = NULL, *temp = NULL;
	struct bes2600_pwr_exit_cb_item *item1 = NULL, *temp1 = NULL;

	mutex_lock(&hw_priv->bes_power.pwr_cb_mutex);

	/* delete all cb in enter_cb_list */
	list_for_each_entry_safe(item, temp, &hw_priv->bes_power.enter_cb_list, link) {
		list_del(&item->link);
		kfree(item);
	}

	/* delete all cb in exit_cb_list */
	list_for_each_entry_safe(item1, temp1, &hw_priv->bes_power.exit_cb_list, link) {
		list_del(&item1->link);
		kfree(item1);
	}

	mutex_unlock(&hw_priv->bes_power.pwr_cb_mutex);
}

static void bes2600_pwr_device_enter_lp_mode(struct bes2600_common *hw_priv)
{
	int ret = 0;
	struct wsm_operational_mode mode = {
		.power_mode = wsm_power_mode_quiescent,
		.disableMoreFlagUsage = true,
	};

	if (hw_priv->in_reset) {
		bes_info("Skipped power mode op during reset\n");
		return;
	}

	/*
	 * After join -110 the bus is often silent.  Do not send quiescent (or
	 * any WSM): it times out, re-arms mon, and hard-locks the tablet.
	 */
	if (hw_priv->bus_stale || hw_priv->hw_bufs_used > 0) {
		bes_warn("%s: skip sleep (bus_stale=%d bufs=%d)\n",
			 __func__, hw_priv->bus_stale, hw_priv->hw_bufs_used);
		return;
	}
	if (time_is_before_jiffies(hw_priv->rx_timestamp + 2 * HZ)) {
		bes_warn("%s: skip sleep, no RX for %u ms\n",
			 __func__,
			 jiffies_to_msecs(jiffies - hw_priv->rx_timestamp));
		return;
	}

	bes_devel("host unlock lmac\n");
	ret = wsm_set_operational_mode(hw_priv, &mode, 0);
	if (ret) {
		/*
		 * Do not mark mcu_slept / pull GPIO down / deactive: the
		 * quiescent command never completed, so the device is still
		 * in an unknown state.  Stay "awake" from the host POV.
		 */
		bes_err("%s, set operation mode fail — abort sleep\n",
			__func__);
		return;
	}

	/* Only mark slept after WSM confirms quiescent */
	hw_priv->bes_power.mcu_slept = true;

	/* wait other module to finish work */
	bes2600_pwr_call_enter_lp_cb(hw_priv);

	if (hw_priv->sbus_ops->sbus_deactive) {
		ret = hw_priv->sbus_ops->sbus_deactive(hw_priv->sbus_priv,
						       SUBSYSTEM_MCU);
		if (ret)
			bes_err("%s, deactive mcu fail\n", __func__);
	}

	if (hw_priv->sbus_ops->gpio_sleep)
		hw_priv->sbus_ops->gpio_sleep(hw_priv->sbus_priv);
	hw_priv->bes_power.hw_awake = false;
	bes_devel("device enter sleep\n");
}

static int bes2600_pwr_enter_lp_mode(struct bes2600_common *hw_priv)
{
	int i = 0;
	struct bes2600_vif *priv;
	int ret = 0;
	char ip_str[20];
	unsigned long status = 0;

	bes_devel("%s: enter\n", __func__);

	/* set interface low power configuration */
	bes2600_for_each_vif(hw_priv, priv, i) {
		if (i == (CW12XX_MAX_VIFS - 1))
			continue;

		if (!priv)
			continue;

		if (priv->join_status == BES2600_JOIN_STATUS_STA &&
		    priv->bss_params.aid &&
		    priv->setbssparams_done) {
			/* enable arp filter */
			if (priv->filter4.enable) {
				bes_devel("%s, arp filter, enable:%d addr:%s\n",
					__func__, priv->filter4.enable, bes2600_get_mac_str(ip_str, priv->filter4.ipv4Address[0]));
				ret = wsm_set_arp_ipv4_filter(hw_priv, &priv->filter4, priv->if_id);
				if (ret)
					bes_err("%s, set arp filter failed\n", __func__);
			}

			/* skip beacon receive if applications don't have muticast service */
			if(priv->join_dtim_period && !priv->has_multicast_subscription) {
				unsigned listen_interval = 1;
				if(priv->join_dtim_period >= CONFIG_BES2600_LISTEN_INTERVAL) {
					listen_interval = priv->join_dtim_period;
				} else {
					listen_interval = CONFIG_BES2600_LISTEN_INTERVAL / priv->join_dtim_period * priv->join_dtim_period;
				}
				ret = wsm_set_beacon_wakeup_period(hw_priv, 1, listen_interval, priv->if_id);
				if (ret)
					bes_err("%s, set wakeup period failed\n", __func__);
			}

			/* Set Enable Broadcast Address Filter */
			priv->broadcast_filter.action_mode = priv->filter4.enable ? 
						WSM_FILTER_ACTION_FILTER_OUT : WSM_FILTER_ACTION_IGNORE;
			if (priv->join_status == BES2600_JOIN_STATUS_AP)
				priv->broadcast_filter.address_mode = WSM_FILTER_ADDR_MODE_A3;
			ret = bes2600_set_macaddrfilter(hw_priv, priv, (u8 *)&priv->broadcast_filter);
			if (ret)
				bes_err("%s, set bc filter failed\n", __func__);

			/* enter low power mode */
			if(!hw_priv->bes_power.ap_lp_bad) {
				bes_devel("%s, psMode:%s, fastPsmIdlePeriod:%d apPsmChangePeriod:%d minAutoPsPollPeriod:%d\n",
						__func__, bes2600_get_ps_mode_str(priv->powersave_mode.pmMode), priv->powersave_mode.fastPsmIdlePeriod,
						priv->powersave_mode.apPsmChangePeriod, priv->powersave_mode.minAutoPsPollPeriod);
				/*
				 * powersave_mode is often FAST_PS (0x81) while
				 * firmware is still ACTIVE.  Sending that
				 * 0x0010 hangs confirm (associated scan and
				 * WPA stop).  Stay ACTIVE until FAST_PS is
				 * proven on this FW.
				 */
				if (priv->powersave_mode.pmMode & WSM_PSM_PS) {
					bes_info("%s: skip 0x0010 FAST_PS/PS "
						 "(fw=0x%x desired=0x%x)\n",
						 __func__,
						 priv->firmware_ps_mode.pmMode,
						 priv->powersave_mode.pmMode);
				} else {
					atomic_set(&hw_priv->bes_power.pm_set_in_process, 1);
					ret = bes2600_set_pm(priv, &priv->powersave_mode);
					if (ret) {
						atomic_set(&hw_priv->bes_power.pm_set_in_process, 0);
						bes_err("%s, set operation mode fail\n", __func__);
					}

					/* wait power save mode changed indication */
					status = wait_for_completion_timeout(&hw_priv->bes_power.pm_enter_cmpl, 5 * HZ);
					atomic_set(&hw_priv->bes_power.pm_set_in_process, 0);
					reinit_completion(&hw_priv->bes_power.pm_enter_cmpl);
					if (!status)
						bes_err("%s, wait pm ind timeout\n", __func__);
				}
			} else {
				bes_devel("skip enter lp mode\n");
			}
		}
	}

	/* set device low power configuration */
	bes_devel("%s: calling device_enter_lp_mode\n", __func__);
	bes2600_pwr_device_enter_lp_mode(hw_priv);
	bes_devel("%s: exit ret=%d\n", __func__, ret);

	return ret;
}

static void bes2600_pwr_device_exit_lp_mode(struct bes2600_common *hw_priv)
{
	int ret = 0;
	struct wsm_operational_mode mode = {
		.power_mode = wsm_power_mode_active,
		.disableMoreFlagUsage = true,
	};

    if (hw_priv->in_reset) {
        bes_info("Skipped exit_lp_mode during reset\n");
        return;
    }

	bes_devel("host lock lmac\n");
	if(hw_priv->sbus_ops->gpio_wake)
		hw_priv->sbus_ops->gpio_wake(hw_priv->sbus_priv);

	if(hw_priv->sbus_ops->sbus_active) {
		ret = hw_priv->sbus_ops->sbus_active(hw_priv->sbus_priv, SUBSYSTEM_MCU);
		if (ret) {
			bes_err("%s, active mcu fail\n", __func__);
			/*
			 * Do not send 0x0006 operational-mode on a bus that
			 * just failed CMD52.  Nested WSM from wsm_cmd_lock
			 * piled 6s timeouts until force_close hard-locked.
			 */
			return;
		}
	}

	/* Let MCU/SDIO settle before first WSM command after sleep */
	usleep_range(20000, 40000);

	/* Re-sending operational mode while already active routinely times out
	 * (~WSM_CMD_TIMEOUT) with TX locked during join.  After MCU sdio sleep
	 * (mcu_slept) we must restore WSM active mode even if hw_awake is stale.
	 */
	if (hw_priv->bes_power.hw_awake && !hw_priv->bes_power.mcu_slept) {
		bes_devel("%s: hw_awake=1, skip redundant operational mode\n", __func__);
		return;
	}

	ret = wsm_set_operational_mode(hw_priv, &mode, 0);
	if (ret) {
		bes_err("%s, set operation mode fail\n", __func__);
		hw_priv->bes_power.hw_awake = false;
	} else {
		hw_priv->bes_power.hw_awake = true;
		hw_priv->bes_power.mcu_slept = false;
	}
	bes_devel("device exit sleep (hw_awake=%d)\n", hw_priv->bes_power.hw_awake);
}

static int bes2600_pwr_exit_lp_mode(struct bes2600_common *hw_priv)
{
	int i = 0, ret = 0;
	struct bes2600_vif *priv;
	struct wsm_arp_ipv4_filter filter;
	struct wsm_set_pm pm;
	char ip_str[20];

	/* set device low power configuration */
	bes2600_pwr_device_exit_lp_mode(hw_priv);

	/* set interface low power configutation */
	bes2600_for_each_vif(hw_priv, priv, i) {
		if (i == (CW12XX_MAX_VIFS - 1))
			continue;

		if (!priv)
			continue;

		if (priv->join_status == BES2600_JOIN_STATUS_STA &&
		    priv->bss_params.aid &&
		    priv->setbssparams_done) {
			/* disable arp filter */
			if (priv->filter4.enable) {
				filter = priv->filter4;
				filter.enable = false;
				bes_devel("%s, arp filter, enable:%d addr:%s\n",
					__func__, filter.enable, bes2600_get_mac_str(ip_str, filter.ipv4Address[0]));
				ret = wsm_set_arp_ipv4_filter(hw_priv, &filter, priv->if_id);
				if (ret)
					bes_err("%s, set arp filter failed\n", __func__);
			}

			/* set wakeup perioid */
			wsm_set_beacon_wakeup_period(hw_priv, priv->join_dtim_period, 0, priv->if_id);
			if (ret)
				bes_err("%s, set wakeup period failed\n", __func__);

			/* Set Enable Broadcast Address Filter */
			priv->broadcast_filter.action_mode = WSM_FILTER_ACTION_IGNORE;
			if (priv->join_status == BES2600_JOIN_STATUS_AP)
				priv->broadcast_filter.address_mode = WSM_FILTER_ADDR_MODE_NONE;
			bes2600_set_macaddrfilter(hw_priv, priv, (u8 *)&priv->broadcast_filter);
			if (ret)
				bes_err("%s, set bc filter failed\n", __func__);

			/* exit low power mode */
			if(!hw_priv->bes_power.ap_lp_bad) {
				pm = priv->powersave_mode;
				pm.pmMode = WSM_PSM_ACTIVE;
				bes_devel("%s, psMode:%s, fastPsmIdlePeriod:%d apPsmChangePeriod:%d minAutoPsPollPeriod:%d\n",
						__func__, bes2600_get_ps_mode_str(pm.pmMode), pm.fastPsmIdlePeriod, pm.apPsmChangePeriod, pm.minAutoPsPollPeriod);
				ret = bes2600_set_pm(priv, &pm);
				if (ret)
					bes_err("%s, set operation mode fail\n", __func__);
			} else {
				bes_devel("skip exit lp mode\n");
			}
		}
	}

	/* call all exit lower power callback */
	bes2600_pwr_call_exit_lp_cb(hw_priv);

	return ret;
}

static void bes2600_pwr_unlock_device(struct bes2600_common *hw_priv)
{
	unsigned long flags;
	unsigned long max_timeout = 0;
	bool constant_event_exist = false;

	bes_devel("%s: enter (going low-power)\n", __func__);

	/* set device to low power mode */
	mutex_lock(&hw_priv->bes_power.pwr_mutex);

	bes2600_pwr_lock_tx(hw_priv);

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	if (hw_priv->bes_power.power_state == POWER_DOWN_STATE_LOCKED) {
		hw_priv->bes_power.power_state = POWER_DOWN_STATE_UNLOCKING;
		hw_priv->bes_power.power_down_task = current;
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

		/* Busy events may have arrived after power_down_work checked */
		spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
		constant_event_exist = bes2600_update_power_delay_events(
			&hw_priv->bes_power, &max_timeout);
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

		if (!constant_event_exist && max_timeout == 0)
			bes2600_pwr_enter_lp_mode(hw_priv);

		spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
		if (constant_event_exist || max_timeout > 0) {
			hw_priv->bes_power.power_state = POWER_DOWN_STATE_LOCKED;
		} else {
			hw_priv->bes_power.power_state = POWER_DOWN_STATE_UNLOCKED;
		}
		hw_priv->bes_power.power_down_task = NULL;
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

		/* Sleep aborted: release the TX lock taken above */
		if (constant_event_exist || max_timeout > 0)
			bes2600_pwr_unlock_tx(hw_priv);
	} else {
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
	}
	mutex_unlock(&hw_priv->bes_power.pwr_mutex);
	bes_devel("%s: exit pstate=%d\n", __func__,
		 hw_priv->bes_power.power_state);

        /* allow system to enter suspend mode */
        // pm_relax(hw_priv->pdev);
}

static bool bes2600_pwr_needs_hw_wake(struct bes2600_pwr_t *bes_pwr)
{
	return bes_pwr->power_state == POWER_DOWN_STATE_UNLOCKED ||
	       bes_pwr->power_state == POWER_DOWN_STATE_UNLOCKING ||
	       bes_pwr->mcu_slept ||
	       (bes_pwr->power_state == POWER_DOWN_STATE_LOCKED &&
		!bes_pwr->hw_awake);
}

/* Lock the device out of low-power mode */
static int bes2600_pwr_lock_device(struct bes2600_common *hw_priv)
{
	unsigned long flags;
	int ret = 0;

	if (!hw_priv) {
		bes_err("%s: hw_priv is NULL\n", __func__);
		return -EINVAL;
	}

	/* prevent system from entering suspend mode */
	// pm_stay_awake(hw_priv->pdev);

	/* wakeup device from low power mode */
	mutex_lock(&hw_priv->bes_power.pwr_mutex);
	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);

	if (hw_priv->bes_power.power_state == POWER_DOWN_STATE_UNLOCKED) {
		hw_priv->bes_power.power_state = POWER_DOWN_STATE_LOCKING;
		hw_priv->bes_power.power_up_task = current;
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

		/* This is the only call that can actually fail */
		ret = bes2600_pwr_exit_lp_mode(hw_priv);
		if (ret) {
			bes_err("%s: bes2600_pwr_exit_lp_mode FAILED (ret=%d)\n",
				__func__, ret);
			spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
			hw_priv->bes_power.power_up_task = NULL;
			hw_priv->bes_power.power_state = POWER_DOWN_STATE_UNLOCKED;
			spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
			mutex_unlock(&hw_priv->bes_power.pwr_mutex);
			return ret;
		}

		spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
		hw_priv->bes_power.power_up_task = NULL;
		hw_priv->bes_power.power_state = POWER_DOWN_STATE_LOCKED;
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
	} else if (hw_priv->bes_power.power_state == POWER_DOWN_STATE_LOCKED &&
		   (!hw_priv->bes_power.hw_awake ||
		    hw_priv->bes_power.mcu_slept)) {
		/* PM busy but MCU slept; mark power_up_task so WSM_TX nest is safe */
		hw_priv->bes_power.power_up_task = current;
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
		bes_devel("%s: re-wake LOCKED-but-asleep MCU\n", __func__);
		bes2600_pwr_device_exit_lp_mode(hw_priv);
		spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
		hw_priv->bes_power.power_up_task = NULL;
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
	} else {
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
	}

	/* unlock TX path */
	ret = bes2600_pwr_unlock_tx(hw_priv);
	if (ret)
		bes_err("%s: bes2600_pwr_unlock_tx FAILED (ret=%d)\n", __func__, ret);

	mutex_unlock(&hw_priv->bes_power.pwr_mutex);

	return ret;   /* 0 = success, negative = error */
}

static void bes2600_pwr_trigger_delayed_work(struct bes2600_common *hw_priv)
{
	unsigned long max_timeout = 0;
	bool constant_event_exist = false;
	unsigned long flags;

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	constant_event_exist = bes2600_update_power_delay_events(&hw_priv->bes_power, &max_timeout);

	if(!constant_event_exist && max_timeout > 0) {
		bes2600_trigger_power_delay_down(&hw_priv->bes_power, max_timeout);
	}
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
}

static void bes2600_power_down_work(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, bes_power.power_down_work.work);

        unsigned long max_timeout = 0;
        bool constant_event_exist = false;
        unsigned long flags;

	bes_devel("%s: enter pstate=%d\n", __func__,
		 hw_priv->bes_power.power_state);

        spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
        constant_event_exist = bes2600_update_power_delay_events(&hw_priv->bes_power, &max_timeout);
        spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

        if (!constant_event_exist && max_timeout == 0) {
                /* no pending event, unlock device */
		bes_devel("%s: no pending event — unlock device\n", __func__);
                bes2600_pwr_unlock_device(hw_priv);
		bes_devel("%s: unlock device returned pstate=%d\n", __func__,
			 hw_priv->bes_power.power_state);
                wake_up(&hw_priv->bes_power.dev_lp_wq);
        } else {
		bes_devel("%s: keep awake (const=%d max_tmo=%lu) pstate=%d\n",
			 __func__, constant_event_exist, max_timeout,
			 hw_priv->bes_power.power_state);
                /* Wake if hardware is asleep */
                if (bes2600_pwr_needs_hw_wake(&hw_priv->bes_power))
                        bes2600_pwr_lock_device(hw_priv);

                /* only have delayed power busy event, restart delayed work */
                if (!constant_event_exist && max_timeout > 0) {
                        spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
                        bes2600_trigger_power_delay_down(&hw_priv->bes_power, max_timeout);
                        spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
                }
        }
	bes_devel("%s: exit\n", __func__);
}

static void bes2600_power_async_work(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, bes_power.power_async_work);

	bes2600_power_down_work(&hw_priv->bes_power.power_down_work.work);
}

static void bes2600_power_mcu_down_work(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, bes_power.power_mcu_down_work);
	unsigned long max_timeout = 0;
	bool constant_event_exist = false;
	unsigned long flags;
	enum power_down_state power_state;
	int ret = 0;

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	constant_event_exist = bes2600_update_power_delay_events(&hw_priv->bes_power, &max_timeout);
	power_state = hw_priv->bes_power.power_state;
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	if(!constant_event_exist &&
	   max_timeout == 0 &&
	   power_state == POWER_DOWN_STATE_UNLOCKED) {
		bes_devel("mcu sleep directly");
		mutex_lock(&hw_priv->bes_power.pwr_mutex);

		if(hw_priv->sbus_ops->gpio_wake)
			hw_priv->sbus_ops->gpio_wake(hw_priv->sbus_priv);

		if(hw_priv->sbus_ops->sbus_active) {
			ret = hw_priv->sbus_ops->sbus_active(hw_priv->sbus_priv, SUBSYSTEM_MCU);
			if (ret)
				bes_err("%s, active mcu fail\n", __func__);
		}

		 if(hw_priv->sbus_ops->sbus_deactive) {
			ret = hw_priv->sbus_ops->sbus_deactive(hw_priv->sbus_priv, SUBSYSTEM_MCU);
			if (ret)
				bes_err("%s, deactive mcu fail\n", __func__);
		}

		if(hw_priv->sbus_ops->gpio_sleep)
			hw_priv->sbus_ops->gpio_sleep(hw_priv->sbus_priv);
		hw_priv->bes_power.hw_awake = false;
		hw_priv->bes_power.mcu_slept = true;
		mutex_unlock(&hw_priv->bes_power.pwr_mutex);
	}
}

void bes2600_pwr_init(struct bes2600_common *hw_priv)
{
        int i = 0;

        hw_priv->bes_power.power_state = POWER_DOWN_STATE_UNLOCKED;
        hw_priv->bes_power.hw_priv = hw_priv;
        spin_lock_init(&hw_priv->bes_power.pwr_lock);
        INIT_DELAYED_WORK(&hw_priv->bes_power.power_down_work, bes2600_power_down_work);
        INIT_WORK(&hw_priv->bes_power.power_async_work, bes2600_power_async_work);
        INIT_WORK(&hw_priv->bes_power.power_mcu_down_work, bes2600_power_mcu_down_work);
        INIT_LIST_HEAD(&hw_priv->bes_power.async_timeout_list);
        INIT_LIST_HEAD(&hw_priv->bes_power.pending_event_list);
        INIT_LIST_HEAD(&hw_priv->bes_power.free_event_list);
        mutex_init(&hw_priv->bes_power.pwr_cb_mutex);
        INIT_LIST_HEAD(&hw_priv->bes_power.enter_cb_list);
        INIT_LIST_HEAD(&hw_priv->bes_power.exit_cb_list);
        hw_priv->bes_power.pending_lock = false;
        hw_priv->bes_power.hw_awake = false;
        hw_priv->bes_power.mcu_slept = false;
        hw_priv->bes_power.power_down_task = NULL;
        hw_priv->bes_power.power_up_task = NULL;
        mutex_init(&hw_priv->bes_power.pwr_mutex);
        atomic_set(&hw_priv->bes_power.dev_state, 0);
        init_completion(&hw_priv->bes_power.pm_enter_cmpl);
        sema_init(&hw_priv->bes_power.sync_lock, 1);
        device_set_wakeup_capable(hw_priv->pdev, true);
        init_waitqueue_head(&hw_priv->bes_power.dev_lp_wq);

        for(i = 0; i < BES2600_DELAY_EVENT_NUM; i++) {
                hw_priv->bes_power.pwr_events[i].idx = i;
                list_add_tail(&hw_priv->bes_power.pwr_events[i].link, &hw_priv->bes_power.free_event_list);
        }
}

void bes2600_pwr_exit(struct bes2600_common *hw_priv)
{
	bes2600_pwr_delete_all_cb(hw_priv);
}

void bes2600_pwr_prepare(struct bes2600_common *hw_priv)
{
	/* wait stop operation end */
	down(&hw_priv->bes_power.sync_lock);
}


void bes2600_pwr_complete(struct bes2600_common *hw_priv)
{
	/* notify stop operation end */
	up(&hw_priv->bes_power.sync_lock);
}

void bes2600_pwr_ensure_bus_awake(struct bes2600_common *hw_priv)
{
	if (!hw_priv || !bes2600_chrdev_is_signal_mode())
		return;

	if (atomic_read(&hw_priv->bes_power.dev_state) == 0)
		return;

	/* Non-sync: this may run on bes2600_wq (single-threaded).  cancel_*_sync
	 * / flush_* on that queue from the same worker deadlocks the tablet.
	 */
	cancel_delayed_work(&hw_priv->bes_power.power_down_work);
}

int bes2600_pwr_wake_mcu_for_join(struct bes2600_common *hw_priv)
{
	int ret;
	unsigned long flags;
	int pstate;

	if (!hw_priv)
		return -EINVAL;

	bes_devel("%s: enter pstate=%d hw_awake=%d mcu_slept=%d\n",
		 __func__, hw_priv->bes_power.power_state,
		 hw_priv->bes_power.hw_awake, hw_priv->bes_power.mcu_slept);

	/* Must not flush_work(power_async) here — same single-thread WQ as join */
	bes2600_pwr_ensure_bus_awake(hw_priv);

	ret = bes2600_pwr_request_awake(hw_priv, BES_PWR_LOCK_ON_JOIN);
	if (ret) {
		bes_err("%s: request_awake failed %d\n", __func__, ret);
		return ret;
	}

	bes_devel("%s: after request_awake pstate=%d hw_awake=%d mcu_slept=%d\n",
		 __func__, hw_priv->bes_power.power_state,
		 hw_priv->bes_power.hw_awake, hw_priv->bes_power.mcu_slept);

	/*
	 * JOIN hold (from pre-connect scan or request_awake above) may leave
	 * hw_awake=1.  Still hold the wake GPIO — after a long idle the MCU
	 * can be asleep while host flags stay sticky "awake", and join is
	 * then TX'd with no 0x040B (seen as join still pending bufs=1).
	 *
	 * Do NOT call full sbus_active() here while join holds TX lock: that
	 * races BH on sbus_mutex and has hard-locked the tablet.  GPIO-only
	 * is safe (io_mutex only).
	 */
	if (!hw_priv->bes_power.mcu_slept && bes2600_pwr_hw_is_awake(hw_priv)) {
		unsigned long since_rx = jiffies - hw_priv->rx_timestamp;
		unsigned int since_rx_ms = jiffies_to_msecs(since_rx);

		/*
		 * GPIO-only under TX lock.  Do NOT kick BH RX here: after a
		 * long idle the SDIO path can hard-lock (dwmmc "Unexpected
		 * interrupt latency" + watchdog).  Seen after
		 * "no RX for 3371 ms — gpio hold + settle" with no further
		 * join logs.  Leave bus traffic to the normal wsm_join path.
		 */
		if (hw_priv->sbus_ops->gpio_wake)
			hw_priv->sbus_ops->gpio_wake(hw_priv->sbus_priv);
		if (since_rx > 2 * HZ)
			bes_warn("%s: already awake but no RX for %u ms — gpio hold only\n",
				 __func__, since_rx_ms);
		else
			bes_devel("%s: already awake — gpio hold (rx %u ms ago)\n",
				  __func__, since_rx_ms);
		return 0;
	}

	/*
	 * If a LOCKING/UNLOCKING transition is in flight on another path, do
	 * not block forever on pwr_mutex + WSM.  JOIN event is already pending;
	 * BH will finish the transition.
	 */
	pstate = hw_priv->bes_power.power_state;
	if (pstate == POWER_DOWN_STATE_LOCKING ||
	    pstate == POWER_DOWN_STATE_UNLOCKING) {
		bes_warn("%s: power transition in progress (pstate=%d) — continue join\n",
			 __func__, pstate);
		return 0;
	}

	/* MCU slept (or hw_awake false): re-wake with power_up_task set so
	 * nested WSM_TX busy events skip re-entering pwr_mutex.
	 */
	bes_devel("%s: MCU asleep — re-wake via exit_lp\n", __func__);
	if (!mutex_trylock(&hw_priv->bes_power.pwr_mutex)) {
		bes_warn("%s: pwr_mutex busy — skip re-wake, continue join\n",
			 __func__);
		return 0;
	}
	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	hw_priv->bes_power.power_up_task = current;
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	bes2600_pwr_device_exit_lp_mode(hw_priv);

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	hw_priv->bes_power.power_up_task = NULL;
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
	mutex_unlock(&hw_priv->bes_power.pwr_mutex);

	bes_devel("%s: JOIN HW wake done (hw_awake=%d mcu_slept=%d)\n",
		 __func__, hw_priv->bes_power.hw_awake,
		 hw_priv->bes_power.mcu_slept);

	return ret;
}

int bes2600_pwr_request_awake(struct bes2600_common *hw_priv, u32 event)
{
	return bes2600_pwr_set_busy_event(hw_priv, event);
}

int bes2600_pwr_request_awake_async(struct bes2600_common *hw_priv, u32 event)
{
	return bes2600_pwr_set_busy_event_async(hw_priv, event);
}

int bes2600_pwr_release_awake(struct bes2600_common *hw_priv, u32 event)
{
	return bes2600_pwr_clear_busy_event(hw_priv, event);
}

void bes2600_pwr_refresh_hw_for_join(struct bes2600_common *hw_priv)
{
	if (!hw_priv || !bes2600_chrdev_is_signal_mode())
		return;

	if (atomic_read(&hw_priv->bes_power.dev_state) == 0)
		return;

	/* Only wake from real sleep.  Re-sending wsm_set_operational_mode while
	 * join holds TX locked routinely times out (~6s) even when hw_awake=1.
	 */
	if (hw_priv->bes_power.hw_awake) {
		bes_devel("%s: hw_awake=1, skip refresh\n", __func__);
		return;
	}

	mutex_lock(&hw_priv->bes_power.pwr_mutex);
	bes2600_pwr_device_exit_lp_mode(hw_priv);
	mutex_unlock(&hw_priv->bes_power.pwr_mutex);
	bes_devel("%s: woke MCU/WSM for join (hw_awake=%d)\n", __func__,
		 hw_priv->bes_power.hw_awake);
}

bool bes2600_pwr_hw_is_awake(struct bes2600_common *hw_priv)
{
	unsigned long flags;
	bool awake;

	if (!hw_priv)
		return false;

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	awake = hw_priv->bes_power.hw_awake;
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	return awake;
}

void bes2600_pwr_start(struct bes2600_common *hw_priv)
{
	unsigned long flags;

	if (!bes2600_chrdev_is_signal_mode())
		return ;

	/* start power management state machine */
	atomic_set(&hw_priv->bes_power.dev_state, 1);
	bes_devel("start power management.\n");

	/* enable device wakeup function */
	device_wakeup_enable(hw_priv->pdev);

	/* set power_state to busy and clear state */
	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	hw_priv->bes_power.power_state = POWER_DOWN_STATE_LOCKED;
	hw_priv->bes_power.power_down_task = NULL;
	hw_priv->bes_power.power_up_task = NULL;
	hw_priv->bes_power.sys_suspend_task = NULL;
	hw_priv->bes_power.sys_resume_task = NULL;
	hw_priv->bes_power.pending_lock = false;
	hw_priv->bes_power.ap_lp_bad = false;
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	/* Hold the MCU awake while the interface is up.  The idle timer is
	 * started only when the last busy event clears (see
	 * bes2600_pwr_clear_busy_event / bes2600_pwr_trigger_delayed_work).
	 */
	if (hw_priv->sbus_ops->gpio_wake)
		hw_priv->sbus_ops->gpio_wake(hw_priv->sbus_priv);
	hw_priv->bes_power.hw_awake = true;
}

void bes2600_pwr_stop(struct bes2600_common *hw_priv)
{
	unsigned long flags;
	unsigned long max_timeout;

	if (!bes2600_chrdev_is_signal_mode())
		return ;

	/* stop power management state machine */
	atomic_set(&hw_priv->bes_power.dev_state, 0);
	bes_devel("stop power management.\n");

	/* cancel pending work */
	cancel_delayed_work_sync(&hw_priv->bes_power.power_down_work);
	flush_delayed_work(&hw_priv->bes_power.power_down_work);
	cancel_work_sync(&hw_priv->bes_power.power_async_work);
	flush_work(&hw_priv->bes_power.power_async_work);
	cancel_work_sync(&hw_priv->bes_power.power_mcu_down_work);
	flush_work(&hw_priv->bes_power.power_mcu_down_work);

	/* delete all pending event and clear state */
	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	bes2600_update_power_delay_events(&hw_priv->bes_power, &max_timeout);
	bes2600_flush_pending_events(&hw_priv->bes_power);
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	/* allow mcu sleep */
	bes2600_power_down_work(&hw_priv->bes_power.power_down_work.work);
	if (hw_priv->bes_power.power_state != POWER_DOWN_STATE_UNLOCKED)
		bes_warn("power state is not unlocked when stop.\n");

	/* unlock tx if tx is locked */
	bes2600_pwr_unlock_tx(hw_priv);

	/* disable device wakeup function */
	device_wakeup_disable(hw_priv->pdev);
}

bool bes2600_pwr_constant_event_is_pending(struct bes2600_common *hw_priv, u32 event)
{
        unsigned long max_timeout = 0;
        bool constant_event_exist = false;
        unsigned long flags;
        bool pending = false;
        struct bes2600_pwr_event_t *item = NULL;

        spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
        constant_event_exist = bes2600_update_power_delay_events(&hw_priv->bes_power, &max_timeout);

        if(constant_event_exist) {
                BES_PWR_EVENT_SET_CONSTANT(event);
                if(!list_empty(&hw_priv->bes_power.pending_event_list)) {
                        list_for_each_entry(item, &hw_priv->bes_power.pending_event_list, link) {
                                if(item->event == event) {
                                        pending = true;
                                        break;
                                }
                        }
                }
        }

        spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

        return pending;
}

int bes2600_pwr_set_busy_event(struct bes2600_common *hw_priv, u32 event)
{
	int ret = 0;
	int dev_state;
	bool need_lock = false;
	unsigned long flags;

	if(!hw_priv || !&hw_priv->bes_power)
		return -EINVAL;

	dev_state = atomic_read(&hw_priv->bes_power.dev_state);

	/* Minimal logging — only when state is bad or on real events */
	if(dev_state == 0 || event != 0) {
		bes_devel("%s: dev_state=%d, power_state=%d, event=0x%x\n",
		          __func__, dev_state, hw_priv->bes_power.power_state, event);
	}

	if(dev_state == 0) {
		/* Early boot / not ready yet — just allow it, don't fail */
		// Used to be -1
		return 0;
	}

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);


	/* Nested WSM from under pwr_mutex (lock/unlock/re-wake) must not
	 * re-enter lock_device — that deadlocks on pwr_mutex.
	 */
	if (event == BES_PWR_LOCK_ON_WSM_TX) {
		if (hw_priv->bes_power.power_up_task == current ||
		    hw_priv->bes_power.power_down_task == current ||
		    hw_priv->bes_power.sys_suspend_task == current ||
		    hw_priv->bes_power.sys_resume_task == current) {
			spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
			return 0;
		}
	}


	BES_PWR_EVENT_SET_CONSTANT(event);
	ret = bes2600_add_power_delay_event(&hw_priv->bes_power, event, 0);
	if(ret) {
		bes_err("%s: add_power_delay_event failed %d\n", __func__, ret);
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
		return ret;
	}

	/*
	 * Never need_wait on pwr_mutex while LOCKING/UNLOCKING: the transition
	 * thread holds pwr_mutex and waits for a WSM confirm via BH.  BH (or
	 * another CPU) calling set_busy_event then deadlocks forever.
	 * Just leave the event pending; the transition path will leave the
	 * device locked/unlocked appropriately.
	 */
	if (hw_priv->bes_power.power_state == POWER_DOWN_STATE_LOCKING ||
	    hw_priv->bes_power.power_state == POWER_DOWN_STATE_UNLOCKING) {
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
		return 0;
	}

	if (bes2600_pwr_needs_hw_wake(&hw_priv->bes_power))
		need_lock = true;

	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	if (need_lock) {
		/* Non-sync: callers (join/scan) often run on bes2600_wq */
		cancel_delayed_work(&hw_priv->bes_power.power_down_work);

		ret = bes2600_pwr_lock_device(hw_priv);
		if (ret)
			bes_err("%s: pwr_lock_device failed %d\n", __func__, ret);
	}

	return ret;
}

int bes2600_pwr_set_busy_event_async(struct bes2600_common *hw_priv, u32 event)
{
	bool need_lock = false;
	unsigned long flags;

	if(atomic_read(&hw_priv->bes_power.dev_state) == 0) {
		return -1;
	}

	cancel_delayed_work(&hw_priv->bes_power.power_down_work);

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	BES_PWR_EVENT_SET_CONSTANT(event);
	bes2600_add_power_delay_event(&hw_priv->bes_power, event, 0);
	if (bes2600_pwr_needs_hw_wake(&hw_priv->bes_power))
		need_lock = true;
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	if(need_lock && !work_pending(&hw_priv->bes_power.power_async_work)) {
		bes_devel("%s lock device by event:%d\n", __func__, BES_PWR_EVENT_NUMBER(event));
		queue_work(hw_priv->workqueue, &hw_priv->bes_power.power_async_work);
	}

	return 0;

}

int bes2600_pwr_set_busy_event_with_timeout(struct bes2600_common *hw_priv, u32 event, u32 timeout)
{
	int ret = 0;
	bool need_lock = false;
	unsigned long flags;

	if (!hw_priv || !&hw_priv->bes_power) return -EINVAL;

	if(atomic_read(&hw_priv->bes_power.dev_state) == 0) {
		return -1;
	}

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);

	/* add delayed event to pending_event_list */
	bes2600_add_power_delay_event(&hw_priv->bes_power, event, timeout);

	if (hw_priv->bes_power.power_state == POWER_DOWN_STATE_LOCKING ||
	    hw_priv->bes_power.power_state == POWER_DOWN_STATE_UNLOCKING) {
		spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
		return 0;
	}

	if (bes2600_pwr_needs_hw_wake(&hw_priv->bes_power))
		need_lock = true;

	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	if (need_lock) {
		bes_devel("%s lock device by event:%d\n", __func__, event);
		cancel_delayed_work(&hw_priv->bes_power.power_down_work);

		ret = bes2600_pwr_lock_device(hw_priv);
		if (ret)
			bes_err("%s: pwr_lock_device failed %d\n", __func__, ret);
	}

	if (!delayed_work_pending(&hw_priv->bes_power.power_down_work))
		bes2600_pwr_trigger_delayed_work(hw_priv);

	return ret;
}

int bes2600_pwr_set_busy_event_with_timeout_async(struct bes2600_common *hw_priv, u32 event, u32 timeout)
{
	int ret = 0;
	bool need_lock = false;
	unsigned long flags;

	if(atomic_read(&hw_priv->bes_power.dev_state) == 0) {
		return -1;
	}

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	if ((hw_priv->bes_power.power_state == POWER_DOWN_STATE_UNLOCKED) ||
	   (hw_priv->bes_power.power_state == POWER_DOWN_STATE_LOCKED)) {
		bes2600_add_power_delay_event(&hw_priv->bes_power, event, timeout);
		if (bes2600_pwr_needs_hw_wake(&hw_priv->bes_power))
			need_lock = true;
	} else if ((hw_priv->bes_power.power_state == POWER_DOWN_STATE_LOCKING) ||
	   (hw_priv->bes_power.power_state == POWER_DOWN_STATE_UNLOCKING)) {
		bes2600_add_async_timeout_power_delay_event(&hw_priv->bes_power, event, timeout);
		if (bes2600_pwr_needs_hw_wake(&hw_priv->bes_power))
			need_lock = true;
	}
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	if(need_lock && !work_pending(&hw_priv->bes_power.power_async_work)) {
		bes_devel("%s lock device by event:%d\n", __func__, event);
		queue_work(hw_priv->workqueue, &hw_priv->bes_power.power_async_work);
	}

       return ret;
}


int bes2600_pwr_clear_busy_event(struct bes2600_common *hw_priv, u32 event)
{
	int ret = 0;
	u32 constant_event = event;
	unsigned long flags;

	if(atomic_read(&hw_priv->bes_power.dev_state) == 0) {
		return -1;
	}

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);

	/* don't need to clear busy event if the command is for unlocking device */
	if((event == BES_PWR_LOCK_ON_WSM_TX)
	   && (hw_priv->bes_power.power_state == POWER_DOWN_STATE_UNLOCKING)) {
		if(hw_priv->bes_power.power_down_task == current) {
			/* BES_PWR_LOCK_ON_WSM_TX is from power down work */
			spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
			return 0;
		}
	}

	/* don't need also to clear busy event if the command is for locking device */
	if((event == BES_PWR_LOCK_ON_WSM_TX)
	   && (hw_priv->bes_power.power_state == POWER_DOWN_STATE_LOCKING)) {
		if(hw_priv->bes_power.power_up_task == current) {
			/* BES_PWR_LOCK_ON_WSM_TX is for powering up operation */
			spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
			return 0;
		}
	}

	 /* don't set busy event if the command is for suspend/resume */
	if((event == BES_PWR_LOCK_ON_WSM_TX)
	   && (hw_priv->bes_power.power_state == POWER_DOWN_STATE_UNLOCKED)) {
		if(hw_priv->bes_power.sys_suspend_task == current ||
		   hw_priv->bes_power.sys_resume_task == current) {
			/* BES_PWR_LOCK_ON_WSM_TX is from suspend/resume work */
			spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);
			return 0;
		}
	}

	/* change constant event to delay event */
	BES_PWR_EVENT_SET_CONSTANT(constant_event);
	if(bes2600_del_pending_event(&hw_priv->bes_power, constant_event)) {
		bes2600_add_power_delay_event(&hw_priv->bes_power, event, 0);
	}
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	/* trigger power down delayed work */
	if(!delayed_work_pending(&hw_priv->bes_power.power_down_work)) {
		bes_devel("%s restart delayed work\n", __func__);
		bes2600_pwr_trigger_delayed_work(hw_priv);
	}

	return ret;
}

void bes2600_pwr_notify_ps_changed(struct bes2600_common *hw_priv, u8 psmode)
{
	if((psmode & 0x01) != WSM_PSM_ACTIVE) {
		bes_devel("complete pm_enter_cmpl\n");
		complete(&hw_priv->bes_power.pm_enter_cmpl);
	}
}

bool bes2600_pwr_device_is_idle(struct bes2600_common *hw_priv)
{
    bool idle = false;
    unsigned long flags;

    if (!bes2600_chrdev_is_signal_mode())
		return false;

    spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
    idle = (hw_priv->bes_power.power_state == POWER_DOWN_STATE_UNLOCKED);
    spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

    return idle;
}

void bes2600_pwr_register_en_lp_cb(struct bes2600_common *hw_priv, bes_pwr_enter_lp_cb cb)
{
	struct bes2600_pwr_enter_cb_item *item = NULL;

	mutex_lock(&hw_priv->bes_power.pwr_cb_mutex);
	list_for_each_entry(item, &hw_priv->bes_power.enter_cb_list, link) {
		if(item->cb == cb) {
			bes_warn("the enter cb is already exist\n");
			mutex_unlock(&hw_priv->bes_power.pwr_cb_mutex);
			return;
		}
	}

	item = kzalloc(sizeof(struct bes2600_pwr_enter_cb_item), GFP_KERNEL);
	if(item) {
		item->cb = cb;
		list_add_tail(&item->link, &hw_priv->bes_power.enter_cb_list);
	}
	mutex_unlock(&hw_priv->bes_power.pwr_cb_mutex);

	if (item == NULL)
		bes_err("register en_lp_cb fail.\n");
}

void bes2600_pwr_unregister_en_lp_cb(struct bes2600_common *hw_priv, bes_pwr_enter_lp_cb cb)
{
	struct bes2600_pwr_enter_cb_item *item = NULL, *temp = NULL;

	mutex_lock(&hw_priv->bes_power.pwr_cb_mutex);
	list_for_each_entry_safe(item, temp, &hw_priv->bes_power.enter_cb_list, link) {
		if(cb == item->cb) {
			list_del(&item->link);
			kfree(item);
			break;
		}
	}
	mutex_unlock(&hw_priv->bes_power.pwr_cb_mutex);
}

void bes2600_pwr_register_exit_lp_cb(struct bes2600_common *hw_priv, bes_pwr_enter_lp_cb cb)
{
	struct bes2600_pwr_exit_cb_item *item = NULL;

	mutex_lock(&hw_priv->bes_power.pwr_cb_mutex);
	list_for_each_entry(item, &hw_priv->bes_power.exit_cb_list, link) {
		if(item->cb == cb) {
			mutex_unlock(&hw_priv->bes_power.pwr_cb_mutex);
			bes_warn("the exit cb is already exist\n");
			return;
		}
	}

	item = kzalloc(sizeof(struct bes2600_pwr_exit_cb_item), GFP_KERNEL);
	if(item) {
		item->cb = cb;
		list_add_tail(&item->link, &hw_priv->bes_power.exit_cb_list);
	}
	mutex_unlock(&hw_priv->bes_power.pwr_cb_mutex);

	if (item == NULL)
		bes_err("register en_lp_cb fail.\n");
}

void bes2600_pwr_unregister_exit_lp_cb(struct bes2600_common *hw_priv, bes_pwr_enter_lp_cb cb)
{
	struct bes2600_pwr_exit_cb_item *item = NULL, *temp = NULL;

	mutex_lock(&hw_priv->bes_power.pwr_cb_mutex);
	list_for_each_entry_safe(item, temp, &hw_priv->bes_power.exit_cb_list, link) {
		if(cb == item->cb) {
			list_del(&item->link);
			kfree(item);
			break;
		}
	}
	mutex_unlock(&hw_priv->bes_power.pwr_cb_mutex);
}

void bes2600_pwr_suspend_start(struct bes2600_common *hw_priv)
{
	if(atomic_read(&hw_priv->bes_power.dev_state) == 0) {
		return;
	}

        mutex_lock(&hw_priv->bes_power.pwr_mutex);
        hw_priv->bes_power.sys_suspend_task = current;
        bes2600_pwr_device_exit_lp_mode(hw_priv);
        mutex_unlock(&hw_priv->bes_power.pwr_mutex);
}

void bes2600_pwr_suspend_end(struct bes2600_common *hw_priv)
{
	unsigned long max_timeout = 0;
	bool constant_event_exist = false;
	unsigned long flags;

	if(atomic_read(&hw_priv->bes_power.dev_state) == 0) {
		return;
	}

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	constant_event_exist = bes2600_update_power_delay_events(&hw_priv->bes_power, &max_timeout);
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	mutex_lock(&hw_priv->bes_power.pwr_mutex);
	if(!constant_event_exist && max_timeout == 0)
		bes2600_pwr_device_enter_lp_mode(hw_priv);
	hw_priv->bes_power.sys_suspend_task = NULL;
	mutex_unlock(&hw_priv->bes_power.pwr_mutex);
}

void bes2600_pwr_resume_start(struct bes2600_common *hw_priv)
{
	if(atomic_read(&hw_priv->bes_power.dev_state) == 0) {
		return;
	}

        mutex_lock(&hw_priv->bes_power.pwr_mutex);
        hw_priv->bes_power.sys_resume_task = current;
        bes2600_pwr_device_exit_lp_mode(hw_priv);
        mutex_unlock(&hw_priv->bes_power.pwr_mutex);
}

void bes2600_pwr_resume_end(struct bes2600_common *hw_priv)
{
	unsigned long max_timeout = 0;
	bool constant_event_exist = false;
	unsigned long flags;

	if(atomic_read(&hw_priv->bes_power.dev_state) == 0) {
		return ;
	}

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	constant_event_exist = bes2600_update_power_delay_events(&hw_priv->bes_power, &max_timeout);
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	mutex_lock(&hw_priv->bes_power.pwr_mutex);
	if(!constant_event_exist && max_timeout == 0)
		bes2600_pwr_device_enter_lp_mode(hw_priv);
	hw_priv->bes_power.sys_resume_task = NULL;
	mutex_unlock(&hw_priv->bes_power.pwr_mutex);
}

void bes2600_pwr_mcu_sleep_directly(struct bes2600_common *hw_priv)
{
	if(atomic_read(&hw_priv->bes_power.dev_state) == 0) {
		return;
	}

	if (bes2600_pwr_device_is_idle(hw_priv)) {
		if(!work_pending(&hw_priv->bes_power.power_mcu_down_work)) {
			queue_work(hw_priv->workqueue, &hw_priv->bes_power.power_mcu_down_work);
		}
	}
}

void bes2600_pwr_mark_ap_lp_bad(struct bes2600_common *hw_priv)
{
	if(atomic_read(&hw_priv->bes_power.dev_state) == 0) {
		return;
	}

	bes2600_pwr_set_busy_event_with_timeout(hw_priv, BES_PWR_LOCK_ON_AP_LP_BAD, 100);
	hw_priv->bes_power.ap_lp_bad = true;
	bes_devel("mark ap_lp_bad flag\n");
}

void bes2600_pwr_clear_ap_lp_bad_mark(struct bes2600_common *hw_priv)
{
	hw_priv->bes_power.ap_lp_bad = false;
	bes_devel("clear ap_lp_bad flag\n");
}

int bes2600_pwr_busy_event_dump(struct bes2600_common *hw_priv, char *buffer, u32 buf_len)
{
	unsigned long max_timeout = 0;
	int used_len = 0;
	struct bes2600_pwr_event_t *item = NULL;
	unsigned long flags;

	if(!buffer) {
		return -1;
	}

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	bes2600_update_power_delay_events(&hw_priv->bes_power, &max_timeout);
	used_len = snprintf(buffer, buf_len, "Event    \t\tFlag\t\ttimeout(ticks)\n");
	if(!list_empty(&hw_priv->bes_power.pending_event_list)) {
		list_for_each_entry(item, &hw_priv->bes_power.pending_event_list, link) {
					used_len += snprintf(buffer + used_len, buf_len - used_len, "%9s\t\t%s\t\t%lu\n",
					bes2600_get_pwr_busy_event_name(item),
					BES_PWR_IS_CONSTANT_EVENT(item->event) ? "C" : "D",
					bes2600_get_pwr_busy_event_timeout(item));
		}
	}
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	return 0;
}

int bes2600_pwr_busy_event_record(struct bes2600_common *hw_priv, char *buffer, u32 buf_len)
{
	unsigned long max_timeout = 0;
	struct bes2600_pwr_event_t *item = NULL;
	unsigned long flags;

	if(!buffer) {
		return -1;
	}

	spin_lock_irqsave(&hw_priv->bes_power.pwr_lock, flags);
	bes2600_update_power_delay_events(&hw_priv->bes_power, &max_timeout);
	if(!list_empty(&hw_priv->bes_power.pending_event_list)) {
		list_for_each_entry(item, &hw_priv->bes_power.pending_event_list, link) {
					snprintf(buffer, buf_len, "Event: %9s. Flag: %s. timeout(ticks): %lu.\n",
					bes2600_get_pwr_busy_event_name(item),
					BES_PWR_IS_CONSTANT_EVENT(item->event) ? "C" : "D",
					bes2600_get_pwr_busy_event_timeout(item));
		}
	}
	spin_unlock_irqrestore(&hw_priv->bes_power.pwr_lock, flags);

	return 0;
}
