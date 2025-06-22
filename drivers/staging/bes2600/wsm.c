/*
 * WSM host interface (HI) implementation for BES2600 mac80211 drivers.
 *
 * Copyright (c) 2022, Bestechnic
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/skbuff.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/random.h>
#include <linux/etherdevice.h>
#include <net/mac80211.h>
#include "bes2600.h"
#include "wsm.h"
#include "bh.h"
#include "debug.h"
#include "itp.h"
#ifdef ROAM_OFFLOAD
#include "sta.h"
#endif
#ifdef CONFIG_BES2600_TESTMODE
#include "bes_nl80211_testmode_msg.h"
#endif
#include "bes_chardev.h"
#include "bes2600_factory.h"
#include "epta_coex.h"
#include "epta_request.h"
#include "bes_log.h"

#define WSM_CMD_TIMEOUT (6 * HZ)
#define WSM_CMD_JOIN_TIMEOUT (7 * HZ)
#define WSM_CMD_START_TIMEOUT (7 * HZ)
#define WSM_CMD_RESET_TIMEOUT (7 * HZ)
#define WSM_CMD_DEFAULT_TIMEOUT (7 * HZ)

#define WSM_SKIP(buf, size) \
	do { \
		if (unlikely((buf)->data + size > (buf)->end)) \
			goto underflow; \
		(buf)->data += size; \
	} while (0)

#define WSM_GET(buf, ptr, size) \
	do { \
		if (unlikely((buf)->data + size > (buf)->end)) \
			goto underflow; \
		memcpy(ptr, (buf)->data, size); \
		(buf)->data += size; \
	} while (0)

#define __WSM_GET(buf, type, cvt) \
	({ \
		type val; \
		if (unlikely((buf)->data + sizeof(type) > (buf)->end)) \
			goto underflow; \
		val = cvt(*(type *)(buf)->data); \
		(buf)->data += sizeof(type); \
		val; \
	})

#define WSM_GET8(buf) __WSM_GET(buf, u8, (u8))
#define WSM_GET16(buf) __WSM_GET(buf, u16, __le16_to_cpu)
#define WSM_GET32(buf) __WSM_GET(buf, u32, __le32_to_cpu)

#define WSM_PUT(buf, ptr, size) \
	do { \
		if (unlikely((buf)->data + size > (buf)->end)) \
			if (unlikely(wsm_buf_reserve((buf), size))) \
				goto nomem; \
		memcpy((buf)->data, ptr, size); \
		(buf)->data += size; \
	} while (0)

#define __WSM_PUT(buf, val, type, cvt) \
	do { \
		if (unlikely((buf)->data + sizeof(type) > (buf)->end)) \
			if (unlikely(wsm_buf_reserve((buf), sizeof(type)))) \
				goto nomem; \
		*(type *)(buf)->data = cvt(val); \
		(buf)->data += sizeof(type); \
	} while (0)

#define WSM_PUT8(buf, val) __WSM_PUT(buf, val, u8, (u8))
#define WSM_PUT16(buf, val) __WSM_PUT(buf, val, u16, __cpu_to_le16)
#define WSM_PUT32(buf, val) __WSM_PUT(buf, val, u32, __cpu_to_le32)

/* Buffer management */
static void wsm_buf_reset(struct wsm_buf *buf)
{
	if (buf->begin) {
		buf->data = &buf->begin[4];
		*(u32 *)buf->begin = 0;
	} else {
		buf->data = buf->begin;
	}
}

static int wsm_buf_reserve(struct wsm_buf *buf, size_t extra_size)
{
	size_t pos = buf->data - buf->begin;
	size_t size = pos + extra_size;

	if (size & (SDIO_BLOCK_SIZE - 1)) {
		size &= SDIO_BLOCK_SIZE;
		size += SDIO_BLOCK_SIZE;
	}

	buf->begin = krealloc(buf->begin, size, GFP_KERNEL | GFP_DMA);
	if (buf->begin) {
		buf->data = &buf->begin[pos];
		buf->end = &buf->begin[size];
		return 0;
	}
	buf->end = buf->data = buf->begin;
	return -ENOMEM;
}

void wsm_buf_init(struct wsm_buf *buf)
{
	BUG_ON(buf->begin);
	buf->begin = kmalloc(SDIO_BLOCK_SIZE, GFP_KERNEL | GFP_DMA);
	buf->end = buf->begin ? &buf->begin[SDIO_BLOCK_SIZE] : buf->begin;
	wsm_buf_reset(buf);
}

void wsm_buf_deinit(struct wsm_buf *buf)
{
	kfree(buf->begin);
	buf->begin = buf->data = buf->end = NULL;
}

/* Command locking */
static inline void wsm_cmd_lock(struct bes2600_common *hw_priv)
{
	bes2600_pwr_set_busy_event(hw_priv, BES_PWR_LOCK_ON_WSM_TX);
	down(&hw_priv->wsm_cmd_sema);
}

static inline void wsm_cmd_unlock(struct bes2600_common *hw_priv)
{
	up(&hw_priv->wsm_cmd_sema);
	bes2600_pwr_clear_busy_event(hw_priv, BES_PWR_LOCK_ON_WSM_TX);
}

static inline void wsm_oper_lock(struct bes2600_common *hw_priv)
{
	bes2600_pwr_set_busy_event(hw_priv, BES_PWR_LOCK_ON_WSM_OPER);
	down(&hw_priv->wsm_oper_lock);
}

static inline void wsm_oper_unlock(struct bes2600_common *hw_priv)
{
	up(&hw_priv->wsm_oper_lock);
	bes2600_pwr_clear_busy_event(hw_priv, BES_PWR_LOCK_ON_WSM_OPER);
}

/* Generic command confirmation */
static int wsm_generic_confirm(struct bes2600_common *hw_priv,
							  void *arg, struct wsm_buf *buf)
{
	u32 status = WSM_GET32(buf);
	if (WARN(status != WSM_STATUS_SUCCESS, "wsm_generic_confirm ret %u", status))
		return -EINVAL;
	return 0;

underflow:
	WARN_ON(1);
	return -EINVAL;
}

/* Configuration command */
int wsm_configuration(struct bes2600_common *hw_priv,
					  struct wsm_configuration *arg, int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT32(buf, arg->dot11MaxTransmitMsduLifeTime);
	WSM_PUT32(buf, arg->dot11MaxReceiveLifeTime);
	WSM_PUT32(buf, arg->dot11RtsThreshold);
	WSM_PUT16(buf, arg->dpdData_size + 12);
	WSM_PUT16(buf, 1); /* DPD version */
	WSM_PUT(buf, arg->dot11StationId, ETH_ALEN);
	WSM_PUT16(buf, 5); /* DPD flags */
	WSM_PUT(buf, arg->dpdData, arg->dpdData_size);
	ret = wsm_cmd_send(hw_priv, buf, arg, 0x0009, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

static int wsm_configuration_confirm(struct bes2600_common *hw_priv,
									struct wsm_configuration *arg,
									struct wsm_buf *buf)
{
	int i;
	int status = WSM_GET32(buf);
	if (WARN_ON(status != WSM_STATUS_SUCCESS))
		return -EINVAL;

	if (bes2600_chrdev_is_signal_mode()) {
		WSM_GET(buf, arg->dot11StationId, ETH_ALEN);
		arg->dot11FrequencyBandsSupported = WSM_GET8(buf);
		WSM_SKIP(buf, 1);
		arg->supportedRateMask = WSM_GET32(buf);
		for (i = 0; i < 2; ++i) {
			arg->txPowerRange[i].min_power_level = WSM_GET32(buf);
			arg->txPowerRange[i].max_power_level = WSM_GET32(buf);
			arg->txPowerRange[i].stepping = WSM_GET32(buf);
		}
	}
	return 0;

underflow:
	WARN_ON(1);
	return -EINVAL;
}

/* Reset command */
int wsm_reset(struct bes2600_common *hw_priv, const struct wsm_reset *arg,
			  int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;
	u16 cmd = 0x000A | WSM_TX_LINK_ID(arg->link_id);

	wsm_cmd_lock(hw_priv);
	WSM_PUT32(buf, arg->reset_statistics ? 0 : 1);
	ret = wsm_cmd_send(hw_priv, buf, NULL, cmd, WSM_CMD_RESET_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* Read MIB */
int wsm_read_mib(struct bes2600_common *hw_priv, u16 mibId, void *buf,
				 size_t buf_size)
{
	int ret;
	struct wsm_buf *wsm_buf = &hw_priv->wsm_cmd_buf;
	struct wsm_mib mib_buf = {
		.mibId = mibId,
		.buf = buf,
		.buf_size = buf_size,
	};

	wsm_cmd_lock(hw_priv);
	WSM_PUT16(wsm_buf, mibId);
	WSM_PUT16(wsm_buf, 0);
	ret = wsm_cmd_send(hw_priv, wsm_buf, &mib_buf, 0x0005, WSM_CMD_TIMEOUT, -1);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

static int wsm_read_mib_confirm(struct bes2600_common *hw_priv,
								struct wsm_mib *arg, struct wsm_buf *buf)
{
	u16 size;
	if (WARN_ON(WSM_GET32(buf) != WSM_STATUS_SUCCESS))
		return -EINVAL;
	if (WARN_ON(WSM_GET16(buf) != arg->mibId))
		return -EINVAL;
	size = WSM_GET16(buf);
	if (size > arg->buf_size)
		size = arg->buf_size;
	WSM_GET(buf, arg->buf, size);
	arg->buf_size = size;
	return 0;

underflow:
	WARN_ON(1);
	return -EINVAL;
}

/* Write MIB */
int wsm_write_mib(struct bes2600_common *hw_priv, u16 mibId, void *buf,
				  size_t buf_size, int if_id)
{
	int ret;
	struct wsm_buf *wsm_buf = &hw_priv->wsm_cmd_buf;
	struct wsm_mib mib_buf = {
		.mibId = mibId,
		.buf = buf,
		.buf_size = buf_size,
	};

	wsm_cmd_lock(hw_priv);
	WSM_PUT16(wsm_buf, mibId);
	WSM_PUT16(wsm_buf, buf_size);
	WSM_PUT(wsm_buf, buf, buf_size);
	ret = wsm_cmd_send(hw_priv, wsm_buf, &mib_buf, 0x0006, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

static int wsm_write_mib_confirm(struct bes2600_common *hw_priv,
								struct wsm_mib *arg, struct wsm_buf *buf,
								int interface_link_id)
{
	int ret;
	struct bes2600_vif *priv;

	if (!is_hardware_cw1250(hw_priv) || is_hardware_cw1260(hw_priv))
		interface_link_id = 0;

	ret = wsm_generic_confirm(hw_priv, arg, buf);
	if (ret)
		return ret;

	if (arg->mibId == 0x1006) {
		const char *p = arg->buf;
		if (!hw_priv->vif_list[interface_link_id])
			return 0;
		priv = cw12xx_hwpriv_to_vifpriv(hw_priv, interface_link_id);
		if (!priv)
			return 0;
		spin_lock(&priv->vif_lock);
		bes2600_enable_powersave(priv, (p[0] & 0x0F) ? true : false);
		spin_unlock(&priv->vif_lock);
	}
	return 0;
}

/* Scan command */
int wsm_scan(struct bes2600_common *hw_priv, const struct wsm_scan *arg,
			 int if_id)
{
	int i, ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	if (unlikely(arg->numOfChannels > 48) || unlikely(arg->numOfSSIDs > WSM_SCAN_MAX_NUM_OF_SSIDS) ||
		unlikely(arg->band > 1))
		return -EINVAL;

	wsm_oper_lock(hw_priv);
	wsm_cmd_lock(hw_priv);
	WSM_PUT8(buf, arg->band);
	WSM_PUT8(buf, arg->scanType);
	WSM_PUT8(buf, arg->scanFlags);
	WSM_PUT8(buf, arg->maxTransmitRate);
	WSM_PUT32(buf, arg->autoScanInterval);
	WSM_PUT8(buf, arg->numOfProbeRequests);
	WSM_PUT8(buf, arg->numOfChannels);
	WSM_PUT8(buf, arg->numOfSSIDs);
	WSM_PUT8(buf, arg->probeDelay);

	for (i = 0; i < arg->numOfChannels; ++i) {
		WSM_PUT16(buf, arg->ch[i].number);
		WSM_PUT16(buf, 0);
		WSM_PUT32(buf, arg->ch[i].minChannelTime);
		WSM_PUT32(buf, arg->ch[i].maxChannelTime);
		WSM_PUT32(buf, 0);
	}

	for (i = 0; i < arg->numOfSSIDs; ++i) {
		WSM_PUT32(buf, arg->ssids[i].length);
		WSM_PUT(buf, &arg->ssids[i].ssid[0], sizeof(arg->ssids[i].ssid));
	}

	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0007, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	if (ret)
		wsm_oper_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	wsm_oper_unlock(hw_priv);
	return -ENOMEM;
}

/* Stop scan */
int wsm_stop_scan(struct bes2600_common *hw_priv, int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;
	wsm_cmd_lock(hw_priv);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0008, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;
}

/* Join confirmation */
static int wsm_join_confirm(struct bes2600_common *hw_priv,
							struct wsm_join *arg, struct wsm_buf *buf)
{
	u32 status = WSM_GET32(buf);
	wsm_oper_unlock(hw_priv);
	if (status != WSM_STATUS_SUCCESS) {
		bes_warn("wsm_join_confirm ret %u\n", status);
		return -EINVAL;
	}
	arg->minPowerLevel = WSM_GET32(buf);
	arg->maxPowerLevel = WSM_GET32(buf);
	return 0;

underflow:
	WARN_ON(1);
	return -EINVAL;
}

/* Join command */
int wsm_join(struct bes2600_common *hw_priv, struct wsm_join *arg, int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_oper_lock(hw_priv);
	wsm_cmd_lock(hw_priv);
	WSM_PUT8(buf, arg->mode);
	WSM_PUT8(buf, arg->band);
	WSM_PUT16(buf, arg->channelNumber);
	WSM_PUT(buf, &arg->bssid[0], sizeof(arg->bssid));
	WSM_PUT16(buf, arg->atimWindow);
	WSM_PUT8(buf, arg->preambleType);
	WSM_PUT8(buf, arg->probeForJoin);
	WSM_PUT8(buf, arg->dtimPeriod);
	WSM_PUT8(buf, arg->flags);
	WSM_PUT32(buf, arg->ssidLength);
	WSM_PUT(buf, &arg->ssid[0], sizeof(arg->ssid));
	WSM_PUT32(buf, arg->beaconInterval);
	WSM_PUT32(buf, arg->basicRateSet);
	hw_priv->tx_burst_idx = -1;
	ret = wsm_cmd_send(hw_priv, buf, arg, 0x000B, WSM_CMD_JOIN_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	wsm_oper_unlock(hw_priv);
	return -ENOMEM;
}

/* Set BSS params */
int wsm_set_bss_params(struct bes2600_common *hw_priv,
					   const struct wsm_set_bss_params *arg, int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT8(buf, 0);
	WSM_PUT8(buf, arg->beaconLostCount);
	WSM_PUT16(buf, arg->aid);
	WSM_PUT32(buf, arg->operationalRateSet);
	WSM_PUT32(buf, hw_priv->ht_info.ht_cap.mcs.rx_mask[0] << 14);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0011, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* Add key */
int wsm_add_key(struct bes2600_common *hw_priv, const struct wsm_add_key *arg,
				int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT(buf, arg, sizeof(*arg));
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x000C, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* Remove key */
int wsm_remove_key(struct bes2600_common *hw_priv,
				   const struct wsm_remove_key *arg, int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT8(buf, arg->entryIndex);
	WSM_PUT8(buf, 0);
	WSM_PUT16(buf, 0);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x000D, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* Set TX queue params */
int wsm_set_tx_queue_params(struct bes2600_common *hw_priv,
							const struct wsm_set_tx_queue_params *arg,
							u8 id, int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;
	u8 queue_id_to_wmm_aci[] = {3, 2, 0, 1};

	wsm_cmd_lock(hw_priv);
	WSM_PUT8(buf, queue_id_to_wmm_aci[id]);
	WSM_PUT8(buf, 0);
	WSM_PUT8(buf, arg->ackPolicy);
	WSM_PUT8(buf, 0);
	WSM_PUT32(buf, arg->maxTransmitLifetime);
	WSM_PUT16(buf, arg->allowedMediumTime);
	WSM_PUT16(buf, 0);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0012, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* Set EDCA params */
int wsm_set_edca_params(struct bes2600_common *hw_priv,
						const struct wsm_edca_params *arg, int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT16(buf, arg->params[3].cwMin);
	WSM_PUT16(buf, arg->params[2].cwMin);
	WSM_PUT16(buf, arg->params[1].cwMin);
	WSM_PUT16(buf, arg->params[0].cwMin);
	WSM_PUT16(buf, arg->params[3].cwMax);
	WSM_PUT16(buf, arg->params[2].cwMax);
	WSM_PUT16(buf, arg->params[1].cwMax);
	WSM_PUT16(buf, arg->params[0].cwMax);
	WSM_PUT8(buf, arg->params[3].aifns);
	WSM_PUT8(buf, arg->params[2].aifns);
	WSM_PUT8(buf, arg->params[1].aifns);
	WSM_PUT8(buf, arg->params[0].aifns);
	WSM_PUT16(buf, arg->params[3].txOpLimit);
	WSM_PUT16(buf, arg->params[2].txOpLimit);
	WSM_PUT16(buf, arg->params[1].txOpLimit);
	WSM_PUT16(buf, arg->params[0].txOpLimit);
	WSM_PUT32(buf, arg->params[3].maxReceiveLifetime);
	WSM_PUT32(buf, arg->params[2].maxReceiveLifetime);
	WSM_PUT32(buf, arg->params[1].maxReceiveLifetime);
	WSM_PUT32(buf, arg->params[0].maxReceiveLifetime);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0013, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* Switch channel */
int wsm_switch_channel(struct bes2600_common *hw_priv,
					   const struct wsm_switch_channel *arg, int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_lock_tx(hw_priv);
	wsm_cmd_lock(hw_priv);
	WSM_PUT8(buf, arg->channelMode | 0x80);
	WSM_PUT8(buf, arg->channelSwitchCount);
	WSM_PUT16(buf, arg->newChannelNumber);
	hw_priv->channel_switch_in_progress = 1;
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0016, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	if (ret) {
		wsm_unlock_tx(hw_priv);
		hw_priv->channel_switch_in_progress = 0;
	}
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	wsm_unlock_tx(hw_priv);
	return -ENOMEM;
}

/* Set power management */
int wsm_set_pm(struct bes2600_common *hw_priv, const struct wsm_set_pm *arg,
			   int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT8(buf, arg->pmMode);
	WSM_PUT8(buf, arg->fastPsmIdlePeriod);
	WSM_PUT8(buf, arg->apPsmChangePeriod);
	WSM_PUT8(buf, arg->minAutoPsPollPeriod);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0010, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* Start AP/P2P */
int wsm_start(struct bes2600_common *hw_priv, const struct wsm_start *arg,
			  int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT8(buf, arg->mode);
	WSM_PUT8(buf, arg->band);
	WSM_PUT16(buf, arg->channelNumber);
	WSM_PUT32(buf, arg->CTWindow);
	WSM_PUT32(buf, arg->beaconInterval);
	WSM_PUT8(buf, arg->DTIMPeriod);
	WSM_PUT8(buf, arg->preambleType);
	WSM_PUT8(buf, arg->probeDelay);
	WSM_PUT8(buf, arg->ssidLength);
	WSM_PUT(buf, arg->ssid, sizeof(arg->ssid));
	WSM_PUT32(buf, arg->basicRateSet);
	hw_priv->tx_burst_idx = -1;
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0017, WSM_CMD_START_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* Start/stop find */
int wsm_start_find(struct bes2600_common *hw_priv, int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0019, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;
}

int wsm_stop_find(struct bes2600_common *hw_priv, int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x001A, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;
}

/* Map link */
int wsm_map_link(struct bes2600_common *hw_priv, const struct wsm_map_link *arg,
				 int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;
	u16 cmd = 0x001C;

	wsm_cmd_lock(hw_priv);
	WSM_PUT(buf, &arg->mac_addr[0], sizeof(arg->mac_addr));
	if (is_hardware_cw1250(hw_priv) || is_hardware_cw1260(hw_priv)) {
		WSM_PUT8(buf, arg->unmap);
		WSM_PUT8(buf, arg->link_id);
	} else {
		cmd |= WSM_TX_LINK_ID(arg->link_id);
		WSM_PUT16(buf, 0);
	}
	ret = wsm_cmd_send(hw_priv, buf, NULL, cmd, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* Update IE */
int wsm_update_ie(struct bes2600_common *hw_priv,
				  const struct wsm_update_ie *arg, int if_id)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT16(buf, arg->what);
	WSM_PUT16(buf, arg->count);
	WSM_PUT(buf, arg->ies, arg->length);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x001B, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* EPTA command */
int wsm_epta_cmd(struct bes2600_common *hw_priv, struct wsm_epta_msg *arg)
{
#ifdef WIFI_BT_COEXIST_EPTA_ENABLE
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;
	static bool epta_lock_tx = false;

	if (arg->hw_epta_enable & (1 << 11))
		arg->hw_epta_enable &= ~(3 << 10);
	else if (coex_is_fdd_mode())
		arg->hw_epta_enable |= (1 << 10);

	if (arg->hw_epta_enable != 3 || arg->hw_epta_enable != 4) {
		if (coex_is_wifi_inactive()) {
			arg->wlan_duration = 20000;
			arg->bt_duration = 80000;
			arg->hw_epta_enable &= ~(0x3);
		}
	}

	bes_devel("epta cmd: wlan:%d bt:%d enable:%x",
			  arg->wlan_duration, arg->bt_duration, arg->hw_epta_enable);

	if (arg->wlan_duration == 0 && !epta_lock_tx) {
		wsm_lock_tx(hw_priv);
		epta_lock_tx = true;
	} else if (epta_lock_tx && arg->wlan_duration != 0) {
		wsm_unlock_tx(hw_priv);
		epta_lock_tx = false;
	}

	wsm_cmd_lock(hw_priv);
	WSM_PUT32(buf, arg->wlan_duration);
	WSM_PUT32(buf, arg->bt_duration);
	WSM_PUT32(buf, arg->hw_epta_enable);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0029, WSM_CMD_TIMEOUT, 0);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
#else
	return 0;
#endif
}

/* WiFi channel command */
int wsm_epta_wifi_chan_cmd(struct bes2600_common *hw_priv, uint32_t channel,
						   uint32_t type)
{
#ifdef WIFI_BT_COEXIST_EPTA_ENABLE
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT32(buf, BES2600_RF_CMD_CH_INFO);
	WSM_PUT32(buf, channel);
	WSM_PUT16(buf, type);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0C27, WSM_CMD_TIMEOUT, 0);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
#else
	return 0;
#endif
}

/* WiFi status command */
int wsm_wifi_status_cmd(struct bes2600_common *hw_priv, uint32_t status)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT32(buf, BES2600_RF_CMD_WIFI_STATUS);
	WSM_PUT32(buf, status);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0C27, WSM_CMD_TIMEOUT, 0);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* CPU usage command */
int wsm_cpu_usage_cmd(struct bes2600_common *hw_priv)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT32(buf, BES2600_RF_CMD_CPU_USAGE);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0C27, WSM_CMD_TIMEOUT, 0);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* Save factory text to MCU */
int wsm_save_factory_txt_to_mcu(struct bes2600_common *hw_priv, const u8 *data,
								int if_id, enum bes2600_rf_cmd_type cmd_type)
{
	int ret, i;
	const struct factory_t *factory_cali = (const struct factory_t *)data;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	WSM_PUT32(buf, cmd_type);
	WSM_PUT32(buf, factory_cali->data.iQ_offset);
	WSM_PUT16(buf, factory_cali->data.freq_cal);
	for (i = 0; i < 3; i++)
		WSM_PUT16(buf, factory_cali->data.tx_power_ch[i]);
	WSM_PUT8(buf, factory_cali->data.freq_cal_flags);
	WSM_PUT8(buf, factory_cali->data.tx_power_type);
	WSM_PUT16(buf, factory_cali->data.temperature);
	for (i = 0; i < 13; i++)
		WSM_PUT16(buf, factory_cali->data.tx_power_ch_5G[i]);
	WSM_PUT16(buf, factory_cali->data.tx_power_flags_5G);
	for (i = 0; i < 4; i++)
		WSM_PUT32(buf, factory_cali->data.bt_tx_power[i]);
	WSM_PUT16(buf, factory_cali->data.temperature_5G);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0C27, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

/* Vendor RF commands */
#ifdef CONFIG_BES2600_TESTMODE
int wsm_vendor_rf_cmd(struct bes2600_common *hw_priv, int if_id,
					  const struct vendor_rf_cmd_t *vendor_rf_cmd)
{
	int ret;
	struct wsm_buf *buf = &hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(hw_priv);
	bes2600_pwr_set_busy_event(hw_priv, BES_PWR_LOCK_ON_TEST_CMD);
	WSM_PUT32(buf, vendor_rf_cmd->cmd_type);
	WSM_PUT32(buf, vendor_rf_cmd->cmd_argc);
	WSM_PUT32(buf, vendor_rf_cmd->cmd_len);
	WSM_PUT(buf, vendor_rf_cmd->cmd, vendor_rf_cmd->cmd_len);
	ret = wsm_cmd_send(hw_priv, buf, NULL, 0x0C25, WSM_CMD_TIMEOUT, if_id);
	wsm_cmd_unlock(hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(hw_priv);
	return -ENOMEM;
}

int wsm_vendor_rf_cmd_confirm(struct bes2600_common *hw_priv, void *arg,
							 struct wsm_buf *buf)
{
	return 0;
}

int wsm_vendor_rf_test_indication(struct bes2600_common *hw_priv, struct wsm_buf *buf)
{
	int i, ret = 0;
	u16 wsm_len;
	u32 cmd_type;
	struct wifi_power_cali_save_t power_cali_save;
	struct wifi_freq_cali_t wifi_freq_cali;
	struct wifi_get_power_cali_t power_cali_get;
	struct wifi_power_cali_flag_t power_cali_flag;
	struct wsm_mcu_hdr *msg_hdr = (struct wsm_mcu_hdr *)(buf->begin);

	wsm_len = __le16_to_cpu(msg_hdr->hdr.len);
	cmd_type = __le32_to_cpu(msg_hdr->cmd_type);
	buf->data += sizeof(struct wsm_mcu_hdr) - sizeof(struct wsm_hdr);

	switch (cmd_type) {
	case VENDOR_RF_SAVE_FREQOFFSET_CMD:
	case VENDOR_RF_GET_SAVE_FREQOFFSET_CMD:
		wifi_freq_cali.save_type = WSM_GET16(buf);
		wifi_freq_cali.freq_cali = WSM_GET16(buf);
		wifi_freq_cali.status = -WSM_GET16(buf);
		wifi_freq_cali.cali_flag = WSM_GET16(buf);
		if (wifi_freq_cali.save_type == RF_CALIB_DATA_IN_LINUX) {
			if (cmd_type == VENDOR_RF_SAVE_FREQOFFSET_CMD) {
#ifdef CONFIG_BES2600_CALIB_FROM_LINUX
				ret = bes2600_wifi_cali_freq_write(&wifi_freq_cali);
#else
				ret = -FACTORY_SAVE_FREQ_ERR;
#endif
			}
			if (cmd_type == VENDOR_RF_GET_SAVE_FREQOFFSET_CMD) {
#ifdef CONFIG_BES2600_CALIB_FROM_LINUX
				ret = vendor_get_freq_cali(&wifi_freq_cali);
#else
				ret = -FACTORY_SAVE_FILE_NOT_EXIST;
#endif
			}
			wifi_freq_cali.status = ret;
		}
		bes2600_rf_cmd_msg_assembly(cmd_type, &wifi_freq_cali,
									sizeof(struct wifi_freq_cali_t));
		break;
	case VENDOR_RF_SAVE_POWERLEVEL_CMD:
		power_cali_save.save_type = WSM_GET16(buf);
		power_cali_save.mode = WSM_GET16(buf);
		power_cali_save.bandwidth = WSM_GET16(buf);
		power_cali_save.band = WSM_GET16(buf);
		power_cali_save.ch = WSM_GET16(buf);
		power_cali_save.power_cali = WSM_GET16(buf);
		power_cali_save.status = -WSM_GET16(buf);
		if (power_cali_save.save_type == RF_CALIB_DATA_IN_LINUX) {
#ifdef CONFIG_BES2600_CALIB_FROM_LINUX
			ret = bes2600_wifi_power_cali_table_write(&power_cali_save);
#else
			ret = -FACTORY_SAVE_POWER_ERR;
#endif
			power_cali_save.status = ret;
		}
		bes2600_rf_cmd_msg_assembly(cmd_type, &power_cali_save,
									sizeof(struct wifi_power_cali_save_t));
		break;
	case VENDOR_RF_GET_SAVE_POWERLEVEL_CMD:
		power_cali_get.save_type = WSM_GET16(buf);
		if (power_cali_get.save_type == RF_CALIB_DATA_IN_LINUX) {
#ifdef CONFIG_BES2600_CALIB_FROM_LINUX
			ret = vendor_get_power_cali(&power_cali_get);
#else
			ret = -FACTORY_SAVE_FILE_NOT_EXIST;
#endif
			power_cali_get.status = ret;
		} else {
			for (i = 0; i < 3; i++)
				power_cali_get.tx_power_ch[i] = WSM_GET16(buf);
			for (i = 0; i < 13; i++)
				power_cali_get.tx_power_ch_5G[i] = WSM_GET16(buf);
			power_cali_get.status = -WSM_GET16(buf);
		}
		bes2600_rf_cmd_msg_assembly(cmd_type, &power_cali_get,
									sizeof(struct wifi_get_power_cali_t));
		break;
	case VENDOR_RF_POWER_CALIB_FINISH:
		power_cali_flag.save_type = WSM_GET16(buf);
		power_cali_flag.band = WSM_GET16(buf);
		power_cali_flag.status = -WSM_GET16(buf);
		if (power_cali_flag.save_type == RF_CALIB_DATA_IN_LINUX) {
#ifdef CONFIG_BES2600_CALIB_FROM_LINUX
			ret = vendor_set_power_cali_flag(&power_cali_flag);
#else
			ret = -FACTORY_SET_POWER_CALI_FLAG_ERR;
#endif
			power_cali_flag.status = ret;
		}
		bes2600_rf_cmd_msg_assembly(cmd_type, &power_cali_flag,
									sizeof(struct wifi_power_cali_flag_t));
		break;
	case VENDOR_RF_SIGNALING_CMD:
	case VENDOR_RF_NOSIGNALING_CMD:
	case VENDOR_RF_GET_CALI_FROM_EFUSE:
		bes2600_rf_cmd_msg_assembly(cmd_type, buf->data, wsm_len - sizeof(struct wsm_mcu_hdr));
		break;
	default:
		break;
	}

	up(&hw_priv->vendor_rf_cmd_replay_sema);
	bes2600_pwr_clear_busy_event(hw_priv, BES_PWR_LOCK_ON_TEST_CMD);
	return 0;

underflow:
	return -EINVAL;
}
#endif

/* Driver RF command confirmation */
int wsm_driver_rf_cmd_confirm(struct bes2600_common *hw_priv, void *arg,
							  struct wsm_buf *buf)
{
	u32 cmd_type;
	struct wsm_mcu_hdr *msg_hdr = (struct wsm_mcu_hdr *)(buf->begin);
	cmd_type = __le32_to_cpu(msg_hdr->cmd_type);
	buf->data += sizeof(struct wsm_mcu_hdr) - sizeof(struct wsm_hdr);

	if (cmd_type == BES2600_RF_CMD_CALI_TXT_TO_FLASH)
		return WSM_GET32(buf);
	return 0;

underflow:
	return -EINVAL;
}

/* RX handling helpers */
static int get_interface_id_scanning(struct bes2600_common *hw_priv)
{
	if (hw_priv->scan.req)
		return hw_priv->scan.if_id;
	if (hw_priv->scan.direct_probe == 1)
		return hw_priv->scan.if_id;
	return -1;
}

static int wsm_handle_rx_confirm(struct bes2600_common *hw_priv, int id,
								void *wsm_arg, struct wsm_buf *buf,
								int interface_link_id)
{
	int ret = 0;
	u16 wsm_cmd;

	spin_lock(&hw_priv->wsm_cmd.lock);
	wsm_arg = hw_priv->wsm_cmd.arg;
	wsm_cmd = hw_priv->wsm_cmd.cmd & ~WSM_TX_LINK_ID(WSM_TX_LINK_ID_MAX);
	hw_priv->wsm_cmd.cmd = 0xFFFF;
	spin_unlock(&hw_priv->wsm_cmd.lock);

	if ((id & 0x0f00) == 0x0400 && WARN_ON((id & ~0x0400) != wsm_cmd))
		return -EINVAL;

	switch (id) {
	case 0x0409:
		if (likely(wsm_arg))
			ret = wsm_configuration_confirm(hw_priv, wsm_arg, buf);
		break;
	case 0x0405:
		if (likely(wsm_arg))
			ret = wsm_read_mib_confirm(hw_priv, wsm_arg, buf);
		break;
	case 0x0406:
		if (likely(wsm_arg))
			ret = wsm_write_mib_confirm(hw_priv, wsm_arg, buf, interface_link_id);
		break;
	case 0x040B:
		if (likely(wsm_arg))
			ret = wsm_join_confirm(hw_priv, wsm_arg, buf);
		break;
#ifdef MCAST_FWDING
	case 0x0423:
		if (likely(wsm_arg)) {
			int i;
			struct bes2600_vif *priv;
			bes2600_for_each_vif(hw_priv, priv, i) {
				if (priv && priv->join_status == BES2600_JOIN_STATUS_AP) {
					spin_lock(&priv->vif_lock);
					ret = wsm_request_buffer_confirm(priv, wsm_arg, buf);
					spin_unlock(&priv->vif_lock);
				}
			}
		}
		break;
#endif
	case 0x0407: /* start-scan */
#ifdef ROAM_OFFLOAD
		if (hw_priv->auto_scanning && atomic_read(&hw_priv->scan.in_progress))
			hw_priv->auto_scanning = 0;
		else {
			wsm_oper_unlock(hw_priv);
			up(&hw_priv->scan.lock);
		}
#endif
	case 0x0408: /* stop-scan */
	case 0x040A: /* wsm_reset */
	case 0x040C: /* add_key */
	case 0x040D: /* remove_key */
	case 0x0410: /* wsm_set_pm */
	case 0x0411: /* set_bss_params */
	case 0x0412: /* set_tx_queue_params */
	case 0x0413: /* set_edca_params */
	case 0x0416: /* switch_channel */
	case 0x0417: /* start */
	case 0x0418: /* beacon_transmit */
	case 0x0419: /* start_find */
	case 0x041A: /* stop_find */
	case 0x041B: /* update_ie */
	case 0x041C: /* map_link */
	case 0x0429: /* epta */
		WARN_ON(wsm_arg != NULL);
		ret = wsm_generic_confirm(hw_priv, wsm_arg, buf);
		if (ret)
			wiphy_warn(hw_priv->hw->wiphy, "wsm_generic_confirm failed for 0x%.4X\n", id & ~0x0400);
		break;
#ifdef CONFIG_BES2600_TESTMODE
	case 0x0C25:
		ret = wsm_vendor_rf_cmd_confirm(hw_priv, wsm_arg, buf);
		break;
#endif
	case 0x0C27:
		ret = wsm_driver_rf_cmd_confirm(hw_priv, wsm_arg, buf);
		break;
	default:
		if (id == 0x0800)
			wsm_handle_exception(hw_priv, buf->data, buf->end - buf->data);
		else
			bes_err("[WSM] Unknown id: 0x%.4X\n", id);
		ret = -EINVAL;

	}

	spin_lock(&hw_priv->wsm_cmd.lock);
	hw_priv->wsm_cmd.ret = ret;
	hw_priv->wsm_cmd.done = 1;
	spin_unlock(&hw_priv->wsm_cmd.lock);
	wake_up(&hw_priv->wsm_cmd_wq);
	return 0;
}

static int wsm_handle_rx_indication(struct bes2600_common *hw_priv,
								   int interface_link_id, struct wsm_buf *buf,
								   struct sk_buff **skb_p)
{
	struct bes2600_vif *priv;
	struct wsm_rx rx = {
		.status = WSM_GET32(buf),
		.channelNumber = WSM_GET16(buf),
		.rxedRate = WSM_GET8(buf),
		.rcpiRssi = WSM_GET8(buf),
		.flags = WSM_GET32(buf),
	};
	struct ieee80211_hdr *hdr;
	size_t hdr_len;
	__le16 fctl;

	buf->data += 16; /* Skip LMAC header */
	if (is_hardware_cw1250(hw_priv) || is_hardware_cw1260(hw_priv)) {
		if (interface_link_id == CW12XX_GENERIC_IF_ID) {
			interface_link_id = get_interface_id_scanning(hw_priv);
			if (interface_link_id == -1)
				interface_link_id = hw_priv->roc_if_id;
#ifdef ROAM_OFFLOAD
			if (hw_priv->auto_scanning)
				interface_link_id = hw_priv->scan.if_id;
#endif
		}
		rx.link_id = ((rx.flags & (0xf << 25)) >> 25);
		rx.if_id = interface_link_id;
	} else {
		rx.link_id = interface_link_id;
		rx.if_id = 0;
	}

	if (rx.if_id == -1) {
		bes_devel("%s: intf is not match\n", __func__);
		return 0;
	}

	priv = cw12xx_hwpriv_to_vifpriv(hw_priv, rx.if_id);
	if (!priv) {
		bes_devel("%s: NULL priv drop frame\n", __func__);
		return 0;
	}

	spin_lock(&priv->vif_lock);
	hdr = (struct ieee80211_hdr *)buf->data;
	if (!rx.rcpiRssi && (ieee80211_is_probe_resp(hdr->frame_control) ||
						 ieee80211_is_beacon(hdr->frame_control))) {
		spin_unlock(&priv->vif_lock);
		return 0;
	}

	if (!priv->cqm_use_rssi) {
		s8 pkt_signal = rx.rcpiRssi / 2 - 110;
		rx.rcpiRssi = pkt_signal;
		if (ieee80211_is_data(hdr->frame_control)) {
			priv->signal_mul = priv->signal ? priv->signal_mul * 80 / 100 + pkt_signal * 20 : pkt_signal * 100;
			priv->signal = priv->signal_mul / 100;
			bes_devel("pkt signal:%d\n", priv->signal);
		}
	}

	fctl = *(__le16 *)buf->data;
	hdr_len = buf->data - buf->begin;
	skb_pull(*skb_p, hdr_len);

	if (!rx.status && (ieee80211_is_deauth(fctl) || ieee80211_is_disassoc(fctl))) {
		bool ignore = false;
		if (is_multicast_ether_addr(hdr->addr1)) {
			struct ieee80211_mmie *mmie;
			bool has_mmie = false;
			if ((*skb_p)->len >= 24 + sizeof(*mmie)) {
				mmie = (struct ieee80211_mmie *)((*skb_p)->data + (*skb_p)->len - sizeof(*mmie));
				if (mmie->element_id == WLAN_EID_MMIE && mmie->length == sizeof(*mmie) - 2)
					has_mmie = true;
			}
			bes_devel("[WSM] RX broadcast/multicast deauth: len=%d, has_mmie:%u, pmf=%d\n",
					  (*skb_p)->len, has_mmie, priv->pmf);
			if (has_mmie ^ priv->pmf)
				ignore = true;
		} else if (ether_addr_equal(hdr->addr1, priv->vif->addr)) {
			bool has_protected = ieee80211_has_protected(fctl);
			bes_devel("[WSM] RX unicast deauth: protected=%d, pmf=%d, connect_in_process=%d\n",
					  has_protected, priv->pmf, atomic_read(&priv->connect_in_process));
			if ((has_protected ^ priv->pmf) || atomic_read(&priv->connect_in_process))
				ignore = true;
		} else {
			ignore = true;
		}

		if (!ignore && priv->join_status == BES2600_JOIN_STATUS_STA &&
			(ether_addr_equal(hdr->addr3, priv->join_bssid) || ether_addr_equal(hdr->addr3, priv->bssid))) {
			bes_devel("[WSM] Issue unjoin command (RX).\n");
			wsm_lock_tx_async(hw_priv);
			if (queue_work(hw_priv->workqueue, &priv->unjoin_work) <= 0)
				wsm_unlock_tx(hw_priv);
#ifdef CONFIG_PM
			else if (bes2600_suspend_status_get(hw_priv))
				bes2600_pending_unjoin_set(hw_priv, priv->if_id);
#endif
			if (bes2600_chrdev_wakeup_by_event_get() == WAKEUP_EVENT_PEER_DETACH)
				bes2600_chrdev_wifi_update_wakeup_reason(WAKEUP_REASON_WIFI_DEAUTH_DISASSOC, 0);
		}
		bes2600_chrdev_wakeup_by_event_set(WAKEUP_EVENT_NONE);
	}

	hw_priv->wsm_cbc.rx(priv, &rx, skb_p);
	if (*skb_p)
		skb_push(*skb_p, hdr_len);
	spin_unlock(&priv->vif_lock);
	return 0;

underflow:
	spin_unlock(&priv->vif_lock);
	return -EINVAL;
}

static int wsm_handle_event_indication(struct bes2600_common *hw_priv,
									  struct wsm_buf *buf, int interface_link_id)
{
	struct bes2600_wsm_event *event;
	int first;

	if (!is_hardware_cw1250(hw_priv) && !is_hardware_cw1260(hw_priv))
		interface_link_id = 0;

	struct bes2600_vif *priv = cw12xx_hwpriv_to_vifpriv(hw_priv, interface_link_id);
	if (unlikely(!priv)) {
		bes_devel("[WSM] Not find corresponding interface\n");
		return 0;
	}
	spin_lock(&priv->vif_lock);
	if (unlikely(priv->mode == NL80211_IFTYPE_UNSPECIFIED)) {
		spin_unlock(&priv->vif_lock);
		return 0;
	}
	spin_unlock(&priv->vif_lock);

	event = kzalloc(sizeof(struct bes2600_wsm_event), GFP_KERNEL);
	if (!event)
		return -ENOMEM;

	event->evt.eventId = __le32_to_cpu(WSM_GET32(buf));
	event->evt.eventData = __le32_to_cpu(WSM_GET32(buf));
	event->if_id = interface_link_id;
	bes_devel("[WSM] Event: %d(%d)\n", event->evt.eventId, event->evt.eventData);

	spin_lock(&hw_priv->event_queue_lock);
	first = list_empty(&hw_priv->event_queue);
	list_add_tail(&event->link, &hw_priv->event_queue);
	spin_unlock(&hw_priv->event_queue_lock);

	if (first)
		queue_work(hw_priv->workqueue, &hw_priv->event_handler);
	return 0;

underflow:
	kfree(event);
	return -EINVAL;
}

static int wsm_channel_switch_indication(struct bes2600_common *hw_priv,
										struct wsm_buf *buf)
{
	if (WARN_ON(WSM_GET32(buf)))
		return -EINVAL;
	wsm_unlock_tx(hw_priv);
	hw_priv->channel_switch_in_progress = 0;
	wake_up(&hw_priv->channel_switch_done);
	if (hw_priv->wsm_cbc.channel_switch)
		hw_priv->wsm_cbc.channel_switch(hw_priv);
	return 0;

underflow:
	return -EINVAL;
}

static int wsm_set_pm_indication(struct bes2600_common *hw_priv,
								 struct wsm_buf *buf)
{
	struct wsm_set_pm_complete arg = {
		.status = WSM_GET32(buf),
		.psm = WSM_GET8(buf),
	};

	if (arg.status == WSM_STATUS_SUCCESS)
		bes2600_pwr_notify_ps_changed(hw_priv, arg.psm);
	else
		bes_err("[WSM] PM Ind status:%d psm:%d\n", arg.status, arg.psm);
	return 0;

underflow:
	return -EINVAL;
}

static int wsm_scan_complete_indication(struct bes2600_common *hw_priv,
									   struct wsm_buf *buf)
{
#ifdef ROAM_OFFLOAD
	if (hw_priv->auto_scanning == 0)
		wsm_oper_unlock(hw_priv);
#else
	wsm_oper_unlock(hw_priv);
#endif
	if (hw_priv->wsm_cbc.scan_complete) {
		struct wsm_scan_complete arg = {
			.status = WSM_GET32(buf),
			.psm = WSM_GET8(buf),
			.numChannels = WSM_GET8(buf),
		};
		hw_priv->wsm_cbc.scan_complete(hw_priv, &arg);
	}
	return 0;

underflow:
	return -EINVAL;
}

static int wsm_find_complete_indication(struct bes2600_common *hw_priv,
										struct wsm_buf *buf)
{
	return 0; /* Stub */
}

static int wsm_suspend_resume_indication(struct bes2600_common *hw_priv,
										int interface_link_id, struct wsm_buf *buf)
{
	if (hw_priv->wsm_cbc.suspend_resume) {
		struct wsm_suspend_resume arg;
		struct bes2600_vif *priv;
		u32 flags = WSM_GET32(buf);

		if (is_hardware_cw1250(hw_priv) || is_hardware_cw1260(hw_priv)) {
			int i;
			arg.if_id = interface_link_id;
			bes2600_for_each_vif(hw_priv, priv, i) {
				if (!priv)
					continue;
				if (priv->join_status == BES2600_JOIN_STATUS_AP) {
					arg.if_id = priv->if_id;
					break;
				}
				arg.link_id = 0;
			}
		} else {
			arg.if_id = 0;
			arg.link_id = interface_link_id;
		}

		arg.stop = !(flags & 1);
		arg.multicast = !!(flags & 8);
		arg.queue = (flags >> 1) & 3;

		priv = cw12xx_hwpriv_to_vifpriv(hw_priv, arg.if_id);
		if (unlikely(!priv)) {
			bes_devel("[WSM] suspend-resume indication for removed interface!\n");
			return 0;
		}
		spin_lock(&priv->vif_lock);
		hw_priv->wsm_cbc.suspend_resume(priv, &arg);
		spin_unlock(&priv->vif_lock);
	}
	return 0;

underflow:
	return -EINVAL;
}

#ifdef MCAST_FWDING
static int wsm_give_buffer_confirm(struct bes2600_common *hw_priv,
								   struct wsm_buf *buf)
{
	bes_devel("[WSM] HW Buf count %d\n", hw_priv->hw_bufs_used);
	if (!hw_priv->hw_bufs_used)
		wake_up(&hw_priv->bh_evt_wq);
	return 0;
}

int wsm_init_release_buffer_request(struct bes2600_common *hw_priv, u8 index)
{
	struct wsm_buf *buf = &hw_priv->wsm_release_buf[index];
	size_t buf_len;

	wsm_buf_init(buf);
	WSM_PUT8(buf, index ? 0 : 0x1);
	WSM_PUT8(buf, 0);
	WSM_PUT16(buf, 0);
	buf_len = buf->data - buf->begin;
	((__le16 *)buf->begin)[0] = __cpu_to_le16(buf_len);
	((__le16 *)buf->begin)[1] = __cpu_to_le16(0x0022);
	return 0;

nomem:
	return -ENOMEM;
}

int wsm_request_buffer_request(struct bes2600_vif *priv, u8 *arg)
{
	int ret;
	struct wsm_buf *buf = &priv->hw_priv->wsm_cmd_buf;

	wsm_cmd_lock(priv->hw_priv);
	WSM_PUT8(buf, (*arg));
	WSM_PUT8(buf, 0);
	WSM_PUT16(buf, 0);
	ret = wsm_cmd_send(priv->hw_priv, buf, arg, 0x0023, WSM_CMD_JOIN_TIMEOUT, priv->if_id);
	wsm_cmd_unlock(priv->hw_priv);
	return ret;

nomem:
	wsm_cmd_unlock(priv->hw_priv);
	return -ENOMEM;
}

static int wsm_request_buffer_confirm(struct bes2600_vif *priv, u8 *arg,
									 struct wsm_buf *buf)
{
	u8 count;
	u32 sta_asleep_mask = WSM_GET32(buf);
	count = WSM_GET8(buf) - 1; /* FW workaround */
	u32 change_mask, mask;
	int i, ret = 0;
	struct bes2600_common *hw_priv = priv->hw_priv;

	spin_lock_bh(&priv->ps_state_lock);
	change_mask = priv->sta_asleep_mask ^ sta_asleep_mask;
	bes_devel("CM %x, HM %x, FWM %x\n", change_mask, priv->sta_asleep_mask, sta_asleep_mask);
	spin_unlock_bh(&priv->ps_state_lock);

	if (change_mask) {
		struct ieee80211_sta *sta;
		rcu_read_lock();
		for (i = 0; i < CW1250_MAX_STA_IN_AP_MODE; ++i) {
			if (priv->link_id_db[i].status != BES2600_LINK_HARD)
				continue;
			mask = BIT(i + 1);
			if (change_mask & mask) {
				bes_devel("PS State Changed %d for sta %pM\n",
						  (sta_asleep_mask & mask) ? 1 : 0, priv->link_id_db[i].mac);
				sta = ieee80211_find_sta(priv->vif, priv->link_id_db[i].mac);
				if (!sta) {
					bes_err("[WSM] WRBC - could not find sta %pM\n",
							priv->link_id_db[i].mac);
				} else {
					ret = ieee80211_sta_ps_transition_ni(sta, (sta_asleep_mask & mask) ? true : false);
					bes_devel("PS State NOTIFIED %d\n", ret);
					WARN_ON(ret);
				}
			}
		}
		rcu_read_unlock();
	}

	spin_lock_bh(&priv->ps_state_lock);
	priv->sta_asleep_mask = sta_asleep_mask;
	spin_unlock_bh(&priv->ps_state_lock);
	bes_devel("[WSM] WRBC - HW Buf count %d SleepMask %d\n",
			  hw_priv->hw_bufs_used, sta_asleep_mask);
	hw_priv->buf_released = 0;
	WARN_ON(count != (hw_priv->wsm_caps.numInpChBufs - 1));
	return ret;

underflow:
	WARN_ON(1);
	return -EINVAL;
}
#endif

/* TX data handling */
static int wsm_handle_tx_probe(struct bes2600_common *hw_priv,
							   struct bes2600_vif *priv, struct wsm_tx *wsm,
							   const struct bes2600_txpriv *txpriv,
							   struct bes2600_queue *queue)
{
#ifdef CONFIG_BES2600_TESTMODE
	if (hw_priv->enable_advance_scan &&
		priv->join_status == BES2600_JOIN_STATUS_STA &&
		hw_priv->advanceScanElems.scanMode == BES2600_SCAN_MEASUREMENT_ACTIVE)
		return 0; /* Transmit probe in active scan */
#endif
	bes_devel("[WSM] Convert probe request to scan.\n");
	wsm_lock_tx_async(hw_priv);
	hw_priv->pending_frame_id = __le32_to_cpu(wsm->packetID);
	queue_delayed_work(hw_priv->workqueue, &hw_priv->scan.probe_work, 0);
	return 1;
}

static int wsm_handle_tx_drop(struct bes2600_common *hw_priv,
							  struct wsm_tx *wsm, struct bes2600_queue *queue)
{
#ifdef CONFIG_BES2600_TESTMODE
	BUG_ON(bes2600_queue_remove(hw_priv, queue, __le32_to_cpu(wsm->packetID)));
#else
	BUG_ON(bes2600_queue_remove(queue, __le32_to_cpu(wsm->packetID)));
#endif
	return 1;
}

static int wsm_handle_tx_join(struct bes2600_common *hw_priv,
							  struct bes2600_vif *priv, struct wsm_tx *wsm)
{
	bes_devel("[WSM] Issue join command.\n");
	wsm_lock_tx_async(hw_priv);
	hw_priv->pending_frame_id = __le32_to_cpu(wsm->packetID);
#ifdef WIFI_BT_COEXIST_EPTA_ENABLE
	if (hw_priv->channel->band != NL80211_BAND_2GHZ)
		bwifi_change_current_status(hw_priv, BWIFI_STATUS_CONNECTING_5G);
	else
		bwifi_change_current_status(hw_priv, BWIFI_STATUS_CONNECTING);
#endif
	if (queue_work(hw_priv->workqueue, &priv->join_work) <= 0)
		wsm_unlock_tx(hw_priv);
	return 1;
}

static int wsm_handle_tx_wep(struct bes2600_common *hw_priv,
							 struct bes2600_vif *priv, struct wsm_tx *wsm,
							 const struct ieee80211_tx_info *tx_info)
{
	bes_devel("[WSM] Issue set_default_wep_key.\n");
	wsm_lock_tx_async(hw_priv);
	priv->wep_default_key_id = tx_info->control.hw_key->keyidx;
	hw_priv->pending_frame_id = __le32_to_cpu(wsm->packetID);
	if (queue_work(hw_priv->workqueue, &priv->wep_key_work) <= 0)
		wsm_unlock_tx(hw_priv);
	return 1;
}

static int wsm_handle_tx_deauth(struct bes2600_common *hw_priv,
								struct bes2600_vif *priv, struct wsm_tx *wsm)
{
	bes_devel("[WSM] Issue unjoin command (TX).\n");
	atomic_set(&priv->connect_in_process, 0);
#ifdef WIFI_BT_COEXIST_EPTA_ENABLE
	bwifi_change_current_status(hw_priv, BWIFI_STATUS_DISCONNECTING);
#endif
	wsm_lock_tx_async(hw_priv);
	if (queue_work(hw_priv->workqueue, &priv->unjoin_work) <= 0)
		wsm_unlock_tx(hw_priv);
	return 1;
}

static bool wsm_handle_tx_data(struct bes2600_vif *priv, const struct wsm_tx *wsm,
							   const struct ieee80211_tx_info *tx_info,
							   struct bes2600_txpriv *txpriv,
							   struct bes2600_queue *queue)
{
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	struct ieee80211_hdr *frame = (struct ieee80211_hdr *)&((u8 *)wsm)[txpriv->offset];
	__le16 fctl = frame->frame_control;
	bool handled = false;

	spin_lock(&priv->vif_lock);
	switch (priv->mode) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_DEVICE:
		if (unlikely(priv->join_status == BES2600_JOIN_STATUS_STA &&
					 ieee80211_is_nullfunc(fctl))) {
			spin_lock(&priv->bss_loss_lock);
			if (priv->bss_loss_status == BES2600_BSS_LOSS_CHECKING) {
				priv->bss_loss_status = BES2600_BSS_LOSS_CONFIRMING;
				priv->bss_loss_confirm_id = wsm->packetID;
			}
			spin_unlock(&priv->bss_loss_lock);
		} else if (unlikely(priv->join_status <= BES2600_JOIN_STATUS_MONITOR ||
						   memcmp(frame->addr1, priv->join_bssid, sizeof(priv->join_bssid)))) {
#ifdef P2P_MULTIVIF
			struct bes2600_vif *p2p_if_vif = __cw12xx_hwpriv_to_vifpriv(hw_priv, 2);
			if (p2p_if_vif && p2p_if_vif->join_status > BES2600_JOIN_STATUS_MONITOR &&
				priv->join_status < BES2600_JOIN_STATUS_MONITOR) {
				txpriv->raw_if_id = 0;
			} else
#endif
			if (ieee80211_is_auth(fctl))
				handled = wsm_handle_tx_join(hw_priv, priv, (struct wsm_tx *)wsm);
			else if (ieee80211_is_probe_req(fctl))
				handled = wsm_handle_tx_probe(hw_priv, priv, (struct wsm_tx *)wsm, txpriv, queue);
			else if (memcmp(frame->addr1, priv->join_bssid, sizeof(priv->join_bssid)) &&
					 priv->join_status == BES2600_JOIN_STATUS_STA && ieee80211_is_data(fctl))
				handled = wsm_handle_tx_drop(hw_priv, (struct wsm_tx *)wsm, queue);
			else if (priv->join_status >= BES2600_JOIN_STATUS_MONITOR)
				handled = false;
			else if (get_interface_id_scanning(hw_priv) != -1) {
				wiphy_warn(priv->hw->wiphy, "Scan ONGOING dropping offchannel eligible frame.\n");
				handled = wsm_handle_tx_drop(hw_priv, (struct wsm_tx *)wsm, queue);
			}
		}
		break;
	case NL80211_IFTYPE_AP:
		if (unlikely(!priv->join_status) ||
			unlikely(!(BIT(txpriv->raw_link_id) & (BIT(0) | priv->link_id_map))) ||
			bes2600_queue_get_generation(wsm->packetID) > BES2600_MAX_REQUEUE_ATTEMPTS) {
			wiphy_warn(priv->hw->wiphy, "Frame dropped: no join, expired link, or too many retries.\n");
			handled = wsm_handle_tx_drop(hw_priv, (struct wsm_tx *)wsm, queue);
		}
		break;
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_MESH_POINT:
	case NL80211_IFTYPE_MONITOR:
	default:
		handled = wsm_handle_tx_drop(hw_priv, (struct wsm_tx *)wsm, queue);
		break;
	}

	if (!handled && ieee80211_is_probe_req(fctl))
		handled = wsm_handle_tx_probe(hw_priv, priv, (struct wsm_tx *)wsm, txpriv, queue);
	else if (!handled && (fctl & __cpu_to_le32(IEEE80211_FCTL_PROTECTED)) &&
			tx_info->control.hw_key &&
			tx_info->control.hw_key->keyidx != priv->wep_default_key_id &&
			(tx_info->control.hw_key->cipher == WLAN_CIPHER_SUITE_WEP40 ||
			 tx_info->control.hw_key->cipher == WLAN_CIPHER_SUITE_WEP104))
		handled = wsm_handle_tx_wep(hw_priv, priv, (struct wsm_tx *)wsm, tx_info);
	else if (!handled && (ieee80211_is_deauth(fctl) || ieee80211_is_disassoc(fctl)))
		handled = wsm_handle_tx_deauth(hw_priv, priv, (struct wsm_tx *)wsm);

	spin_unlock(&priv->vif_lock);
	return handled;
}

/* RX handling */
int wsm_handle_rx(struct bes2600_common *hw_priv, int id,
				  struct wsm_hdr *wsm, struct sk_buff **skb_p)
{
	int ret = 0;
	struct wsm_buf buf;
	int interface_link_id = (id >> 6) & WSM_TX_LINK_ID_MAX;

	buf.begin = (u8 *)wsm;
	buf.data = buf.begin + sizeof(struct wsm_hdr);
	buf.end = buf.begin + __le16_to_cpu(wsm->len);

	bes_info("[WSM] Raw ID: 0x%.4X, len: %d\n", id, __le16_to_cpu(wsm->len)); // Debug raw ID
	id = WSM_MSG_ID_GET(id);
	bes_info("[WSM] Parsed ID: 0x%.4X\n", id); // Debug parsed ID
	if (id & 0x0400) {
		ret = wsm_handle_rx_confirm(hw_priv, id, NULL, &buf, interface_link_id);
	} else {
		switch (id) {
		case 0x0801:
			bes_info("[WSM] Ignoring unknown message ID: 0x%.4X, len: %d\n", id, __le16_to_cpu(wsm->len));
			print_hex_dump(KERN_DEBUG, "[WSM] Unknown msg: ", DUMP_PREFIX_OFFSET, 16, 1, buf.data, buf.end - buf.data, false);
			return 0;
		case 0x0804:
			ret = wsm_handle_rx_indication(hw_priv, interface_link_id, &buf, skb_p);
			break;
		case 0x0805:
			ret = wsm_handle_event_indication(hw_priv, &buf, interface_link_id);
			break;
		case 0x0806:
			ret = wsm_channel_switch_indication(hw_priv, &buf);
			break;
		case 0x0809:
			ret = wsm_set_pm_indication(hw_priv, &buf);
			break;
		case 0x080A:
			ret = wsm_scan_complete_indication(hw_priv, &buf);
			break;
		case 0x080B:
			ret = wsm_find_complete_indication(hw_priv, &buf);
			break;
		case 0x080C:
			ret = wsm_suspend_resume_indication(hw_priv, interface_link_id, &buf);
			break;
#ifdef MCAST_FWDING
		case 0x0822:
			ret = wsm_give_buffer_confirm(hw_priv, &buf);
			break;
#endif
#ifdef CONFIG_BES2600_TESTMODE
		case 0x0C26:
			ret = wsm_vendor_rf_test_indication(hw_priv, &buf);
			break;
#endif
		case 0x0C31:
			ret = wsm_handle_event_indication(hw_priv, &buf, interface_link_id);
			break;
		default:
			if (id == 0x0800)
				wsm_handle_exception(hw_priv, buf.data, buf.end - buf.data);
			else
				bes_err("[WSM] Unknown id: 0x%.4X\n", id);
			ret = -EINVAL;
		}
	}
	return ret;
}

/* Exception handling */
int wsm_handle_exception(struct bes2600_common *hw_priv, u8 *data, size_t len)
{
	struct wsm_buf buf;
	u32 code;

	if (WARN_ON(len < 4))
		return -EINVAL;

	buf.begin = data;
	buf.data = buf.begin;
	buf.end = &buf.begin[len];

	code = WSM_GET32(&buf);
	bes_err("[WSM] Exception: 0x%.8X\n", code);
	print_hex_dump(KERN_DEBUG, "EXCEPTION> ", DUMP_PREFIX_OFFSET, 16, 1,
				   buf.data, buf.end - buf.data, false);
	return 0;

underflow:
	WARN_ON(1);
	return -EINVAL;
}

/* Command sending */
int wsm_cmd_send(struct bes2600_common *hw_priv, struct wsm_buf *buf,
				 void *arg, u16 cmd, long tmo, int if_id)
{
	int ret;

	if (is_hardware_cw1250(hw_priv) || is_hardware_cw1260(hw_priv)) {
		if (if_id == CW12XX_GENERIC_IF_ID)
			if_id = get_interface_id_scanning(hw_priv);
		cmd |= WSM_TX_IF_ID(if_id);
	} else if (if_id >= 0) {
		cmd |= WSM_TX_LINK_ID(if_id);
	}

	spin_lock(&hw_priv->wsm_cmd.lock);
	BUG_ON(hw_priv->wsm_cmd.ptr);
	buf->begin -= 4;
	((__le16 *)buf->begin)[0] = __cpu_to_le16(buf->data - buf->begin);
	((__le16 *)buf->begin)[1] = __cpu_to_le16(cmd);
	hw_priv->wsm_cmd.arg = arg;
	hw_priv->wsm_cmd.cmd = cmd;
	hw_priv->wsm_cmd.ptr = buf->begin;
	hw_priv->wsm_cmd.len = buf->data - buf->begin;
	bes2600_bh_wakeup(hw_priv);
	spin_unlock(&hw_priv->wsm_cmd.lock);

	tmo = wait_event_timeout(hw_priv->wsm_cmd_wq, hw_priv->wsm_cmd.done, tmo);
	if (unlikely(tmo == 0)) {
		spin_lock(&hw_priv->wsm_cmd.lock);
		ret = (hw_priv->wsm_cmd.ptr) ? -ETIMEDOUT : -EBUSY;
		hw_priv->wsm_cmd.ptr = NULL;
		hw_priv->wsm_cmd.done = 0;
		hw_priv->wsm_cmd.arg = NULL;
		hw_priv->wsm_cmd.cmd = 0xFFFF;
		spin_unlock(&hw_priv->wsm_cmd.lock);
	} else {
		spin_lock(&hw_priv->wsm_cmd.lock);
		ret = hw_priv->wsm_cmd.ret;
		hw_priv->wsm_cmd.ptr = NULL;
		hw_priv->wsm_cmd.done = 0;
		hw_priv->wsm_cmd.arg = NULL;
		hw_priv->wsm_cmd.cmd = 0xFFFF;
		spin_unlock(&hw_priv->wsm_cmd.lock);
	}

	wsm_buf_reset(buf);
	return ret;
}

/* TX queue handling */
static int bes2600_get_prio_queue(struct bes2600_vif *priv,
								  u32 link_id_map, int *total)
{
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	static u32 urgent;
	struct wsm_edca_queue_params *edca;
	unsigned score, best = -1;
	int winner = -1;
	size_t queued;
	int i;
	urgent = BIT(priv->link_id_after_dtim) | BIT(priv->link_id_uapsd);

	for (i = 0; i < 4; ++i) {
		queued = bes2600_queue_get_num_queued(priv, &hw_priv->tx_queue[i], link_id_map);
		if (!queued)
			continue;
		*total += queued;
		edca = &priv->edca.params[i];
		score = ((edca->aifns + edca->cwMin) << 16) +
				(edca->cwMax - edca->cwMin) * (get_random_u32() & 0xFFFF);
		if (score < best && (winner < 0 || i != 3)) {
			best = score;
			winner = i;
		}
	}

	if (winner >= 0 && hw_priv->tx_burst_idx >= 0 &&
		winner != hw_priv->tx_burst_idx &&
		!bes2600_queue_get_num_queued(priv, &hw_priv->tx_queue[winner], link_id_map & urgent) &&
		bes2600_queue_get_num_queued(priv, &hw_priv->tx_queue[hw_priv->tx_burst_idx], link_id_map))
		winner = hw_priv->tx_burst_idx;

	return winner;
}

static int wsm_get_tx_queue_and_mask(struct bes2600_vif *priv,
									 struct bes2600_queue **queue_p,
									 u32 *tx_allowed_mask_p,
									 bool *more)
{
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	int idx;
	u32 tx_allowed_mask;
	int total = 0;

	if (priv->tx_multicast) {
		tx_allowed_mask = BIT(priv->link_id_after_dtim);
		idx = bes2600_get_prio_queue(priv, tx_allowed_mask, &total);
		if (idx >= 0) {
			*more = total > 1;
			goto found;
		}
	}

	tx_allowed_mask = ~priv->sta_asleep_mask;
	tx_allowed_mask |= BIT(priv->link_id_uapsd);
	if (priv->sta_asleep_mask) {
		tx_allowed_mask |= priv->pspoll_mask;
		tx_allowed_mask &= ~BIT(priv->link_id_after_dtim);
	} else {
		tx_allowed_mask |= BIT(priv->link_id_after_dtim);
	}
	idx = bes2600_get_prio_queue(priv, tx_allowed_mask, &total);
	if (idx < 0)
		return -ENOENT;

found:
	*queue_p = &hw_priv->tx_queue[idx];
	*tx_allowed_mask_p = tx_allowed_mask;
	return 0;
}

static struct bes2600_vif *wsm_select_vif(struct bes2600_common *hw_priv)
{
	struct bes2600_vif *priv = NULL;
	int i = hw_priv->if_id_selected;
	int num_vifs = CW12XX_MAX_VIFS;

	spin_lock(&hw_priv->vif_list_lock);
	for (int j = 0; j < num_vifs; j++) {
		i = (i + 1) % num_vifs;
		if (hw_priv->vif_list[i]) {
			priv = cw12xx_get_vif_from_ieee80211(hw_priv->vif_list[i]);
			if (priv && atomic_read(&priv->enabled)) {
				hw_priv->if_id_selected = i;
				break;
			}
		}
	}
	spin_unlock(&hw_priv->vif_list_lock);

	return priv;
}

static bool wsm_can_transmit(struct bes2600_common *hw_priv)
{
	if (atomic_read(&hw_priv->tx_lock))
		return false;
	if (hw_priv->hw_bufs_used >= hw_priv->wsm_caps.numInpChBufs)
		return false;
	return true;
}

static int wsm_select_queue(struct bes2600_vif *priv, struct bes2600_queue **queue,
							u32 *tx_allowed_mask, bool *more)
{
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	int ret;

	spin_lock_bh(&priv->ps_state_lock);
	ret = wsm_get_tx_queue_and_mask(priv, queue, tx_allowed_mask, more);
	if (!ret && priv->buffered_multicasts && (!priv->tx_multicast || !priv->sta_asleep_mask)) {
		priv->buffered_multicasts = false;
		if (priv->tx_multicast) {
			priv->tx_multicast = false;
			queue_work(hw_priv->workqueue, &priv->multicast_stop_work);
		}
	}
	spin_unlock_bh(&priv->ps_state_lock);

	return ret;
}

static int wsm_fetch_packet(struct bes2600_vif *priv, struct bes2600_queue *queue,
							u32 tx_allowed_mask, u8 **data, size_t *tx_len,
							int *vif_selected, unsigned int *burst)
{
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	struct wsm_tx *wsm;
	struct ieee80211_tx_info *tx_info;
	struct bes2600_txpriv *txpriv;
	struct ieee80211_hdr *hdr;
	int queue_num = queue - hw_priv->tx_queue;
	bool more = false;

	spin_lock(&priv->vif_lock);
	if (bes2600_queue_get(queue, priv->if_id, tx_allowed_mask, &wsm, &tx_info, &txpriv)) {
		spin_unlock(&priv->vif_lock);
		return 0;
	}

	if (wsm_handle_tx_data(priv, wsm, tx_info, txpriv, queue)) {
		spin_unlock(&priv->vif_lock);
		return 0;
	}

	wsm->hdr.id &= __cpu_to_le16(~WSM_TX_IF_ID(WSM_TX_IF_ID_MAX));
	wsm->hdr.id |= cpu_to_le16(WSM_TX_IF_ID(txpriv->raw_if_id ? txpriv->raw_if_id : priv->if_id));

	*vif_selected = priv->if_id;
	*data = (u8 *)wsm;
	*tx_len = __le16_to_cpu(wsm->hdr.len);

	if (priv->edca.params[queue_num].txOpLimit)
		*burst = min(*burst, bes2600_queue_get_num_queued(priv, queue, tx_allowed_mask) + 1);
	else
		*burst = 1;

	hw_priv->tx_burst_idx = (*burst > 1) ? queue_num : -1;

	if (more) {
		hdr = (struct ieee80211_hdr *)&((u8 *)wsm)[txpriv->offset];
		if (strstr(&priv->ssid[0], "6.1.12") && (hdr->addr1[0] & 0x01))
			hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_MOREDATA);
		else
			hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_MOREDATA);
	}

	priv->pspoll_mask &= ~BIT(txpriv->raw_link_id);
	spin_unlock(&priv->vif_lock);

	return 1;
}

static int wsm_handle_cmd(struct bes2600_common *hw_priv, u8 **data, size_t *tx_len,
						  unsigned int *burst, int *vif_selected)
{
	spin_lock(&hw_priv->wsm_cmd.lock);
	if (hw_priv->wsm_cmd.ptr) {
		*data = hw_priv->wsm_cmd.ptr;
		*tx_len = hw_priv->wsm_cmd.len;
		*burst = 1;
		*vif_selected = -1;
		spin_unlock(&hw_priv->wsm_cmd.lock);
		return 1;
	}
	spin_unlock(&hw_priv->wsm_cmd.lock);
	return 0;
}

int wsm_get_tx(struct bes2600_common *hw_priv, u8 **data, size_t *tx_len,
			   unsigned int *burst, int *vif_selected)
{
	struct bes2600_vif *priv;
	struct bes2600_queue *queue;
	u32 tx_allowed_mask;
	bool more;
	int count = 0;
	int num_vifs = CW12XX_MAX_VIFS;

	count = bes2600_itp_get_tx(hw_priv, data, tx_len, burst);
	if (count)
		return count;

	count = wsm_handle_cmd(hw_priv, data, tx_len, burst, vif_selected);
	if (count)
		return count;

	for (int i = 0; i < num_vifs; i++) {
		if (!wsm_can_transmit(hw_priv))
			break;

		priv = wsm_select_vif(hw_priv);
		if (!priv)
			break;

		if (wsm_select_queue(priv, &queue, &tx_allowed_mask, &more))
			continue;

		count = wsm_fetch_packet(priv, queue, tx_allowed_mask, data, tx_len,
								 vif_selected, burst);
		if (count)
			break;
	}

	return count;
}

void wsm_txed(struct bes2600_common *hw_priv, u8 *data)
{
	if (data == hw_priv->wsm_cmd.ptr) {
		spin_lock(&hw_priv->wsm_cmd.lock);
		hw_priv->wsm_cmd.ptr = NULL;
		spin_unlock(&hw_priv->wsm_cmd.lock);
	}
}

int wsm_set_probe_responder(struct bes2600_vif *priv, bool enable)
{
		struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);

		priv->rx_filter.probeResponder = enable;
		return wsm_set_rx_filter(hw_priv, &priv->rx_filter, priv->if_id);
}

int wsm_set_keepalive_filter(struct bes2600_vif *priv, bool enable)
{
		struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);

		priv->rx_filter.keepalive = enable;
		return wsm_set_rx_filter(hw_priv, &priv->rx_filter, priv->if_id);
}

void wsm_lock_tx(struct bes2600_common *hw_priv)
{
		wsm_cmd_lock(hw_priv);
		if (atomic_add_return(1, &hw_priv->tx_lock) == 1) {
				if (wsm_flush_tx(hw_priv))
						bes_devel("[WSM] TX is locked.\n");
		}
		wsm_cmd_unlock(hw_priv);
}

void wsm_lock_tx_async(struct bes2600_common *hw_priv)
{
		if (atomic_add_return(1, &hw_priv->tx_lock) == 1)
				bes_devel("[WSM] TX is locked (async).\n");
}

void wsm_unlock_tx(struct bes2600_common *hw_priv)
{
		int tx_lock;
		if (atomic_read(&hw_priv->bh_error))
				bes_err("fatal error occured, unlock is unsafe\n");
		else {
				tx_lock = atomic_sub_return(1, &hw_priv->tx_lock);
				if (tx_lock < 0) {
						BUG_ON(1);
				} else if (tx_lock == 0) {
						bes2600_bh_wakeup(hw_priv);
						bes_devel("[WSM] TX is unlocked.\n");
				}
		}
}

void wsm_vif_lock_tx(struct bes2600_vif *priv)
{
		struct bes2600_common *hw_priv = priv->hw_priv;

		wsm_cmd_lock(hw_priv);
		if (atomic_add_return(1, &hw_priv->tx_lock) == 1) {
				if (wsm_vif_flush_tx(priv))
						bes_devel("[WSM] TX is locked for"
										" if_id %d.\n", priv->if_id);
		else
			bes_devel("[WSM] TX is locked for if_id %d.\n", priv->if_id);

		}
		wsm_cmd_unlock(hw_priv);
}

bool wsm_vif_flush_tx(struct bes2600_vif *priv)
{
	struct bes2600_common *hw_priv = priv->hw_priv;
	unsigned long timestamp = jiffies;
	unsigned long timeout;
	int i, if_id = priv->if_id;
	BUG_ON(!atomic_read(&hw_priv->tx_lock));
	if (!hw_priv->hw_bufs_used_vif[if_id])
		return true;
	if (atomic_read(&hw_priv->bh_error)) {
		bes_err("[WSM] Fatal error occurred, will not flush TX.\n");
		return false;
	}
	for (i = 0; i < 4; ++i)
		bes2600_queue_get_xmit_timestamp(&hw_priv->tx_queue[i], &timestamp, if_id, 0xffffffff);
	timeout = timestamp + WSM_CMD_LAST_CHANCE_TIMEOUT;
	timeout = jiffies_to_msecs(time_after(timeout, jiffies) ? (timeout - jiffies) : (ULONG_MAX - jiffies + timeout));
	if (wait_event_timeout(hw_priv->bh_evt_wq, !hw_priv->hw_bufs_used_vif[if_id], timeout) <= 0) {
		bes2600_chrdev_wifi_force_close(hw_priv, true);
		return false;
	}
	return true;
}

bool wsm_flush_tx(struct bes2600_common *hw_priv)
{
	unsigned long timestamp = jiffies;
	bool pending = false;
	unsigned long timeout;
	int i;
	BUG_ON(!atomic_read(&hw_priv->tx_lock));
	if (!hw_priv->hw_bufs_used)
		return true;
	if (atomic_read(&hw_priv->bh_error)) {
		bes_err("[WSM] Fatal error occurred, will not flush TX.\n");
		return false;
	}
	for (i = 0; i < 4; ++i)
		pending |= bes2600_queue_get_xmit_timestamp(&hw_priv->tx_queue[i], &timestamp, CW12XX_ALL_IFS, 0xffffffff);
	if (!pending)
		return true;
	timeout = timestamp + WSM_CMD_LAST_CHANCE_TIMEOUT;
	timeout = jiffies_to_msecs(time_after(timeout, jiffies) ? (timeout - jiffies) : (ULONG_MAX - jiffies + timeout));
	if (wait_event_timeout(hw_priv->bh_evt_wq, !hw_priv->hw_bufs_used, timeout) <= 0) {
		bes2600_chrdev_wifi_force_close(hw_priv, true);
		return false;
	}
	return true;
}

