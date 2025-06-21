/*
 * WSM host interface (HI) interface for BES2600 mac80211 drivers
 *
 * Copyright (c) 2022, Bestechnic
 * Author:
 *
 * Based on BES2600 UMAC WSM API, which is
 * Copyright (C) SA 2010
 * Author: Stewart Mathers <stewart.mathers@stericsson.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef BES2600_WSM_H_INCLUDED
#define BES2600_WSM_H_INCLUDED

#include <linux/spinlock.h>

struct bes2600_common;
struct bes2600_vif;

/* Bands */
#define WSM_PHY_BAND_2_4G		(0)
#define WSM_PHY_BAND_5G			(1)

/* Transmit rates */
#define WSM_TRANSMIT_RATE_1		(0)
#define WSM_TRANSMIT_RATE_2		(1)
#define WSM_TRANSMIT_RATE_6		(6)
#define WSM_TRANSMIT_RATE_9		(7)
#define WSM_TRANSMIT_RATE_12		(8)
#define WSM_TRANSMIT_RATE_18		(9)
#define WSM_TRANSMIT_RATE_24		(10)
#define WSM_TRANSMIT_RATE_36		(11)
#define WSM_TRANSMIT_RATE_48		(12)
#define WSM_TRANSMIT_RATE_54		(13)
#define WSM_TRANSMIT_RATE_HT_6		(14)
#define WSM_TRANSMIT_RATE_HT_13		(15)
#define WSM_TRANSMIT_RATE_HT_19		(16)
#define WSM_TRANSMIT_RATE_HT_26		(17)
#define WSM_TRANSMIT_RATE_HT_39		(18)
#define WSM_TRANSMIT_RATE_HT_52		(19)
#define WSM_TRANSMIT_RATE_HT_58		(20)
#define WSM_TRANSMIT_RATE_HT_65		(21)

/* Scan types */
#define WSM_SCAN_TYPE_FOREGROUND	(0)
#define WSM_SCAN_TYPE_BACKGROUND	(1)
#define WSM_SCAN_TYPE_AUTO		(2)

/* Scan flags */
#define WSM_SCAN_FLAG_FORCE_BACKGROUND	(BIT(0))
#define WSM_SCAN_FLAG_SPLIT_METHOD	(BIT(1))
#define WSM_SCAN_FLAG_SHORT_PREAMBLE	(BIT(2))
#define WSM_SCAN_FLAG_11N_GREENFIELD	(BIT(3))
#define WSM_FLAG_MAC_INSTANCE_1		(BIT(4))
#define WSM_FLAG_MAC_INSTANCE_0		(~(BIT(4)))

/* Scan constraints */
#define WSM_SCAN_MAX_NUM_OF_CHANNELS	(48)
#define WSM_SCAN_MAX_NUM_OF_SSIDS	(2)
#ifdef CONFIG_BES2600_TESTMODE
#define WSM_TX_FLAG_EXPIRY_TIME		(BIT(0))
#endif /*CONFIG_BES2600_TESTMODE*/

/* Power management modes */
#define WSM_PSM_ACTIVE			(0)
#define WSM_PSM_PS			BIT(0)
#define WSM_PSM_FAST_PS_FLAG		BIT(7)
#define WSM_PSM_FAST_PS			(BIT(0) | BIT(7))
#define WSM_PSM_UNKNOWN			BIT(1)

/* Queue IDs */
#define WSM_QUEUE_BEST_EFFORT		(0)
#define WSM_QUEUE_BACKGROUND		(1)
#define WSM_QUEUE_VIDEO			(2)
#define WSM_QUEUE_VOICE			(3)

/* HT TX parameters */
#define WSM_HT_TX_NON_HT		(0)
#define WSM_HT_TX_MIXED			(1)
#define WSM_HT_TX_GREENFIELD		(2)
#define WSM_HT_TX_STBC			(BIT(7))

/* EPTA priority flags for BT Coex */
#define WSM_EPTA_PRIORITY_DEFAULT	4
#define WSM_EPTA_PRIORITY_DATA		4
#define WSM_EPTA_PRIORITY_MGT		5
#define WSM_EPTA_PRIORITY_ACTION	5
#define WSM_EPTA_PRIORITY_VIDEO		5
#define WSM_EPTA_PRIORITY_VOICE		6
#define WSM_EPTA_PRIORITY_EAPOL		7

/* TX status */
#define WSM_TX_STATUS_AGGREGATION	(BIT(0))
#define WSM_TX_STATUS_REQUEUE		(BIT(1))
#define WSM_TX_STATUS_NORMAL_ACK	(0<<2)
#define WSM_TX_STATUS_NO_ACK		(1<<2)
#define WSM_TX_STATUS_NO_EXPLICIT_ACK	(2<<2)
#define WSM_TX_STATUS_BLOCK_ACK		(3<<2)

/* RX status */
#define WSM_RX_STATUS_UNENCRYPTED	(0<<0)
#define WSM_RX_STATUS_WEP		(1<<0)
#define WSM_RX_STATUS_TKIP		(2<<0)
#define WSM_RX_STATUS_AES		(3<<0)
#define WSM_RX_STATUS_WAPI		(4<<0)
#define WSM_RX_STATUS_DECRYPTED		(7<<0)
#define WSM_RX_STATUS_ENCRYPTION(status) ((status) & 0x07)
#define WSM_RX_STATUS_AGGREGATE		(BIT(3))
#define WSM_RX_STATUS_AGGREGATE_FIRST	(BIT(4))
#define WSM_RX_STATUS_AGGREGATE_LAST	(BIT(5))
#define WSM_RX_STATUS_DEFRAGMENTED	(BIT(6))
#define WSM_RX_STATUS_BEACON		(BIT(7))
#define WSM_RX_STATUS_TIM		(BIT(8))
#define WSM_RX_STATUS_MULTICAST		(BIT(9))
#define WSM_RX_STATUS_MATCHING_SSID	(BIT(10))
#define WSM_RX_STATUS_MATCHING_BSSI	(BIT(11))
#define WSM_RX_STATUS_MORE_DATA		(BIT(12))
#define WSM_RX_STATUS_MEASUREMENT	(BIT(13))
#define WSM_RX_STATUS_HT		(BIT(14))
#define WSM_RX_STATUS_STBC		(BIT(15))
#define WSM_RX_STATUS_ADDRESS1		(BIT(16))
#define WSM_RX_STATUS_GROUP		(BIT(17))
#define WSM_RX_STATUS_BROADCAST		(BIT(18))
#define WSM_RX_STATUS_GROUP_KEY		(BIT(19))
#define WSM_RX_STATUS_KEY_IDX(status)	(((status >> 20)) & 0x0F)
#define WSM_TX_2BYTES_SHIFT		(BIT(7))

/* Join mode */
#define WSM_JOIN_MODE_IBSS		(0)
#define WSM_JOIN_MODE_BSS		(1)

/* PLCP preamble type */
#define WSM_JOIN_PREAMBLE_LONG		(0)
#define WSM_JOIN_PREAMBLE_SHORT		(1)
#define WSM_JOIN_PREAMBLE_SHORT_2	(2)

/* Join flags */
#define WSM_JOIN_FLAGS_UNSYNCRONIZED	BIT(0)
#define WSM_JOIN_FLAGS_P2P_GO		BIT(1)
#define WSM_JOIN_FLAGS_FORCE		BIT(2)
#define WSM_JOIN_FLAGS_PRIO		BIT(3)

/* Key types */
#define WSM_KEY_TYPE_WEP_DEFAULT	(0)
#define WSM_KEY_TYPE_WEP_PAIRWISE	(1)
#define WSM_KEY_TYPE_TKIP_GROUP		(2)
#define WSM_KEY_TYPE_TKIP_PAIRWISE	(3)
#define WSM_KEY_TYPE_AES_GROUP		(4)
#define WSM_KEY_TYPE_AES_PAIRWISE	(5)
#define WSM_KEY_TYPE_WAPI_GROUP		(6)
#define WSM_KEY_TYPE_WAPI_PAIRWISE	(7)
#define WSM_KEY_TYPE_IGTK_GROUP		(8)

/* Key indexes */
#define CW1250_WSM_KEY_MAX_INDEX	(15)
#define WSM_KEY_MAX_INDEX		(10)

/* ACK policy */
#define WSM_ACK_POLICY_NORMAL		(0)
#define WSM_ACK_POLICY_NO_ACK		(1)

/* Start modes */
#define WSM_START_MODE_AP		(0)
#define WSM_START_MODE_P2P_GO		(1)
#define WSM_START_MODE_P2P_DEV		(2)

/* SetAssociationMode MIB flags */
#define WSM_ASSOCIATION_MODE_USE_PREAMBLE_TYPE		(BIT(0))
#define WSM_ASSOCIATION_MODE_USE_HT_MODE		(BIT(1))
#define WSM_ASSOCIATION_MODE_USE_BASIC_RATE_SET		(BIT(2))
#define WSM_ASSOCIATION_MODE_USE_MPDU_START_SPACING	(BIT(3))
#define WSM_ASSOCIATION_MODE_SNOOP_ASSOC_FRAMES		(BIT(4))

/* RcpiRssiThreshold MIB flags */
#define WSM_RCPI_RSSI_THRESHOLD_ENABLE	(BIT(0))
#define WSM_RCPI_RSSI_USE_RSSI		(BIT(1))
#define WSM_RCPI_RSSI_DONT_USE_UPPER	(BIT(2))
#define WSM_RCPI_RSSI_DONT_USE_LOWER	(BIT(3))

/* Update-ie constants */
#define WSM_UPDATE_IE_BEACON		(BIT(0))
#define WSM_UPDATE_IE_PROBE_RESP	(BIT(1))
#define WSM_UPDATE_IE_PROBE_REQ		(BIT(2))

/* PS Mode Error */
#define WSM_PS_ERROR_NO_ERROR		0
#define WSM_PS_ERROR_AP_NOT_RESP_TO_POLL	1
#define WSM_PS_ERROR_AP_NOT_RESP_TO_UAPSD_TRIGGER	2
#define WSM_PS_ERROR_AP_SENT_UNICAST_IN_DOZE	3
#define WSM_PS_ERROR_AP_NO_DATA_AFTER_TIM	4

/* eth filter extended flags */
#define WSM_ETH_FILTER_EXT_DISABLE_IPV6_MATCH		BIT(0)

/* WSM events */
#define WSM_EVENT_ERROR			(0)
#define WSM_EVENT_BSS_LOST		(1)
#define WSM_EVENT_BSS_REGAINED		(2)
#define WSM_EVENT_RADAR_DETECTED	(3)
#define WSM_EVENT_RCPI_RSSI		(4)
#define WSM_EVENT_BT_INACTIVE		(5)
#define WSM_EVENT_BT_ACTIVE		(6)
#define WSM_EVENT_PS_MODE_ERROR		(7)
#define WSM_EVENT_INACTIVITY		(9)
#define WSM_EVENT_WAKEUP_EVENT		(10)

/* MIB IDs */
#define WSM_MIB_ID_DOT11_STATION_ID		0x0000
#define WSM_MIB_ID_DOT11_MAX_TRANSMIT_LIFTIME	0x0001
#define WSM_MIB_ID_DOT11_MAX_RECEIVE_LIFETIME	0x0002
#define WSM_MIB_ID_DOT11_SLOT_TIME		0x0003
#define WSM_MIB_ID_DOT11_GROUP_ADDRESSES_TABLE	0x0004
#define WSM_MAX_GRP_ADDRTABLE_ENTRIES		8
#define WSM_MIB_ID_DOT11_WEP_DEFAULT_KEY_ID	0x0005
#define WSM_MIB_ID_DOT11_CURRENT_TX_POWER_LEVEL	0x0006
#define WSM_MIB_ID_DOT11_RTS_THRESHOLD		0x0007
#define WSM_MIB_ID_NON_ERP_PROTECTION		0x1000
#define WSM_MIB_ID_ARP_IP_ADDRESSES_TABLE	0x1001
#define WSM_MAX_ARP_IP_ADDRTABLE_ENTRIES	1
#define WSM_MIB_ID_TEMPLATE_FRAME		0x1002
#define WSM_MIB_ID_RX_FILTER			0x1003
#define WSM_MIB_ID_BEACON_FILTER_TABLE		0x1004
#define WSM_MIB_ID_BEACON_FILTER_ENABLE		0x1005
#define WSM_MIB_ID_OPERATIONAL_POWER_MODE	0x1006
#define WSM_MIB_ID_BEACON_WAKEUP_PERIOD		0x1007
#define WSM_MIB_ID_RCPI_RSSI_THRESHOLD		0x1009
#define WSM_MIB_ID_STATISTICS_TABLE		0x100A
#define WSM_MIB_ID_IBSS_PS_CONFIG		0x100B
#define WSM_MIB_ID_COUNTERS_TABLE		0x100C
#define WSM_MIB_ID_BLOCK_ACK_INFO		0x100D
#define WSM_MIB_ID_BLOCK_ACK_POLICY		0x100E
#define WSM_MIB_ID_OVERRIDE_INTERNAL_TX_RATE	0x100F
#define WSM_MIB_ID_SET_ASSOCIATION_MODE		0x1010
#define WSM_MIB_ID_UPDATE_EPTA_CONFIG_DATA	0x1011
#define WSM_MIB_ID_SELECT_CCA_METHOD		0x1012
#define WSM_MIB_ID_SET_UAPSD_INFORMATION	0x1013
#define WSM_MIB_ID_SET_AUTO_CALIBRATION_MODE	0x1015
#define WSM_MIB_ID_SET_TX_RATE_RETRY_POLICY	0x1016
#define WSM_MIB_ID_SET_HOST_MSG_TYPE_FILTER	0x1017
#define WSM_MIB_ID_P2P_FIND_INFO		0x1018
#define WSM_MIB_ID_P2P_PS_MODE_INFO		0x1019
#define WSM_MIB_ID_SET_ETHERTYPE_DATAFRAME_FILTER 0x101A
#define WSM_MIB_ID_SET_UDPPORT_DATAFRAME_FILTER	0x101B
#define WSM_MIB_ID_SET_MAGIC_DATAFRAME_FILTER	0x101C
#define WSM_MIB_ID_P2P_DEVICE_INFO		0x101D
#define WSM_MIB_ID_SET_WCDMA_BAND		0x101E
#define WSM_MIB_ID_GRP_SEQ_COUNTER		0x101F
#define WSM_MIB_ID_PROTECTED_MGMT_POLICY	0x1020
#define WSM_MID_ID_SET_HT_PROTECTION		0x1021
#define WSM_MIB_ID_GPIO_COMMAND			0x1022
#define WSM_MIB_ID_TSF_COUNTER			0x1023
#define WSM_MIB_USE_MULTI_TX_CONF		0x1024
#define WSM_MIB_ID_KEEP_ALIVE_PERIOD		0x1025
#define WSM_MIB_ID_DISABLE_BSSID_FILTER		0x1026
#define WSM_MIB_ID_SET_INACTIVITY		0x1035
#define WSM_MIB_ID_MAC_ADDR_FILTER		0x1030
#define WSM_MIB_ID_IPV4_ADDR_FILTERING		0x1031
#ifdef MCAST_FWDING
#define WSM_MIB_ID_FORWARDING_OFFLOAD		0x1033
#endif
#ifdef IPV6_FILTERING
#define WSM_MIB_IP_IPV6_ADDR_FILTER		0x1032
#define WSM_MIB_ID_NS_IP_ADDRESSES_TABLE	0x1034
#define WSM_MAX_NDP_IP_ADDRTABLE_ENTRIES	1
#endif
#define WSM_MIB_ID_EXT_BASE			0x1040
#define WSM_MIB_ID_EXT_RF_ENABLE		(WSM_MIB_ID_EXT_BASE + 0)
#define WSM_MIB_ID_EXT_TCP_KEEP_ALIVE_FRAME	(WSM_MIB_ID_EXT_BASE + 1)
#define WSM_MIB_ID_EXT_TCP_KEEP_ALIVE_PERIOD	(WSM_MIB_ID_EXT_BASE + 2)
#define WSM_MIB_ID_EXT_TX_SHORT_GI_ENABLED	(WSM_MIB_ID_EXT_BASE + 3)
#define WSM_MIB_ID_EXT_TXRX_OPT_PARAM		(WSM_MIB_ID_EXT_BASE + 4)
#define WSM_MIB_ID_EXT_PWR_TBL_UPDATE		(WSM_MIB_ID_EXT_BASE + 5)

/* Frame template types */
#define WSM_FRAME_TYPE_PROBE_REQUEST	(0)
#define WSM_FRAME_TYPE_BEACON		(1)
#define WSM_FRAME_TYPE_NULL		(2)
#define WSM_FRAME_TYPE_QOS_NULL		(3)
#define WSM_FRAME_TYPE_PS_POLL		(4)
#define WSM_FRAME_TYPE_PROBE_RESPONSE	(5)
#define WSM_FRAME_TYPE_ARP_REPLY	(6)
#ifdef IPV6_FILTERING
#define WSM_FRAME_TYPE_NA		(7)
#define WSM_MAX_IPV6_ADDR_FILTER_ELTS	(8)
#endif
#define WSM_FRAME_GREENFIELD		(0x80)

/* Status */
#define WSM_STATUS_SUCCESS		(0)
#define WSM_STATUS_FAILURE		(1)
#define WSM_INVALID_PARAMETER		(2)
#define WSM_ACCESS_DENIED		(3)
#define WSM_STATUS_DECRYPTFAILURE	(4)
#define WSM_STATUS_MICFAILURE		(5)
#define WSM_STATUS_RETRY_EXCEEDED	(6)
#define WSM_STATUS_TX_LIFETIME_EXCEEDED	(7)
#define WSM_STATUS_LINK_LOST		(8)
#define WSM_STATUS_NO_KEY_FOUND		(9)
#define WSM_STATUS_JAMMER_DETECTED	(10)
#define WSM_REQUEUE			(11)

/* Advanced filtering options */
#define WSM_MAX_FILTER_ELEMENTS		(4)
#define WSM_FILTER_ACTION_IGNORE	(0)
#define WSM_FILTER_ACTION_FILTER_OUT	(1)
#define WSM_FILTER_ACTION_FILTER_IN	(2)
#define WSM_FILTER_ADDR_MODE_NONE	(0)
#define WSM_FILTER_ADDR_MODE_A1		(1)
#define WSM_FILTER_ADDR_MODE_A2		(2)
#define WSM_FILTER_ADDR_MODE_A3		(3)
#define WSM_FILTER_PORT_TYPE_DST	(0)
#define WSM_FILTER_PORT_TYPE_SRC	(1)
#define WSM_IP_DATA_FRAME_ADDRMODE_SRC	(0x01)
#define WSM_IP_DATA_FRAME_ADDRMODE_DEST	(0x02)
#define WSM_IP_DATA_FRAME_ADDRMODE_TCPACK	(0x03)

struct wsm_hdr {
    __le16 len;
    __le16 id;
};

struct wsm_mcu_hdr {
    struct wsm_hdr hdr;
    uint32_t m_reserve;
    uint32_t handle_label;
    uint32_t cmd_type;
};

#define WSM_MSG_ID_MASK			(0x0C3F)
#define WSM_MSG_ID_GET(x)		((x) & WSM_MSG_ID_MASK)
#define WSM_TX_SEQ_MAX			(7)
#define WSM_TX_SEQ(seq)			((seq & WSM_TX_SEQ_MAX) << 13)
#define WSM_MSG_SEQ_GET(x)		(((x) >> 13) & WSM_TX_SEQ_MAX)
#define WSM_BES2600_CMD_ID_LABLE	(0x0C00)
#define IS_DRIVER_TO_MCU_CMD(id)	((id & WSM_BES2600_CMD_ID_LABLE) == WSM_BES2600_CMD_ID_LABLE)
#define WSM_TXRX_SEQ_IDX(id)		(((id & 0xF00) == WSM_BES2600_CMD_ID_LABLE) ? 1 : 0)
#define WSM_TO_MCU_CMD_CONFIRM_LABEL	(0x0f)
#define WSM_TO_MCU_CMD_INDICATION_LABEL	(0xf0)
#define WSM_CONFIRM_CONDITION(id, confirm_label) (((id & 0x0f00) == 0x0400 || \
        (((id & 0x0f00) == WSM_BES2600_CMD_ID_LABLE) && confirm_label == WSM_TO_MCU_CMD_CONFIRM_LABEL)))
#define WSM_TO_MCU_CMD_IND_CONDITION(id, ind_label) (((id & 0x0f00) == WSM_BES2600_CMD_ID_LABLE) && \
        ind_label == WSM_TO_MCU_CMD_INDICATION_LABEL)
#define WSM_TX_LINK_ID_MAX		(0x0F)
#define WSM_TX_LINK_ID(link_id)		((link_id & WSM_TX_LINK_ID_MAX) << 6)
#define WSM_TX_IF_ID_MAX		(0x0F)
#define WSM_TX_IF_ID(if_id)		((if_id & WSM_TX_IF_ID_MAX) << 6)
#define MAX_BEACON_SKIP_TIME_MS		1000
#ifdef FPGA_SETUP
#define WSM_CMD_LAST_CHANCE_TIMEOUT	(HZ * 9 / 2)
#else
#define WSM_CMD_LAST_CHANCE_TIMEOUT	(HZ * 20 / 2)
#endif
#define WSM_CMD_EXTENDED_TIMEOUT	(HZ * 20 / 2)
#define WSM_RI_GET_PEER_ID_FROM_FLAGS(_f) (((_f)&(0xF<<25)>>25))

/* WSM capability */
struct wsm_caps {
    u16 numInpChBufs;
    u16 sizeInpChBuf;
    u16 hardwareId;
    u16 hardwareSubId;
    u16 firmwareCap;
    u16 firmwareType;
    u16 firmwareApiVer;
    u16 firmwareBuildNumber;
    u16 firmwareVersion;
    int firmwareReady;
};

/* WSM commands */
struct wsm_tx_power_range {
    int min_power_level;
    int max_power_level;
    u32 stepping;
};

struct wsm_configuration {
    u32 dot11MaxTransmitMsduLifeTime;
    u32 dot11MaxReceiveLifeTime;
    u32 dot11RtsThreshold;
    u8 *dot11StationId;
    const void *dpdData;
    size_t dpdData_size;
    u8 dot11FrequencyBandsSupported;
    u32 supportedRateMask;
    struct wsm_tx_power_range txPowerRange[2];
};

int wsm_configuration(struct bes2600_common *hw_priv,
              struct wsm_configuration *arg,
              int if_id);

struct wsm_reset {
    int link_id;
    bool reset_statistics;
};

int wsm_reset(struct bes2600_common *hw_priv, const struct wsm_reset *arg,
          int if_id);

int wsm_read_mib(struct bes2600_common *hw_priv, u16 mibId, void *buf,
         size_t buf_size);

int wsm_write_mib(struct bes2600_common *hw_priv, u16 mibId, void *buf,
          size_t buf_size, int if_id);

struct wsm_ssid {
    u8 ssid[32];
    u32 length;
};

struct wsm_scan_ch {
    u16 number;
    u32 minChannelTime;
    u32 maxChannelTime;
    u32 txPowerLevel;
};

struct wsm_scan_complete {
    u32 status;
    u8 psm;
    u8 numChannels;
#ifdef ROAM_OFFLOAD
    u16 reserved;
#endif
};

typedef void (*wsm_scan_complete_cb) (struct bes2600_common *hw_priv,
                      struct wsm_scan_complete *arg);

struct wsm_scan {
    u8 band;
    u8 scanType;
    u8 scanFlags;
    u8 maxTransmitRate;
    u32 autoScanInterval;
    u32 numOfProbeRequests;
    u8 numOfChannels;
    u8 numOfSSIDs;
    u8 probeDelay;
    struct wsm_ssid *ssids;
    struct wsm_scan_ch *ch;
};

int wsm_scan(struct bes2600_common *hw_priv, const struct wsm_scan *arg,
         int if_id);

int wsm_stop_scan(struct bes2600_common *hw_priv, int if_id);

struct wsm_tx_confirm {
    u32 packetID;
    u32 status;
    u8 txedRate;
    u8 ackFailures;
    u16 flags;
    u32 mediaDelay;
    u32 txQueueDelay;
    u32 link_id;
    int if_id;
};

typedef void (*wsm_tx_confirm_cb) (struct bes2600_common *hw_priv,
                   struct wsm_tx_confirm *arg);

struct wsm_tx {
    struct wsm_hdr hdr;
    __le32 packetID;
    u8 maxTxRate;
    u8 queueId;
    u8 more;
    u8 flags;
    __le32 reserved;
    __le32 expireTime;
    __le32 htTxParameters;
};

#define WSM_TX_EXTRA_HEADROOM (28)

struct wsm_rx {
    u32 status;
    u16 channelNumber;
    u8 rxedRate;
    u8 rcpiRssi;
    u32 flags;
    void *frame;
    size_t frame_size;
    int link_id;
    int if_id;
};

#define WSM_RX_EXTRA_HEADROOM (16)

typedef void (*wsm_rx_cb) (struct bes2600_vif *priv, struct wsm_rx *arg,
               struct sk_buff **skb_p);

struct wsm_event {
    u32 eventId;
    u32 eventData;
};

struct bes2600_wsm_event {
    struct list_head link;
    struct wsm_event evt;
    u8 if_id;
};

typedef void (*wsm_event_cb) (struct bes2600_common *hw_priv,
                  struct wsm_event *arg);

struct wsm_join {
    u8 mode;
    u8 band;
    u16 channelNumber;
    u8 bssid[6];
    u16 atimWindow;
    u8 preambleType;
    u8 probeForJoin;
    u8 dtimPeriod;
    u8 flags;
    u32 ssidLength;
    u8 ssid[32];
    u32 beaconInterval;
    u32 basicRateSet;
    int minPowerLevel;
    int maxPowerLevel;
};

int wsm_join(struct bes2600_common *hw_priv, struct wsm_join *arg, int if_id);

struct wsm_set_pm {
    u8 pmMode;
    u8 fastPsmIdlePeriod;
    u8 apPsmChangePeriod;
    u8 minAutoPsPollPeriod;
};

int wsm_set_pm(struct bes2600_common *hw_priv, const struct wsm_set_pm *arg,
           int if_id);

struct wsm_set_pm_complete {
    u32 status;
    u8 psm;
};

typedef void (*wsm_set_pm_complete_cb) (struct bes2600_common *hw_priv,
                    struct wsm_set_pm_complete *arg);

struct wsm_set_bss_params {
    u8 beaconLostCount;
    u16 aid;
    u32 operationalRateSet;
};

int wsm_set_bss_params(struct bes2600_common *hw_priv,
               const struct wsm_set_bss_params *arg, int if_id);

struct wsm_add_key {
    u8 type;
    u8 entryIndex;
    u16 reserved;
    union {
        struct {
            u8 peerAddress[6];
            u8 reserved;
            u8 keyLength;
            u8 keyData[16];
        } __packed wepPairwiseKey;
        struct {
            u8 keyId;
            u8 keyLength;
            u16 reserved;
            u8 keyData[16];
        } __packed wepGroupKey;
        struct {
            u8 peerAddress[6];
            u8 reserved[2];
            u8 tkipKeyData[16];
            u8 rxMicKey[8];
            u8 txMicKey[8];
        } __packed tkipPairwiseKey;
        struct {
            u8 tkipKeyData[16];
            u8 rxMicKey[8];
            u8 keyId;
            u8 reserved[3];
            u8 rxSeqCounter[8];
        } __packed tkipGroupKey;
        struct {
            u8 peerAddress[6];
            u16 reserved;
            u8 aesKeyData[16];
        } __packed aesPairwiseKey;
        struct {
            u8 aesKeyData[16];
            u8 keyId;
            u8 reserved[3];
            u8 rxSeqCounter[8];
        } __packed aesGroupKey;
        struct {
            u8 peerAddress[6];
            u8 keyId;
            u8 reserved;
            u8 wapiKeyData[16];
            u8 micKeyData[16];
        } __packed wapiPairwiseKey;
        struct {
            u8 wapiKeyData[16];
            u8 micKeyData[16];
            u8 keyId;
            u8 reserved[3];
        } __packed wapiGroupKey;
        struct {
            u8 IGTKKeyData[16];
            u8 keyId;
            u8 Reserved[3];
            u8 IPN[8];
        } __packed igtkGroupKey;
    } __packed;
} __packed;

int wsm_add_key(struct bes2600_common *hw_priv, const struct wsm_add_key *arg,
            int if_id);

struct wsm_remove_key {
    u8 entryIndex;
};

int wsm_remove_key(struct bes2600_common *hw_priv,
           const struct wsm_remove_key *arg, int if_id);

struct wsm_set_tx_queue_params {
    u8 ackPolicy;
    u16 allowedMediumTime;
    u32 maxTransmitLifetime;
};

struct wsm_tx_queue_params {
    struct wsm_set_tx_queue_params params[4];
};

#define WSM_TX_QUEUE_SET(queue_params, queue, ack_policy, allowed_time,     \
             max_life_time)					    \
do {									    \
    struct wsm_set_tx_queue_params *p = &(queue_params)->params[queue]; \
    p->ackPolicy = (ack_policy);				\
    p->allowedMediumTime = (allowed_time);				\
    p->maxTransmitLifetime = (max_life_time);			\
} while (0)

int wsm_set_tx_queue_params(struct bes2600_common *hw_priv,
                const struct wsm_set_tx_queue_params *arg,
                u8 id, int if_id);

struct wsm_edca_queue_params {
    u16 cwMin;
    u16 cwMax;
    u8 aifns;
    u16 txOpLimit;
    u32 maxReceiveLifetime;
    bool uapsdEnable;
};

struct wsm_edca_params {
    struct wsm_edca_queue_params params[4];
};

#define TXOP_UNIT 32
#define WSM_EDCA_SET(edca, queue, aifs, cw_min, cw_max, txop, life_time,\
        uapsd)	\
    do {							\
        struct wsm_edca_queue_params *p = &(edca)->params[queue]; \
        p->cwMin = (cw_min);				\
        p->cwMax = (cw_max);				\
        p->aifns = (aifs);				\
        p->txOpLimit = ((txop) * TXOP_UNIT);		\
        p->maxReceiveLifetime = (life_time);		\
        p->uapsdEnable = (uapsd);			\
    } while (0)

int wsm_set_edca_params(struct bes2600_common *hw_priv,
            const struct wsm_edca_params *arg, int if_id);

int wsm_set_uapsd_param(struct bes2600_common *hw_priv,
            const struct wsm_edca_params *arg);

struct wsm_switch_channel {
    u8 channelMode;
    u8 channelSwitchCount;
    u16 newChannelNumber;
};

int wsm_switch_channel(struct bes2600_common *hw_priv,
               const struct wsm_switch_channel *arg, int if_id);

typedef void (*wsm_channel_switch_cb) (struct bes2600_common *hw_priv);

struct wsm_start {
    u8 mode;
    u8 band;
    u16 channelNumber;
    u32 CTWindow;
    u32 beaconInterval;
    u8 DTIMPeriod;
    u8 preambleType;
    u8 probeDelay;
    u8 ssidLength;
    u8 ssid[32];
    u32 basicRateSet;
};

int wsm_start(struct bes2600_common *hw_priv, const struct wsm_start *arg,
        int if_id);

int wsm_start_find(struct bes2600_common *hw_priv, int if_id);

int wsm_stop_find(struct bes2600_common *hw_priv, int if_id);

typedef void (*wsm_find_complete_cb) (struct bes2600_common *hw_priv,
                      u32 status);

struct wsm_suspend_resume {
    int link_id;
    bool stop;
    bool multicast;
    int queue;
    int if_id;
};

typedef void (*wsm_suspend_resume_cb) (struct bes2600_vif *priv,
                       struct wsm_suspend_resume *arg);

struct wsm_update_ie {
    u16 what;
    u16 count;
    u8 *ies;
    size_t length;
};

int wsm_update_ie(struct bes2600_common *hw_priv,
          const struct wsm_update_ie *arg, int if_id);

struct wsm_map_link {
    u8 mac_addr[6];
    u8 unmap;
    u8 link_id;
};

int wsm_map_link(struct bes2600_common *hw_priv, const struct wsm_map_link *arg,
        int if_id);

struct wsm_cbc {
    wsm_scan_complete_cb scan_complete;
    wsm_tx_confirm_cb tx_confirm;
    wsm_rx_cb rx;
    wsm_event_cb event;
    wsm_set_pm_complete_cb set_pm_complete;
    wsm_channel_switch_cb channel_switch;
    wsm_find_complete_cb find_complete;
    wsm_suspend_resume_cb suspend_resume;
};

#ifdef MCAST_FWDING
int wsm_init_release_buffer_request(struct bes2600_common *priv, u8 index);
int wsm_request_buffer_request(struct bes2600_vif *priv, u8 *arg);
#endif

/* MIB shortcuts */
struct wsm_mib {
    u16 mibId;
    void *buf;
    size_t buf_size;
};

static inline int wsm_set_output_power(struct bes2600_common *hw_priv,
                       int power_level, int if_id)
{
    __le32 val = __cpu_to_le32(power_level);
    return wsm_write_mib(hw_priv, WSM_MIB_ID_DOT11_CURRENT_TX_POWER_LEVEL,
                 &val, sizeof(val), if_id);
}

static inline int wsm_set_beacon_wakeup_period(struct bes2600_common *hw_priv,
                           unsigned dtim_interval,
                           unsigned listen_interval,
                           int if_id)
{
    struct {
        u8 numBeaconPeriods;
        u8 reserved;
        __le16 listenInterval;
    } val = {
        dtim_interval, 0, __cpu_to_le16(listen_interval)};
    if (dtim_interval > 0xFF || listen_interval > 0xFFFF)
        return -EINVAL;
    else
        return wsm_write_mib(hw_priv, WSM_MIB_ID_BEACON_WAKEUP_PERIOD,
                     &val, sizeof(val), if_id);
}

struct wsm_rcpi_rssi_threshold {
    u8 rssiRcpiMode;
    u8 lowerThreshold;
    u8 upperThreshold;
    u8 rollingAverageCount;
};

static inline int wsm_set_rcpi_rssi_threshold(struct bes2600_common *hw_priv,
                    struct wsm_rcpi_rssi_threshold *arg,
                    int if_id)
{
    return wsm_write_mib(hw_priv, WSM_MIB_ID_RCPI_RSSI_THRESHOLD, arg,
                 sizeof(*arg), if_id);
}

struct wsm_counters_table {
    __le32 countPlcpErrors;
    __le32 countFcsErrors;
    __le32 countTxPackets;
    __le32 countRxPackets;
    __le32 countRxPacketErrors;
    __le32 countRxDecryptionFailures;
    __le32 countRxMicFailures;
    __le32 countRxNoKeyFailures;
    __le32 countTxMulticastFrames;
    __le32 countTxFramesSuccess;
    __le32 countTxFrameFailures;
    __le32 countTxFramesRetried;
    __le32 countTxFramesMultiRetried;
    __le32 countRxFrameDuplicates;
    __le32 countRtsSuccess;
    __le32 countRtsFailures;
    __le32 countAckFailures;
    __le32 countRxMulticastFrames;
    __le32 countRxFramesSuccess;
    __le32 countRxCMACICVErrors;
    __le32 countRxCMACReplays;
    __le32 countRxMgmtCCMPReplays;
};

static inline int wsm_get_counters_table(struct bes2600_common *hw_priv,
                     struct wsm_counters_table *arg)
{
    return wsm_read_mib(hw_priv, WSM_MIB_ID_COUNTERS_TABLE,
            arg, sizeof(*arg));
}

static inline int wsm_get_station_id(struct bes2600_common *hw_priv, u8 *mac)
{
    return wsm_read_mib(hw_priv, WSM_MIB_ID_DOT11_STATION_ID, mac,
                ETH_ALEN);
}

struct wsm_rx_filter {
    bool promiscuous;
    bool bssid;
    bool fcs;
    bool probeResponder;
    bool keepalive;
};

static inline int wsm_set_rx_filter(struct bes2600_common *hw_priv,
                    const struct wsm_rx_filter *arg,
                    int if_id)
{
    __le32 val = 0;
    if (arg->promiscuous)
        val |= __cpu_to_le32(BIT(0));
    if (arg->bssid)
        val |= __cpu_to_le32(BIT(1));
    if (arg->fcs)
        val |= __cpu_to_le32(BIT(2));
    if (arg->probeResponder)
        val |= __cpu_to_le32(BIT(3));
    if (arg->keepalive)
        val |= __cpu_to_le32(BIT(4));
    return wsm_write_mib(hw_priv, WSM_MIB_ID_RX_FILTER, &val, sizeof(val),
            if_id);
}

int wsm_set_probe_responder(struct bes2600_vif *priv, bool enable);
int wsm_set_keepalive_filter(struct bes2600_vif *priv, bool enable);

#define WSM_BEACON_FILTER_IE_HAS_CHANGED	BIT(0)
#define WSM_BEACON_FILTER_IE_NO_LONGER_PRESENT	BIT(1)
#define WSM_BEACON_FILTER_IE_HAS_APPEARED	BIT(2)

struct wsm_beacon_filter_table_entry {
    u8	ieId;
    u8	actionFlags;
    u8	oui[3];
    u8	matchData[3];
} __packed;

struct wsm_beacon_filter_table {
    __le32 numOfIEs;
    struct wsm_beacon_filter_table_entry entry[10];
} __packed;

static inline int wsm_set_beacon_filter_table(struct bes2600_common *hw_priv,
                    struct wsm_beacon_filter_table *ft,
                    int if_id)
{
    static __le32 numOfIEs;
    size_t size = __le32_to_cpu(ft->numOfIEs) *
             sizeof(struct wsm_beacon_filter_table_entry) +
             sizeof(__le32);

    if (numOfIEs == ft->numOfIEs && numOfIEs == 0) {
        return 0;
    }
    numOfIEs = ft->numOfIEs;
    return wsm_write_mib(hw_priv, WSM_MIB_ID_BEACON_FILTER_TABLE, ft, size,
            if_id);
}

#define WSM_BEACON_FILTER_ENABLE	BIT(0)
#define WSM_BEACON_FILTER_AUTO_ERP	BIT(1)
#define WSM_BEACON_FILTER_AUTO_HT	BIT(2)

struct wsm_beacon_filter_control {
    __le32 enabled;
    __le32 bcn_count;
};

static inline int wsm_beacon_filter_control(struct bes2600_common *hw_priv,
                    struct wsm_beacon_filter_control *arg,
                    int if_id)
{
    static __le32 enabled, bcn_count;

    if (enabled == arg->enabled && bcn_count == arg->bcn_count) {
        return 0;
    }
    enabled = arg->enabled;
    bcn_count = arg->bcn_count;
    return wsm_write_mib(hw_priv, WSM_MIB_ID_BEACON_FILTER_ENABLE, arg,
                 sizeof(struct wsm_beacon_filter_control), if_id);
}

enum wsm_power_mode {
    wsm_power_mode_active = 0,
    wsm_power_mode_doze = 1,
    wsm_power_mode_quiescent = 2,
};

struct wsm_operational_mode {
    enum wsm_power_mode power_mode;
    int disableMoreFlagUsage;
    int performAntDiversity;
};

static inline int wsm_set_operational_mode(struct bes2600_common *hw_priv,
                    const struct wsm_operational_mode *arg,
                    int if_id)
{
    u32 val = arg->power_mode;
    if (arg->disableMoreFlagUsage)
        val |= BIT(4);
    if (arg->performAntDiversity)
        val |= BIT(5);
    return wsm_write_mib(hw_priv, WSM_MIB_ID_OPERATIONAL_POWER_MODE, &val,
                 sizeof(val), if_id);
}

struct wsm_inactivity {
    u8 max_inactivity;
    u8 min_inactivity;
};

static inline int wsm_set_inactivity(struct bes2600_common *hw_priv,
                    const struct wsm_inactivity *arg,
                    int if_id)
{
    struct {
           u8	min_inactive;
           u8	max_inactive;
           u16	reserved;
    } val;

    val.max_inactive = arg->max_inactivity;
    val.min_inactive = arg->min_inactivity;
    val.reserved = 0;

    return wsm_write_mib(hw_priv, WSM_MIB_ID_SET_INACTIVITY, &val,
                 sizeof(val), if_id);
}

struct wsm_template_frame {
    u8 frame_type;
    u8 rate;
    bool disable;
    struct sk_buff *skb;
};

static inline int wsm_set_template_frame(struct bes2600_common *hw_priv,
                     struct wsm_template_frame *arg,
                     int if_id)
{
    int ret;
    u8 *p = skb_push(arg->skb, 4);
    p[0] = arg->frame_type;
    p[1] = arg->rate;
    if (arg->disable)
        ((u16 *) p)[1] = 0;
    else
        ((u16 *) p)[1] = __cpu_to_le16(arg->skb->len - 4);
    ret = wsm_write_mib(hw_priv, WSM_MIB_ID_TEMPLATE_FRAME, p,
                arg->skb->len, if_id);
    skb_pull(arg->skb, 4);
    return ret;
}

struct wsm_protected_mgmt_policy {
    bool protectedMgmtEnable;
    bool unprotectedMgmtFramesAllowed;
    bool encryptionForAuthFrame;
};

static inline int
wsm_set_protected_mgmt_policy(struct bes2600_common *hw_priv,
                  struct wsm_protected_mgmt_policy *arg,
                  int if_id)
{
    __le32 val = 0;
    int ret;
    if (arg->protectedMgmtEnable)
        val |= __cpu_to_le32(BIT(0));
    if (arg->unprotectedMgmtFramesAllowed)
        val |= __cpu_to_le32(BIT(1));
    if (arg->encryptionForAuthFrame)
        val |= __cpu_to_le32(BIT(2));
    ret = wsm_write_mib(hw_priv, WSM_MIB_ID_PROTECTED_MGMT_POLICY, &val,
                sizeof(val), if_id);
    return ret;
}

static inline int wsm_set_block_ack_policy(struct bes2600_common *hw_priv,
                       u8 blockAckTxTidPolicy,
                       u8 blockAckRxTidPolicy,
                       int if_id)
{
    struct {
        u8 blockAckTxTidPolicy;
        u8 reserved1;
        u8 blockAckRxTidPolicy;
        u8 reserved2;
    } val = {
        .blockAckTxTidPolicy = blockAckTxTidPolicy,
        .blockAckRxTidPolicy = blockAckRxTidPolicy,
    };
    return wsm_write_mib(hw_priv, WSM_MIB_ID_BLOCK_ACK_POLICY, &val,
                 sizeof(val), if_id);
}

struct wsm_association_mode {
    u8 flags;
    u8 preambleType;
    u8 greenfieldMode;
    u8 mpduStartSpacing;
    __le32 basicRateSet;
};

static inline int wsm_set_association_mode(struct bes2600_common *hw_priv,
                       struct wsm_association_mode *arg,
                       int if_id)
{
    return wsm_write_mib(hw_priv, WSM_MIB_ID_SET_ASSOCIATION_MODE, arg,
                 sizeof(*arg), if_id);
}

struct wsm_set_tx_rate_retry_policy_header {
    u8 numTxRatePolicies;
    u8 reserved[3];
} __packed;

struct wsm_set_tx_rate_retry_policy_policy {
    u8 policyIndex;
    u8 shortRetryCount;
    u8 longRetryCount;
    u8 policyFlags;
    u8 rateRecoveryCount;
    u8 reserved[3];
    __le32 rateCountIndices[3];
} __packed;

struct wsm_set_tx_rate_retry_policy {
    struct wsm_set_tx_rate_retry_policy_header hdr;
    struct wsm_set_tx_rate_retry_policy_policy tbl[8];
} __packed;

static inline int wsm_set_tx_rate_retry_policy(struct bes2600_common *hw_priv,
                struct wsm_set_tx_rate_retry_policy *arg,
                int if_id)
{
    size_t size = sizeof(struct wsm_set_tx_rate_retry_policy_header) +
        arg->hdr.numTxRatePolicies *
        sizeof(struct wsm_set_tx_rate_retry_policy_policy);
    return wsm_write_mib(hw_priv, WSM_MIB_ID_SET_TX_RATE_RETRY_POLICY, arg,
                 size, if_id);
}

struct wsm_ether_type_filter_hdr {
    u8 nrFilters;
    u8 extFlags;
    u8 reserved[2];
} __packed;

struct wsm_ether_type_filter {
    u8 filterAction;
    u8 reserved;
    __le16 etherType;
} __packed;

static inline int wsm_set_ether_type_filter(struct bes2600_common *hw_priv,
                struct wsm_ether_type_filter_hdr *arg,
                int if_id)
{
    size_t size = sizeof(struct wsm_ether_type_filter_hdr) +
        arg->nrFilters * sizeof(struct wsm_ether_type_filter);
    return wsm_write_mib(hw_priv, WSM_MIB_ID_SET_ETHERTYPE_DATAFRAME_FILTER,
        arg, size, if_id);
}

struct wsm_udp_port_filter_hdr {
    u8 nrFilters;
    u8 reserved[3];
} __packed;

struct wsm_udp_port_filter {
    u8 filterAction;
    u8 portType;
    __le16 udpPort;
} __packed;

static inline int wsm_set_udp_port_filter(struct bes2600_common *hw_priv,
                struct wsm_udp_port_filter_hdr *arg,
                int if_id)
{
    size_t size = sizeof(struct wsm_udp_port_filter_hdr) +
        arg->nrFilters * sizeof(struct wsm_udp_port_filter);
    return wsm_write_mib(hw_priv, WSM_MIB_ID_SET_UDPPORT_DATAFRAME_FILTER,
        arg, size, if_id);
}

#define D11_MAX_SSID_LEN		(32)

struct wsm_p2p_device_type {
    __le16 categoryId;
    u8 oui[4];
    __le16 subCategoryId;
} __packed;

struct wsm_p2p_device_info {
    struct wsm_p2p_device_type primaryDevice;
    u8 reserved1[3];
    u8 devNameSize;
    u8 localDevName[D11_MAX_SSID_LEN];
    u8 reserved2[3];
    u8 numSecDevSupported;
    struct wsm_p2p_device_type secondaryDevices[0];
} __packed;

struct wsm_cdma_band {
    u8 WCDMA_Band;
    u8 reserved[3];
} __packed;

struct wsm_group_tx_seq {
    __le32 bits_47_16;
    __le16 bits_15_00;
    __le16 reserved;
} __packed;

#define WSM_DUAL_CTS_PROT_ENB		(1 << 0)
#define WSM_NON_GREENFIELD_STA_PRESENT	(1 << 1)
#define WSM_HT_PROT_MODE__NO_PROT	(0 << 2)
#define WSM_HT_PROT_MODE__NON_MEMBER	(1 << 2)
#define WSM_HT_PROT_MODE__20_MHZ	(2 << 2)
#define WSM_HT_PROT_MODE__NON_HT_MIXED	(3 << 2)
#define WSM_LSIG_TXOP_PROT_FULL		(1 << 4)
#define WSM_LARGE_L_LENGTH_PROT		(1 << 5)

struct wsm_ht_protection {
    __le32 flags;
} __packed;

#define WSM_GPIO_COMMAND_SETUP	0
#define WSM_GPIO_COMMAND_READ	1
#define WSM_GPIO_COMMAND_WRITE	2
#define WSM_GPIO_COMMAND_RESET	3
#define WSM_GPIO_ALL_PINS	0xFF

struct wsm_gpio_command {
    u8 GPIO_Command;
    u8 pin;
    __le16 config;
} __packed;

struct wsm_tsf_counter {
    __le64 TSF_Counter;
} __packed;

struct wsm_keep_alive_period {
    __le16 keepAlivePeriod;
    u8 reserved[2];
} __packed;

static inline int wsm_keep_alive_period(struct bes2600_common *hw_priv,
                    int period, int if_id)
{
    struct wsm_keep_alive_period arg = {
        .keepAlivePeriod = __cpu_to_le16(period),
    };
    return wsm_write_mib(hw_priv, WSM_MIB_ID_KEEP_ALIVE_PERIOD,
            &arg, sizeof(arg), if_id);
};

struct wsm_set_bssid_filtering {
    u8 filter;
    u8 reserved[3];
} __packed;

static inline int wsm_set_bssid_filtering(struct bes2600_common *hw_priv,
                      bool enabled, int if_id)
{
    struct wsm_set_bssid_filtering arg = {
        .filter = !enabled,
    };
    return wsm_write_mib(hw_priv, WSM_MIB_ID_DISABLE_BSSID_FILTER,
            &arg, sizeof(arg), if_id);
}

struct wsm_multicast_filter {
    __le32 enable;
    __le32 numOfAddresses;
    u8 macAddress[WSM_MAX_GRP_ADDRTABLE_ENTRIES][ETH_ALEN];
} __packed;

struct wsm_mac_addr_info {
    u8 filter_mode;
    u8 address_mode;
    u8 MacAddr[6];
} __packed;

struct wsm_mac_addr_filter {
    u8 numfilter;
    u8 action_mode;
    u8 Reserved[2];
    struct wsm_mac_addr_info macaddrfilter[0];
} __packed;

struct wsm_broadcast_addr_filter {
    u8 action_mode;
    u8 nummacaddr;
    u8 filter_mode;
    u8 address_mode;
    u8 MacAddr[6];
} __packed;

static inline int wsm_set_multicast_filter(struct bes2600_common *hw_priv,
                       struct wsm_multicast_filter *fp,
                       int if_id)
{
    return wsm_write_mib(hw_priv, WSM_MIB_ID_DOT11_GROUP_ADDRESSES_TABLE,
                 fp, sizeof(*fp), if_id);
}

struct wsm_arp_ipv4_filter {
    __le32 enable;
    __be32 ipv4Address[WSM_MAX_ARP_IP_ADDRTABLE_ENTRIES];
} __packed;

#ifdef IPV6_FILTERING
struct wsm_ndp_ipv6_filter {
    __le32 enable;
    struct in6_addr ipv6Address[WSM_MAX_NDP_IP_ADDRTABLE_ENTRIES];
} __packed;

struct wsm_ip6_addr_info {
    u8 filter_mode;
    u8 address_mode;
    u8 Reserved[2];
    u8 ipv6[16];
};

struct wsm_ipv6_filter_header {
    u8 numfilter;
    u8 action_mode;
    u8 Reserved[2];
};

struct wsm_ipv6_filter {
    struct wsm_ipv6_filter_header hdr;
    struct wsm_ip6_addr_info ipv6filter[WSM_MAX_IPV6_ADDR_FILTER_ELTS];
} __packed;

static inline int wsm_set_ipv6_filter(struct bes2600_common *hw_priv,
                    struct wsm_ipv6_filter_header *arg,
                    int if_id)
{
    size_t size = sizeof(struct wsm_ipv6_filter_header) +
        arg->numfilter * sizeof(struct wsm_ip6_addr_info);

    return wsm_write_mib(hw_priv, WSM_MIB_IP_IPV6_ADDR_FILTER,
        arg, size, if_id);
}
#endif

struct wsm_ip4_addr_info {
    u8 filter_mode;
    u8 address_mode;
    u8 Reserved[2];
    u8 ipv4[4];
};

struct wsm_ipv4_filter {
    u8 numfilter;
    u8 action_mode;
    u8 Reserved[2];
    struct wsm_ip4_addr_info ipv4filter[0];
} __packed;

static inline int wsm_set_arp_ipv4_filter(struct bes2600_common *hw_priv,
                      struct wsm_arp_ipv4_filter *fp,
                      int if_id)
{
    return wsm_write_mib(hw_priv, WSM_MIB_ID_ARP_IP_ADDRESSES_TABLE,
                fp, sizeof(*fp), if_id);
}

#ifdef IPV6_FILTERING
static inline int wsm_set_ndp_ipv6_filter(struct bes2600_common *priv,
                      struct wsm_ndp_ipv6_filter *fp,
                      int if_id)
{
    return wsm_write_mib(priv, WSM_MIB_ID_NS_IP_ADDRESSES_TABLE,
                fp, sizeof(*fp), if_id);
}
#endif

struct wsm_p2p_ps_modeinfo {
    u8	oppPsCTWindow;
    u8	count;
    u8	reserved;
    u8	dtimCount;
    __le32	duration;
    __le32	interval;
    __le32	startTime;
} __packed;

static inline int wsm_set_p2p_ps_modeinfo(struct bes2600_common *hw_priv,
                      struct wsm_p2p_ps_modeinfo *mi,
                      int if_id)
{
    return wsm_write_mib(hw_priv, WSM_MIB_ID_P2P_PS_MODE_INFO,
                 mi, sizeof(*mi), if_id);
}

static inline int wsm_get_p2p_ps_modeinfo(struct bes2600_common *hw_priv,
                      struct wsm_p2p_ps_modeinfo *mi)
{
    return wsm_read_mib(hw_priv, WSM_MIB_ID_P2P_PS_MODE_INFO,
                mi, sizeof(*mi));
}

static inline int wsm_use_multi_tx_conf(struct bes2600_common *hw_priv,
                    bool enabled, int if_id)
{
    __le32 arg = enabled ? __cpu_to_le32(1) : 0;

    return wsm_write_mib(hw_priv, WSM_MIB_USE_MULTI_TX_CONF,
            &arg, sizeof(arg), if_id);
}

struct wsm_uapsd_info {
    __le16 uapsdFlags;
    __le16 minAutoTriggerInterval;
    __le16 maxAutoTriggerInterval;
    __le16 autoTriggerStep;
};

static inline int wsm_set_uapsd_info(struct bes2600_common *hw_priv,
                     struct wsm_uapsd_info *arg,
                     int if_id)
{
    return wsm_write_mib(hw_priv, WSM_MIB_ID_SET_UAPSD_INFORMATION,
                arg, sizeof(*arg), if_id);
}

struct wsm_override_internal_txrate {
    u8 internalTxRate;
    u8 nonErpInternalTxRate;
    u8 reserved[2];
} __packed;

static inline int
wsm_set_override_internal_txrate(struct bes2600_common *hw_priv,
                     struct wsm_override_internal_txrate *arg,
                     int if_id)
{
    return wsm_write_mib(hw_priv, WSM_MIB_ID_OVERRIDE_INTERNAL_TX_RATE,
                arg, sizeof(*arg), if_id);
}

#ifdef MCAST_FWDING
struct wsm_forwarding_offload {
    u8 fwenable;
    u8 flags;
    u8 reserved[2];
} __packed;

static inline int wsm_set_forwarding_offlad(struct bes2600_common *hw_priv,
                     struct wsm_forwarding_offload *arg,int if_id)
{
    return wsm_write_mib(hw_priv, WSM_MIB_ID_FORWARDING_OFFLOAD,
                arg, sizeof(*arg),if_id);
}
#endif

/* WSM TX port control */
void wsm_lock_tx(struct bes2600_common *hw_priv);
void wsm_vif_lock_tx(struct bes2600_vif *priv);
void wsm_lock_tx_async(struct bes2600_common *hw_priv);
bool wsm_flush_tx(struct bes2600_common *hw_priv);
bool wsm_vif_flush_tx(struct bes2600_vif *priv);
void wsm_unlock_tx(struct bes2600_common *hw_priv);

/* WSM / BH API */
int wsm_handle_exception(struct bes2600_common *hw_priv, u8 *data, size_t len);
int wsm_handle_rx(struct bes2600_common *hw_priv, int id, struct wsm_hdr *wsm,
          struct sk_buff **skb_p);

/* wsm_buf API */
struct wsm_buf {
    u8 *begin;
    u8 *data;
    u8 *end;
};

void wsm_buf_init(struct wsm_buf *buf);
void wsm_buf_deinit(struct wsm_buf *buf);

/* wsm_cmd API */
struct wsm_cmd {
    spinlock_t lock;
    int done;
    u8 *ptr;
    size_t len;
    void *arg;
    int ret;
    u16 cmd;
};

/* WSM TX buffer access */
int wsm_get_tx(struct bes2600_common *hw_priv, u8 **data,
           size_t *tx_len, unsigned int *burst, int *vif_selected);
void wsm_txed(struct bes2600_common *hw_priv, u8 *data);

/* Queue mapping: WSM <---> linux */
static inline u8 wsm_queue_id_to_linux(u8 queueId)
{
    static const u8 queue_mapping[] = {2, 3, 1, 0};
    return queue_mapping[queueId];
}

static inline u8 wsm_queue_id_to_wsm(u8 queueId)
{
    static const u8 queue_mapping[] = {3, 2, 0, 1};
    return queue_mapping[queueId];
}

#ifdef CONFIG_BES2600_TESTMODE
struct vendor_rf_cmd_t {
    u32 cmd_type;
    u32 cmd_argc;
    u32 cmd_len;
    u8 cmd[0];
};

int wsm_vendor_rf_cmd(struct bes2600_common *hw_priv, int if_id,
                      const struct vendor_rf_cmd_t *vendor_rf_cmd);
int wsm_vendor_rf_cmd_confirm(struct bes2600_common *hw_priv,
                              void *arg, struct wsm_buf *buf);
int wsm_vendor_rf_test_indication(struct bes2600_common *hw_priv, struct wsm_buf *buf);
#endif

enum bes2600_rf_cmd_type {
    BES2600_RF_CMD_CALI_TXT_TO_FLASH = 0,
    BES2600_RF_CMD_CH_INFO           = 1,
    BES2600_RF_CMD_CPU_USAGE         = 2,
    BES2600_RF_CMD_WIFI_STATUS       = 3,
    BES2600_RF_CMD_CALI_TXT_TO_EFUSE = 4,
    BES2600_RF_CMD_MAX,
};

int wsm_driver_rf_cmd_confirm(struct bes2600_common *hw_priv,
                              void *arg, struct wsm_buf *buf);

struct wsm_epta_msg {
    int    wlan_duration;
    int    bt_duration;
    int    hw_epta_enable;
};

#define PROCTECT_MODE_RTS_CTS		0x1
#define PROCTECT_MODE_CTS		0x2
#define PROCTECT_MODE_RTS_CTS_RETRY	0x4

typedef struct MIB_TXRX_OPT_PARAM_S
{
    u8  rts_retry_limit;
    u8  protect_mode;
    u16 rts_duration;
} MIB_TXRX_OPT_PARAM;

int wsm_epta_cmd(struct bes2600_common *hw_priv, struct wsm_epta_msg *arg);

int wsm_epta_wifi_chan_cmd(struct bes2600_common *hw_priv, uint32_t channel, uint32_t type);

int wsm_cpu_usage_cmd(struct bes2600_common *hw_priv);

int wsm_wifi_status_cmd(struct bes2600_common *hw_priv, uint32_t status);

#if defined(STANDARD_FACTORY_EFUSE_FLAG)
int wsm_save_factory_txt_to_mcu(struct bes2600_common *hw_priv, const u8 *data, int if_id, enum bes2600_rf_cmd_type cmd_type);
#endif

int wsm_cmd_send(struct bes2600_common *hw_priv, struct wsm_buf *buf,
				 void *arg, u16 cmd, long tmo, int if_id);

#endif /* BES2600_WSM_H_INCLUDED */
