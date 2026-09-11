/*
 * Mac80211 SDIO driver for BES2600 device
 *
 * Copyright (c) 2010, Bestechnic
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DEBUG 1
#define SDIO_RETRY_MAX 30 // Arbitrary. Maybe something else?
#include <linux/version.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/spinlock.h>
#include <net/mac80211.h>
#include <linux/scatterlist.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/version.h>
#include <linux/of_gpio.h>

#include "bes2600.h"
#include "sbus.h"
#include "bes2600_plat.h"
#include "hwio.h"
#include "bh.h"
#include "bes_chardev.h"
#include "bes_log.h"

void sdio_work_debug(struct sbus_priv *self);
static void bes2600_sdio_power_down(struct sbus_priv *self);
struct bes2600_platform_data_sdio *bes2600_get_platform_data(void);
int bes2600_register_net_dev(struct sbus_priv *bus_priv);
int bes2600_unregister_net_dev(struct sbus_priv *bus_priv);
bool bes2600_is_net_dev_created(struct sbus_priv *bus_priv);
static void bes2600_gpio_wakeup_mcu(struct sbus_priv *self);
static void bes2600_gpio_allow_mcu_sleep(struct sbus_priv *self);

MODULE_AUTHOR("Dmitry Tarnyagin <dmitry.tarnyagin@stericsson.com>");
MODULE_DESCRIPTION("mac80211 BES2600 SDIO driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("bes2600_wlan");

struct sbus_priv {
	struct sdio_func	*func;
	struct bes2600_common	*core;
	const struct bes2600_platform_data_sdio *pdata;
	spinlock_t		lock;
	sbus_irq_handler	irq_handler;
	void			*irq_priv;
	struct device *dev;
	struct workqueue_struct *sdio_wq;
	bool fw_started;
	struct mutex io_mutex;
//	long unsigned int gpio_wakup_flags;
	atomic_t gpio_wakeup_ref;   /* instead of unsigned long gpio_wakup_flags; */
	struct mutex sbus_mutex;
	bool retune_protected;
#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
	u8 next_toggle;
	int tx_data_toggle;
	int rx_data_toggle;
#endif
#ifdef CONFIG_BES_SDIO_RX_MULTIPLE_ENABLE
	spinlock_t rx_queue_lock;
	struct sk_buff_head rx_queue;
	u8 *rx_buffer;
	struct work_struct rx_work;
	u32 rx_last_ctrl;
	u32 rx_valid_ctrl;
	u32 rx_total_ctrl_cnt;
	u32 rx_continuous_ctrl_cnt;
	u32 rx_zero_ctrl_cnt;
	u32 rx_remain_ctrl_cnt;
	u8 rx_ebusy_streak;
	u32 rx_data_cnt;
	u32 rx_xfer_cnt;
	u32 rx_proc_cnt;
	long unsigned int last_irq_timestamp;
	long unsigned int last_rx_data_timestamp;
#endif
#ifdef CONFIG_BES_SDIO_TX_MULTIPLE_ENABLE
	u8 *tx_buffer;
	u8 *tx_pad;
	struct list_head tx_bufferlist;
	struct kmem_cache *tx_bufferlistpool;
	spinlock_t tx_bufferlock;
	struct work_struct tx_work;
	struct scatterlist tx_sg[BES_SDIO_TX_MULTIPLE_NUM + 1];
	struct scatterlist tx_sg_nosignal[BES_SDIO_TX_MULTIPLE_NUM_NOSIGNAL + 1];
	u32 tx_data_cnt;
	u32 tx_xfer_cnt;
	u32 tx_proc_cnt;
	u8 tx_ebusy_streak;
	long unsigned int last_tx_data_timestamp;
#endif
	bool unregister_in_process;
};

#define IS_DRIVER_VENDOR_CMD(X) ((X & 0x0C00) == 0x0C00)
struct HI_MSG_HDR {
	uint16_t MsgLen;
	uint16_t MsgId;
};

enum DRIVER_TO_MCU_MSG_ST {
	ST_ENTER,
	ST_EXIT,
};

#define BES_VENDOR_ID	0xbe57
#define BES_DEVICE_ID_2002	0x2002

static const struct sdio_device_id bes2600_sdio_ids[] = {
	{ SDIO_DEVICE(BES_VENDOR_ID, BES_DEVICE_ID_2002) },
	{ /* end: all zeroes */			},
};
MODULE_DEVICE_TABLE(sdio, bes2600_sdio_ids);

#ifdef CONFIG_BES2600_GPIO_WAKEUP_AP
static int bes2600_gpio_wakeup_ap_config(struct sbus_priv *priv);
#endif

/* sbus_ops implemetation */

#ifdef CONFIG_BES2600_WLAN_BES
static inline unsigned int sdio_max_byte_size(struct sdio_func *func)
{
	unsigned mval = min(func->card->host->max_seg_size, func->card->host->max_blk_size);

	if (func->card->quirks & MMC_QUIRK_BLKSZ_FOR_BYTE_MODE)
		mval = min(mval, func->cur_blksize);
	else
		mval = min(mval, func->max_blksize);

	return min(mval, 512u);
}

static int bes_sdio_memcpy_io_helper(struct sdio_func *func, int write, void *data_buf, unsigned size)
{
	int ret = 0;
	unsigned remainder = size;
	unsigned max_blocks, align_blocks, pads;

#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
	struct sbus_priv *self = NULL;
#endif

	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;
	struct scatterlist sg[2];
	struct scatterlist *next;

	u8 *pad_buf = (u8 *)kmalloc(func->cur_blksize, GFP_KERNEL);
	if (!pad_buf)
		return -ENOMEM;

	if (!func || (func->num > 7) || (!data_buf) || (!size)) {
		ret = -EINVAL;
		goto out;
	}

#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
	self = sdio_get_drvdata(func);
	if (!self) {
		bes_err("%s: no sbus_priv\n", __func__);
		ret = -ENODEV;
		goto out;
	}
#endif

	if (func->card->cccr.multi_block && size > sdio_max_byte_size(func) ) {
		max_blocks = min(func->card->host->max_blk_count, 511u);
		align_blocks = (size + func->cur_blksize - 1) / func->cur_blksize;
		if (align_blocks > max_blocks) {
			/* to be simplified, consider this should not
			 * happen, and to be continued;
			 */
			bes_devel("%s warning to be continued, align=%d max=%d", __func__, align_blocks, max_blocks);
			ret = -EINVAL;
			goto out;
		}
		pads = align_blocks * func->cur_blksize - size;
		bes_devel("%s sz=%u blk=%u pad=%u,dir=%d", __func__, size, align_blocks, pads, write);

		memset(&mrq, 0, sizeof(mrq));
		memset(&cmd, 0, sizeof(cmd));
		memset(&data, 0, sizeof(data));

		mrq.cmd = &cmd;
		mrq.data = &data;

		cmd.opcode = SD_IO_RW_EXTENDED;
		cmd.arg = write ? 0x80000000 : 0x00000000;
		cmd.arg |= func->num << 28;
		cmd.arg |= 0x04000000;
		cmd.arg |= size << 9;

#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
		if (likely(self->fw_started == true)) {
			cmd.arg &= ~(1 << 25);
			if (write) {
				cmd.arg |= ((self->tx_data_toggle & 0x1) << 25);
				++self->tx_data_toggle;
			} else {
				cmd.arg |= ((self->rx_data_toggle & 0x1) << 25);
				++self->rx_data_toggle;
			}
		}
#endif

		cmd.arg |= (0x08000000 | align_blocks);
		cmd.flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_ADTC;

		data.blksz = func->cur_blksize;
		data.blocks = align_blocks;
		data.flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;

		data.sg = sg;
		data.sg_len = 1;
		sg_init_table(sg, 2);
		sg_set_buf(sg, data_buf, size);
		if (pads) {
			next = sg_next(sg);
			sg_set_buf(next, pad_buf, pads);
			data.sg_len = 2;
		}

		mmc_set_data_timeout(&data, func->card);

		mmc_wait_for_req(func->card->host, &mrq);

		if (cmd.error){
			ret = cmd.error;
			goto out;
		}
		if (data.error) {
			ret = data.error;
			goto out;
		}
		if (cmd.resp[0] & R5_ERROR) {
			ret = -EIO;
			goto out;
		}
		if (cmd.resp[0] & R5_FUNCTION_NUMBER) {
			ret = -EINVAL;
			goto out;
		}
		if (cmd.resp[0] & R5_OUT_OF_RANGE) {
			ret = -ERANGE;
			goto out;
		}
	} else {
		while (remainder) {
			size = min(remainder, sdio_max_byte_size(func));

			bes_devel("%s size=%d dir=%d", __func__, size, write);
			if (write) {
#ifndef CONFIG_BES_SDIO_RXTX_TOGGLE
				ret = sdio_memcpy_toio(func, size, data_buf, size);
#else
				if (likely(self->fw_started == true)) {
					ret = sdio_memcpy_toio(func, size | ((self->tx_data_toggle & 0x1) << 16), data_buf, size);
					++self->tx_data_toggle;
				} else {
					ret = sdio_memcpy_toio(func, size, data_buf, size);
				}
#endif
			} else {
#ifndef CONFIG_BES_SDIO_RXTX_TOGGLE
				ret = sdio_memcpy_fromio(func, data_buf, size, size);
#else
				if (likely(self->fw_started == true)) {
					ret = sdio_memcpy_fromio(func, data_buf, size | ((self->rx_data_toggle & 0x1) << 16), size);
					++self->rx_data_toggle;
				} else {
					ret = sdio_memcpy_fromio(func, data_buf, size, size);
				}
#endif
			}
			if (ret)
				goto out;

			remainder -= size;
			data_buf += size;
		}
	}
out:
	kfree(pad_buf);

	if (ret) {
		bes_err("%s, err=%d(%d:%p:%d)",
				__func__, ret, func->num, data_buf, size);
#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
				if (self && self->fw_started == true) {
					if (write)
						--self->tx_data_toggle;
					else
						--self->rx_data_toggle;
					bes_err("%s,toggle count:%u,%u\n", __func__, self->tx_data_toggle, self->rx_data_toggle);
				}
#endif
	}
	return ret;
}
#endif

static int bes2600_sdio_memcpy_fromio(struct sbus_priv *self,
				     unsigned int addr,
				     void *dst, int count)
{
	return bes_sdio_memcpy_io_helper(self->func, 0, dst, count);
}

static int bes2600_sdio_memcpy_toio(struct sbus_priv *self,
				   unsigned int addr,
				   const void *src, int count)
{
	return bes_sdio_memcpy_io_helper(self->func, 1, (void *)src, count);
}

static void bes2600_sdio_lock(struct sbus_priv *self)
{
	sdio_claim_host(self->func);
}

static void bes2600_sdio_unlock(struct sbus_priv *self)
{
	sdio_release_host(self->func);
}

/* bes sdio slave regs can only be accessed by command52
 * if a WORD or DWORD reg wants to be accessed,
 * please combine the results of multiple command52
 */
static int bes2600_sdio_reg_read(struct sbus_priv *self, u32 reg,
					void *dst, int count)
{
	int ret = 0;
	if (count <= 0 || !dst)
		return -EINVAL;
	while(count && !ret) {
		*(u8 *)dst = sdio_readb(self->func, reg, &ret);
		dst ++;
		reg ++;
		count--;
	}
	return ret;

}

static int bes2600_sdio_reg_write(struct sbus_priv *self, u32 reg,
					const void *src, int count)
{
	int ret = 0;
	if (count <= 0 || !src)
		return -EINVAL;
	while (count && !ret) {
		sdio_writeb(self->func, *(u8 *)src, reg, &ret);
		src ++;
		reg ++;
		count --;
	}
	return ret;
}

#ifndef CONFIG_BES2600_USE_GPIO_IRQ
static void bes2600_sdio_irq_handler(struct sdio_func *func)
{
	if (!func) {
		bes_err("SDIO IRQ handler called with NULL func\n");
		return;
	}

	struct sbus_priv *self = sdio_get_drvdata(func);
	unsigned long flags;

	if (!self) {
		bes_err("%s: no sbus_priv\n", __func__);
		return;
	}

	bes_devel("%s called, fw_started:%d \n",
			 __func__, self->fw_started);
	if (likely(self->fw_started && self->core)) {
		queue_work(self->sdio_wq, &self->rx_work);
		self->last_irq_timestamp = jiffies;
	} else if(self->irq_handler) {
		spin_lock_irqsave(&self->lock, flags);
		self->irq_handler(self->irq_priv);
		spin_unlock_irqrestore(&self->lock, flags);
	}
}
#else /* CONFIG_BES2600_USE_GPIO_IRQ */
static u32 bes2600_gpio_irq_handler(void *dev_id)
{
	struct sbus_priv *self = (struct sbus_priv *)dev_id;

	bes_devel("\n %s called \n", __func__);
	if (!self) {
		bes_err("%s: no sbus_priv\n", __func__);
		return 0;
	}
	if (self->irq_handler)
		self->irq_handler(self->irq_priv);
	return 0;
}

static int bes2600_request_irq(struct sbus_priv *self,
			      u32 handler)
{
	int ret = 0;
	int func_num;
	const struct resource *irq = self->pdata->irq;
	u8 cccr;
	int ret0 = 0;

	/* Hack to access Fuction-0 */
	func_num = self->func->num;
	self->func->num = 0;

	cccr = sdio_readb(self->func, SDIO_CCCR_IENx, &ret);
	if (ret) {
		bes_err("%s: CCCR_IENx read failed %d\n", __func__, ret);
		goto set_func;
	}

	/* Master interrupt enable ... */
	cccr |= BIT(0);

	/* ... for our function */
	cccr |= BIT(func_num);

	sdio_writeb(self->func, cccr, SDIO_CCCR_IENx, &ret);
	if (ret) {
		bes_err("%s: CCCR_IENx write failed %d\n", __func__, ret);
		goto set_func;
	}

	/* Restore the WLAN function number */
	self->func->num = func_num;
	return 0;

set_func:
	//AW judge sdio read write timeout, 1s
	ret0 = sw_mci_check_r1_ready(self->func->card->host, 1000);
	if (ret0 != 0)
		bes_err(("%s data timeout.\n", __FUNCTION__));

	self->func->num = func_num;
	bes_err("[%s]  fail exiting sw_gpio_irq_request..   :%d\n",__func__, ret);
	return ret;
}
#endif /* CONFIG_BES2600_USE_GPIO_IRQ */

static int bes2600_sdio_irq_subscribe(struct sbus_priv *self,
				     sbus_irq_handler handler,
				     void *priv)
{
	int ret;
	unsigned long flags;

	if (!handler)
		return -EINVAL;

	spin_lock_irqsave(&self->lock, flags);
	self->irq_priv = priv;
	self->irq_handler = handler;
	spin_unlock_irqrestore(&self->lock, flags);

	bes_devel( "SW IRQ subscribe\n");
	sdio_claim_host(self->func);
#ifndef CONFIG_BES2600_USE_GPIO_IRQ
	ret = sdio_claim_irq(self->func, bes2600_sdio_irq_handler);
#else
	mdelay(10);
	ret = bes2600_request_irq(self, bes2600_gpio_irq_handler);
#endif
	sdio_release_host(self->func);
	return ret;
}

static int bes2600_sdio_irq_unsubscribe(struct sbus_priv *self)
{
	int ret = 0;
	unsigned long flags;
#ifdef CONFIG_BES2600_USE_GPIO_IRQ
	const struct resource *irq = self->pdata->irq;
#endif

	if (!self->irq_handler) {
		bes_err("%s: no irq_handler\n", __func__);
		return 0;
	}

	bes_devel("SW IRQ unsubscribe\n");

/*
#ifndef CONFIG_BES2600_USE_GPIO_IRQ
	sdio_claim_host(self->func);
	ret = sdio_release_irq(self->func);
	sdio_release_host(self->func);
#else
	free_irq(irq->start, self);
#endif  //CONFIG_BES2600_USE_GPIO_IRQ
*/

	spin_lock_irqsave(&self->lock, flags);
	self->irq_priv = NULL;
	self->irq_handler = NULL;
	spin_unlock_irqrestore(&self->lock, flags);

	return ret;
}

static void bes2600_sdio_off(const struct bes2600_platform_data_sdio *pdata)
{
    struct sbus_priv *sbus = bes2600_chrdev_get_sbus_priv_data();
    struct sdio_func *func = sbus ? sbus->func : NULL;

    bes_info("%s: Performing software SDIO reset\n", __func__);

    if (func) {
        sdio_claim_host(func);
        sdio_writeb(func, 0x08, SDIO_CCCR_ABORT, NULL);  // IO reset
    	bes_devel("Sleeping in: %s\n", __func__);
        msleep(10);
        sdio_release_host(func);
        // Force bus rescan
        mmc_detect_change(func->card->host, 0);
        bes_info("%s: Reset and rescan complete\n", __func__);
    } else {
        bes_warn("%s: No func for reset\n", __func__);
    }

	// Both pins are not availiable? being used for other things?
	//gpiod_direction_output(pdata->powerup, GPIOD_OUT_LOW);
	//gpiod_direction_output(pdata->reset, GPIOD_OUT_LOW);
}

static void bes2600_sdio_on(const struct bes2600_platform_data_sdio *pdata)
{
	bes_devel("%s\n", __func__);
	// Both pins are not availiable? being used for other things?
	// gpiod_direction_output(pdata->powerup, GPIOD_OUT_HIGH);
}

static size_t bes2600_sdio_align_size(struct sbus_priv *self, size_t size)
{
	size_t aligned  = size;
	if (self->func->cur_blksize > size)
	       aligned = sdio_align_size(self->func, size);
	else
		aligned = (aligned + 3) & (~3);

	return aligned;
}

static int bes2600_sdio_set_block_size(struct sbus_priv *self, size_t size)
{
	return sdio_set_block_size(self->func, size);
}

void sdio_work_debug(struct sbus_priv *self)
{
	/*
	 * Print-only.  Claiming the SDIO host and reading CTRL after a
	 * silent pipe has hard-locked CPU2 (log: four lines, never
	 * "realtime ctrl=", then LOCKUP).  Do not poke the bus here.
	 */
	bes_err("%s now=%u last irq timestamp=%u\n", __func__,
			(u32)jiffies_to_msecs(jiffies), jiffies_to_msecs(self->last_irq_timestamp));
	bes_err("%s rx ctrl: total=%u continuous=%u xfer=%u remain=%u zero=%u last=%x(%x) next=%d\n", __func__,
			self->rx_total_ctrl_cnt, self->rx_continuous_ctrl_cnt, self->rx_xfer_cnt, self->rx_remain_ctrl_cnt, self->rx_zero_ctrl_cnt,
			self->rx_last_ctrl, self->rx_valid_ctrl, self->next_toggle);
	bes_err("%s rx: last timestamp=%u, total=%u(%u), proc=%u\n", __func__,
			(u32)jiffies_to_msecs(self->last_rx_data_timestamp),
			self->rx_data_cnt, self->rx_xfer_cnt, self->rx_proc_cnt);
	bes_err("%s tx: last timestamp=%u, total=%u,%u, proc=%u\n", __func__,
			(u32)jiffies_to_msecs(self->last_tx_data_timestamp),
			self->tx_data_cnt, self->tx_xfer_cnt, self->tx_proc_cnt);
}


static int bes2600_sdio_read_ctrl(struct sbus_priv *self, u32 *ctrl_reg)
{
	u8 data[4];
	int ret = 0, again = 0;
	*ctrl_reg = 0;

	/* clear sdio slave gen interrupt */
	ret = bes2600_sdio_reg_read(self, BES_TX_CTRL_REG_ID + 1, data, 1);
	if (unlikely(ret)) {
		if (ret != -EBUSY && ret != -ETIMEDOUT)
			bes_err("[SBUS] Failed(%d) to read control register.\n",
				ret);
		return ret;
	}
	self->rx_total_ctrl_cnt++;

	if (data[0] & 0x7f) {
		/* length field valid */
		again = 1;
		if (((data[0] >> 7) & 0x1) == self->next_toggle) {
			*ctrl_reg = (data[0] & 0x7f) << 9;
			self->rx_valid_ctrl = data[0];
			if (self->rx_last_ctrl && (((self->rx_last_ctrl >> 7) & 0x1) != self->next_toggle))
				self->rx_continuous_ctrl_cnt++;
			self->next_toggle ^= 1;
		} else {
			self->rx_remain_ctrl_cnt++;
		}
	} else {
		/* distinguish zero true or false */
		self->rx_zero_ctrl_cnt++;
		ret = bes2600_sdio_reg_read(self, BES_TX_CTRL_REG_ID, &data[1], 1);
		if (!ret && (data[1] & 0x01))
			again = 0;
		else
			again = 1;
	}
	self->rx_last_ctrl = data[0];

	return again;
}

#ifdef CONFIG_BES_SDIO_RX_MULTIPLE_ENABLE

static int bes2600_sdio_packets_check(u32 ctrl_reg, u8 *packets)
{
	int i;
	u16 single, total_cal = 0;
	u16 packets_length;
	struct HI_MSG_HDR *pMsg;

	/* bit 23-16 indicate count of packets */
	u32 packets_cnt;
	packets_cnt = BES_SDIO_RX_MULTIPLE_NUM;

	/* bit 15-0 indicate totoal length of packets */
	packets_length = PACKET_TOTAL_LEN(ctrl_reg);

	/* first 32-bit: addr in mcu;
	 * second 32-bit: packet length;
	 * next: data
	 */
	for (i = 0; i < packets_cnt; i++) {
		pMsg = (struct HI_MSG_HDR *)(packets + total_cal);
		bes_devel("%s, %x,%x\n", __func__, pMsg->MsgId, pMsg->MsgLen);
		single = pMsg->MsgLen;
		single = (single + 3) & (~0x3);
		if (unlikely(single > 1632)) {
			bes_warn("%s %d,len=%u,%dth,total=%u,%u\n", __func__, __LINE__, single, i,
					packets_length, total_cal);
			if (i >= 1) {
				return -201;
			}
		}
		total_cal += single;
		if ((!pMsg->MsgLen) || (total_cal == packets_length)) {
			//bes_devel("%s, contain %d packets\n", __func__, i);
			break;
		}
	}
	bes_devel("%s, %d,%u,%u\n", __func__, packets_cnt, packets_length, total_cal);

	if (packets_length < total_cal) {
		bes_err("%s,%d pkt len=%u, total len=%u", __func__, __LINE__, packets_length, total_cal);
		return -202;
	}

	return 0;
}

static int bes2600_sdio_extract_packets(struct sbus_priv *self, u32 ctrl_reg, u8 *data)
{
	int i, alloc_retry = 0;
	u8 packets_cnt = BES_SDIO_RX_MULTIPLE_NUM;
	u16 packet_len, pos = 0;
	struct sk_buff *skb;

	for (i = 0; i < packets_cnt; i++) {
		packet_len = ((struct HI_MSG_HDR *)&(data[pos]))->MsgLen;
		if (!packet_len)
			break;

		do {
			skb = dev_alloc_skb(packet_len);
			if (likely(skb))
				break;
			bes_warn("%s,%d no memory and sleep\n", __func__, __LINE__);
    		bes_devel("Sleeping in: %s\n", __func__);
			msleep(100);
			++alloc_retry;
		} while(alloc_retry < 10);
		if (!skb) {
			bes_err("%s: no skb for packet_len %u\n",
				__func__, packet_len);
			return -ENOMEM;
		}
		skb_trim(skb, 0);
		skb_put(skb, packet_len);
		memcpy(skb->data, &data[pos], packet_len);
		bes_devel("%s, %d,%d\n", __func__, packet_len, pos);
		spin_lock(&self->rx_queue_lock);
		skb_queue_tail(&self->rx_queue, skb);
		self->rx_data_cnt++;
		spin_unlock(&self->rx_queue_lock);
		packet_len = (packet_len + 3) & (~0x3);
		pos += packet_len;
		if (pos == PACKET_TOTAL_LEN(ctrl_reg))
			break;
	}
	return 0;
}

static void sdio_rx_work(struct work_struct *work)
{
	int ret, again = 0, retry = 0, crc_retry = 0, ebusy_tries = 0;
	u32 ctrl_reg = 0;
	int total_len;
	struct sbus_priv *self = container_of(work, struct sbus_priv, rx_work);
	u8 *buf = self->rx_buffer;

	/* don't read/write sdio when sdio error */
	if (bes2600_chrdev_is_bus_error())
		return;

	bes2600_gpio_wakeup_mcu(self);

	do {
		bes2600_sdio_lock(self);
		again = bes2600_sdio_read_ctrl(self, &ctrl_reg);

		if (again == -EBUSY || again == -ETIMEDOUT) {
			/*
			 * CMD52 during MCU wake returns -EBUSY.  Dropping
			 * GPIO and returning lost the WSM confirm (idle
			 * 0x0006 timeout, CONFIRM MISMATCH).  Keep the
			 * wake line and retry; never force_close.
			 */
			bes2600_sdio_unlock(self);
			if (++ebusy_tries <= 30) {
				usleep_range(1000, 2000);
				continue;
			}
			if (self->rx_ebusy_streak < 3) {
				self->rx_ebusy_streak++;
				bes_err("%s ctrl read %d — retry later (%u)\n",
					__func__, again, self->rx_ebusy_streak);
				bes2600_gpio_allow_mcu_sleep(self);
				msleep(20);
				if (!bes2600_chrdev_is_bus_error())
					queue_work(self->sdio_wq, &self->rx_work);
				return;
			}
			bes_err("%s ctrl read %d — skip force_close\n",
				__func__, again);
			if (self->core)
				bes2600_bh_mark_bus_stale(self->core);
			bes2600_gpio_allow_mcu_sleep(self);
			return;
		}
		self->rx_ebusy_streak = 0;
		ebusy_tries = 0;

		total_len = PACKET_TOTAL_LEN(ctrl_reg);
		if (!total_len) {
			bes2600_sdio_unlock(self);
			if ((again == 1) && retry <= 5) {
				retry++;
				continue;
			} else {
				break;
			}
		}

		do {
			ret = bes2600_sdio_memcpy_fromio(self, 0, buf, total_len);
			if (likely(ret != BES_SDIO_CRC_ERROR)) {
				crc_retry = 0;
				break;
			} else {
				crc_retry++;
				bes_err("%s sdio read crc error(%d)\n", __func__, crc_retry);
			}
		} while (crc_retry <= SDIO_RETRY_MAX);
		if (self->retune_protected == true) {
			sdio_retune_release(self->func);
			self->retune_protected = false;
		}
		bes2600_sdio_unlock(self);
		if (ret) {
			bes_err("%s,%d error=%d\n", __func__, __LINE__, ret);
			sdio_work_debug(self);
			goto failed;
		}

		retry = 0;
		self->rx_xfer_cnt++;
		self->last_rx_data_timestamp = jiffies;

		if ((ret = bes2600_sdio_packets_check(ctrl_reg, buf))) {
			bes_err("%s,%d error=%d\n", __func__, __LINE__, ret);
			sdio_work_debug(self);
			goto failed;
		}

		if ((ret = bes2600_sdio_extract_packets(self, ctrl_reg, buf))) {
			bes_err("%s,%d error=%d\n", __func__, __LINE__, ret);
			goto failed;
		}

		ctrl_reg = 0;

		if (likely(self->irq_handler)) {
			self->irq_handler(self->irq_priv);
		} else {
			bes_err("%s,%d\n", __func__, __LINE__);
			goto failed;
		}

	} while (again);

	bes2600_gpio_allow_mcu_sleep(self);
	return;

failed:
	bes2600_gpio_allow_mcu_sleep(self);
	bes2600_chrdev_wifi_force_close(self->core, false);
	bes_err("%s: SDIO rx failed\n", __func__);
}

static void *bes2600_sdio_pipe_read(struct sbus_priv *self)
{
	struct sk_buff *skb;

	if (bes2600_chrdev_is_bus_error()) {
		return bes2600_tx_loop_read(self->core);
	}

	spin_lock(&self->rx_queue_lock);
	skb = skb_dequeue(&self->rx_queue);
	if (skb)
		self->rx_proc_cnt++;
	spin_unlock(&self->rx_queue_lock);
	if (likely(self->fw_started == true &&
		!bes2600_pwr_device_is_idle(self->core) &&
		self->core->hw_bufs_used > 0))
		if (!skb)
			queue_work(self->sdio_wq, &self->rx_work);
	return skb;
}

#endif

#ifdef CONFIG_BES_SDIO_TX_MULTIPLE_ENABLE

struct bes_sdio_tx_list_t {
	struct list_head node;
	u8 *buf;
	u32 len;
};

static int bes_sdio_memcpy_to_io_helper(struct sdio_func *func, unsigned origin_size, struct scatterlist *sg, u32 sg_num)
{
	int ret = 0;
	u32 align_blocks;
	u32 compensate;
	unsigned size = origin_size & (~0x3);

#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
	struct sbus_priv *self = NULL;
#endif

	u32 sg_compensate_num = sg_num;

	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_data data;

#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
	self = sdio_get_drvdata(func);
#endif

	if (size && func->card->cccr.multi_block) {

		align_blocks = (size + func->cur_blksize - 1) / func->cur_blksize;
		bes_devel("%s sz=%u blk=%u", __func__, size, align_blocks);
		compensate = align_blocks * func->cur_blksize - size;
		if (compensate) {
			sg_set_buf(&sg[sg_num], self->tx_buffer, compensate);
			sg_compensate_num = sg_num + 1;
		}
		sg_mark_end(&sg[sg_compensate_num - 1]);

		memset(&mrq, 0, sizeof(mrq));
		memset(&cmd, 0, sizeof(cmd));
		memset(&data, 0, sizeof(data));

		mrq.cmd = &cmd;
		mrq.data = &data;

		cmd.opcode = SD_IO_RW_EXTENDED;
		cmd.arg = 0x80000000;
		cmd.arg |= func->num << 28;
		cmd.arg |= 0x04000000;
		cmd.arg |= (origin_size) << 9;

#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
		if (likely(self->fw_started == true)) {
			cmd.arg &= ~(1 << 25);
			cmd.arg |= ((self->tx_data_toggle & 0x1) << 25);
			++self->tx_data_toggle;
		}
#endif

		cmd.arg |= 0x08000000 | align_blocks;
		cmd.flags = MMC_RSP_SPI_R5 | MMC_RSP_R5 | MMC_CMD_ADTC;

		data.blksz = func->cur_blksize;
		data.blocks = align_blocks;
		data.flags = MMC_DATA_WRITE;

		data.sg = sg;
		data.sg_len = sg_compensate_num;
		mmc_set_data_timeout(&data, func->card);
		mmc_wait_for_req(func->card->host, &mrq);

		if (cmd.error){
			ret = cmd.error;
			goto out;
		}
		if (data.error) {
			ret = data.error;
			goto out;
		}
		if (cmd.resp[0] & R5_ERROR) {
			ret = -EIO;
			goto out;
		}
		if (cmd.resp[0] & R5_FUNCTION_NUMBER) {
			ret = -EINVAL;
			goto out;
		}
		if (cmd.resp[0] & R5_OUT_OF_RANGE) {
			ret = -ERANGE;
			goto out;
		}
	} else {
		bes_err("%s,%d (%u)\n", __func__, __LINE__, size);
		ret = -EINVAL;
	}
out:
#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
	if (unlikely(ret))
		self->tx_data_toggle--;
#endif
	return ret;
}

static void sdio_tx_work(struct work_struct *work)
{
	int ret, crc_retry = 0;
	u32 blks, cur_blk = 0, align, total_len = 0, scatters = 0;
	struct list_head proc_list;
	struct bes_sdio_tx_list_t *tx_buffer, *temp;
	struct sbus_priv *self = container_of(work, struct sbus_priv, tx_work);
	struct scatterlist *sg = NULL;
	int bes_sdio_tx_multiple_num;
	struct HI_MSG_HDR *pMsg;
	enum DRIVER_TO_MCU_MSG_ST driver_to_mcu = ST_EXIT;

	/* don't read/write sdio when sdio error */
	if (bes2600_chrdev_is_bus_error())
		return;

	if (bes2600_chrdev_is_signal_mode()) {
		sg = self->tx_sg;
		bes_sdio_tx_multiple_num = BES_SDIO_TX_MULTIPLE_NUM;
	} else {
		sg = self->tx_sg_nosignal;
		bes_sdio_tx_multiple_num = BES_SDIO_TX_MULTIPLE_NUM_NOSIGNAL;
	}

	INIT_LIST_HEAD(&proc_list);

	for (;;) {
		spin_lock(&self->tx_bufferlock);
		list_splice_tail_init(&self->tx_bufferlist, &proc_list);
		spin_unlock(&self->tx_bufferlock);
		if (list_empty(&proc_list))
			break;
		sg_init_table(sg, bes_sdio_tx_multiple_num + 1);
		list_for_each_entry_safe(tx_buffer, temp, &proc_list, node) {
			blks = (tx_buffer->len + self->func->cur_blksize - 1) / self->func->cur_blksize;
			align = blks * self->func->cur_blksize;
			if (blks >= 4) {
				align = 1632;
			}
			if (unlikely(blks >= 5)) {
				bes_err("%s,%d skip error-len packet:%u,%d\n", __func__, __LINE__, tx_buffer->len, blks);
				list_del_init(&tx_buffer->node);
				kmem_cache_free(self->tx_bufferlistpool, tx_buffer);
				continue;
			}
			bes_devel("%s,%p,%u->%u\n", __func__, tx_buffer->buf, tx_buffer->len, align);
			if (!cur_blk)
				cur_blk = blks;
			else if (cur_blk != blks)
				goto flush_previous;

			pMsg = (struct HI_MSG_HDR *)tx_buffer->buf;
			if (unlikely(IS_DRIVER_VENDOR_CMD(pMsg->MsgId))) {
				if (driver_to_mcu == ST_EXIT) {
					driver_to_mcu = ST_ENTER;
					goto flush_previous;
				}
			}

			/*
			 * FW wants each packet padded to 'align' in the
			 * CMD53 stream.  Do not DMA 'align' bytes from the
			 * skb (KFENCE: 704B object, 1632 map).  Bounce.
			 */
			{
				u8 *slot = self->tx_pad + scatters * 1632;

				if (align > 1632)
					align = 1632;
				memcpy(slot, tx_buffer->buf, tx_buffer->len);
				if (align > tx_buffer->len)
					memset(slot + tx_buffer->len, 0,
					       align - tx_buffer->len);
				sg_set_buf(&sg[scatters], slot, align);
			}
			total_len += align;
			++scatters;
/*del_node:*/
			list_del_init(&tx_buffer->node);
			kmem_cache_free(self->tx_bufferlistpool, tx_buffer);
			self->tx_proc_cnt++;
			if (unlikely(IS_DRIVER_VENDOR_CMD(pMsg->MsgId))) {
				if (driver_to_mcu == ST_ENTER) {
					driver_to_mcu = ST_EXIT;
					break;
				}
			}
			if (scatters >= bes_sdio_tx_multiple_num) {
				break;
			}
		}
flush_previous:
		if (likely(scatters)) {
			if (total_len & 0x3) {
				bes_err("%s: unaligned total_len %u\n",
					__func__, total_len);
				break;
			}
			else
				total_len |= (cur_blk - 1);
			sdio_claim_host(self->func);
			if (self->retune_protected == false) {
				sdio_retune_hold_now(self->func);
				self->retune_protected = true;
			}
			do {
				ret = bes_sdio_memcpy_to_io_helper(self->func, total_len, sg, scatters);
				if (likely(ret != BES_SDIO_CRC_ERROR)) {
					crc_retry = 0;
					break;
				} else {
					crc_retry++;
					bes_err("%s sdio write crc error(%d)\n", __func__, crc_retry);
				}
			} while (crc_retry <= 10);
			sdio_release_host(self->func);
			if (ret) {
				bes_err("%s,%d err=%d,%d,%d\n", __func__,
					__LINE__, ret, scatters, cur_blk);
				sdio_work_debug(self);
				/*
				 * Brief CMD53 -EBUSY is MCU/MMC contention.
				 * Minutes of it (shutdown log) is a wedged
				 * LMAC.  Re-queueing forever blocked NM
				 * poweroff.  Do not force_close.
				 */
				if (ret == -EBUSY || ret == -ETIMEDOUT) {
					if (self->tx_ebusy_streak < 3)
						self->tx_ebusy_streak++;
					bes_err("%s: TX %d — skip force_close (%u)\n",
						__func__, ret,
						self->tx_ebusy_streak);
					if (self->tx_ebusy_streak >= 3) {
						if (self->core)
							bes2600_bh_mark_bus_stale(self->core);
						list_for_each_entry_safe(tx_buffer, temp,
									 &proc_list, node) {
							list_del_init(&tx_buffer->node);
							kmem_cache_free(self->tx_bufferlistpool,
									tx_buffer);
						}
						goto tx_done;
					}
					spin_lock(&self->tx_bufferlock);
					list_splice_tail_init(&proc_list,
							      &self->tx_bufferlist);
					spin_unlock(&self->tx_bufferlock);
					goto tx_done;
				}
				bes2600_chrdev_wifi_force_close(self->core,
								false);
			}
			self->tx_ebusy_streak = 0;
			queue_work(self->sdio_wq, &self->rx_work);
			scatters = 0;
			total_len = 0;
			cur_blk = 0;
			self->tx_xfer_cnt++;
			self->last_tx_data_timestamp = jiffies;
		}
	}
tx_done:
	return;
}

static int bes2600_sdio_pipe_send(struct sbus_priv *self, u8 pipe, u32 len, u8 *buf)
{
	struct bes_sdio_tx_list_t * desc = NULL;

	if (bes2600_chrdev_is_bus_error() ||
	    (self->core && self->core->bus_stale)) {
		bes2600_tx_loop_pipe_send(self->core, buf, len);
		return 0;
	}

	desc = kmem_cache_alloc(self->tx_bufferlistpool, GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	INIT_LIST_HEAD(&desc->node);
	desc->buf = buf;
	desc->len = len;
	if (!buf || !len)
		return -EINVAL;

	spin_lock(&self->tx_bufferlock);
	list_add_tail(&desc->node, &self->tx_bufferlist);
	self->tx_data_cnt++;
	spin_unlock(&self->tx_bufferlock);
	queue_work(self->sdio_wq, &self->tx_work);
	return 0;

}
#endif

static int bes2600_sdio_misc_init(struct sbus_priv *self, struct bes2600_common *core)
{
#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
	self->rx_data_toggle = 0;
	self->tx_data_toggle = 0;
	self->next_toggle = 0;
#endif
#ifdef CONFIG_BES_SDIO_RX_MULTIPLE_ENABLE
	spin_lock_init(&self->rx_queue_lock);
	skb_queue_head_init(&self->rx_queue);
	self->rx_buffer = (u8 *)__get_dma_pages(GFP_KERNEL, get_order(1632 * BES_SDIO_RX_MULTIPLE_NUM));
	if (!self->rx_buffer)
		return -ENOMEM;
	INIT_WORK(&self->rx_work, sdio_rx_work);
#endif
#ifdef CONFIG_BES_SDIO_TX_MULTIPLE_ENABLE
	INIT_LIST_HEAD(&self->tx_bufferlist);
	spin_lock_init(&self->tx_bufferlock);
	self->tx_buffer = (u8 *)kmalloc(512, GFP_KERNEL);
	if (!self->tx_buffer) {
		goto err2;
	}
	self->tx_pad = (u8 *)__get_dma_pages(GFP_KERNEL,
		get_order(1632 * BES_SDIO_TX_MULTIPLE_NUM));
	if (!self->tx_pad) {
		kfree(self->tx_buffer);
		self->tx_buffer = NULL;
		goto err2;
	}
	self->tx_bufferlistpool = kmem_cache_create("sdio_tx_bufferlistpool", sizeof(struct bes_sdio_tx_list_t), 0, SLAB_HWCACHE_ALIGN, NULL);
	if (!self->tx_bufferlistpool)
		goto err1;
	self->sdio_wq = alloc_workqueue("bes_sdio", WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 2);
	if (!self->sdio_wq)
		goto err0;
	INIT_WORK(&self->tx_work, sdio_tx_work);
	return 0;
err0:
	kmem_cache_destroy(self->tx_bufferlistpool);
err1:
	if (self->tx_pad) {
		free_pages((unsigned long)self->tx_pad,
			   get_order(1632 * BES_SDIO_TX_MULTIPLE_NUM));
		self->tx_pad = NULL;
	}
	kfree(self->tx_buffer);
err2:
	free_pages((unsigned long)self->rx_buffer, get_order(1632 * BES_SDIO_RX_MULTIPLE_NUM));
	return -ENOMEM;
#endif
	return 0;
}

static struct bes2600_platform_data_sdio bes_sdio_plat_data = {
	.inited = false
};

struct bes2600_platform_data_sdio *bes2600_get_platform_data(void)
{
	return &bes_sdio_plat_data;
}

static int bes2600_platform_data_init(struct device *dev)
{
	struct bes2600_platform_data_sdio *pdata = bes2600_get_platform_data();
	struct device_node *np;

	// skip reinit if already inited
	if (pdata->inited)
		return 0;

	np = of_find_compatible_node(NULL, NULL, "bestechnic,bes2600-sdio");
	if (!np) {
		bes_err("bes2600-sdio device node not found!\n");
		return -ENXIO;
	}

	/* Ensure I/Os are pulled low */
	/* The hardware for these pins have an error? No connected?
	pdata->reset = devm_fwnode_gpiod_get_index(dev, &np->fwnode, "reset", 0, GPIOD_OUT_LOW, "bes2600_wlan_reset");
	if (IS_ERR(pdata->reset)) {
		bes_err("can't request reset_gpio (%ld)\n", PTR_ERR(pdata->reset));
		pdata->reset = NULL;
 	}

	pdata->powerup = devm_fwnode_gpiod_get_index(dev, &np->fwnode, "powerup", 0, GPIOD_OUT_LOW, "bes2600_wlan_powerup");
	if (IS_ERR(pdata->powerup)) {
		bes_err("can't request powerup_gpio (%ld)\n", PTR_ERR(pdata->powerup));
		pdata->powerup = NULL;
 	}
	*/

	pdata->wakeup = devm_fwnode_gpiod_get_index(dev, &np->fwnode, "wakeup", 0, GPIOD_OUT_LOW, "bes2600_wakeup");
	if (IS_ERR(pdata->wakeup)) {
		bes_err("can't request wakeup_gpio (%ld)\n", PTR_ERR(pdata->wakeup));
		pdata->wakeup = NULL;
 	}

	pdata->host_wakeup = devm_fwnode_gpiod_get_index(dev, &np->fwnode, "host-wakeup", 0, GPIOD_IN, "bes2600_host_irq");
	if (IS_ERR(pdata->host_wakeup)) {
		bes_err("can't request host_wake_gpio (%ld)\n", PTR_ERR(pdata->host_wakeup));
		pdata->host_wakeup = NULL;
 	}

	pdata->wlan_bt_hostwake_registered = false;
	pdata->inited = true;
	return 0;
}

static int bes2600_sdio_reset(struct sbus_priv *self)
{
	// As not used due to below comment, build fails.
	//const struct bes2600_platform_data_sdio *pdata = bes2600_get_platform_data();

	bes_devel("%s\n", __func__);

/*	Due to error in hardware?, reset and power pins are not availiable?
	gpiod_direction_output(pdata->reset, GPIOD_OUT_HIGH);
	mdelay(50);
	gpiod_direction_output(pdata->reset, GPIOD_OUT_LOW);
*/

	return 0;
}

static int bes2600_sdio_readb_once(struct sdio_func *func, unsigned int addr)
{
	int ret = 0;
	u8 val = sdio_readb(func, addr, &ret);

	return (ret < 0) ? ret : val;
}

static int bes2600_sdio_readb_safe(struct sdio_func *func, unsigned int addr)
{
	int ret = 0;
	u8 val = 0;
	u8 retry = 0;

	do {
		val = sdio_readb(func, addr, &ret);
	} while((ret < 0) && ++retry < SDIO_RETRY_MAX);

	if (ret)
		bes_err("%s failed addr=0x%x ret=%d retry=%d\n", __func__, addr, ret, retry);

	return (ret < 0) ? ret : val;
}

static int bes2600_sdio_writeb_safe(struct sdio_func *func, unsigned int addr, u8 val)
{
	int ret;
	u8 retry = 0;

	do {
		sdio_writeb(func, val, addr, &ret);
	} while((ret < 0) && ++retry < SDIO_RETRY_MAX);

	if (ret)
		bes_err("%s failed, ret:%d\n", __func__, ret);

	return ret;
}

static void bes2600_gpio_wakeup_mcu(struct sbus_priv *self)
{
	const struct bes2600_platform_data_sdio *pdata = bes2600_get_platform_data();

	mutex_lock(&self->io_mutex);

	if (atomic_inc_return(&self->gpio_wakeup_ref) == 1) {
		bes_devel("pull high gpio (first user)\n");
		gpiod_direction_output(pdata->wakeup, GPIOD_OUT_HIGH);
		mutex_unlock(&self->io_mutex);
		/* MCU ignores CMD52 until the wake GPIO has settled. */
		usleep_range(2000, 4000);
		return;
	}

	mutex_unlock(&self->io_mutex);
}

static void bes2600_gpio_allow_mcu_sleep(struct sbus_priv *self)
{
	const struct bes2600_platform_data_sdio *pdata = bes2600_get_platform_data();

	mutex_lock(&self->io_mutex);

	int ref = atomic_dec_return(&self->gpio_wakeup_ref);
	if (ref == 0) {
		bes_devel("pull low gpio (no more users)\n");
		gpiod_direction_output(pdata->wakeup, GPIOD_OUT_LOW);
	} else if (ref < 0) {
		bes_err("gpio wakeup refcount went negative!\n");
		atomic_set(&self->gpio_wakeup_ref, 0);
	}

	//bes_info("gpio wakeup ref now: %d\n", atomic_read(&self->gpio_wakeup_ref));
	mutex_unlock(&self->io_mutex);
}

static int bes2600_sdio_active(struct sbus_priv *self, int sub_system)
{
	u16 cfg;
	u8 cfm = 0;
	int ret = 0, retries = 0;
	u8 tmp_val = 0;
	u32 cnt = 0;
//	u32 delay_cnt = 2;

	/* nosignal mode only allow SUBSYSTEM_WIFI */
	if (!bes2600_chrdev_is_signal_mode() && sub_system != SUBSYSTEM_WIFI)
		return -EINVAL;

	/* don't read/write sdio when sdio error */
	if (bes2600_chrdev_is_bus_error())
		return 0;

	/* prevent concurrent access */
	mutex_lock(&self->sbus_mutex);

	/* set config and confirm value */
	if (sub_system == SUBSYSTEM_MCU) {
		cfg = BES_HOST_INT | BES_SUBSYSTEM_MCU_ACTIVE;
		cfm = BES_SLAVE_STATUS_MCU_WAKEUP_READY;
		//delay_cnt = 2;
	} else if (sub_system == SUBSYSTEM_WIFI) {
		cfg = BES_HOST_INT | BES_SUBSYSTEM_WIFI_ACTIVE;
		cfm = BES_SLAVE_STATUS_WIFI_READY;
		//delay_cnt = 25;
	} else if(sub_system == SUBSYSTEM_BT) {
		cfg = BES_HOST_INT | BES_SUBSYSTEM_BT_ACTIVE;
		cfm = BES_SLAVE_STATUS_BT_READY;
		//delay_cnt = 25;
	} else if(sub_system == SUBSYSTEM_BT_LP) {
		cfg = BES_HOST_INT | BES_SUBSYSTEM_BT_WAKEUP;
		cfm = BES_SLAVE_STATUS_BT_WAKE_READY;
		//delay_cnt = 2;
	} else {
		mutex_unlock(&self->sbus_mutex);
		return -EINVAL;
	}

	/* set fw_started flag in advance */
	if(sub_system == SUBSYSTEM_WIFI) {
		self->fw_started = true;
	}

	/* wait until device ready */
	do {
		sdio_claim_host(self->func);
		ret = bes2600_sdio_readb_once(self->func, BES_SLAVE_STATUS_REG_ID);
		sdio_release_host(self->func);
		bes_devel("active wait mcu ready cnt:%d, reg:%d\n", cnt, ret);
		if (ret < 0) {
			/*
			 * -EBUSY is MMC contention / MCU still waking, not a
			 * dead chip.  force_close from here (scan_work →
			 * wsm_cmd_lock → exit_lp) WARNed in tx_loop then
			 * hard-locked CPU3.
			 */
			if ((ret == -EBUSY || ret == -ETIMEDOUT) && ++cnt <= 500) {
				usleep_range(1000, 2000);
				continue;
			}
			goto err;
		}
		if ((ret & BES_SLAVE_STATUS_MCU_READY) == 0) {
			if (++cnt > 500) {
				bes_err("active wait MCU_READY timeout, subsys:%d\n",
					sub_system);
				mutex_unlock(&self->sbus_mutex);
				return -ETIMEDOUT;
			}
			usleep_range(1000, 2000);
		}
	} while (ret < 0 || (ret & BES_SLAVE_STATUS_MCU_READY) == 0);

	/* Already active? Re-sending ACTIVE can confuse the firmware. */
	if (cfm && (ret & cfm)) {
		mutex_unlock(&self->sbus_mutex);
		return 0;
	}

	do {
		/* claim sdio host */
		sdio_claim_host(self->func);

		/* write first segment */
		tmp_val = (cfg >> 8) & 0xff;
		ret = bes2600_sdio_writeb_safe(self->func, BES_HOST_INT_REG_ID + 1, tmp_val);
		if(ret < 0) {
			sdio_release_host(self->func);
			bes_err("active write 1st seg failed\n");
			goto err;
		}

		/* write second segment */
		tmp_val = cfg & 0xff;
		ret = bes2600_sdio_writeb_safe(self->func, BES_HOST_INT_REG_ID, tmp_val);
		if(ret < 0) {
			sdio_release_host(self->func);
			bes_err("active write 2nd seg failed\n");
			goto err;
		}

		/* release sdio host */
		sdio_release_host(self->func);

		/* wait for device to response */
    	//bes_devel("Sleeping in: %s\n", __func__);
		usleep_range(10000, 12000);  // Atomic-safe

		/* read device response result */
		sdio_claim_host(self->func);
		ret = bes2600_sdio_readb_once(self->func, BES_SLAVE_STATUS_REG_ID);
		sdio_release_host(self->func);
		if (ret < 0) {
			if ((ret == -EBUSY || ret == -ETIMEDOUT) &&
			    ++retries <= 200) {
				usleep_range(10000, 12000);
				continue;
			}
			bes_err("active read response failed\n");
			goto err;
		}
		bes_devel("active resp cnt:%d, reg:%d, sub_sys:%d\n", retries, ret, sub_system);
	} while ((cfm != 0) && (ret & cfm) == 0 && ++retries <= 200);	// check if cfm bit is set

	if (retries > 200) {
		bes_err("bes2600_sdio_active failed, subsys:%d\n", sub_system);
		/* open wifi failed, restore fw_started flag */
		if(sub_system == SUBSYSTEM_WIFI) {
			self->fw_started = false;
		}

		mutex_unlock(&self->sbus_mutex);
		return -EFAULT;
	} else {
		ret = 0;
	}

#ifdef CONFIG_BES2600_GPIO_WAKEUP_AP
	if (sub_system == SUBSYSTEM_WIFI ||
		sub_system == SUBSYSTEM_BT)
		ret = bes2600_gpio_wakeup_ap_config(self);
#else
	ret = 0;
#endif
	/* prevent concurrent access */
	mutex_unlock(&self->sbus_mutex);

	return ret;
err:
	mutex_unlock(&self->sbus_mutex);
	if (ret == -EBUSY || ret == -ETIMEDOUT) {
		bes_err("bes2600_sdio_active: %d — skip force_close, subsys:%d\n",
			ret, sub_system);
		if (sub_system == SUBSYSTEM_WIFI)
			self->fw_started = false;
		return ret;
	}
	bes2600_chrdev_wifi_force_close(self->core, false);
	return -ENODEV;
}

static void bes2600_sdio_empty_work(struct sbus_priv *self)
{
#ifdef CONFIG_BES_SDIO_RX_MULTIPLE_ENABLE
	struct sk_buff *skb;
#endif
#ifdef CONFIG_BES_SDIO_TX_MULTIPLE_ENABLE
	struct bes_sdio_tx_list_t *tx_buffer, *temp;
#endif

#ifdef CONFIG_BES_SDIO_RX_MULTIPLE_ENABLE
	cancel_work_sync(&self->rx_work);
	while (1) {
		skb = skb_dequeue(&self->rx_queue);
		if (skb)
			dev_kfree_skb(skb);
		else
			break;
	}
	self->rx_last_ctrl = 0;
	self->rx_total_ctrl_cnt = 0;
	self->rx_continuous_ctrl_cnt = 0;
	self->rx_remain_ctrl_cnt = 0;
	self->rx_zero_ctrl_cnt = 0;
	self->rx_data_cnt = 0;
	self->rx_xfer_cnt = 0;
	self->rx_proc_cnt = 0;
#endif

#ifdef CONFIG_BES_SDIO_TX_MULTIPLE_ENABLE
	cancel_work_sync(&self->tx_work);
	list_for_each_entry_safe(tx_buffer, temp, &self->tx_bufferlist, node) {
		list_del_init(&tx_buffer->node);
		kmem_cache_free(self->tx_bufferlistpool, tx_buffer);
	}
	self->tx_data_cnt = 0;
	self->tx_xfer_cnt = 0;
	self->tx_proc_cnt = 0;
#endif

#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
	self->rx_data_toggle = 0;
	self->tx_data_toggle = 0;
	self->next_toggle = 0;
#endif
}

#ifdef CONFIG_BES2600_GPIO_WAKEUP_AP
static void bes2600_wlan_bt_hostwake_unregister(void);
#endif

static int bes2600_sdio_deactive(struct sbus_priv *self, int sub_system)
{
	u16 cfg = 0;
	u8 cfm = 0;
	u8 tmp_val = 0;
	u16 retries = 0;
	u32 cnt = 0;
	// u32 delay_cnt = 2;
	int ret;

	/* don't read/write sdio when sdio error */
	if (bes2600_chrdev_is_bus_error() ||
	    (self->core && self->core->bus_stale))
		return 0;

	/* notify device deactive event */
	if (bes2600_chrdev_is_signal_mode()) {
		/* prevent concurrent access */
		mutex_lock(&self->sbus_mutex);

		/* set config and confirm value */
		if (sub_system == SUBSYSTEM_MCU) {
			cfg = BES_HOST_INT | BES_SUBSYSTEM_MCU_DEACTIVE;
			cfm = BES_SLAVE_STATUS_MCU_WAKEUP_READY;
		} else if (sub_system == SUBSYSTEM_WIFI) {
			cfg = BES_HOST_INT | BES_SUBSYSTEM_WIFI_DEACTIVE;
			cfm = BES_SLAVE_STATUS_WIFI_READY;
		} else if(sub_system == SUBSYSTEM_BT) {
			cfg = BES_HOST_INT | BES_SUBSYSTEM_BT_DEACTIVE;
			cfm = BES_SLAVE_STATUS_BT_READY;
		} else if(sub_system == SUBSYSTEM_BT_LP) {
			cfg = BES_HOST_INT | BES_SUBSYSTEM_BT_SLEEP;
			cfm = BES_SLAVE_STATUS_BT_WAKE_READY;
		} else {
			mutex_unlock(&self->sbus_mutex);
			return -EINVAL;
		}

		/* wait until device ready */
		do {
			sdio_claim_host(self->func);
			ret = bes2600_sdio_readb_once(self->func, BES_SLAVE_STATUS_REG_ID);
			sdio_release_host(self->func);
			bes_devel("deactive wait mcu ready cnt:%d, reg:%d\n", cnt, ret);

			if (ret < 0) {
				if ((ret == -EBUSY || ret == -ETIMEDOUT) &&
				    ++cnt <= 500) {
					usleep_range(1000, 2000);
					continue;
				}
				goto err;
			}
			if ((ret & BES_SLAVE_STATUS_MCU_READY) == 0) {
				if (++cnt > 500) {
					bes_err("deactive wait MCU_READY timeout, subsys:%d\n",
						sub_system);
					mutex_unlock(&self->sbus_mutex);
					return -ETIMEDOUT;
				}
				usleep_range(1000, 2000);
			}
		} while (ret < 0 || (ret & BES_SLAVE_STATUS_MCU_READY) == 0);

		do {
			/* claim sdio host */
			sdio_claim_host(self->func);

			/* write first segment */
			tmp_val = (cfg >> 8) & 0xff;
			ret = bes2600_sdio_writeb_safe(self->func, BES_HOST_INT_REG_ID + 1, tmp_val);
			if(ret < 0) {
				sdio_release_host(self->func);
				bes_err("deactive write 1st seg failed\n");
				goto err;
			}

			/* write second segment */
			tmp_val = cfg & 0xff;
			ret = bes2600_sdio_writeb_safe(self->func, BES_HOST_INT_REG_ID, tmp_val);
			if(ret < 0) {
				sdio_release_host(self->func);
				bes_err("deactive write 2nd seg failed\n");
				goto err;
			}

			/* release sdio host */
			sdio_release_host(self->func);

			/* wait device to response */
    		bes_devel("Sleeping in: %s\n", __func__);
			usleep_range(10000, 12000);  // Atomic-safe
			//msleep(delay_cnt);

			/* read device response result */
			sdio_claim_host(self->func);
			ret = bes2600_sdio_readb_safe(self->func, BES_SLAVE_STATUS_REG_ID);
			sdio_release_host(self->func);
			if(ret < 0) {
				bes_err("deactive read response failed\n");
				if (sub_system == SUBSYSTEM_MCU) {
					/* cmd52 may return error when 2600 is sleeping */
					ret = 0;
					break;
				} else {
					goto err;
				}
			}
			bes_devel("deactive resp cnt:%d, reg:%d, sub_sys:%d\n", retries, ret, sub_system);
		} while((cfm != 0) && (ret & cfm) != 0 && ++retries < 200);

		/* set fw_started flag to false */
		if(bes2600_chrdev_is_signal_mode()
		   && sub_system == SUBSYSTEM_WIFI)
			self->fw_started = false;

		/* reset sdio send and receive control variable */
		if(sub_system == SUBSYSTEM_WIFI) {
			bes2600_sdio_empty_work(self);
		}

		/* prevent concurrent access */
		mutex_unlock(&self->sbus_mutex);

		return (ret < 0) ? ret : 0;
	} else {
		return 0;
	}

err:
	mutex_unlock(&self->sbus_mutex);
	if (ret == -EBUSY || ret == -ETIMEDOUT) {
		bes_err("bes2600_sdio_deactive: %d — skip force_close, subsys:%d\n",
			ret, sub_system);
		if (self->core)
			bes2600_bh_mark_bus_stale(self->core);
		return ret;
	}
	bes2600_chrdev_wifi_force_close(self->core, false);
	return -ENODEV;
}

static void bes2600_sdio_power_down(struct sbus_priv *self)
{
#ifdef POWER_DOWN_BY_MSG
	u32 cfg = BES_HOST_INT | BES_SUBSYSTEM_SYSTEM_CLOSE;
	u8 tmp_val = 0;
	int ret = 0;

	sdio_claim_host(self->func);
	tmp_val = (cfg >> 8) & 0xff;
	sdio_writeb(self->func, tmp_val, BES_HOST_INT_REG_ID + 1, &ret);
	tmp_val = cfg & 0xff;
	sdio_writeb(self->func, tmp_val, BES_HOST_INT_REG_ID, &ret);
	sdio_release_host(self->func);
#else
	// As not used due to below comment, build fails.
	//struct bes2600_platform_data_sdio *pdata = bes2600_get_platform_data();
	// Both pins are not availiable? being used for other things?
	// gpiod_direction_output(pdata->powerup, GPIOD_OUT_LOW);
#endif

    bes_devel("Sleeping in: %s\n", __func__);
	msleep(10);

	self->func->card->host->caps &= ~MMC_CAP_NONREMOVABLE;

}

static int bes2600_sdio_power_switch(struct sbus_priv *self, int on)
{
	// TODO: return something meaningful, perhaps
	struct bes2600_platform_data_sdio *pdata = bes2600_get_platform_data();
	if (on) {
		bes2600_sdio_on(pdata);
		return 0;
	}
	bes2600_sdio_power_down(self);
	return 0;
}

static void bes2600_sdio_halt_device(struct sbus_priv *self)
{
	sdio_work_debug(self);
}

static bool bes2600_sdio_wakeup_source(struct sbus_priv *self)
{
	struct bes2600_platform_data_sdio *pdata = bes2600_get_platform_data();

	if(pdata->wakeup_source) {
		pdata->wakeup_source = false;
		return true;
	}

	return false;
}

static struct sbus_ops bes2600_sdio_sbus_ops = {
	.sbus_memcpy_fromio	= bes2600_sdio_memcpy_fromio,
	.sbus_memcpy_toio	= bes2600_sdio_memcpy_toio,
	.lock			= bes2600_sdio_lock,
	.unlock			= bes2600_sdio_unlock,
	.irq_subscribe		= bes2600_sdio_irq_subscribe,
	.irq_unsubscribe	= bes2600_sdio_irq_unsubscribe,
	.reset			= bes2600_sdio_reset,
	.align_size		= bes2600_sdio_align_size,
	.set_block_size		= bes2600_sdio_set_block_size,
	.sbus_reg_read		= bes2600_sdio_reg_read,
	.sbus_reg_write		= bes2600_sdio_reg_write,
	.init			= bes2600_sdio_misc_init,
#ifdef CONFIG_BES_SDIO_RX_MULTIPLE_ENABLE
	.pipe_read		= bes2600_sdio_pipe_read,
#endif
#ifdef CONFIG_BES_SDIO_TX_MULTIPLE_ENABLE
	.pipe_send		= bes2600_sdio_pipe_send,
#endif
	.sbus_active		= bes2600_sdio_active,
	.sbus_deactive		= bes2600_sdio_deactive,
	.power_switch		= bes2600_sdio_power_switch,
	.gpio_wake		= bes2600_gpio_wakeup_mcu,
	.gpio_sleep		= bes2600_gpio_allow_mcu_sleep,
	.halt_device		= bes2600_sdio_halt_device,
	.wakeup_source		= bes2600_sdio_wakeup_source,
};

static void bes2600_sdio_en_lp_cb(struct bes2600_common *hw_priv)
{
	long unsigned int old_ts, new_ts;
	struct sbus_priv *self = hw_priv->sbus_priv;

	do {
		old_ts = self->last_irq_timestamp;
		flush_work(&self->rx_work);
		new_ts = self->last_irq_timestamp;
	} while(old_ts != new_ts);
}

/* Probe Function to be called by SDIO stack when device is discovered */
static int bes2600_sdio_probe(struct sdio_func *func,
			      const struct sdio_device_id *id)
{
	struct sbus_priv *self;
	int status=0;

	bes_devel("Probe called:%p,%d\n", func, func->num);
	if (func->num > 1)
		return 0;

	func->card->host->caps |= MMC_CAP_NONREMOVABLE;
	bes2600_chrdev_bus_probe_notify();

	self = kzalloc(sizeof(*self), GFP_KERNEL);
	if (!self) {
		bes_devel("Can't allocate SDIO sbus_priv.");
		return -ENOMEM;
	}

	atomic_set(&self->gpio_wakeup_ref, 0);
	spin_lock_init(&self->lock);

	struct device *dev = &func->dev;
	status = bes2600_platform_data_init(dev);
	if (status) {
		bes_err("platform data init failed: %d\n", status);
		goto err;
	}

	self->pdata = bes2600_get_platform_data();
	self->func = func;
	self->dev = &func->dev;
	self->retune_protected = false;
	self->unregister_in_process = false;
	mutex_init(&self->io_mutex);
	mutex_init(&self->sbus_mutex);
#ifdef CONFIG_BES_SDIO_RXTX_TOGGLE
	self->fw_started = false;
#endif
	bes2600_gpio_wakeup_mcu(self);

	sdio_set_drvdata(func, self);
	sdio_claim_host(func);
	sdio_enable_func(func);
	sdio_release_host(func);

	bes2600_reg_set_object(&bes2600_sdio_sbus_ops, self);
	status = bes2600_load_firmware(&bes2600_sdio_sbus_ops, self);
	if(status > 0) {	// for wifi closed case
		bes_devel("interrupt init process beacuse device be closed.\n");
		goto out;
	} else if(status < 0) {	// for download fail case
		bes_err("Loading of firmware failed: %d\n", status);
		goto err;
	}

	status = bes2600_register_net_dev(self);
	if (status) {
		bes_err("Register net dev failed: %d\n", status);
		goto err;
	}

out:
	bes2600_chrdev_set_sbus_priv_data(self, false);
	bes2600_gpio_allow_mcu_sleep(self);
	return 0;

err:
	bes_err("%s failed, func:%d\n", __func__, func->num);
	func->card->host->caps &= ~MMC_CAP_NONREMOVABLE;
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
	bes2600_gpio_allow_mcu_sleep(self);
	sdio_set_drvdata(func, NULL);
	bes2600_reg_set_object(NULL, NULL);
	bes2600_chrdev_set_sbus_priv_data(NULL, true);
	kfree(self);

	if (status < 0) {
		return status;
	} else {
		bes_err("Unspecified probe failure.\n");
		return -EIO; // Generic I/O error
	}
}

int bes2600_register_net_dev(struct sbus_priv *bus_priv)
{
	int status = 0;

	if (!bus_priv) {
		bes_err("%s: NULL bus_priv\n", __func__);
		return -EINVAL;
	}
	status = bes2600_core_probe(&bes2600_sdio_sbus_ops,
			      bus_priv, bus_priv->dev, &bus_priv->core);
	if(!status)
		bes2600_pwr_register_en_lp_cb(bus_priv->core, bes2600_sdio_en_lp_cb);

	return status;
}

int bes2600_unregister_net_dev(struct sbus_priv *bus_priv)
{
	if (!bus_priv) {
		bes_err("%s: NULL bus_priv\n", __func__);
		return -EINVAL;
	}
	if (bus_priv->core && !bus_priv->unregister_in_process) {
		bus_priv->unregister_in_process = true;
		bes2600_core_release(bus_priv->core);
		bes2600_pwr_unregister_en_lp_cb(bus_priv->core, bes2600_sdio_en_lp_cb);
		bus_priv->core = NULL;

		if (bus_priv->sdio_wq) {
			flush_workqueue(bus_priv->sdio_wq);
			destroy_workqueue(bus_priv->sdio_wq);
			bus_priv->sdio_wq = NULL;
		}

		if (bus_priv->rx_buffer) {
			free_pages((unsigned long)bus_priv->rx_buffer, get_order(1632 * BES_SDIO_RX_MULTIPLE_NUM));
			bus_priv->rx_buffer = NULL;
		}

#ifdef CONFIG_BES_SDIO_TX_MULTIPLE_ENABLE
		if (bus_priv->tx_buffer) {
			kfree(bus_priv->tx_buffer);
			bus_priv->tx_buffer = NULL;
		}
		if (bus_priv->tx_pad) {
			free_pages((unsigned long)bus_priv->tx_pad,
				   get_order(1632 * BES_SDIO_TX_MULTIPLE_NUM));
			bus_priv->tx_pad = NULL;
		}

		if (bus_priv->tx_bufferlistpool) {
			kmem_cache_destroy(bus_priv->tx_bufferlistpool);
			bus_priv->tx_bufferlistpool = NULL;
		}
#endif
		bus_priv->unregister_in_process = false;
	}
	return 0;
}

bool bes2600_is_net_dev_created(struct sbus_priv *bus_priv)
{
	if (!bus_priv) {
		bes_err("%s: NULL bus_priv\n", __func__);
		return false;
	}
	return (bus_priv->core != NULL);
}

/* Disconnect Function to be called by SDIO stack when
 * device is disconnected */
static void bes2600_sdio_remove(struct sdio_func *func)
{
	struct sbus_priv *self = sdio_get_drvdata(func);

	func->card->host->caps &= ~MMC_CAP_NONREMOVABLE;
	bes_devel("%s called:%p,%d\n", __func__, func, func->num);

	if (self) {
#ifndef CONFIG_BES2600_USE_GPIO_IRQ
		sdio_claim_host(func);
		sdio_release_irq(func);
		sdio_release_host(func);
#else
		free_irq(irq->start, self);
#endif  //CONFIG_BES2600_USE_GPIO_IRQ
		sdio_claim_host(func);
		sdio_disable_func(func);
		sdio_release_host(func);
		bes2600_reg_set_object(NULL, NULL);
		bes2600_chrdev_set_sbus_priv_data(NULL, false);
		sdio_set_drvdata(func, NULL);
		if (self->retune_protected == true) {
			sdio_retune_release(func);
		}
		kfree(self);
	}
}

#ifdef CONFIG_BES2600_GPIO_WAKEUP_AP

static irqreturn_t bes2600_wlan_bt_hostwake_thread(int irq, void *dev_id)
{
	struct bes2600_platform_data_sdio *pdata = bes2600_get_platform_data();

	bes_devel("bes2600_wlan_hostwake:%d\n", dev_id == (void *)pdata);

	if (dev_id == (void *)pdata) {
		bes2600_chrdev_wakeup_by_event_set(WAKEUP_EVENT_SETTING);
		pdata->wakeup_source = true;
		disable_irq_nosync(irq);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static int bes2600_wlan_bt_hostwake_register(void)
{
	int ret = 0;
	struct bes2600_platform_data_sdio *pdata = bes2600_get_platform_data();

	// flipping internal struct registers considered as nothing
	bes_warn("%s: this function does nothing\n", __FUNCTION__);

	if (pdata->wlan_bt_hostwake_registered == true) {
		bes_err("wlan hostwake register repeatedly.\n");
		return -1;
	}

	pdata->wlan_bt_hostwake_registered = true;
	pdata->wakeup_source = false;

	return ret;
}

static void bes2600_wlan_bt_hostwake_unregister(void)
{
	int ret = 0;
	struct bes2600_platform_data_sdio *pdata = bes2600_get_platform_data();

	// flipping internal struct registers considered as nothing
	bes_warn("%s: this function does nothing\n", __FUNCTION__);

	if (pdata->wlan_bt_hostwake_registered == false)
		return;

	pdata->wlan_bt_hostwake_registered = false;
}

static int bes2600_gpio_wakeup_ap_config(struct sbus_priv *self)
{
	u8 wakeup_cfg = 0;
	int ret = 0, irq_flags = 0, irq = 0;

    if (!bes2600_chrdev_is_signal_mode())
        return 0;

	if (irq_flags & IRQF_TRIGGER_HIGH) {
		wakeup_cfg = BES_AP_WAKEUP_GPIO_HIGH | BES_AP_WAKEUP_CFG_VALID;
	} else if (irq_flags & IRQF_TRIGGER_LOW) {
		wakeup_cfg = BES_AP_WAKEUP_GPIO_LOW | BES_AP_WAKEUP_CFG_VALID;
	} else if (irq_flags & IRQF_TRIGGER_RISING) {
		wakeup_cfg = BES_AP_WAKEUP_GPIO_RISE | BES_AP_WAKEUP_CFG_VALID;
	} else if (irq_flags & IRQF_TRIGGER_FALLING) {
		wakeup_cfg = BES_AP_WAKEUP_GPIO_FALL | BES_AP_WAKEUP_CFG_VALID;
	}

	if (wakeup_cfg & BES_AP_WAKEUP_CFG_VALID)
		wakeup_cfg |= (BES_AP_WAKEUP_TYPE_GPIO << BES_AP_WAKEUP_TYPE_SHIFT);

	bes_devel("%s config:%x\n", __func__, wakeup_cfg);

	sdio_claim_host(self->func);
	sdio_writeb(self->func, wakeup_cfg, BES_AP_WAKEUP_REG_ID, &ret);
	if (!ret) {
		sdio_writeb(self->func, 0, BES_HOST_INT_REG_ID + 1, &ret);
	}
	if (!ret) {
		sdio_writeb(self->func, (BES_HOST_INT | BES_AP_WAKEUP_CFG), BES_HOST_INT_REG_ID, &ret);
	}
	sdio_release_host(self->func);
	if (ret) {
		bes_err("%s failed:%d\n", __func__, ret);
		free_irq(irq, &bes_sdio_plat_data);
		return ret;
	}

	return 0;
}
#endif

static int bes2600_sdio_prepare(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct sbus_priv *self = sdio_get_drvdata(func);

	bes_devel("%s (%p,%d)enter\n", __func__, func, func->num);

	if (func->num > 1)
		return 0;

	if(bes2600_sdio_sbus_ops.gpio_wake)
		bes2600_sdio_sbus_ops.gpio_wake(self);

	return 0;
}

static int bes2600_sdio_suspend(struct device *dev)
{
	int ret;
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct sbus_priv *self = sdio_get_drvdata(func);

	bes_devel("%s (%p,%d)enter\n", __func__, func, func->num);
	if (func->num > 1)
		return 0;

#ifndef CONFIG_BES2600_WOWLAN
	if(bes2600_chrdev_check_system_close() == false)
		return -EBUSY;
#endif

	/* Notify SDIO that BES2600 will remain powered during suspend */
	ret = sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
	if (ret) {
		bes_err("Error setting SDIO pm flags: %i\n", ret);
		return ret;
	}

	if (bes2600_chrdev_is_bt_opened() == true) {
		if ((ret = bes2600_sdio_deactive(self, SUBSYSTEM_BT_LP))) {
			bes_err("bt sleep in suspend failed:%d.\n", ret);
			return ret;
		}
	}

#ifdef CONFIG_BES2600_GPIO_WAKEUP_AP
	return bes2600_wlan_bt_hostwake_register();
#endif

	return 0;
}

static int bes2600_sdio_suspend_noirq(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct sbus_priv *self = sdio_get_drvdata(func);

	bes_devel("%s (%p,%d)enter\n", __func__, func, func->num);

	if (func->num > 1)
		return 0;

	if(self->core &&
	   (work_pending(&self->rx_work) || atomic_read(&self->core->bh_rx))) {
		bes_devel("%s: Suspend interrupted.\n", __func__);
		return -EAGAIN;
	}

	if(bes2600_sdio_sbus_ops.gpio_sleep)
		bes2600_sdio_sbus_ops.gpio_sleep(self);

	if (self->retune_protected == true)
		bes_warn("retune is closed while ap sleep.\n");

	return 0;
}

static int bes2600_sdio_resume_noirq(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct sbus_priv *self = sdio_get_drvdata(func);

	bes_devel("%s (%p,%d)enter\n", __func__, func, func->num);

	if (func->num > 1)
		return 0;

	if(bes2600_sdio_sbus_ops.gpio_wake)
		bes2600_sdio_sbus_ops.gpio_wake(self);

	return 0;
}

static int bes2600_sdio_resume(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);

	bes_devel("%s (%p,%d)enter\n", __func__, func, func->num);

	if (func->num > 1)
		return 0;

#ifdef CONFIG_BES2600_GPIO_WAKEUP_AP
	bes2600_wlan_bt_hostwake_unregister();
#endif

	return 0;
}

static void bes2600_sdio_complete(struct device *dev)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct sbus_priv *self = sdio_get_drvdata(func);

	bes_devel("%s (%p,%d)enter\n", __func__, func, func->num);

	if (func->num > 1)
		return;

	/* wakeup bt if bt is on */
	bes2600_chrdev_wakeup_bt();

	/* clear resume gpio wake flag */
	if(bes2600_sdio_sbus_ops.gpio_sleep)
		bes2600_sdio_sbus_ops.gpio_sleep(self);
}

static const struct dev_pm_ops bes2600_pm_ops = {
	.prepare = bes2600_sdio_prepare,
	.suspend = bes2600_sdio_suspend,
	.suspend_noirq = bes2600_sdio_suspend_noirq,
	.resume_noirq = bes2600_sdio_resume_noirq,
	.resume = bes2600_sdio_resume,
	.complete = bes2600_sdio_complete,
};

static struct sdio_driver sdio_driver = {
	.name		= "bes2600_wlan",
	.id_table	= bes2600_sdio_ids,
	.probe		= bes2600_sdio_probe,
	.remove		= bes2600_sdio_remove,
	.drv = {
		.pm = &bes2600_pm_ops,
	}
};

/* Init Module function -> Called by insmod */
static int __init bes2600_sdio_init(void)
{
	const struct bes2600_platform_data_sdio *pdata = NULL;

	bes_devel("------Driver: bes2600.ko version :%s\n", BES2600_DRV_VERSION);

	bes2600_chrdev_update_signal_mode();
	int ret = bes2600_chrdev_init(&bes2600_sdio_sbus_ops);
	if (ret)
		return ret;

	ret = sdio_register_driver(&sdio_driver);
	if (ret) {
		bes2600_chrdev_free();
		return ret;
	}

	pdata = bes2600_get_platform_data();
	if (!pdata->inited) {
		bes_err("pdata->inited = %d, platform data must be inited at this point\n", pdata->inited);
		sdio_unregister_driver(&sdio_driver);
		bes2600_chrdev_free();
		// probably does nothing, but still
		if (pdata)
			bes2600_sdio_off(pdata);
		return -ECANCELED;
	}

	return 0;
}

/* Called at Driver Unloading */
static void __exit bes2600_sdio_exit(void)
{
	const struct bes2600_platform_data_sdio *pdata = bes2600_get_platform_data();
	struct sbus_priv *priv =  bes2600_chrdev_get_sbus_priv_data();
	bes_devel("%s called\n", __func__);

	bes2600_unregister_net_dev(priv);
	sdio_unregister_driver(&sdio_driver);
	bes2600_chrdev_free();
	bes2600_sdio_off(pdata);
}

module_init(bes2600_sdio_init);
module_exit(bes2600_sdio_exit);
