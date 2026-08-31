#ifndef BES_LOG_H
#define BES_LOG_H

extern struct device *global_dev;

#ifdef CONFIG_BES2600_ENABLE_DEVEL_LOGS
#define bes_devel(fmt, ...) dev_dbg(global_dev, fmt, ##__VA_ARGS__)
#else
#define bes_devel(fmt, ...) no_printk(fmt, ##__VA_ARGS__)
#endif
#define bes_info(fmt, ...) dev_info(global_dev, fmt, ##__VA_ARGS__)
#define bes_warn(fmt, ...) dev_warn(global_dev, fmt, ##__VA_ARGS__)
#define bes_err(fmt, ...) dev_err(global_dev, fmt, ##__VA_ARGS__)
/* KERN_ERR so it hits the console even if kmsg dies mid-line */
#define bes_pin(fmt, ...) printk(KERN_ERR "bes2600 %s: " fmt, __func__, ##__VA_ARGS__)

/*
 * Firmware NAK / SDIO timeout / "AP did not ACK" are expected.  Do not
 * WARN_ON() them (stack dump + TAINT_WARN) or BUG_ON() (panic).
 * Keep WARN_ON() only for true programmer invariants (NULL about to
 * be dereferenced, queue id that would corrupt memory).
 */
static inline int bes_fail(int ret, const char *what)
{
	if (ret)
		bes_err("%s failed: %d\n", what, ret);
	return ret;
}

#endif /* BES_LOG_H */
