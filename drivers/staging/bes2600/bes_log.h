#ifndef BES_LOG_H_INCLUDED
#define BES_LOG_H_INCLUDED

extern struct device *global_dev;

#ifdef CONFIG_BES2600_ENABLE_DEVEL_LOGS
#define bes_devel(fmt, ...) dev_dbg(global_dev, fmt, ##__VA_ARGS__)
#else
#define bes_devel(fmt, ...) no_printk(fmt, ##__VA_ARGS__)
#endif
#define bes_info(fmt, ...) dev_info(global_dev, fmt, ##__VA_ARGS__)
#define bes_warn(fmt, ...) dev_warn(global_dev, fmt, ##__VA_ARGS__)
#define bes_err(fmt, ...) dev_err(global_dev, fmt, ##__VA_ARGS__)

/* Firmware NAK / WSM timeout: log, keep errno, do not WARN_ON (taint). */
static inline int bes_fail(const char *fn, int ret)
{
	if (unlikely(ret))
		bes_err("%s: failed %d\n", fn, ret);
	return ret;
}
/* KERN_ERR so it hits the console even if kmsg dies mid-line */
#define bes_pin(fmt, ...) printk(KERN_ERR "bes2600 %s: " fmt, __func__, ##__VA_ARGS__)

#endif /* BES_LOG_H_INCLUDED */
