#ifndef BES_LOG_H_INCLUDED
#define BES_LOG_H_INCLUDED

#include <linux/device.h>
#include <linux/printk.h>

extern struct device *global_dev;

#ifdef CONFIG_BES2600_ENABLE_DEVEL_LOGS
#define bes_devel(fmt, ...) \
	do { \
		if (global_dev) \
			dev_dbg(global_dev, fmt, ##__VA_ARGS__); \
		else \
			pr_debug("bes2600: " fmt, ##__VA_ARGS__); \
	} while (0)
#else
#define bes_devel(fmt, ...) no_printk(fmt, ##__VA_ARGS__)
#endif
#define bes_info(fmt, ...) \
	do { \
		if (global_dev) \
			dev_info(global_dev, fmt, ##__VA_ARGS__); \
		else \
			pr_info("bes2600: " fmt, ##__VA_ARGS__); \
	} while (0)
#define bes_warn(fmt, ...) \
	do { \
		if (global_dev) \
			dev_warn(global_dev, fmt, ##__VA_ARGS__); \
		else \
			pr_warn("bes2600: " fmt, ##__VA_ARGS__); \
	} while (0)
#define bes_err(fmt, ...) \
	do { \
		if (global_dev) \
			dev_err(global_dev, fmt, ##__VA_ARGS__); \
		else \
			pr_err("bes2600: " fmt, ##__VA_ARGS__); \
	} while (0)

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
