KERN_DIR = /lib/modules/$(shell uname -r)/build

ifeq ($(CONFIG_BES2600_KEEP_ALIVE),y)
ccflags-y += -DVENDOR_XM_KEEPALIVE
endif

BES2600_DRV_VERSION := bes2600_0.3.5_2024.0116

# basic function
define string_flag
$(strip $(if $($(1)),-D$(1)=\"$($(1))\"))
endef

ccflags-y += -Werror
ccflags-y += -DCONFIG_BES2600_WLAN_BES
ccflags-y += -DBES_SDIO_RXTX_TOGGLE
ccflags-y += -DBES_SDIO_TX_MULTIPLE_ENABLE
ccflags-y += -DBES_SDIO_RX_MULTIPLE_ENABLE
ccflags-y += -DBES_UNIFIED_PM
ccflags-y += -DBES_SDIO_OPTIMIZED_LEN
ccflags-y += -DBES2600_HOST_TIMESTAMP_DEBUG

# compilation macros setting
ccflags-y += $(call string_flag,BES2600_DRV_VERSION)

# internal feature options
ccflags-y += -DAP_HT_CAP_UPDATE
ccflags-y += -DBES2600_RX_IN_BH
ccflags-y += -DDCW1260_DETECTION_LOGIC

# sdd options
ccflags-y += -DTEST_11B=0
ccflags-y += -DDPD_CALI=0
ccflags-y += -DALI_CONFG=0
ccflags-y += -DCHIP_WIFI_ROM_VER=1
ccflags-y += -DWIFI_OUT_FEM=0
ccflags-y += -DRF_TX_CONTROL_IO=16

# stbc rx option
ccflags-y += -DSTBC_RX_24G=0
ccflags-y += -DSTBC_RX_5G=0
ccflags-y += -DIPV6_FILTERING

# basic files for building module
bes2600-y := \
	fwio.o \
	txrx.o \
	main.o \
	queue.o \
	hwio.o \
	bh.o \
	wsm.o \
	sta.o \
	ap.o \
	scan.o \
	txrx_opt.o \
	bes_chardev.o \
	tx_loop.o \
	bes_fw.o \
	bes_fw_common.o \
	bes2600_factory.o \
	bes2600_sdio.o \
	epta_coex.o \
	epta_request.o

# compilation files select
bes2600-$(CONFIG_BES2600_DEBUGFS)       += debug.o
bes2600-$(CONFIG_BES2600_ITP)           += itp.o
bes2600-$(CONFIG_PM)                    += pm.o
bes2600-$(CONFIG_BES2600_BT)            += bes2600_btusb.o
bes2600-$(CONFIG_BES2600_TESTMODE)      += wifi_testmode_cmd.o
bes2600-$(CONFIG_BES2600_WOWLAN)        += bes_pwr.o

obj-$(CONFIG_BES2600) += bes2600.o

all: modules

modules clean:
	$(MAKE) -C $(KERN_DIR) M=$(shell pwd) $@
