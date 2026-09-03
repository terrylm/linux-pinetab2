# compilation macros setting
ccflags-y += -Werror
ccflags-y += -DBES2600_DRV_VERSION=\"bes2600_0.3.5_2024.0116\"
# sdd options
ccflags-y += -DCHIP_WIFI_ROM_VER=1
ccflags-y += -DRF_TX_CONTROL_IO=16
# stbc rx option
ccflags-y += -DSTBC_RX_24G=0
ccflags-y += -DSTBC_RX_5G=0

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
bes2600-$(CONFIG_BES2600_DEBUGFS)	+= debug.o
bes2600-$(CONFIG_BES2600_ITP)		+= itp.o
bes2600-$(CONFIG_PM)			+= pm.o
bes2600-$(CONFIG_BES2600_BT)		+= bes2600_btusb.o
bes2600-$(CONFIG_BES2600_TESTMODE)	+= wifi_testmode_cmd.o
bes2600-$(CONFIG_BES2600_WOWLAN)	+= bes_pwr.o

obj-$(CONFIG_BES2600) += bes2600.o
