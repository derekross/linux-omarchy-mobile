# SPDX-License-Identifier: GPL-2.0
#
# Google's CPIF for the S5300 on gs201 (google-modules/radio/samsung/s5300),
# stage 1: boot, IPC and RFS over the PCIe link, legacy buffers only.

ccflags-y += -I$(src) -I$(src)/gs-include
ccflags-y += -DCPIF_WAKEPKT_SET_MARK=0x80000000
ccflags-y += -DCONFIG_OPTION_REGION=\"\"

obj-$(CONFIG_SHM_IPC) += shm_ipc.o

obj-$(CONFIG_EXYNOS_MODEM_IF) += cpif.o
cpif-y += modem_main.o modem_variation.o
cpif-y += modem_io_device.o net_io_device.o bootdump_io_device.o ipc_io_device.o modem_toe_device.o
cpif-y += modem_utils.o modem_dump.o
cpif-y += link_device.o link_device_memory_flow_control.o
cpif-y += link_device_memory_debug.o
cpif-y += link_device_memory_snapshot.o link_device_memory_legacy.o
cpif-$(CONFIG_LINK_DEVICE_PCIE) += s51xx_pcie.o pcie-gs201-compat.o
cpif-$(CONFIG_SEC_MODEM_S5100) += modem_ctrl_s5100.o modem_ctrl.o
