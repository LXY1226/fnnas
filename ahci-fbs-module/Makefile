# SPDX-License-Identifier: GPL-2.0-or-later
# Makefile for ahci_dwc_fbs out-of-tree module
#
# Build inside chroot (native ARM64):
#   make -C /usr/src/linux-headers-<ver> M=$(pwd) modules
#
# The local copy of ahci.h (drivers/ata/ahci.h) is not shipped
# in the kernel headers package; we carry our own here.

obj-m := ahci_dwc_fbs.o

# Put src dir first so our local ahci.h shadows any stale copy
ccflags-y += -I$(src)
