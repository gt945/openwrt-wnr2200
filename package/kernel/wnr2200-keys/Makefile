#
# Copyright (C) 2008-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=wnr2200-keys
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/wnr2200-keys
  SUBMENU:=Other modules
  TITLE:=WNR2200 GPIO Button driver
  FILES:=$(PKG_BUILD_DIR)/wnr2200-keys.ko
  #AUTOLOAD:=$(call AutoProbe,wnr2200-keys)
  KCONFIG:=
endef

define KernelPackage/button-hotplug/description
  Kernel module for WNR2200 button driver
endef

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)"

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,wnr2200-keys))
