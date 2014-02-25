#
# Copyright (C) 2007-2011 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=rpcapd
PKG_REV:=ea593f0ec4540f861882b151dccd9f8cfbf8b201
PKG_VERSION:=ea593f0ec4
PKG_RELEASE:=1

PKG_SOURCE_PROTO:=git
PKG_SOURCE_VERSION:=$(PKG_REV)
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_URL:=git://github.com/frgtn/rpcapd-linux.git
PKG_SOURCE:=$(PKG_SOURCE_SUBDIR).tar.gz

include $(INCLUDE_DIR)/package.mk

define Package/rpcapd
  SECTION:=net
  CATEGORY:=Network
  DEPENDS:=+libpthread
  TITLE:=rpcapd
endef

define Package/rpcapd/description
    remote packet capture daemon
endef

CONFIGURE_ARGS += \
	--enable-static \
	--disable-yydebug \
	--with-build-cc="$(HOSTCC)" \
	--with-pcap=linux \
	--without-septel \
	--without-dag \
	--without-libnl \
	--disable-can
MAKE_FLAGS += \
	
	
MAKE_FLAGS += \
	CCOPT="$(TARGET_CFLAGS) -I$(BUILD_DIR)/linux/include" \
	LDFLAGS="$(TARGET_LDFLAGS)" \
	OSNAME=Linux \
	prefix="/usr" \
	unstable="true" \
	CFLAGS="-pthread -D_DEBUG -g -Wall -DHAVE_REMOTE -DHAVE_SNPRINTF"

define Build/Configure
# 	(cd $(PKG_BUILD_DIR)/libpcap; ./configure );
	$(call Build/Configure/Default,,,libpcap)
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR)/libpcap
	$(MAKE) -C $(PKG_BUILD_DIR) $(MAKE_FLAGS)
endef

define Package/rpcapd/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/rpcapd $(1)/usr/bin/rpcapd
endef

$(eval $(call BuildPackage,rpcapd))
