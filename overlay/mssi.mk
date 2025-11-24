#
# Copyright (C) 2026 The LineageOS Project
#
# SPDX-License-Identifier: Apache-2.0
#

MTK_NON_5G_PLATFORMS := mt6761 mt6765 mt6768 mt6779 mt6781 mt6785 mt6789
MTK_NON_MULTISTA_PLATFORMS := mt6761 mt6765 mt6768 mt6781 mt6785 mt6789 mt6833 mt6835
MTK_NON_6GHZ_PLATFORMS := $(MTK_NON_MULTISTA_PLATFORMS) mt6779 mt6853 mt6873 mt6875 mt6877 mt6883 mt6885 mt6889 mt6891 mt6893

PRODUCT_PACKAGES += \
    MssiFrameworkOverlay \
	MssiNetworkStackOverlay \
	MssiWifiOverlay

ifeq ($(ENABLE_VENDOR_RIL_SERVICE), true)
PRODUCT_PACKAGES += \
    MssiFrameworkTelephonyOverlay \
	MssiTelephonyOverlay

ifeq (,$(filter $(TARGET_BOARD_PLATFORM),$(MTK_NON_5G_PLATFORMS)))
PRODUCT_PACKAGES += \
    MssiFrameworkTelephony5gOverlay
endif
endif

ifeq (,$(filter $(TARGET_BOARD_PLATFORM),$(MTK_NON_MULTISTA_PLATFORMS)))
PRODUCT_PACKAGES += \
    MssiWifiMultiStaOverlay

ifeq (,$(filter $(TARGET_BOARD_PLATFORM),$(MTK_NON_6GHZ_PLATFORMS)))
PRODUCT_PACKAGES += \
    MssiWifi6gOverlay
endif
endif
