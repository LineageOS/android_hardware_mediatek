LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := android.hardware.power-service-mediatek
LOCAL_VENDOR_MODULE := true
LOCAL_MULTILIB := 64
LOCAL_VINTF_FRAGMENTS := power-mtk.xml
LOCAL_SRC_FILES := Power.cpp

LOCAL_SHARED_LIBRARIES := \
    libbase \
    libbinder_ndk \
    android.hardware.power-V2-ndk

ifneq ($(TARGET_TAP_TO_WAKE_NODE),)
    LOCAL_CFLAGS += -DTAP_TO_WAKE_NODE=\"$(TARGET_TAP_TO_WAKE_NODE)\"
endif

ifneq ($(TARGET_POWERHAL_MODE_EXT),)
    LOCAL_CFLAGS += -DMODE_EXT
    LOCAL_SRC_FILES += ../../../../$(TARGET_POWERHAL_MODE_EXT)
endif

include $(BUILD_SHARED_LIBRARY)
