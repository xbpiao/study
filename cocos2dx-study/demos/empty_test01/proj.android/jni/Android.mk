LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := empty_test01_shared

LOCAL_MODULE_FILENAME := libempty_test01

LOCAL_SRC_FILES := main.cpp \
                   ../../Classes/AppDelegate.cpp \
                   ../../Classes/HelloWorldScene.cpp

LOCAL_C_INCLUDES := $(LOCAL_PATH)/../../Classes \
                    $(LOCAL_PATH)/../../../../component/cocos2d-x-3.14.1/extensions \
                    $(LOCAL_PATH)/../../../../component/cocos2d-x-3.14.1 \
                    $(LOCAL_PATH)/../../../../component/cocos2d-x-3.14.1/cocos/editor-support

LOCAL_STATIC_LIBRARIES := cocos2dx_static

include $(BUILD_SHARED_LIBRARY)

$(call import-module,.)
