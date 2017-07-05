LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := testbox2d02_shared

LOCAL_MODULE_FILENAME := libtestbox2d02

LOCAL_SRC_FILES := main.cpp \
                   ../../Classes/AppDelegate.cpp \
                   ../../Classes/GLES-Render.cpp \
                   ../../Classes/VisibleRect.cpp \
                   ../../Classes/Mybox2dWorld.cpp \
                   ../../Classes/HelloWorldScene.cpp

LOCAL_C_INCLUDES := $(LOCAL_PATH)/../../Classes \
                    $(LOCAL_PATH)/../../../../component/cocos2d-x-3.15/extensions \
                    $(LOCAL_PATH)/../../../../component/cocos2d-x-3.15/external \
                    $(LOCAL_PATH)/../../../../component/cocos2d-x-3.15 \
                    $(LOCAL_PATH)/../../../../component/cocos2d-x-3.15/cocos/editor-support

LOCAL_STATIC_LIBRARIES := cocos2dx_static
LOCAL_STATIC_LIBRARIES += cocos2dx_static

include $(BUILD_SHARED_LIBRARY)

$(call import-module,.)
