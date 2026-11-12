#include "gpu/Context.h"

#include <cstdio>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>

// emdawnwebgpu throws instead of failing when the browser has no WebGPU at all
EM_JS(int, jsHasWebGpu, (), {
    return (typeof navigator !== 'undefined' && navigator.gpu) ? 1 : 0;
});
#else
#include <webgpu/wgpu.h>
#endif

namespace gpu
{
    std::string toString(WGPUStringView text)
    {
        if (text.data == nullptr)
        {
            return {};
        }
        if (text.length == WGPU_STRLEN)
        {
            return std::string(text.data);
        }
        return std::string(text.data, text.length);
    }

    Context::~Context()
    {
        mClosing = true;
        if (mQueue != nullptr)
        {
            wgpuQueueRelease(mQueue);
        }
        if (mDevice != nullptr)
        {
            wgpuDeviceRelease(mDevice);
        }
        if (mAdapter != nullptr)
        {
            wgpuAdapterRelease(mAdapter);
        }
        if (mInstance != nullptr)
        {
            // Lets callbacks for the released objects finish while this object still exists
            wgpuInstanceProcessEvents(mInstance);
            wgpuInstanceRelease(mInstance);
        }
    }

    void Context::start()
    {
        if (mState != ContextState::IDLE)
        {
            return;
        }

#ifdef __EMSCRIPTEN__
        if (!jsHasWebGpu())
        {
            fail("This browser has no WebGPU");
            return;
        }
        mInstance = wgpuCreateInstance(nullptr);
#else
        // No OpenGL backend: it would switch the current GL context out from under the renderer
        WGPUInstanceExtras extras = {};
        extras.chain.sType = (WGPUSType)WGPUSType_InstanceExtras;
        extras.backends = WGPUInstanceBackend_Vulkan | WGPUInstanceBackend_Metal | WGPUInstanceBackend_DX12;
        WGPUInstanceDescriptor descriptor = WGPU_INSTANCE_DESCRIPTOR_INIT;
        descriptor.nextInChain = &extras.chain;
        mInstance = wgpuCreateInstance(&descriptor);
#endif
        if (mInstance == nullptr)
        {
            fail("WebGPU is not available");
            return;
        }

        mState = ContextState::STARTING;
        mStatus = "Starting";

        WGPURequestAdapterOptions options = WGPU_REQUEST_ADAPTER_OPTIONS_INIT;
        options.powerPreference = WGPUPowerPreference_HighPerformance;
        WGPURequestAdapterCallbackInfo callback = WGPU_REQUEST_ADAPTER_CALLBACK_INFO_INIT;
        callback.mode = WGPUCallbackMode_AllowProcessEvents;
        callback.callback = &Context::onAdapter;
        callback.userdata1 = this;
        wgpuInstanceRequestAdapter(mInstance, &options, callback);
    }

    void Context::poll()
    {
        if (mInstance == nullptr)
        {
            return;
        }
        wgpuInstanceProcessEvents(mInstance);
#ifndef __EMSCRIPTEN__
        // wgpu-native finishes buffer mappings when the device is polled
        if (mDevice != nullptr)
        {
            wgpuDevicePoll(mDevice, false, nullptr);
        }
#endif
    }

    void Context::fail(const std::string& reason)
    {
        mState = ContextState::UNAVAILABLE;
        mStatus = reason;
        std::fprintf(stderr, "gpu: %s\n", reason.c_str());
    }

    void Context::onAdapter(WGPURequestAdapterStatus status, WGPUAdapter adapter, WGPUStringView message, void* pContext, void*)
    {
        auto* pSelf = static_cast<Context*>(pContext);
        if (pSelf->mClosing)
        {
            return;
        }
        if (status != WGPURequestAdapterStatus_Success || adapter == nullptr)
        {
            std::string detail = toString(message);
            pSelf->fail(detail.empty() ? "No WebGPU adapter" : "No WebGPU adapter: " + detail);
            return;
        }
        pSelf->mAdapter = adapter;

        WGPUDeviceDescriptor descriptor = WGPU_DEVICE_DESCRIPTOR_INIT;
        descriptor.label = WGPUStringView{ "gpu::Context", WGPU_STRLEN };
        descriptor.uncapturedErrorCallbackInfo.callback = &Context::onError;
        descriptor.deviceLostCallbackInfo.mode = WGPUCallbackMode_AllowProcessEvents;
        descriptor.deviceLostCallbackInfo.callback = &Context::onDeviceLost;
        descriptor.deviceLostCallbackInfo.userdata1 = pSelf;

        WGPURequestDeviceCallbackInfo callback = WGPU_REQUEST_DEVICE_CALLBACK_INFO_INIT;
        callback.mode = WGPUCallbackMode_AllowProcessEvents;
        callback.callback = &Context::onDevice;
        callback.userdata1 = pSelf;
        wgpuAdapterRequestDevice(adapter, &descriptor, callback);
    }

    void Context::onDevice(WGPURequestDeviceStatus status, WGPUDevice device, WGPUStringView message, void* pContext, void*)
    {
        auto* pSelf = static_cast<Context*>(pContext);
        if (pSelf->mClosing)
        {
            return;
        }
        if (status != WGPURequestDeviceStatus_Success || device == nullptr)
        {
            pSelf->fail("Could not create a WebGPU device: " + toString(message));
            return;
        }
        pSelf->mDevice = device;
        pSelf->mQueue = wgpuDeviceGetQueue(device);

        std::string name = "WebGPU";
        WGPUAdapterInfo info = WGPU_ADAPTER_INFO_INIT;
        if (wgpuAdapterGetInfo(pSelf->mAdapter, &info) == WGPUStatus_Success)
        {
            std::string deviceName = toString(info.device);
            std::string description = toString(info.description);
            if (!deviceName.empty())
            {
                name = deviceName;
            }
            else if (!description.empty())
            {
                name = description;
            }
            wgpuAdapterInfoFreeMembers(info);
        }
        pSelf->mStatus = name;
        pSelf->mState = ContextState::READY;
    }

    void Context::onDeviceLost(const WGPUDevice*, WGPUDeviceLostReason reason, WGPUStringView message, void* pContext, void*)
    {
        auto* pSelf = static_cast<Context*>(pContext);
        if (pSelf == nullptr || pSelf->mClosing || reason == WGPUDeviceLostReason_Destroyed)
        {
            return;
        }
        pSelf->fail("WebGPU device lost: " + toString(message));
    }

    void Context::onError(const WGPUDevice*, WGPUErrorType, WGPUStringView message, void*, void*)
    {
        std::fprintf(stderr, "gpu: WebGPU error: %s\n", toString(message).c_str());
    }
}
