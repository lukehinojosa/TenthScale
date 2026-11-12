#pragma once

#include <string>

#include <webgpu/webgpu.h>

namespace gpu
{
    enum class ContextState
    {
        IDLE,
        STARTING,
        READY,
        UNAVAILABLE
    };

    // The WebGPU instance, adapter, and device. Starting is asynchronous: call poll() every frame until it settles.
    class Context
    {
    public:
        Context() = default;
        ~Context();

        Context(const Context&) = delete;
        Context& operator=(const Context&) = delete;

        void start();

        // Runs WebGPU's pending callbacks: device startup, finished readbacks, errors. Call once per frame.
        void poll();

        ContextState getState() const { return mState; }
        bool isReady() const { return mState == ContextState::READY; }

        // The adapter's name when ready, or why WebGPU can't be used
        const std::string& getStatus() const { return mStatus; }

        WGPUInstance getInstance() const { return mInstance; }
        WGPUDevice getDevice() const { return mDevice; }
        WGPUQueue getQueue() const { return mQueue; }

    private:
        static void onAdapter(WGPURequestAdapterStatus status, WGPUAdapter adapter, WGPUStringView message, void* pContext, void*);
        static void onDevice(WGPURequestDeviceStatus status, WGPUDevice device, WGPUStringView message, void* pContext, void*);
        static void onDeviceLost(const WGPUDevice* pDevice, WGPUDeviceLostReason reason, WGPUStringView message, void* pContext, void*);
        static void onError(const WGPUDevice* pDevice, WGPUErrorType type, WGPUStringView message, void*, void*);

        void fail(const std::string& reason);

        WGPUInstance mInstance = nullptr;
        WGPUAdapter mAdapter = nullptr;
        WGPUDevice mDevice = nullptr;
        WGPUQueue mQueue = nullptr;
        ContextState mState = ContextState::IDLE;
        std::string mStatus = "Not started";
        bool mClosing = false;
    };

    std::string toString(WGPUStringView text);
}
