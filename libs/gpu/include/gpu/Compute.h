#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <webgpu/webgpu.h>

#include "Context.h"

namespace gpu
{
    // Owns one WebGPU object and releases it; Traits supplies the release call
    template<typename T, typename Traits>
    class Handle
    {
    public:
        Handle() = default;
        explicit Handle(T handle) : mHandle(handle) {}
        ~Handle()
        {
            reset();
        }

        Handle(const Handle&) = delete;
        Handle& operator=(const Handle&) = delete;
        Handle(Handle&& other) noexcept : mHandle(std::exchange(other.mHandle, nullptr)) {}
        Handle& operator=(Handle&& other) noexcept
        {
            if (this != &other)
            {
                reset(std::exchange(other.mHandle, nullptr));
            }
            return *this;
        }

        void reset(T handle = nullptr)
        {
            if (mHandle != nullptr)
            {
                Traits::release(mHandle);
            }
            mHandle = handle;
        }

        T get() const { return mHandle; }
        explicit operator bool() const { return mHandle != nullptr; }

    private:
        T mHandle = nullptr;
    };

    struct BindGroupTraits
    {
        static void release(WGPUBindGroup handle) { wgpuBindGroupRelease(handle); }
    };
    using BindGroup = Handle<WGPUBindGroup, BindGroupTraits>;

    // A GPU buffer. Sizes round up to a multiple of 4, as WebGPU copies require.
    class Buffer
    {
    public:
        static constexpr WGPUBufferUsage kStorage = WGPUBufferUsage_Storage | WGPUBufferUsage_CopySrc | WGPUBufferUsage_CopyDst;
        static constexpr WGPUBufferUsage kUniform = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;

        Buffer() = default;
        ~Buffer();

        Buffer(const Buffer&) = delete;
        Buffer& operator=(const Buffer&) = delete;
        Buffer(Buffer&& other) noexcept;
        Buffer& operator=(Buffer&& other) noexcept;

        bool create(const Context& context, std::uint64_t size, WGPUBufferUsage usage, const char* label = nullptr);
        void release();

        // Queued: happens before any work submitted after this call
        void write(const Context& context, std::uint64_t offset, const void* pData, std::uint64_t size);

        WGPUBuffer get() const { return mBuffer; }
        std::uint64_t getSize() const { return mSize; }

    private:
        WGPUBuffer mBuffer = nullptr;
        std::uint64_t mSize = 0;
    };

    // Compiled WGSL; one module can hold many compute entry points
    class ShaderModule
    {
    public:
        ShaderModule() = default;
        ~ShaderModule();

        ShaderModule(const ShaderModule&) = delete;
        ShaderModule& operator=(const ShaderModule&) = delete;

        bool load(const Context& context, const std::string& wgsl, const char* label = nullptr);
        WGPUShaderModule get() const { return mModule; }

    private:
        WGPUShaderModule mModule = nullptr;
    };

    struct BufferBinding
    {
        std::uint32_t binding = 0;
        const Buffer* pBuffer = nullptr;
        std::uint64_t offset = 0;
        std::uint64_t size = WGPU_WHOLE_SIZE;
    };

    // One compute entry point. Its bind group layout comes from the shader, so a bind group must list
    // exactly the bindings that entry point uses.
    class ComputeProgram
    {
    public:
        static constexpr std::uint32_t kWorkgroupSize = 256; // Every entry point must use @workgroup_size(256)

        ComputeProgram() = default;
        ~ComputeProgram();

        ComputeProgram(const ComputeProgram&) = delete;
        ComputeProgram& operator=(const ComputeProgram&) = delete;

        bool create(const Context& context, const ShaderModule& module, const char* entryPoint);
        BindGroup makeBindGroup(const Context& context, std::initializer_list<BufferBinding> bindings) const;
        BindGroup makeBindGroup(const Context& context, const std::vector<BufferBinding>& bindings) const;

        WGPUComputePipeline get() const { return mPipeline; }

    private:
        WGPUComputePipeline mPipeline = nullptr;
    };

    // Records dispatches and copies, then sends them to the GPU in one submission
    class CommandList
    {
    public:
        explicit CommandList(const Context& context);
        ~CommandList();

        CommandList(const CommandList&) = delete;
        CommandList& operator=(const CommandList&) = delete;

        // Runs enough workgroups to cover threadCount threads, spread over two dimensions past 65535 groups
        void dispatch(const ComputeProgram& program, const BindGroup& bindGroup, std::uint64_t threadCount);
        void copy(const Buffer& source, std::uint64_t sourceOffset, const Buffer& destination, std::uint64_t destinationOffset, std::uint64_t size);
        void submit();

    private:
        void endPass();

        WGPUQueue mQueue = nullptr;
        WGPUCommandEncoder mEncoder = nullptr;
        WGPUComputePassEncoder mPass = nullptr;
    };

    struct ReadRange
    {
        const Buffer* pBuffer = nullptr;
        std::uint64_t offset = 0;
        std::uint64_t size = 0;
    };

    // Brings GPU data back to the CPU without stalling: up to two copies in flight, the newest one wins
    class Readback
    {
    public:
        Readback();
        ~Readback();

        Readback(const Readback&) = delete;
        Readback& operator=(const Readback&) = delete;

        // Copies the ranges, back to back, after all work submitted so far. Returns the copy's serial number,
        // or 0 while both copies are busy.
        std::uint64_t request(const Context& context, std::initializer_list<ReadRange> ranges);

        // The newest finished copy, once each; false when nothing new has arrived since the last call
        bool take(std::vector<std::uint8_t>& out, std::uint64_t* pSerial = nullptr);

        bool isPending() const;

    private:
        struct Slot;
        static void onMapped(WGPUMapAsyncStatus status, WGPUStringView message, void* pHolder, void*);

        std::shared_ptr<Slot> mSlots[2];
        std::uint64_t mNextSerial = 1;
        std::uint64_t mLastTakenSerial = 0;
    };
}
