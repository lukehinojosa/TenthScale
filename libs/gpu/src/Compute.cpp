#include "gpu/Compute.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace gpu
{
    namespace
    {
        constexpr std::uint32_t MAX_GROUPS_PER_DIMENSION = 65535;

        WGPUStringView view(const char* text)
        {
            return WGPUStringView{ text, text != nullptr ? WGPU_STRLEN : 0 };
        }

        std::uint64_t roundUp4(std::uint64_t size)
        {
            return (size + 3) & ~std::uint64_t(3);
        }
    }

    Buffer::~Buffer()
    {
        release();
    }

    Buffer::Buffer(Buffer&& other) noexcept
        : mBuffer(std::exchange(other.mBuffer, nullptr)), mSize(std::exchange(other.mSize, 0))
    {
    }

    Buffer& Buffer::operator=(Buffer&& other) noexcept
    {
        if (this != &other)
        {
            release();
            mBuffer = std::exchange(other.mBuffer, nullptr);
            mSize = std::exchange(other.mSize, 0);
        }
        return *this;
    }

    bool Buffer::create(const Context& context, std::uint64_t size, WGPUBufferUsage usage, const char* label)
    {
        release();
        if (!context.isReady())
        {
            return false;
        }
        WGPUBufferDescriptor descriptor = WGPU_BUFFER_DESCRIPTOR_INIT;
        descriptor.label = view(label);
        descriptor.size = roundUp4(std::max<std::uint64_t>(size, 4));
        descriptor.usage = usage;
        mBuffer = wgpuDeviceCreateBuffer(context.getDevice(), &descriptor);
        mSize = mBuffer != nullptr ? descriptor.size : 0;
        return mBuffer != nullptr;
    }

    void Buffer::release()
    {
        if (mBuffer != nullptr)
        {
            wgpuBufferDestroy(mBuffer);
            wgpuBufferRelease(mBuffer);
            mBuffer = nullptr;
        }
        mSize = 0;
    }

    void Buffer::write(const Context& context, std::uint64_t offset, const void* pData, std::uint64_t size)
    {
        if (mBuffer == nullptr || size == 0)
        {
            return;
        }
        wgpuQueueWriteBuffer(context.getQueue(), mBuffer, offset, pData, (size_t)roundUp4(size));
    }

    ShaderModule::~ShaderModule()
    {
        if (mModule != nullptr)
        {
            wgpuShaderModuleRelease(mModule);
        }
    }

    bool ShaderModule::load(const Context& context, const std::string& wgsl, const char* label)
    {
        if (mModule != nullptr)
        {
            wgpuShaderModuleRelease(mModule);
            mModule = nullptr;
        }
        if (!context.isReady())
        {
            return false;
        }
        WGPUShaderSourceWGSL source = WGPU_SHADER_SOURCE_WGSL_INIT;
        source.code = WGPUStringView{ wgsl.data(), wgsl.size() };
        WGPUShaderModuleDescriptor descriptor = WGPU_SHADER_MODULE_DESCRIPTOR_INIT;
        descriptor.nextInChain = &source.chain;
        descriptor.label = view(label);
        mModule = wgpuDeviceCreateShaderModule(context.getDevice(), &descriptor);
        return mModule != nullptr;
    }

    ComputeProgram::~ComputeProgram()
    {
        if (mPipeline != nullptr)
        {
            wgpuComputePipelineRelease(mPipeline);
        }
    }

    bool ComputeProgram::create(const Context& context, const ShaderModule& module, const char* entryPoint)
    {
        if (mPipeline != nullptr)
        {
            wgpuComputePipelineRelease(mPipeline);
            mPipeline = nullptr;
        }
        if (!context.isReady() || module.get() == nullptr)
        {
            return false;
        }
        WGPUComputePipelineDescriptor descriptor = WGPU_COMPUTE_PIPELINE_DESCRIPTOR_INIT;
        descriptor.label = view(entryPoint);
        descriptor.compute.module = module.get();
        descriptor.compute.entryPoint = view(entryPoint);
        mPipeline = wgpuDeviceCreateComputePipeline(context.getDevice(), &descriptor);
        return mPipeline != nullptr;
    }

    BindGroup ComputeProgram::makeBindGroup(const Context& context, std::initializer_list<BufferBinding> bindings) const
    {
        return makeBindGroup(context, std::vector<BufferBinding>(bindings));
    }

    BindGroup ComputeProgram::makeBindGroup(const Context& context, const std::vector<BufferBinding>& bindings) const
    {
        if (mPipeline == nullptr)
        {
            return BindGroup();
        }

        std::vector<WGPUBindGroupEntry> entries;
        entries.reserve(bindings.size());
        for (const BufferBinding& binding : bindings)
        {
            if (binding.pBuffer == nullptr || binding.pBuffer->get() == nullptr)
            {
                return BindGroup();
            }
            WGPUBindGroupEntry entry = WGPU_BIND_GROUP_ENTRY_INIT;
            entry.binding = binding.binding;
            entry.buffer = binding.pBuffer->get();
            entry.offset = binding.offset;
            entry.size = binding.size;
            entries.push_back(entry);
        }

        WGPUBindGroupLayout layout = wgpuComputePipelineGetBindGroupLayout(mPipeline, 0);
        WGPUBindGroupDescriptor descriptor = WGPU_BIND_GROUP_DESCRIPTOR_INIT;
        descriptor.layout = layout;
        descriptor.entryCount = entries.size();
        descriptor.entries = entries.data();
        WGPUBindGroup group = wgpuDeviceCreateBindGroup(context.getDevice(), &descriptor);
        wgpuBindGroupLayoutRelease(layout);
        return BindGroup(group);
    }

    CommandList::CommandList(const Context& context) : mQueue(context.getQueue())
    {
        if (context.isReady())
        {
            WGPUCommandEncoderDescriptor descriptor = WGPU_COMMAND_ENCODER_DESCRIPTOR_INIT;
            mEncoder = wgpuDeviceCreateCommandEncoder(context.getDevice(), &descriptor);
        }
    }

    CommandList::~CommandList()
    {
        endPass();
        if (mEncoder != nullptr)
        {
            wgpuCommandEncoderRelease(mEncoder);
        }
    }

    void CommandList::dispatch(const ComputeProgram& program, const BindGroup& bindGroup, std::uint64_t threadCount)
    {
        if (mEncoder == nullptr || program.get() == nullptr || !bindGroup || threadCount == 0)
        {
            return;
        }
        if (mPass == nullptr)
        {
            WGPUComputePassDescriptor descriptor = WGPU_COMPUTE_PASS_DESCRIPTOR_INIT;
            mPass = wgpuCommandEncoderBeginComputePass(mEncoder, &descriptor);
        }

        std::uint64_t groups = (threadCount + ComputeProgram::kWorkgroupSize - 1) / ComputeProgram::kWorkgroupSize;
        std::uint32_t groupsX = (std::uint32_t)std::min<std::uint64_t>(groups, MAX_GROUPS_PER_DIMENSION);
        std::uint32_t groupsY = (std::uint32_t)((groups + groupsX - 1) / groupsX);

        wgpuComputePassEncoderSetPipeline(mPass, program.get());
        wgpuComputePassEncoderSetBindGroup(mPass, 0, bindGroup.get(), 0, nullptr);
        wgpuComputePassEncoderDispatchWorkgroups(mPass, groupsX, groupsY, 1);
    }

    void CommandList::copy(const Buffer& source, std::uint64_t sourceOffset, const Buffer& destination, std::uint64_t destinationOffset, std::uint64_t size)
    {
        if (mEncoder == nullptr || source.get() == nullptr || destination.get() == nullptr || size == 0)
        {
            return;
        }
        endPass();
        wgpuCommandEncoderCopyBufferToBuffer(mEncoder, source.get(), sourceOffset, destination.get(), destinationOffset, roundUp4(size));
    }

    void CommandList::submit()
    {
        if (mEncoder == nullptr)
        {
            return;
        }
        endPass();
        WGPUCommandBufferDescriptor descriptor = WGPU_COMMAND_BUFFER_DESCRIPTOR_INIT;
        WGPUCommandBuffer commands = wgpuCommandEncoderFinish(mEncoder, &descriptor);
        wgpuQueueSubmit(mQueue, 1, &commands);
        wgpuCommandBufferRelease(commands);
        wgpuCommandEncoderRelease(mEncoder);
        mEncoder = nullptr;
    }

    void CommandList::endPass()
    {
        if (mPass != nullptr)
        {
            wgpuComputePassEncoderEnd(mPass);
            wgpuComputePassEncoderRelease(mPass);
            mPass = nullptr;
        }
    }

    // Shared with the map callback, which can arrive after the Readback is gone
    struct Readback::Slot
    {
        enum class State
        {
            FREE,
            MAPPING,
            READY
        };

        ~Slot()
        {
            if (buffer != nullptr)
            {
                wgpuBufferRelease(buffer);
            }
        }

        WGPUBuffer buffer = nullptr;
        std::uint64_t capacity = 0;
        std::uint64_t size = 0;
        std::uint64_t serial = 0;
        State state = State::FREE;
    };

    Readback::Readback()
    {
        mSlots[0] = std::make_shared<Slot>();
        mSlots[1] = std::make_shared<Slot>();
    }

    Readback::~Readback() = default;

    std::uint64_t Readback::request(const Context& context, std::initializer_list<ReadRange> ranges)
    {
        if (!context.isReady())
        {
            return 0;
        }

        std::shared_ptr<Slot> pSlot;
        for (auto& pCandidate : mSlots)
        {
            if (pCandidate->state == Slot::State::FREE)
            {
                pSlot = pCandidate;
                break;
            }
        }
        if (pSlot == nullptr)
        {
            return 0;
        }

        std::uint64_t total = 0;
        for (const ReadRange& range : ranges)
        {
            total += roundUp4(range.size);
        }
        if (total == 0)
        {
            return 0;
        }

        if (pSlot->capacity < total)
        {
            if (pSlot->buffer != nullptr)
            {
                wgpuBufferRelease(pSlot->buffer);
            }
            WGPUBufferDescriptor descriptor = WGPU_BUFFER_DESCRIPTOR_INIT;
            descriptor.label = WGPUStringView{ "gpu::Readback", WGPU_STRLEN };
            descriptor.size = total;
            descriptor.usage = WGPUBufferUsage_MapRead | WGPUBufferUsage_CopyDst;
            pSlot->buffer = wgpuDeviceCreateBuffer(context.getDevice(), &descriptor);
            pSlot->capacity = pSlot->buffer != nullptr ? total : 0;
            if (pSlot->buffer == nullptr)
            {
                return 0;
            }
        }

        WGPUCommandEncoderDescriptor encoderDescriptor = WGPU_COMMAND_ENCODER_DESCRIPTOR_INIT;
        WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(context.getDevice(), &encoderDescriptor);
        std::uint64_t offset = 0;
        for (const ReadRange& range : ranges)
        {
            if (range.pBuffer != nullptr && range.pBuffer->get() != nullptr && range.size > 0)
            {
                wgpuCommandEncoderCopyBufferToBuffer(encoder, range.pBuffer->get(), range.offset, pSlot->buffer, offset, roundUp4(range.size));
            }
            offset += roundUp4(range.size);
        }
        WGPUCommandBufferDescriptor commandDescriptor = WGPU_COMMAND_BUFFER_DESCRIPTOR_INIT;
        WGPUCommandBuffer commands = wgpuCommandEncoderFinish(encoder, &commandDescriptor);
        wgpuQueueSubmit(context.getQueue(), 1, &commands);
        wgpuCommandBufferRelease(commands);
        wgpuCommandEncoderRelease(encoder);

        pSlot->size = total;
        pSlot->serial = mNextSerial++;
        pSlot->state = Slot::State::MAPPING;

        WGPUBufferMapCallbackInfo callback = WGPU_BUFFER_MAP_CALLBACK_INFO_INIT;
        callback.mode = WGPUCallbackMode_AllowProcessEvents;
        callback.callback = &Readback::onMapped;
        callback.userdata1 = new std::shared_ptr<Slot>(pSlot);
        wgpuBufferMapAsync(pSlot->buffer, WGPUMapMode_Read, 0, (size_t)total, callback);
        return pSlot->serial;
    }

    void Readback::onMapped(WGPUMapAsyncStatus status, WGPUStringView message, void* pHolder, void*)
    {
        auto* pShared = static_cast<std::shared_ptr<Slot>*>(pHolder);
        Slot& slot = **pShared;
        if (status == WGPUMapAsyncStatus_Success)
        {
            slot.state = Slot::State::READY;
        }
        else
        {
            slot.state = Slot::State::FREE;
            if (status != WGPUMapAsyncStatus_Aborted && status != WGPUMapAsyncStatus_CallbackCancelled)
            {
                std::fprintf(stderr, "gpu: readback failed: %s\n", toString(message).c_str());
            }
        }
        delete pShared;
    }

    bool Readback::take(std::vector<std::uint8_t>& out, std::uint64_t* pSerial)
    {
        Slot* pNewest = nullptr;
        for (auto& pSlot : mSlots)
        {
            if (pSlot->state != Slot::State::READY)
            {
                continue;
            }
            if (pSlot->serial <= mLastTakenSerial)
            {
                // Finished after a newer copy was already taken
                wgpuBufferUnmap(pSlot->buffer);
                pSlot->state = Slot::State::FREE;
            }
            else if (pNewest == nullptr || pSlot->serial > pNewest->serial)
            {
                pNewest = pSlot.get();
            }
        }
        if (pNewest == nullptr)
        {
            return false;
        }
        mLastTakenSerial = pNewest->serial;
        if (pSerial != nullptr)
        {
            *pSerial = pNewest->serial;
        }

        const void* pData = wgpuBufferGetConstMappedRange(pNewest->buffer, 0, (size_t)pNewest->size);
        if (pData != nullptr)
        {
            out.resize((size_t)pNewest->size);
            std::memcpy(out.data(), pData, (size_t)pNewest->size);
        }

        // An older finished copy is out of date now, so free it too
        for (auto& pSlot : mSlots)
        {
            if (pSlot->state == Slot::State::READY)
            {
                wgpuBufferUnmap(pSlot->buffer);
                pSlot->state = Slot::State::FREE;
            }
        }
        return pData != nullptr;
    }

    bool Readback::isPending() const
    {
        return mSlots[0]->state != Slot::State::FREE || mSlots[1]->state != Slot::State::FREE;
    }
}
