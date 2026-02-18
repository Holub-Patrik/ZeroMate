/*
 * An implementation of "lockless" thread-safe queue
 *
 * Thread safety is guaranteed only when certain conditions are met:
 * - There are only 2 threads accessing the queue
 * - Each thread has a clear role, and never switch during the run of the
 * program
 * - Basically it is meant for Single Producer Single Consumer scenarios
 *
 * The conditions need to be met, since the implementation expects that:
 * - only the writer can advance the write position
 * - only the reader can advance the read position
 *
 * The implementation uses std::array as backing type for circular buffers
 * The buffers will be aligned to a power of 2 to allow for faster math
 * - Instead of modulo, bit masking is used. The same is done in linux kernel
 *
 * The classes are generic thanks to templates
 * Within templates:
 * - Size specifies the backing array size
 * - Type specifies the element type
 */

#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#define ALIGNMENT 64

// align array size to size of 2 and use &
consteval std::size_t align_array_size(const std::size_t size)
{
    const auto left_most_index = (sizeof(size) * 8) - std::countl_zero(size);
    const auto ret_val = 1U << (left_most_index - 1);
    return ret_val;
}

namespace CB
{

    template<typename Type, std::size_t Size>
    struct Buffer
    {
        std::array<Type, align_array_size(Size)> data{};
        // so the atomics don't suffer from false sharing
        alignas(ALIGNMENT) std::atomic<std::uint64_t> read_pos{ 0 };
        alignas(ALIGNMENT) std::atomic<std::uint64_t> write_pos{ 1 };

        // doesn't advance position, only returns the position as if it was advanced
        [[nodiscard]] std::uint64_t advanced_pos(const std::uint64_t cur_pos) const noexcept
        {
            return (cur_pos + 1) & (align_array_size(Size) - 1);
        }
    };

    template<typename Type, std::size_t Size>
    class Reader final
    {
    private:
        Buffer<Type, align_array_size(Size)>& buffer;
        std::uint64_t cached_write_pos;

    public:
        Reader() = delete;
        ~Reader() = default;

        explicit Reader(Buffer<Type, align_array_size(Size)>& buf)
        : buffer(buf)
        , cached_write_pos(buffer.write_pos.load(std::memory_order_relaxed))
        {
        }

        Reader(const Reader<Type, Size>& other) = delete;
        Reader& operator=(const Reader<Type, Size>& other) = delete;

        Reader(Reader<Type, Size>&& other) = delete;
        Reader& operator=(Reader<Type, Size>&& other) = delete;

        const Type& peek() const
        {
            return buffer.data[buffer.read_pos.load(std::memory_order_relaxed)];
        }

        bool try_advance() noexcept
        {
            const auto current_read = buffer.read_pos.load(std::memory_order_relaxed);
            const auto next_read = buffer.advanced_pos(current_read);

            if (next_read == cached_write_pos)
            {
                cached_write_pos = buffer.write_pos.load(std::memory_order_acquire);
                if (next_read == cached_write_pos)
                {
                    return false;
                }
            }

            return true;
        }

        bool advance() noexcept
        {
            const auto current_read = buffer.read_pos.load(std::memory_order_relaxed);
            const auto next_read = buffer.advanced_pos(current_read);

            if (next_read == cached_write_pos)
            {
                cached_write_pos = buffer.write_pos.load(std::memory_order_acquire);
                if (next_read == cached_write_pos)
                {
                    return false;
                }
            }

            buffer.read_pos.store(next_read, std::memory_order_release);
            return true;
        }
    };

    template<typename Type, std::size_t Size>
    class Writer final
    {
    private:
        Buffer<Type, align_array_size(Size)>& buffer;
        std::uint64_t cached_read_pos;

    public:
        Writer() = delete;
        ~Writer() = default;

        explicit Writer(Buffer<Type, align_array_size(Size)>& buf)
        : buffer(buf)
        , cached_read_pos(buffer.read_pos.load(std::memory_order_relaxed))
        {
        }

        Writer(const Writer<Type, Size>& other) = delete;
        Writer& operator=(const Writer<Type, Size>& other) = delete;

        Writer(Writer<Type, Size>&& other) = delete;
        Writer& operator=(Writer<Type, Size>&& other) = delete;

        bool insert(const Type& item) noexcept
        {
            const auto current_write = buffer.write_pos.load(std::memory_order_relaxed);

            if (current_write == cached_read_pos)
            {
                cached_read_pos = buffer.read_pos.load(std::memory_order_acquire);
                if (current_write == cached_read_pos)
                {
                    return false;
                }
            }

            buffer.data[buffer.write_pos] = item;
            const auto next_write = buffer.advanced_pos(current_write);
            buffer.write_pos.store(next_write, std::memory_order_release);
            return true;
        }

        bool try_insert() noexcept
        {
            const auto current_write = buffer.write_pos.load(std::memory_order_relaxed);

            if (current_write == cached_read_pos)
            {
                cached_read_pos = buffer.read_pos.load(std::memory_order_acquire);
                if (current_write == cached_read_pos)
                {
                    return false;
                }
            }

            return true;
        }
    };
} // namespace CB
