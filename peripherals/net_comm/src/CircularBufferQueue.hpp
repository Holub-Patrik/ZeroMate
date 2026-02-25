/*
 * High Throughput Thread-Safe Primitives Library
 *
 * Implemented Primitives
 *
 * "Lockless" Single Produce Single Consumer Queue
 * - Buffer class representing the data backing for the queue
 * - Writer class representing an interface into data allowing to write into a queue
 * - Reader class representing an interface into data allowing to read from the queue
 *
 * "Exponential" Backoffs
 * - Usefull when spinning while waiting for other primitives
 * - Basic Backoff
 *   - Spins for a certain amount of cycles as fast as it can
 *   - After that turns to spinning with a cpu_relax() instruction
 *
 * - Sleep Backoff
 *   - Spins the same way as a basic backoff
 *   - When it finishes fast and relaxed spinning, it start spinning with a sleep
 *
 * - Semaphore Backoff
 *   - Spins the same way as basic backoff
 *   - When it finishes fast and relaxed spinning, it puts the thread to sleep on binary semaphore
 *   - This backoff needs to be waked from the sleeping state
 *   - Usefull when there are bursts of high throughput and then for example wait for network
 *
 * To be implemented:
 * - Spinlocks
 * - Multiple Producer Multiple Consumer
 * - More os specific optimizations to improve performance
 * - Bulk data access (insert N, take as many as possible)
 *
 * Considering:
 * - More data structures possibly
 */

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#define ALIGNMENT 64

// align array size to size of 2 and use &
consteval std::size_t align_array_size(const std::size_t size)
{
    const auto left_most_index = (sizeof(size) * 8) - std::countl_zero(size);
    const auto ret_val = 1U << (left_most_index - 1);
    return ret_val;
}

inline void cpu_relax()
{
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    #ifdef _MSC_VER
        #include <intrin.h>
    _mm_pause();
    #else
    __builtin_ia32_pause();
    #endif
#elif defined(__aarch64__) || defined(__arm__)
    asm volatile("yield");
#else
    // Fallback compiler barrier to force memory reload
    asm volatile("" ::: "memory");
#endif
}

namespace TSP
{

    namespace BF
    {

        template<class Derived>
        class IBackoff
        {
        private:
            IBackoff() = default;

        public:
            virtual ~IBackoff() = default;

            IBackoff(const IBackoff&) = delete;
            IBackoff& operator=(const IBackoff&) = delete;

            IBackoff(IBackoff&&) = delete;
            IBackoff& operator=(IBackoff&&) = delete;

            void wait() noexcept
            {
                static_cast<Derived*>(this)->wait_impl();
            };

            template<typename Predicate>
            void wait(Predicate&& pred) noexcept
            {
                static_cast<Derived*>(this)->wait_impl(std::forward(pred));
            }

            void reset() noexcept
            {
                static_cast<Derived*>(this)->reset_impl();
            };

            void wake() noexcept
            {
                static_cast<Derived*>(this)->wake_impl();
            }

            friend Derived;
        };

        class Backoff final : public IBackoff<Backoff>
        {
        private:
            const std::uint64_t max_cycles;
            std::uint64_t cycles{ 0 };

        public:
            Backoff() = delete;
            ~Backoff() final = default;

            explicit Backoff(const std::uint64_t max_cycles)
            : max_cycles(max_cycles)
            {
            }

            Backoff(const Backoff& other) = delete;
            Backoff& operator=(const Backoff& other) = delete;

            Backoff(Backoff&& other) = delete;
            Backoff& operator=(Backoff&& other) = delete;

            template<typename Predicate>
            void wait_impl(Predicate&& /*pred*/) noexcept
            {
                wait_impl();
            }

            void wait_impl() noexcept
            {
                if (cycles < max_cycles)
                {
                    cycles++;
                    return;
                }
                cpu_relax();
            }

            void reset_impl() noexcept
            {
                cycles = 0;
            }

            void wake_impl() const noexcept
            {
            }
        };

        template<typename TimeUnit>
        class SleepBackoff final : public IBackoff<SleepBackoff<TimeUnit>>
        {
        private:
            const std::uint64_t max_cycles_fast;
            const std::uint64_t max_cycles_relaxed;

            const TimeUnit wait_time{ std::chrono::microseconds{ 100 } };

            std::uint64_t cycles_fast{ 0 };
            std::uint64_t cycles_relaxed{ 0 };

        public:
            SleepBackoff<TimeUnit>() = delete;
            ~SleepBackoff<TimeUnit>() final = default;

            explicit SleepBackoff<TimeUnit>(const std::uint64_t max_cycles_fast,
                                            const std::uint64_t max_cycles_relaxed,
                                            const TimeUnit& wait_time)
            : max_cycles_fast(max_cycles_fast)
            , max_cycles_relaxed(max_cycles_relaxed)
            , wait_time(wait_time)
            {
            }

            SleepBackoff<TimeUnit>(const SleepBackoff<TimeUnit>& other) = delete;
            SleepBackoff<TimeUnit>& operator=(const SleepBackoff<TimeUnit>& other) = delete;

            SleepBackoff<TimeUnit>(SleepBackoff<TimeUnit>&& other) = delete;
            SleepBackoff<TimeUnit>& operator=(SleepBackoff<TimeUnit>&& other) = delete;

            template<typename Predicate>
            void wait_impl(Predicate&& /*pred*/)
            {
                wait_impl();
            }

            void wait_impl() noexcept
            {
                if (cycles_fast < max_cycles_fast)
                {
                    cycles_fast++;
                    return;
                }

                if (cycles_relaxed < max_cycles_relaxed)
                {
                    cycles_relaxed++;
                    cpu_relax();
                    return;
                }

                std::this_thread::sleep_for(wait_time);
            }

            void reset_impl() noexcept
            {
                cycles_fast = 0;
                cycles_relaxed = 0;
            }

            void wake_impl() const noexcept
            {
            }
        };

        class SemBackoff final : public IBackoff<SemBackoff>
        {
        private:
            const std::uint64_t max_cycles_fast;
            const std::uint64_t max_cycles_relaxed;

            std::uint64_t cycles_fast{ 0 };
            std::uint64_t cycles_relaxed{ 0 };

            std::binary_semaphore sem{ 0 };
            std::atomic<bool> is_sleeping{ false };

        public:
            SemBackoff() = delete;
            ~SemBackoff() final = default;

            SemBackoff(const SemBackoff&) = delete;
            SemBackoff& operator=(const SemBackoff&) = delete;

            SemBackoff(SemBackoff&&) = delete;
            SemBackoff& operator=(SemBackoff&&) = delete;

            explicit SemBackoff(const std::uint64_t max_cycles_fast, const std::uint64_t max_cycles_relaxed)
            : max_cycles_fast(max_cycles_fast)
            , max_cycles_relaxed(max_cycles_relaxed)
            {
            }

            template<typename Predicate>
            void wait_impl(Predicate&& condition) noexcept
            {
                if (cycles_fast < max_cycles_fast)
                {
                    cycles_fast++;
                    return;
                }

                if (cycles_relaxed < max_cycles_relaxed)
                {
                    cycles_relaxed++;
                    cpu_relax();
                    return;
                }

                is_sleeping.store(false, std::memory_order_seq_cst);
                if (condition())
                {
                    is_sleeping.store(false, std::memory_order_relaxed);
                    return;
                }

                sem.acquire();
                is_sleeping.store(false, std::memory_order_relaxed);
            }

            void reset_impl() noexcept
            {
                cycles_fast = 0;
                cycles_relaxed = 0;
                is_sleeping.store(false, std::memory_order_relaxed);
            }

            void wake_impl() noexcept
            {
                if (is_sleeping.load(std::memory_order_seq_cst))
                {
                    sem.release();
                }
            }
        };
    }

    namespace Queue
    {
        template<typename Type, std::size_t Size>
        struct Buffer
        {
            std::array<Type, align_array_size(Size)> data{};
            // so the atomics don't suffer from false sharing
            alignas(ALIGNMENT) std::atomic<std::uint64_t> read_pos{ 0 };
            alignas(ALIGNMENT) std::atomic<std::uint64_t> write_pos{ 1 };

            // doesn't advance position, only returns the position as if it was advanced
            [[nodiscard]] static std::uint64_t advanced_pos(const std::uint64_t cur_pos) noexcept
            {
                return (cur_pos + 1) & (align_array_size(Size) - 1);
            }
        };

        template<typename Type, std::size_t Size>
        class Reader final
        {
        private:
            Buffer<Type, align_array_size(Size)>* buffer;
            std::uint64_t cached_write_pos;

        public:
            Reader()
            : buffer(nullptr)
            , cached_write_pos(0) { };
            ~Reader() = default;

            explicit Reader(Buffer<Type, align_array_size(Size)>& buf)
            : buffer(&buf)
            , cached_write_pos(buffer->write_pos.load(std::memory_order_relaxed))
            {
            }

            Reader(const Reader<Type, Size>& other) = delete;
            Reader& operator=(const Reader<Type, Size>& other) = delete;

            Reader(Reader<Type, Size>&& other) noexcept
            : buffer(other.buffer)
            , cached_write_pos(other.buffer->write_pos.load(std::memory_order_relaxed))
            {
            }
            Reader& operator=(Reader<Type, Size>&& other) noexcept
            {
                std::swap(buffer, other.buffer);
                cached_write_pos = buffer->write_pos.load(std::memory_order_relaxed);
            };

            [[nodiscard]] const Type& peek() const
            {
                return buffer->data[buffer->read_pos.load(std::memory_order_relaxed)];
            }

            template<typename BackoffType>
            [[nodiscard]] const Type& read(BackoffType& backoff) noexcept
            {
                while (!try_advance())
                {
                    backoff.wait([this]() { return this->try_advance(); });
                }
                backoff.reset();

                // advance without extra checks
                const auto ret = peek();
                const auto current_read = buffer->read_pos.load(std::memory_order_relaxed);
                const auto next_read = Buffer<Type, align_array_size(Size)>::advanced_pos(current_read);
                buffer->read_pos.store(next_read, std::memory_order_release);

                return ret;
            }

            bool try_advance() noexcept
            {
                const auto current_read = buffer->read_pos.load(std::memory_order_relaxed);
                const auto next_read = Buffer<Type, align_array_size(Size)>::advanced_pos(current_read);

                if (next_read == cached_write_pos)
                {
                    cached_write_pos = buffer->write_pos.load(std::memory_order_acquire);
                    if (next_read == cached_write_pos)
                    {
                        return false;
                    }
                }

                return true;
            }

            bool advance() noexcept
            {
                const auto current_read = buffer->read_pos.load(std::memory_order_relaxed);
                const auto next_read = Buffer<Type, align_array_size(Size)>::advanced_pos(current_read);

                if (next_read == cached_write_pos)
                {
                    cached_write_pos = buffer->write_pos.load(std::memory_order_acquire);
                    if (next_read == cached_write_pos)
                    {
                        return false;
                    }
                }

                buffer->read_pos.store(next_read, std::memory_order_release);
                return true;
            }
        };

        template<typename Type, std::size_t Size>
        class Writer final
        {
        private:
            Buffer<Type, align_array_size(Size)>* buffer;
            std::uint64_t cached_read_pos;

        public:
            Writer()
            : buffer(nullptr)
            , cached_read_pos(0) { };
            ~Writer() = default;

            explicit Writer(Buffer<Type, align_array_size(Size)>& buf)
            : buffer(buf)
            , cached_read_pos(buffer->read_pos.load(std::memory_order_relaxed))
            {
            }

            Writer(const Writer<Type, Size>& other) = delete;
            Writer& operator=(const Writer<Type, Size>& other) = delete;

            Writer(Writer<Type, Size>&& other) noexcept
            : buffer(other.buffer)
            , cached_read_pos(other.buffer->read_pos.load(std::memory_order_relaxed)) { };
            Writer& operator=(Writer<Type, Size>&& other) noexcept
            {
                std::swap(buffer, other.buffer);
                cached_read_pos = buffer->read_pos.load(std::memory_order_relaxed);
            };

            template<typename Backoff>
            void insert_with_backoff(const Type& item, Backoff& backoff)
            {
                while (!try_insert())
                {
                    backoff.wait([this]() { return this->try_insert(); });
                }
                backoff.reset();

                const auto current_write = buffer->write_pos.load(std::memory_order_relaxed);
                buffer->data[current_write] = item;

                const auto next_write = Buffer<Type, align_array_size(Size)>::advanced_pos(current_write);
                buffer->write_pos.store(next_write, std::memory_order_release);
            }

            bool insert(const Type& item) noexcept
            {
                const auto current_write = buffer->write_pos.load(std::memory_order_relaxed);

                if (current_write == cached_read_pos)
                {
                    cached_read_pos = buffer->read_pos.load(std::memory_order_acquire);
                    if (current_write == cached_read_pos)
                    {
                        return false;
                    }
                }

                buffer->data[current_write] = item;
                const auto next_write = buffer->advanced_pos(current_write);
                buffer->write_pos.store(next_write, std::memory_order_release);
                return true;
            }

            bool try_insert() noexcept
            {
                const auto current_write = buffer->write_pos.load(std::memory_order_relaxed);

                if (current_write == cached_read_pos)
                {
                    cached_read_pos = buffer->read_pos.load(std::memory_order_acquire);
                    if (current_write == cached_read_pos)
                    {
                        return false;
                    }
                }

                return true;
            }
        };
    }
}
