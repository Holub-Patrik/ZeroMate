#pragma once

#include <cstdint>
#include <array>
#include <atomic>

#include "CircularBufferQueue.hpp"

class FastMap
{
public:
    static constexpr std::size_t GPIOMapSize = 64;

private:
    std::array<std::uint8_t, GPIOMapSize> _map;

public:
    FastMap()
    : _map()
    {
        for (std::uint8_t& mapping : _map)
        {
            mapping = UINT8_MAX;
        }
    }

    void set(const std::uint8_t src, const std::uint8_t dst) noexcept
    {
        _map[src] = dst;
    }

    [[nodiscard]] std::uint8_t get(const std::uint8_t src) const noexcept
    {
        return _map[src];
    }

    [[nodiscard]] bool contains(const std::uint8_t pos) const noexcept
    {
        return _map[pos] != UINT8_MAX;
    }

    [[nodiscard]] const std::array<std::uint8_t, GPIOMapSize>& _get_arr() const noexcept
    {
        return _map;
    }
};

class Spinlock final
{
private:
    std::atomic<bool> lock_ = { false };

public:
    Spinlock() = default;
    ~Spinlock() = default;

    Spinlock(const Spinlock& other) = delete;
    Spinlock& operator=(const Spinlock& other) = delete;

    Spinlock(Spinlock&& other) = delete;
    Spinlock& operator=(Spinlock&& other) = delete;

    void lock() noexcept
    {
        for (;;)
        {
            if (!lock_.exchange(true, std::memory_order_acquire))
            {
                return;
            }
            while (lock_.load(std::memory_order_relaxed))
            {
                cpu_relax();
            }
        }
    }

    bool try_lock() noexcept
    {
        return !lock_.load(std::memory_order_relaxed) && !lock_.exchange(true, std::memory_order_acquire);
    }

    void unlock() noexcept
    {
        lock_.store(false, std::memory_order_release);
    }
};
