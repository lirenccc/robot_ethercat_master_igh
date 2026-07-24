/**
 * @file seqlock.hpp
 * @brief 单写多读序列锁：实时写侧无阻塞，读侧在撕裂时重试。
 */

#ifndef ETHERCAT_JOINT_UTIL_SEQLOCK_HPP_
#define ETHERCAT_JOINT_UTIL_SEQLOCK_HPP_

#include <atomic>
#include <cstdint>

namespace ethercat_joint {

class SeqLock {
public:
    void writeBegin() noexcept
    {
        const uint64_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_release);  // odd = writing
    }

    void writeEnd() noexcept
    {
        const uint64_t s = seq_.load(std::memory_order_relaxed);
        seq_.store(s + 1, std::memory_order_release);  // even = stable
    }

    uint64_t readBegin() const noexcept
    {
        uint64_t s;
        do {
            s = seq_.load(std::memory_order_acquire);
        } while (s & 1ULL);
        return s;
    }

    bool readRetry(uint64_t start) const noexcept
    {
        std::atomic_thread_fence(std::memory_order_acquire);
        return seq_.load(std::memory_order_relaxed) != start;
    }

private:
    std::atomic<uint64_t> seq_{0};
};

/** vector<bool> 不可用于跨线程；可放入 vector 的原子布尔包装。 */
struct AtomicBool {
    std::atomic<uint8_t> v{0};

    AtomicBool() noexcept = default;
    AtomicBool(bool b) noexcept : v(b ? 1 : 0) {}
    AtomicBool(const AtomicBool& o) noexcept : v(o.v.load(std::memory_order_relaxed)) {}
    AtomicBool& operator=(const AtomicBool& o) noexcept
    {
        v.store(o.v.load(std::memory_order_relaxed), std::memory_order_relaxed);
        return *this;
    }
    AtomicBool& operator=(bool b) noexcept
    {
        v.store(b ? 1 : 0, std::memory_order_release);
        return *this;
    }

    bool load(std::memory_order order = std::memory_order_acquire) const noexcept
    {
        return v.load(order) != 0;
    }
    void store(bool b, std::memory_order order = std::memory_order_release) noexcept
    {
        v.store(b ? 1 : 0, order);
    }
    operator bool() const noexcept { return load(); }
};

struct AtomicU8 {
    std::atomic<uint8_t> v{0};

    AtomicU8() noexcept = default;
    AtomicU8(uint8_t x) noexcept : v(x) {}
    AtomicU8(const AtomicU8& o) noexcept : v(o.v.load(std::memory_order_relaxed)) {}
    AtomicU8& operator=(const AtomicU8& o) noexcept
    {
        v.store(o.v.load(std::memory_order_relaxed), std::memory_order_relaxed);
        return *this;
    }
    AtomicU8& operator=(uint8_t x) noexcept
    {
        v.store(x, std::memory_order_release);
        return *this;
    }

    uint8_t load(std::memory_order order = std::memory_order_acquire) const noexcept
    {
        return v.load(order);
    }
    void store(uint8_t x, std::memory_order order = std::memory_order_release) noexcept
    {
        v.store(x, order);
    }
    operator uint8_t() const noexcept { return load(); }
    AtomicU8& operator++() noexcept
    {
        v.fetch_add(1, std::memory_order_acq_rel);
        return *this;
    }
    uint8_t operator++(int) noexcept { return v.fetch_add(1, std::memory_order_acq_rel); }
};

struct AtomicU16 {
    std::atomic<uint16_t> v{0};

    AtomicU16() noexcept = default;
    AtomicU16(uint16_t x) noexcept : v(x) {}
    AtomicU16(const AtomicU16& o) noexcept : v(o.v.load(std::memory_order_relaxed)) {}
    AtomicU16& operator=(const AtomicU16& o) noexcept
    {
        v.store(o.v.load(std::memory_order_relaxed), std::memory_order_relaxed);
        return *this;
    }
    AtomicU16& operator=(uint16_t x) noexcept
    {
        v.store(x, std::memory_order_release);
        return *this;
    }

    uint16_t load(std::memory_order order = std::memory_order_acquire) const noexcept
    {
        return v.load(order);
    }
    void store(uint16_t x, std::memory_order order = std::memory_order_release) noexcept
    {
        v.store(x, order);
    }
    operator uint16_t() const noexcept { return load(); }
    AtomicU16& operator--() noexcept
    {
        v.fetch_sub(1, std::memory_order_acq_rel);
        return *this;
    }
    uint16_t operator--(int) noexcept { return v.fetch_sub(1, std::memory_order_acq_rel); }
};

}  // namespace ethercat_joint

#endif  // ETHERCAT_JOINT_UTIL_SEQLOCK_HPP_
