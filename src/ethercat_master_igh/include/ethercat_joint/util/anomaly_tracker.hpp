/**
 * @file anomaly_tracker.hpp
 * @brief 连续/滑窗异常策略与 healthy dwell（纯头文件，无堆/无日志）
 *
 * 对齐 ethercat_control_v2 control_core/anomaly_tracker.hpp。
 */

#ifndef ETHERCAT_JOINT_UTIL_ANOMALY_TRACKER_HPP
#define ETHERCAT_JOINT_UTIL_ANOMALY_TRACKER_HPP

#include <algorithm>
#include <cstdint>
#include <limits>

namespace ethercat_joint {

struct AnomalyPolicy {
    uint32_t consecutive_stop = 1U;
    uint32_t window_size = 1U;
    uint32_t window_stop = 1U;
};

struct AnomalyStats {
    uint64_t total = 0U;
    uint32_t consecutive = 0U;
    uint32_t window_count = 0U;
    bool stop_required = false;
};

class AnomalyTracker {
public:
    explicit AnomalyTracker(AnomalyPolicy policy = {}) noexcept
        : policy_(sanitize(policy))
    {
    }

    /**
     * 每周期喂入一次异常布尔。window_bits_ 是最多 64 拍的环形位图：cursor_ 指向下一写入位，
     * window_count_ 等于窗口内 1 的个数。sanitize 把 window_size 钳到 1…64，并把阈值为 0
     * 抬到 1（fail-closed：配置错误时宁可更容易停机）。
     */
    AnomalyStats observe(bool anomaly) noexcept
    {
        if (anomaly) {
            if (total_ != std::numeric_limits<uint64_t>::max()) {
                ++total_;
            }
            if (consecutive_ != std::numeric_limits<uint32_t>::max()) {
                ++consecutive_;
            }
        } else {
            consecutive_ = 0U;
        }

        const uint64_t bit = uint64_t{1} << cursor_;
        if ((window_bits_ & bit) != 0U && window_count_ > 0U) {
            --window_count_;
        }
        if (anomaly) {
            window_bits_ |= bit;
            ++window_count_;
        } else {
            window_bits_ &= ~bit;
        }
        cursor_ = (cursor_ + 1U) % policy_.window_size;

        return {total_, consecutive_, window_count_,
                consecutive_ >= policy_.consecutive_stop ||
                    window_count_ >= policy_.window_stop};
    }

    AnomalyStats stats() const noexcept
    {
        return {total_, consecutive_, window_count_,
                consecutive_ >= policy_.consecutive_stop ||
                    window_count_ >= policy_.window_stop};
    }

    void reset() noexcept
    {
        total_ = 0U;
        consecutive_ = 0U;
        window_bits_ = 0U;
        window_count_ = 0U;
        cursor_ = 0U;
    }

    const AnomalyPolicy& policy() const noexcept { return policy_; }

private:
    static AnomalyPolicy sanitize(AnomalyPolicy policy) noexcept
    {
        policy.consecutive_stop = std::max<uint32_t>(1U, policy.consecutive_stop);
        policy.window_size = std::clamp<uint32_t>(policy.window_size, 1U, 64U);
        policy.window_stop =
            std::clamp<uint32_t>(policy.window_stop, 1U, policy.window_size);
        return policy;
    }

    AnomalyPolicy policy_;
    uint64_t total_ = 0U;
    uint64_t window_bits_ = 0U;
    uint32_t consecutive_ = 0U;
    uint32_t window_count_ = 0U;
    uint32_t cursor_ = 0U;
};

/**
 * 安全复位后的健康驻留：连续 K 个健康周期后才允许再使能。
 * 进入故障时禁止使能；显式 requestReset() 后开始计数。
 */
class HealthyDwell {
public:
    explicit HealthyDwell(uint32_t required_cycles = 1U) noexcept
        : required_(std::max<uint32_t>(1U, required_cycles))
    {
    }

    void onFault() noexcept
    {
        armed_ = false;
        consecutive_ok_ = 0U;
        allow_enable_ = false;
    }

    /** 操作员显式安全复位：开始 dwell，尚不允许使能 */
    void requestReset() noexcept
    {
        armed_ = true;
        consecutive_ok_ = 0U;
        allow_enable_ = false;
    }

    void observe(bool healthy) noexcept
    {
        if (!armed_) {
            return;
        }
        if (healthy) {
            if (consecutive_ok_ != std::numeric_limits<uint32_t>::max()) {
                ++consecutive_ok_;
            }
            if (consecutive_ok_ >= required_) {
                // 驻留完成：保持放行直至下次 onFault；勿因偶发 RX 抖动再次清零
                allow_enable_ = true;
                armed_ = false;
                consecutive_ok_ = 0U;
            }
        } else {
            consecutive_ok_ = 0U;
            allow_enable_ = false;
        }
    }

    bool allowEnable() const noexcept { return allow_enable_; }
    bool armed() const noexcept { return armed_; }
    uint32_t consecutiveOk() const noexcept { return consecutive_ok_; }
    uint32_t required() const noexcept { return required_; }

    /** 首次上电：允许使能（尚无故障闩锁） */
    void allowInitialEnable() noexcept
    {
        armed_ = false;
        consecutive_ok_ = 0U;
        allow_enable_ = true;
    }

    void reset(uint32_t required_cycles) noexcept
    {
        required_ = std::max<uint32_t>(1U, required_cycles);
        allowInitialEnable();
    }

private:
    uint32_t required_ = 1U;
    uint32_t consecutive_ok_ = 0U;
    bool armed_ = false;
    bool allow_enable_ = true;
};

}  // namespace ethercat_joint

#endif  // ETHERCAT_JOINT_UTIL_ANOMALY_TRACKER_HPP
