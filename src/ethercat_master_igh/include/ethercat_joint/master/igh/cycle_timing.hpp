/**
 * @file cycle_timing.hpp
 * @brief 周期跳拍与 deadline 度量纯函数（对齐 ethercat_control_v2 control_timing.hpp）
 *
 * Timing 落后时用 nextCycleDeadline 跳到未来合法槽，禁止连发追赶。
 */

#ifndef ETHERCAT_JOINT_MASTER_IGH_CYCLE_TIMING_HPP
#define ETHERCAT_JOINT_MASTER_IGH_CYCLE_TIMING_HPP

#include <cstdint>

namespace ethercat_joint {

struct CycleTimingStats {
    uint64_t current_lateness_ns = 0U;
    uint64_t current_execution_ns = 0U;
    uint64_t max_lateness_ns = 0U;
    uint64_t max_execution_ns = 0U;
    uint64_t deadline_miss_count = 0U;
    bool deadline_met = true;
};

/**
 * 由上一拍 scheduled 与当前时刻计算下一拍绝对 deadline。
 * 若已超期则跳过错过的槽，不补跑。
 */
constexpr uint64_t nextCycleDeadline(uint64_t scheduled_wakeup_ns,
                                     uint64_t now_ns,
                                     uint64_t cycle_period_ns) noexcept
{
    if (cycle_period_ns == 0U ||
        scheduled_wakeup_ns > UINT64_MAX - cycle_period_ns) {
        return UINT64_MAX;
    }
    uint64_t next = scheduled_wakeup_ns + cycle_period_ns;
    if (now_ns < next) {
        return next;
    }
    const uint64_t skipped = ((now_ns - next) / cycle_period_ns) + 1U;
    // 落后过多时禁止跳到久远未来（会导致 Job 长时间等不到 tick 而忙等）。
    // 超过 16 拍直接以当前时刻重新基准，宁可丢周期也不让整条实时链睡死。
    constexpr uint64_t kMaxCatchUpSlots = 16U;
    if (skipped > kMaxCatchUpSlots) {
        return now_ns + cycle_period_ns;
    }
    if (skipped > (UINT64_MAX - next) / cycle_period_ns) {
        return UINT64_MAX;
    }
    return next + skipped * cycle_period_ns;
}

/** 本拍跳过的槽数（0 = 准时或仅推进一拍）。 */
constexpr uint64_t skippedSlotsBetween(uint64_t previous_scheduled_ns,
                                       uint64_t next_scheduled_ns,
                                       uint64_t cycle_period_ns) noexcept
{
    if (cycle_period_ns == 0U || next_scheduled_ns <= previous_scheduled_ns) {
        return 0U;
    }
    const uint64_t delta = next_scheduled_ns - previous_scheduled_ns;
    if (delta <= cycle_period_ns) {
        return 0U;
    }
    return (delta / cycle_period_ns) - 1U;
}

constexpr CycleTimingStats observeCycleTiming(CycleTimingStats stats,
                                              uint64_t scheduled_wakeup_ns,
                                              uint64_t actual_wakeup_ns,
                                              uint64_t cycle_end_ns,
                                              uint64_t cycle_period_ns) noexcept
{
    stats.current_lateness_ns = actual_wakeup_ns > scheduled_wakeup_ns
        ? actual_wakeup_ns - scheduled_wakeup_ns
        : 0U;
    stats.current_execution_ns = cycle_end_ns > actual_wakeup_ns
        ? cycle_end_ns - actual_wakeup_ns
        : 0U;
    if (stats.current_lateness_ns > stats.max_lateness_ns) {
        stats.max_lateness_ns = stats.current_lateness_ns;
    }
    if (stats.current_execution_ns > stats.max_execution_ns) {
        stats.max_execution_ns = stats.current_execution_ns;
    }
    const uint64_t deadline = scheduled_wakeup_ns > UINT64_MAX - cycle_period_ns
        ? UINT64_MAX
        : scheduled_wakeup_ns + cycle_period_ns;
    stats.deadline_met = cycle_period_ns != 0U && cycle_end_ns <= deadline;
    if (!stats.deadline_met && stats.deadline_miss_count != UINT64_MAX) {
        ++stats.deadline_miss_count;
    }
    return stats;
}

}  // namespace ethercat_joint

#endif  // ETHERCAT_JOINT_MASTER_IGH_CYCLE_TIMING_HPP
