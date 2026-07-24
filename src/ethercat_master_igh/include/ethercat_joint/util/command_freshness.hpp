/**
 * @file command_freshness.hpp
 * @brief 外部 CST/CSV 命令新鲜度（纯头文件，无堆/无日志）
 *
 * 外部命令新鲜度：外部命令源必须在超时内刷新；超时则要求安全输出。
 * Job 内推进的文件轨迹走 JobInternal，不武装外部 watchdog。
 */

#ifndef ETHERCAT_JOINT_UTIL_COMMAND_FRESHNESS_HPP
#define ETHERCAT_JOINT_UTIL_COMMAND_FRESHNESS_HPP

#include <cstdint>

namespace ethercat_joint {

/** 设定点来源：External 走新鲜度；JobInternal 由 Job hook 推进，不超时 */
enum class SetpointSource : uint8_t {
    External = 0,
    JobInternal = 1,
};

struct CommandFreshnessState {
    bool armed = false;
    uint64_t deadline_ns = 0;
};

inline void armExternalCommand(
    CommandFreshnessState & state,
    uint64_t now_ns,
    uint64_t timeout_ns) noexcept
{
    if (timeout_ns == 0U) {
        state.armed = false;
        state.deadline_ns = 0U;
        return;
    }
    state.armed = true;
    state.deadline_ns = now_ns + timeout_ns;
}

inline void disarmCommandFreshness(CommandFreshnessState & state) noexcept
{
    state.armed = false;
    state.deadline_ns = 0U;
}

/** Job 文件轨迹：解除外部武装，避免误杀 */
inline void markJobInternalCommand(CommandFreshnessState & state) noexcept
{
    disarmCommandFreshness(state);
}

inline bool isExternalCommandStale(
    const CommandFreshnessState & state,
    uint64_t now_ns) noexcept
{
    return state.armed && now_ns > state.deadline_ns;
}

/**
 * 抢锁失败时的安全退化：保持 armed/deadline，本拍应输出零力矩/零速。
 * 返回 true 表示本拍应对该通道做安全退化。
 */
inline bool contentionFallbackActive(const CommandFreshnessState & state) noexcept
{
    return state.armed;
}

}  // namespace ethercat_joint

#endif  // ETHERCAT_JOINT_UTIL_COMMAND_FRESHNESS_HPP
