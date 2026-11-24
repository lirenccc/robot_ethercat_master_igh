/**
 * @file csp_setpoint_blend.hpp
 * @brief 把慢于总线的 CSP 目标拆到每个总线周期（纯函数，无硬件依赖）
 *
 * 主机每拍给出一个绝对目标。总线更快时，本拍位移按间隔均分到后续总线周期。
 * 与总线同拍的目标直接对齐。更慢的目标最多铺到 100 ms，避免一个 2 ms 采样吃完整段位移。
 */

#ifndef ETHERCAT_JOINT_MOTOR_CSP_SETPOINT_BLEND_HPP
#define ETHERCAT_JOINT_MOTOR_CSP_SETPOINT_BLEND_HPP

#include <cstdint>

namespace ethercat_joint {

/** 2 ms 总线下 50 拍 = 100 ms，盖住 10 ms 控制环的抖动。 */
inline constexpr int32_t kCspSetpointBlendMaxSteps = 50;

struct CspSetpointBlend {
    int32_t origin = 0;
    int32_t goal = 0;
    int32_t index = 0;
    int32_t steps = 1;
    int32_t last_output = 0;
    uint64_t last_cmd_ns = 0;
    bool primed = false;
};

inline void resetCspSetpointBlend(
    CspSetpointBlend & blend, int32_t position, uint64_t now_ns) noexcept
{
    blend.origin = position;
    blend.goal = position;
    blend.index = 1;
    blend.steps = 1;
    blend.last_output = position;
    blend.last_cmd_ns = now_ns;
    blend.primed = true;
}

inline int32_t stepCspSetpointBlend(
    CspSetpointBlend & blend,
    int32_t goal,
    uint64_t now_ns,
    uint64_t bus_cycle_ns) noexcept
{
    if (!blend.primed) {
        resetCspSetpointBlend(blend, goal, now_ns);
        return goal;
    }
    if (goal != blend.goal) {
        int32_t steps = 1;
        if (bus_cycle_ns > 0U && now_ns > blend.last_cmd_ns) {
            const uint64_t dt = now_ns - blend.last_cmd_ns;
            const uint64_t n = (dt + bus_cycle_ns / 2U) / bus_cycle_ns;
            if (n >= 2U) {
                const uint64_t capped = n > static_cast<uint64_t>(kCspSetpointBlendMaxSteps)
                    ? static_cast<uint64_t>(kCspSetpointBlendMaxSteps)
                    : n;
                steps = static_cast<int32_t>(capped);
            }
        }
        blend.origin = blend.last_output;
        blend.goal = goal;
        blend.steps = steps;
        blend.index = 1;
        blend.last_cmd_ns = now_ns;
    } else if (blend.index < blend.steps) {
        ++blend.index;
    }
    const int64_t delta =
        static_cast<int64_t>(blend.goal) - static_cast<int64_t>(blend.origin);
    const int32_t out = static_cast<int32_t>(
        static_cast<int64_t>(blend.origin) +
        delta * static_cast<int64_t>(blend.index) / static_cast<int64_t>(blend.steps));
    blend.last_output = out;
    return out;
}

}  // namespace ethercat_joint

#endif  // ETHERCAT_JOINT_MOTOR_CSP_SETPOINT_BLEND_HPP
