/**
 * @file safe_output.hpp
 * @brief Job 内 safe-output 纯逻辑（无硬件可测）
 *
 * 严重通信故障时：钉住故障瞬间实际位置、速度/力矩清零、控制字 Shutdown(0x06)。
 * 不在此路径隐式 Fault Reset(0x0080) 或再使能。只有上层通过显式、
 * 禁用态门禁后的复位请求，才可在有限周期内覆盖控制字为 0x0080。
 */

#ifndef ETHERCAT_JOINT_MASTER_IGH_SAFE_OUTPUT_HPP
#define ETHERCAT_JOINT_MASTER_IGH_SAFE_OUTPUT_HPP

#include "ethercat_joint/servo/cia402.hpp"

#include <cstdint>

namespace ethercat_joint {

struct SafeProcessImageOutputs {
    int32_t target_position = 0;
    int32_t target_velocity = 0;
    int16_t target_torque = 0;
    uint16_t control_word = CONTROL_WORD_SWITCH_ON;  // Shutdown 0x06
};

/** 由故障边沿实际位置计算安全 PDO 设定。 */
inline SafeProcessImageOutputs makeSafeProcessImageOutputs(int32_t actual_position) noexcept
{
    SafeProcessImageOutputs out;
    out.target_position = actual_position;
    out.target_velocity = 0;
    out.target_torque = 0;
    out.control_word = CONTROL_WORD_SWITCH_ON;
    return out;
}

/**
 * 显式 Fault Reset 只覆盖已选中且仍处于 Fault 的轴。
 * 非故障轴和复位窗口之外始终保持 Shutdown(0x06)。
 */
inline uint16_t selectSafeControlWord(bool reset_window_active,
                                      bool axis_selected,
                                      bool axis_faulted) noexcept
{
    return reset_window_active && axis_selected && axis_faulted
        ? CONTROL_WORD_FAULT_RESET
        : CONTROL_WORD_SWITCH_ON;
}

/**
 * 边沿锁存：刚进入 safe 时捕获 actual；持续 safe 时保持 latched。
 * @param entering_safe 本拍是否为进入 safe 的上升沿
 */
inline int32_t latchSafePosition(bool entering_safe,
                                 int32_t latched_position,
                                 int32_t actual_position) noexcept
{
    return entering_safe ? actual_position : latched_position;
}

}  // namespace ethercat_joint

#endif  // ETHERCAT_JOINT_MASTER_IGH_SAFE_OUTPUT_HPP
