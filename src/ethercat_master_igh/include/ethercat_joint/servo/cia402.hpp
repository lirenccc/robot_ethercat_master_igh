/**
 * @file cia402.hpp
 * @brief CiA402 / CoE 类型与状态字解码
 *
 * 状态字解码遵循 DS402：Not ready / Switch on disabled / Fault* 的 bit5
 * （Quick stop）为 don't care，用掩码 0x4F；Ready / Switched on /
 * Operation enabled / Quick stop active 用掩码 0x6F。
 */

#ifndef ETHERCAT_JOINT_SERVO_CIA402_HPP
#define ETHERCAT_JOINT_SERVO_CIA402_HPP

#include <cstdint>

namespace ethercat_joint {

/** CANopen over EtherCAT (CoE) 操作模式 */
enum class OperationMode : int8_t {
    NONE = 0,
    PROFILE_POSITION = 1,          // PP
    PROFILE_VELOCITY = 3,          // PV
    PROFILE_TORQUE = 4,            // PT
    HOMING = 6,                    // HM
    INTERPOLATED_POSITION = 7,     // IP
    CYCLIC_SYNC_POSITION = 8,      // CSP
    CYCLIC_SYNC_VELOCITY = 9,      // CSV
    CYCLIC_SYNC_TORQUE = 10        // CST
};

/** CIA402 状态机状态（值为状态字关键位模式） */
enum class CIA402State : uint16_t {
    NOT_READY_TO_SWITCH_ON = 0x00,
    SWITCH_ON_DISABLED = 0x40,
    READY_TO_SWITCH_ON = 0x21,
    SWITCHED_ON = 0x23,
    OPERATION_ENABLED = 0x27,
    QUICK_STOP_ACTIVE = 0x07,
    FAULT_REACTION_ACTIVE = 0x0F,
    FAULT = 0x08
};

/**
 * 控制字命令值（0x6040）。
 * 历史命名与 CiA402 命令名不完全一致，数值勿改：
 *   CONTROL_WORD_SWITCH_ON      = Shutdown (0x06)
 *   CONTROL_WORD_ENABLE_VOLTAGE = Switch on (0x07)
 *   CONTROL_WORD_ENABLE_OPERATION / KEEP_OPERATION / FAULT_RESET 名实相符
 */
constexpr uint16_t CONTROL_WORD_SWITCH_ON = 0x06;
constexpr uint16_t CONTROL_WORD_ENABLE_VOLTAGE = 0x07;
constexpr uint16_t CONTROL_WORD_ENABLE_OPERATION = 0x0F;
constexpr uint16_t CONTROL_WORD_KEEP_OPERATION = 0x1F;
constexpr uint16_t CONTROL_WORD_FAULT_RESET = 0x80;

constexpr uint16_t kCia402EnableSequence[4] = {
    CONTROL_WORD_SWITCH_ON,
    CONTROL_WORD_ENABLE_VOLTAGE,
    CONTROL_WORD_ENABLE_OPERATION,
    CONTROL_WORD_KEEP_OPERATION};

constexpr uint16_t kCia402DisableSequence[4] = {
    CONTROL_WORD_KEEP_OPERATION,
    CONTROL_WORD_ENABLE_OPERATION,
    CONTROL_WORD_ENABLE_VOLTAGE,
    CONTROL_WORD_SWITCH_ON};

/** bit2=Operation enabled 且 bit3=Fault 清除（兼容部分厂商扩展状态字） */
inline bool isCiA402OperationEnabled(uint16_t status_word)
{
    return (status_word & 0x08) == 0 && (status_word & 0x04) != 0;
}

/**
 * CiA402 Fault 判定：状态字 bit3。
 * Fault 与 Fault reaction active 均置该位；勿仅用 decode 结果 == FAULT。
 */
inline bool isCiA402Fault(uint16_t status_word) noexcept
{
    return (status_word & 0x08U) != 0U;
}

inline CIA402State decodeCia402State(uint16_t status_word)
{
    const uint16_t key_bits = status_word & 0x4F;
    if (key_bits == 0x00) {
        return CIA402State::NOT_READY_TO_SWITCH_ON;
    }
    if (key_bits == 0x40) {
        return CIA402State::SWITCH_ON_DISABLED;
    }
    if ((status_word & 0x6F) == 0x21) {
        return CIA402State::READY_TO_SWITCH_ON;
    }
    if ((status_word & 0x6F) == 0x23) {
        return CIA402State::SWITCHED_ON;
    }
    if ((status_word & 0x6F) == 0x27) {
        return CIA402State::OPERATION_ENABLED;
    }
    if ((status_word & 0x6F) == 0x07) {
        return CIA402State::QUICK_STOP_ACTIVE;
    }
    if (key_bits == 0x0F) {
        return CIA402State::FAULT_REACTION_ACTIVE;
    }
    if ((key_bits & 0x08) == 0x08) {
        return CIA402State::FAULT;
    }
    return CIA402State::NOT_READY_TO_SWITCH_ON;
}

inline const char* cia402StateName(uint16_t state)
{
    switch (state) {
        case 0x00: return "NOT_READY_TO_SWITCH_ON";
        case 0x40: return "SWITCH_ON_DISABLED";
        case 0x21: return "READY_TO_SWITCH_ON";
        case 0x23: return "SWITCHED_ON";
        case 0x27: return "OPERATION_ENABLED";
        case 0x07: return "QUICK_STOP_ACTIVE";
        case 0x0F: return "FAULT_REACTION_ACTIVE";
        case 0x08: return "FAULT";
        default: return "UNKNOWN";
    }
}

inline const char* cia402StateName(CIA402State state)
{
    return cia402StateName(static_cast<uint16_t>(state));
}

}  // namespace ethercat_joint

#endif  // ETHERCAT_JOINT_SERVO_CIA402_HPP
