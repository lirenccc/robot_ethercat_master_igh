/**
 * @file motor_profile.hpp
 * @brief 关节模组型号注册表：VID/PID、PDO 布局、默认运动学参数
 *
 * 新增模组：在 motor_profile.cpp 的 kProfiles 中追加一条 MotorProfile，
 * 并在 motors_xinqi.yaml / motors_sjd17.yaml（或新建实例 yaml）中设置
 * motor_model 与可选覆盖参数。
 */

#ifndef MOTOR_PROFILE_HPP
#define MOTOR_PROFILE_HPP

#include "ethercat_joint/motor/motor_kinematics.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace ethercat_joint {

/** EtherCAT 从站 PDO 映射类型 */
enum class PdoLayout {
    UNKNOWN = 0,
    JOINT_MODULE,   ///< 标准关节模组 PDO：0x1600/0x1A00（IgH 新奇 14B；SJD17 Tx 22B 含 0x2020/0x2021；新奇 ENI 另可含 0x603F）
    GATEWAY,
    /** CoolDrive JMDT（天机 Marvin）：Rx 6040/6060/5FFE/607A/60FF/6071；Tx 6041/6061/5FFE/6064/606C/6077/310B */
    COOLDRIVE_JMDT,
};

struct SlaveIdentity {
    uint32_t vendor_id = 0;
    uint32_t product_code = 0;
};

/** 单型号模组静态描述（不含运行时从站 position） */
struct MotorProfile {
    std::string model_id;
    std::string display_name;
    std::vector<SlaveIdentity> identities;
    PdoLayout pdo_layout = PdoLayout::UNKNOWN;
    MotorKinematicsParams kinematics{};
    double velocity_limit_deg_s = 246.0;
    uint32_t dc_shift_ns = 0;  ///< DC SYNC0 偏移，新奇模组建议 0
    uint32_t dc_sync0_ns = 0;  ///< DC SYNC0 周期（ns）；0→沿用 bus_cycle_us*1000。JMDT 需 2000000 (2ms)。
    /** DC AssignActivate 字（厂商特定；0x0000=关闭 DC）。NH17 用 0x0300（激活 SYNC0，IgH 标准 DC），模组文档要求激活 DC 后模式显示才会跟随。 */
    uint16_t dc_assign_activate = 0x0300;
    /** 启动时是否校验 0x60C2 与总线周期一致；无 0x60C2 的型号设 false */
    bool require_interpolation_period_gate = true;
};

class MotorProfileRegistry {
public:
    static const MotorProfile* findByModelId(const std::string& model_id);
    static const MotorProfile* findByIdentity(uint32_t vendor_id, uint32_t product_code);
    static const std::vector<MotorProfile>& all();

    /** 所有已注册关节模组的 VID/PID（用于从站扫描） */
    static std::vector<SlaveIdentity> allMotorIdentities();

    /** VID/PID 是否匹配任一已注册关节模组 */
    static bool isKnownMotor(uint32_t vendor_id, uint32_t product_code);

    static const char* pdoLayoutName(PdoLayout layout);
};

}  // namespace ethercat_joint

#endif
