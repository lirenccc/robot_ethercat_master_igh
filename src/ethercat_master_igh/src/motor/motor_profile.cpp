#include "ethercat_joint/motor/motor_profile.hpp"

#include <algorithm>

namespace ethercat_joint {

namespace {

MotorKinematicsParams makeNh17Kinematics()
{
    MotorKinematicsParams p;
    p.gear_ratio = 101.0;
    p.encoder_resolution = 65536.0;
    p.motor_encoder_resolution = 65536.0;
    p.output_side_encoder = false;
    // 0x2016=0 时位置切外圈，但 0x606C/0x60FF 仍为电机侧 cnt/s
    p.velocity_on_motor_encoder = true;
    p.rated_torque_motor = 0.297;
    p.max_current_ma = 4602.0;
    p.torque_constant_kt = 0.030;
    p.gear_efficiency = 0.6;
    return p;
}

MotorKinematicsParams makeSjd17Kinematics()
{
    MotorKinematicsParams p;
    p.gear_ratio = 1.0;
    p.torque_gear_ratio = 120.0;
    p.encoder_resolution = 524288.0;
    p.output_side_encoder = true;
    // 模组瞬间峰值转矩 112 Nm；千分比按峰值标定 → 电机侧 112/120
    p.rated_torque_motor = 112.0 / 120.0;
    p.max_current_ma = 0.0;
    p.torque_constant_kt = 0.1075;
    p.gear_efficiency = 1.0;
    return p;
}

MotorKinematicsParams makePmslmLinearKinematics()
{
    // Linear axis abuse of rotary kinematics: treat 1 mm as 1 "degree".
    // pulse = mm/360 * encoder_resolution → with enc=360000 → pulse = mm * 1000
    // (matches legacy ethercat_controller position_scale=1000 pulses/mm).
    MotorKinematicsParams p;
    p.gear_ratio = 1.0;
    p.torque_gear_ratio = 1.0;
    p.encoder_resolution = 360000.0;
    p.motor_encoder_resolution = 360000.0;
    p.output_side_encoder = true;
    p.velocity_on_motor_encoder = false;
    p.rated_torque_motor = 15.6;  // thrust scale companion (legacy thrust_scale=1000/15.6)
    p.max_current_ma = 0.0;
    p.torque_constant_kt = 0.0;
    p.gear_efficiency = 1.0;
    return p;
}

MotorKinematicsParams makeJmdtKinematics()
{
    // 默认取 J1/J2（减速比 120、20bit 编码器）；各轴差异由 motors_jmdt*.yaml overlay。
    MotorKinematicsParams p;
    p.gear_ratio = 120.0;
    p.torque_gear_ratio = 120.0;
    p.encoder_resolution = 1048576.0;
    p.motor_encoder_resolution = 1048576.0;
    p.output_side_encoder = false;
    p.velocity_on_motor_encoder = true;
    p.rated_torque_motor = 0.75;
    p.max_current_ma = 0.0;
    p.torque_constant_kt = 0.0;
    p.gear_efficiency = 1.0;
    return p;
}

const std::vector<MotorProfile> kProfiles = {
    {
        "NH17-100-BT-48E",
        "新奇 NH17-100-BT-48E 关节模组",
        {
            {0x00522227, 0x00009253},
        },
        PdoLayout::JOINT_MODULE,
        makeNh17Kinematics(),
        246.0,
        0U,
        4000000U, // dc_sync0_ns: DC 周期 = 4ms（对齐 ENI CycleTime0/总线周期）
        0x0300U,  // dc_assign_activate: 激活 SYNC0（IgH 标准 DC；0x0003 只分配周期不激活 SYNC，
                  //   NH17 模式显示不跟随导致 od=0、拒绝进入 Operation Enabled）
        true,     // require_interpolation_period_gate: ENI 经 SDO 下载 0x60C2:1=4、0x60C2:2=-3
    },
    {
        "SJD-17-120-NN-S00",
        "三木禾 SJD-17-120-NN-S00 关节模组",
        {
            {0x000009CF, 0x00010001},
        },
        PdoLayout::JOINT_MODULE,
        makeSjd17Kinematics(),
        225.0,
        0U,
        0U,
        0x0000U, // dc_assign_activate: SJD 默认关 DC（free-run）
        false,   // require_interpolation_period_gate: SJD 无 0x60C2 支持
    },
    {
        "PMSLM-LINEAR",
        "并联直线电机 PMSLM（1°:=1mm，脉冲/mm=1000）",
        {
            {0x00418108, 0x00009252},
        },
        PdoLayout::JOINT_MODULE,
        makePmslmLinearKinematics(),
        500.0,
        0U,
        0U,
        0x0000U, // dc_assign_activate: 默认关 DC（free-run）
        true,    // require_interpolation_period_gate: PMSLM 支持 0x60C2
    },
    {
        "COOLDRIVE-JMDT",
        "CoolDrive JMDT 关节模组（天机 Marvin）",
        {
            {0x00000748, 0x00000019},
        },
        PdoLayout::COOLDRIVE_JMDT,
        makeJmdtKinematics(),
        180.0,
        720000U,  // dc_shift_ns: SYNC0 shift ≈ 720 µs（天机 Marvin 对齐值）
        2000000U, // dc_sync0_ns: DC SYNC0 周期 = 2ms（天机 Marvin 成功值）
        0x0000U,  // dc_assign_activate: 当前 free-run（关闭 DC）；需 DC 时改 0x0300
        false,    // require_interpolation_period_gate: JMDT 不做 0x60C2 校验
    },
};

bool identityEqual(const SlaveIdentity& a, const SlaveIdentity& b)
{
    return a.vendor_id == b.vendor_id && a.product_code == b.product_code;
}

}  // namespace

const MotorProfile* MotorProfileRegistry::findByModelId(const std::string& model_id)
{
    for (const auto& profile : kProfiles) {
        if (profile.model_id == model_id) {
            return &profile;
        }
    }
    return nullptr;
}

const MotorProfile* MotorProfileRegistry::findByIdentity(
    uint32_t vendor_id, uint32_t product_code)
{
    const SlaveIdentity needle{vendor_id, product_code};
    for (const auto& profile : kProfiles) {
        for (const auto& id : profile.identities) {
            if (identityEqual(id, needle)) {
                return &profile;
            }
        }
    }
    return nullptr;
}

const std::vector<MotorProfile>& MotorProfileRegistry::all()
{
    return kProfiles;
}

std::vector<SlaveIdentity> MotorProfileRegistry::allMotorIdentities()
{
    std::vector<SlaveIdentity> out;
    for (const auto& profile : kProfiles) {
        for (const auto& id : profile.identities) {
            if (std::none_of(out.begin(), out.end(),
                             [&](const SlaveIdentity& existing) {
                                 return identityEqual(existing, id);
                             })) {
                out.push_back(id);
            }
        }
    }
    return out;
}

bool MotorProfileRegistry::isKnownMotor(uint32_t vendor_id, uint32_t product_code)
{
    return findByIdentity(vendor_id, product_code) != nullptr;
}

const char* MotorProfileRegistry::pdoLayoutName(PdoLayout layout)
{
    switch (layout) {
        case PdoLayout::JOINT_MODULE: return "joint_module";
        case PdoLayout::GATEWAY: return "gateway";
        case PdoLayout::COOLDRIVE_JMDT: return "cooldrive_jmdt";
        default: return "unknown";
    }
}

}  // namespace ethercat_joint
