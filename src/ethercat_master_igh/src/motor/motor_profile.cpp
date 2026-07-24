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
        true,
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
        false,
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
        default: return "unknown";
    }
}

}  // namespace ethercat_joint
