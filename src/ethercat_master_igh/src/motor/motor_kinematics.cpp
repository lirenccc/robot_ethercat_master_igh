#include "ethercat_joint/motor/motor_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace ethercat_joint {

namespace {

std::vector<MotorKinematicsParams> g_params{MotorKinematicsParams{}};

}  // namespace

void MotorKinematics::setParams(const std::vector<MotorKinematicsParams>& params)
{
    if (!params.empty()) {
        g_params = params;
    }
}

const MotorKinematicsParams& MotorKinematics::get(size_t motor_id)
{
    static const MotorKinematicsParams kDefault;
    if (motor_id < g_params.size()) {
        return g_params[motor_id];
    }
    return kDefault;
}

bool MotorKinematics::inferOutputSideEncoder(double encoder_resolution)
{
    // 外圈 18bit=262144；内圈 16bit=65536（新奇 0x2016=2）
    return encoder_resolution >= 200000.0;
}

double MotorKinematics::encoderFactor(size_t motor_id)
{
    const auto& p = get(motor_id);
    if (p.output_side_encoder) {
        return p.encoder_resolution;
    }
    return p.gear_ratio * p.encoder_resolution;
}

double MotorKinematics::velocityEncoderFactor(size_t motor_id)
{
    const auto& p = get(motor_id);
    if (p.velocity_on_motor_encoder) {
        const double motor_enc =
            p.motor_encoder_resolution > 0.0 ? p.motor_encoder_resolution : 65536.0;
        return p.gear_ratio * motor_enc;
    }
    return encoderFactor(motor_id);
}

std::string MotorKinematics::describe(size_t motor_id)
{
    const auto& p = get(motor_id);
    std::ostringstream oss;
    oss << (p.output_side_encoder ? "outer" : "inner")
        << " enc=" << static_cast<int>(p.encoder_resolution)
        << " ratio=" << p.gear_ratio;
    if (p.torque_gear_ratio != p.gear_ratio) {
        oss << " torque_ratio=" << p.torque_gear_ratio;
    }
    const double pos_factor = encoderFactor(motor_id);
    const double vel_factor = velocityEncoderFactor(motor_id);
    oss << " pos_factor=" << static_cast<int>(pos_factor);
    if (std::abs(vel_factor - pos_factor) > 0.5) {
        oss << " vel_factor=" << static_cast<int>(vel_factor)
            << " (motor_enc=" << static_cast<int>(p.motor_encoder_resolution) << ")";
    } else {
        oss << " factor=" << static_cast<int>(pos_factor);
    }
    oss << " (deg/360×factor→cnt)";
    return oss.str();
}

bool MotorKinematics::useCurrentBasedTorqueFormula(size_t motor_id)
{
    const auto& p = get(motor_id);
    return p.max_current_ma > 0.0 && p.torque_constant_kt > 0.0;
}

double MotorKinematics::outputTorqueScale(size_t motor_id)
{
    const auto& p = get(motor_id);
    if (useCurrentBasedTorqueFormula(motor_id)) {
        return p.max_current_ma / 1000.0 * p.torque_constant_kt
             * p.torque_gear_ratio * p.gear_efficiency;
    }
    return p.rated_torque_motor * p.torque_gear_ratio;
}

double MotorKinematics::pulseToDegree(int32_t pulse, size_t motor_id)
{
    const auto& p = get(motor_id);
    const double factor = encoderFactor(motor_id);
    double motor_degree = pulse / factor * kDegreesPerRevolution;
    motor_degree *= p.joint_direction;
    return motor_degree + p.position_offset_deg;
}

int32_t MotorKinematics::degreeToPulse(double degree, size_t motor_id)
{
    const auto& p = get(motor_id);
    double motor_degree = (degree - p.position_offset_deg) / p.joint_direction;
    const double factor = encoderFactor(motor_id);
    return static_cast<int32_t>(motor_degree / kDegreesPerRevolution * factor);
}

double MotorKinematics::pulseToRadian(int32_t pulse, size_t motor_id)
{
    return pulseToDegree(pulse, motor_id) * kRadiansPerRevolution / kDegreesPerRevolution;
}

int32_t MotorKinematics::radianToPulse(double radian, size_t motor_id)
{
    const auto& p = get(motor_id);
    double motor_rad = (radian - p.position_offset_deg * kRadiansPerRevolution / kDegreesPerRevolution)
                       / p.joint_direction;
    const double factor = encoderFactor(motor_id);
    return static_cast<int32_t>(motor_rad / kRadiansPerRevolution * factor);
}

double MotorKinematics::pulsePerSecToDegreePerSec(int32_t pulse_per_sec, size_t motor_id)
{
    const auto& p = get(motor_id);
    const double factor = velocityEncoderFactor(motor_id);
    return pulse_per_sec / factor * kDegreesPerRevolution * p.joint_direction;
}

int32_t MotorKinematics::degreePerSecToPulsePerSec(double degree_per_sec, size_t motor_id)
{
    const auto& p = get(motor_id);
    degree_per_sec /= p.joint_direction;
    const double factor = velocityEncoderFactor(motor_id);
    return static_cast<int32_t>(degree_per_sec / kDegreesPerRevolution * factor);
}

double MotorKinematics::outputRpmToDegreePerSec(double output_rpm)
{
    return output_rpm * kDegreesPerRevolution / 60.0;
}

double MotorKinematics::degreePerSecToOutputRpm(double degree_per_sec)
{
    return degree_per_sec * 60.0 / kDegreesPerRevolution;
}

int32_t MotorKinematics::motorRpmToPulsePerSec(double motor_rpm, size_t motor_id)
{
    const auto& p = get(motor_id);
    const double motor_enc =
        p.motor_encoder_resolution > 0.0 ? p.motor_encoder_resolution : p.encoder_resolution;
    return static_cast<int32_t>(motor_rpm / 60.0 * motor_enc * p.joint_direction);
}

double MotorKinematics::pulsePerSecToMotorRpm(int32_t pulse_per_sec, size_t motor_id)
{
    const auto& p = get(motor_id);
    const double motor_enc =
        p.motor_encoder_resolution > 0.0 ? p.motor_encoder_resolution : p.encoder_resolution;
    if (motor_enc <= 0.0) {
        return 0.0;
    }
    return static_cast<double>(pulse_per_sec) / motor_enc * 60.0 / p.joint_direction;
}

int32_t MotorKinematics::outputRpmToPulsePerSec(double output_rpm, size_t motor_id)
{
    // 统一经 °/s，保证 NH17 外圈位置 + 电机侧速度时使用 velocityEncoderFactor
    return degreePerSecToPulsePerSec(outputRpmToDegreePerSec(output_rpm), motor_id);
}

double MotorKinematics::pulsePerSecToOutputRpm(int32_t pulse_per_sec, size_t motor_id)
{
    return degreePerSecToOutputRpm(pulsePerSecToDegreePerSec(pulse_per_sec, motor_id));
}

double MotorKinematics::rawTorqueToMotorTorque(int16_t torque_raw, size_t motor_id)
{
    const auto& p = get(motor_id);
    if (useCurrentBasedTorqueFormula(motor_id)) {
        return torque_raw / 1000.0 * p.max_current_ma / 1000.0 * p.torque_constant_kt;
    }
    return torque_raw / 1000.0 * p.rated_torque_motor;
}

double MotorKinematics::rawTorqueToOutputTorque(int16_t torque_raw, size_t motor_id)
{
    return torque_raw / 1000.0 * outputTorqueScale(motor_id);
}

int16_t MotorKinematics::outputTorqueToRaw(double output_torque_nm, size_t motor_id)
{
    const double scale = outputTorqueScale(motor_id);
    if (scale <= 0.0) {
        return 0;
    }
    const double permille = output_torque_nm / scale * 1000.0;
    const double clamped = std::clamp(permille, -32768.0, 32767.0);
    return static_cast<int16_t>(std::lround(clamped));
}

int16_t MotorKinematics::currentAmpereToRaw(double current_a, size_t motor_id)
{
    const auto& p = get(motor_id);
    if (useCurrentBasedTorqueFormula(motor_id)) {
        const double imax_a = p.max_current_ma / 1000.0;
        if (imax_a <= 0.0) {
            return 0;
        }
        const double permille = current_a / imax_a * 1000.0;
        return static_cast<int16_t>(std::lround(std::clamp(permille, -32768.0, 32767.0)));
    }
    // τ_out ≈ i · Kt · torque_gear_ratio · η（与 outputTorqueScale 一致时 η 已含）
    const double output_nm =
        current_a * p.torque_constant_kt * p.torque_gear_ratio * p.gear_efficiency;
    return outputTorqueToRaw(output_nm, motor_id);
}

double MotorKinematics::rawToCurrentAmpere(int16_t torque_raw, size_t motor_id)
{
    const auto& p = get(motor_id);
    if (useCurrentBasedTorqueFormula(motor_id)) {
        return torque_raw / 1000.0 * p.max_current_ma / 1000.0;
    }
    if (p.torque_constant_kt <= 0.0) {
        return 0.0;
    }
    const double motor_nm = rawTorqueToMotorTorque(torque_raw, motor_id);
    return motor_nm / p.torque_constant_kt;
}

double MotorKinematics::currentToOutputTorquePerAmp(size_t motor_id)
{
    const auto& p = get(motor_id);
    if (p.torque_constant_kt <= 0.0) {
        return 0.0;
    }
    return p.torque_constant_kt * p.torque_gear_ratio * p.gear_efficiency;
}

}  // namespace ethercat_joint
