/**
 * @file motor_kinematics.hpp
 * @brief 新奇关节模组等单位换算（见 motor_profile 与 motor_kinematics 实现）
 */

#ifndef MOTOR_KINEMATICS_HPP
#define MOTOR_KINEMATICS_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace ethercat_joint {

/** 用户输入/显示一律为角度制（°）；文档弧度公式见 radianToPulse */
constexpr double kDegreesPerRevolution = 360.0;
constexpr double kRadiansPerRevolution = 6.283185307179586;  // 2π，与文档 6.28 一致

/**
 * @brief 单关节运动学参数
 *
 * 位置（角度制，输出端 °）：
 *   内圈 PDO：Cnt = deg/360 × gear_ratio × encoder_resolution
 *   外圈 PDO：Cnt = deg/360 × encoder_resolution
 * 速度（0x606C / 0x60FF，输出端 °/s）：
 *   默认与位置同一 encoderFactor；
 *   NH17：即便 0x2016=0（位置外圈），速度仍为电机侧 cnt/s
 *   → Cnt/s = deg/s/360 × gear_ratio × motor_encoder_resolution
 * 力矩（输出端 Nm）：
 *   输出力矩 = permille/1000 × rated_torque_motor × torque_gear_ratio
 *   （NH17 电流法：Imax × kt × torque_gear_ratio × η）
 * 弧度制（仅文档对照，Web/ROS 服务不使用）：
 *   内圈 Cnt = rad/6.28 × gear_ratio × encoder_resolution
 */
struct MotorKinematicsParams {
    double gear_ratio = 101.0;           // 位置内圈 / 力矩 / 电机侧速度换算
    double torque_gear_ratio = 101.0;    // 力矩换算（电机端 permille → 关节输出端 Nm）
    double encoder_resolution = 65536.0;   // 位置 PDO：内圈 16bit；外圈 18bit=262144
    double motor_encoder_resolution = 65536.0;  // 电机侧编码器（速度/电机 RPM）
    bool output_side_encoder = false;      // true=位置 PDO 为输出端计数
    /** NH17：位置可切外圈，但 0x606C/0x60FF 始终为电机侧 cnt/s */
    bool velocity_on_motor_encoder = false;
    double rated_torque_motor = 0.297;       // 旧公式：电机端额定力矩 (Nm)，千分比基准
    double max_current_ma = 4602.0;          // SDO 0x6075，NH17 出厂 Q 轴电流限制
    double torque_constant_kt = 0.030;       // 扭矩常数 (N·m/A)
    double gear_efficiency = 0.6;            // 减速机输出效率 η，文档建议 60~70%
    double position_offset_deg = 0.0;
    double joint_direction = 1.0;
};

class MotorKinematics {
public:
    static void setParams(const std::vector<MotorKinematicsParams>& params);
    static const MotorKinematicsParams& get(size_t motor_id);

    static double pulseToDegree(int32_t pulse, size_t motor_id = 0);
    static int32_t degreeToPulse(double degree, size_t motor_id = 0);
    static int32_t radianToPulse(double radian, size_t motor_id = 0);
    static double pulseToRadian(int32_t pulse, size_t motor_id = 0);
    static double pulsePerSecToDegreePerSec(int32_t pulse_per_sec, size_t motor_id = 0);
    static int32_t degreePerSecToPulsePerSec(double degree_per_sec, size_t motor_id = 0);

    /** @brief 输出端 RPM ↔ °/s（docx 速度换算配套） */
    static double outputRpmToDegreePerSec(double output_rpm);
    static double degreePerSecToOutputRpm(double degree_per_sec);

    /** @brief 电机端 RPM ↔ cnt/s（docx: cnt/s = rpm/60 × 编码器分辨率） */
    static int32_t motorRpmToPulsePerSec(double motor_rpm, size_t motor_id = 0);
    static double pulsePerSecToMotorRpm(int32_t pulse_per_sec, size_t motor_id = 0);

    /** @brief 输出端 RPM ↔ cnt/s（经 °/s，使用 velocityEncoderFactor） */
    static int32_t outputRpmToPulsePerSec(double output_rpm, size_t motor_id = 0);
    static double pulsePerSecToOutputRpm(int32_t pulse_per_sec, size_t motor_id = 0);

    /** @brief 根据 0x2016 位置模式推断内/外圈（65536=内圈，262144=外圈） */
    static bool inferOutputSideEncoder(double encoder_resolution);

    /** @brief 位置 PDO 计数因子（deg/360×factor→cnt） */
    static double encoderFactor(size_t motor_id = 0);
    /** @brief 速度 PDO 计数因子（°/s → cnt/s；NH17 外圈位置时仍为电机侧） */
    static double velocityEncoderFactor(size_t motor_id = 0);
    static std::string describe(size_t motor_id = 0);

    static double rawTorqueToMotorTorque(int16_t torque_raw, size_t motor_id = 0);
    static double rawTorqueToOutputTorque(int16_t torque_raw, size_t motor_id = 0);
    static int16_t outputTorqueToRaw(double output_torque_nm, size_t motor_id = 0);

    /**
     * @brief 电机 q 轴电流 (A) ↔ CST 千分比
     * 电流法：1000‰ ≈ max_current_ma/1000 A；否则经 τ=Kt·i 再换算输出力矩千分比
     */
    static int16_t currentAmpereToRaw(double current_a, size_t motor_id = 0);
    static double rawToCurrentAmpere(int16_t torque_raw, size_t motor_id = 0);

    /** @brief 电机电流 1 A → 关节输出端力矩 (Nm)，即 K_t·N·η */
    static double currentToOutputTorquePerAmp(size_t motor_id = 0);

    /** @brief 千分比 → 负载端力矩系数 (Nm @ permille=1000) */
    static double outputTorqueScale(size_t motor_id = 0);

private:
    static bool useCurrentBasedTorqueFormula(size_t motor_id);
};

}  // namespace ethercat_joint

#endif
