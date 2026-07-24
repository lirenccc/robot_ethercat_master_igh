/**
 * @file csp_interpolation_period.hpp
 * @brief CiA402 0x60C2 插补周期编解码（纯函数，无硬件依赖）
 *
 * value * 10^exponent 秒；4 ms 规范编码为 value=4, exponent=-3。
 */

#ifndef ETHERCAT_JOINT_MOTOR_CSP_INTERPOLATION_PERIOD_HPP
#define ETHERCAT_JOINT_MOTOR_CSP_INTERPOLATION_PERIOD_HPP

#include <cstdint>
#include <optional>

namespace ethercat_joint {

struct CspInterpolationPeriod {
    uint8_t value = 0U;
    int8_t exponent = 0;
};

inline std::optional<CspInterpolationPeriod> encodeCspInterpolationPeriod(
    uint64_t period_ns) noexcept
{
    if (period_ns == 0U) {
        return std::nullopt;
    }
    uint64_t unit_ns = 1000000000ULL;
    for (int exponent = 0; exponent >= -9; --exponent) {
        if (period_ns % unit_ns == 0U) {
            const uint64_t value = period_ns / unit_ns;
            if (value >= 1U && value <= 255U) {
                return CspInterpolationPeriod{static_cast<uint8_t>(value),
                                              static_cast<int8_t>(exponent)};
            }
        }
        if (exponent > -9) {
            unit_ns /= 10U;
        }
    }
    return std::nullopt;
}

inline uint64_t decodeCspInterpolationPeriodNs(CspInterpolationPeriod period) noexcept
{
    if (period.value == 0U || period.exponent < -9 || period.exponent > 0) {
        return 0U;
    }
    uint64_t unit_ns = 1000000000ULL;
    for (int exponent = 0; exponent > period.exponent; --exponent) {
        unit_ns /= 10U;
    }
    return static_cast<uint64_t>(period.value) * unit_ns;
}

/** 驱动回读的 0x60C2 是否与总线周期一致（纳秒精确匹配）。 */
inline bool interpolationPeriodMatchesBus(uint8_t value,
                                          int8_t exponent,
                                          uint64_t bus_cycle_ns) noexcept
{
    const uint64_t decoded =
        decodeCspInterpolationPeriodNs(CspInterpolationPeriod{value, exponent});
    return decoded != 0U && decoded == bus_cycle_ns;
}

}  // namespace ethercat_joint

#endif  // ETHERCAT_JOINT_MOTOR_CSP_INTERPOLATION_PERIOD_HPP
