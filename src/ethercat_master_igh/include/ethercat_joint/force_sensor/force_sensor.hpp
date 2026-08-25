#pragma once

#include <cstdint>

namespace ethercat_joint
{

/** 六维力传感器单拍采样（SI：N / N·m）。 */
struct ForceSensorSample
{
  uint16_t data_no{0};
  double fx{0.0};
  double fy{0.0};
  double fz{0.0};
  double mx{0.0};
  double my{0.0};
  double mz{0.0};
  bool valid{false};
};

/** M8126i32 PDO 0x6020：Fx…Mz = INT32 / divisor（宇立手册，工程单位 N / N·m）。 */
inline constexpr double kSriM8126Int32Divisor = 10000.0;

inline double sriM8126Int32ToSi(int32_t raw) noexcept
{
  return static_cast<double>(raw) / kSriM8126Int32Divisor;
}

}  // namespace ethercat_joint
