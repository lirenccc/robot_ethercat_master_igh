#pragma once

#include <cstdint>

namespace ethercat_master_igh
{

enum class MotionPolicy : uint8_t
{
  ObservationOnly = 0,
  SupervisedMotion = 1,
  /**
   * Permit explicitly approved PREOP drive configuration, but never authorize
   * cyclic motion commands. This is intended for no-motion commissioning/HIL.
   */
  Commissioning = 2,
};

constexpr bool motionPolicyAllowsDriveConfiguration(
  MotionPolicy policy) noexcept
{
  return policy == MotionPolicy::Commissioning ||
    policy == MotionPolicy::SupervisedMotion;
}

constexpr bool motionPolicyAuthorizesCommands(
  MotionPolicy policy, bool startup_evidence_passed) noexcept
{
  return policy == MotionPolicy::SupervisedMotion &&
    startup_evidence_passed;
}

}  // namespace ethercat_master_igh
