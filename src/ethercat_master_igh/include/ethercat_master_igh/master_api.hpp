#pragma once

/**
 * Stable, ROS-decoupled master surface for robot_rt_task_control adapters.
 * Maps to: init / map_joints / cycle / shutdown (see ETHERCAT_INTEGRATION.md).
 *
 * Hard-RT: Timing+Job own PDO exchange; Master::cycle() only pushes setpoints.
 * AxisConfig identity fields match ethercat_master_ecmaster (no nested MotorConfig).
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ethercat_master_igh/motion_policy.hpp"

namespace ethercat_joint
{
class EtherCATServo;
}

namespace ethercat_master_igh
{

enum class PdoLayout : uint8_t
{
  Unknown = 0,
  JointModule = 1,
  Gateway = 2,
  CoolDriveJmdt = 3,
};

struct AxisConfig
{
  std::string joint_name;
  uint16_t alias{0};
  uint16_t position{0};
  uint32_t vendor_id{0};
  uint32_t product_code{0};
  std::string model_id;
  PdoLayout pdo_layout{PdoLayout::Unknown};
};

enum class AxisConfigError : uint8_t
{
  None = 0,
  AxisCountOutOfRange,
  EmptyJointName,
  MissingIdentity,
  DuplicateJointName,
  DuplicateBusAddress,
};

inline AxisConfigError validate_axis_configs(
  const std::vector<AxisConfig> & axes) noexcept
{
  if (axes.empty() || axes.size() > 7U) {
    return AxisConfigError::AxisCountOutOfRange;
  }
  for (std::size_t axis = 0; axis < axes.size(); ++axis) {
    const auto & candidate = axes[axis];
    if (candidate.joint_name.empty()) {
      return AxisConfigError::EmptyJointName;
    }
    if (candidate.vendor_id == 0U || candidate.product_code == 0U) {
      return AxisConfigError::MissingIdentity;
    }
    for (std::size_t previous = 0; previous < axis; ++previous) {
      if (axes[previous].joint_name == candidate.joint_name) {
        return AxisConfigError::DuplicateJointName;
      }
      if (axes[previous].alias == candidate.alias &&
        axes[previous].position == candidate.position)
      {
        return AxisConfigError::DuplicateBusAddress;
      }
    }
  }
  return AxisConfigError::None;
}

struct AxisCommand
{
  double position{0.0};  // rad; non-finite → skip (leave last target)
  double velocity{0.0};  // rad/s; non-finite → skip
  double effort{0.0};    // Nm; applied only when this cycle requests CST (see cycle())
  bool enable{true};     // always applied each cycle (true=request enable FSM)
  /// CiA402 mode; 0 = do not change this cycle
  int8_t operation_mode{0};
};

struct AxisState
{
  double position{0.0};  // rad
  double velocity{0.0};  // rad/s
  double effort{0.0};    // Nm
  bool enabled{false};
  bool fault{false};
  uint16_t status_word{0};
  uint16_t error_code{0};
};

/**
 * Stable, SDK-independent health snapshot for an upper-layer safety
 * supervisor.  Counters are monotonic for one Master lifetime.
 */
struct MasterHealth
{
  bool initialized{false};
  bool operational{false};
  bool job_thread_running{false};
  bool working_counter_complete{false};
  bool communication_fault{false};
  bool link_connected{false};
  bool safe_output_active{true};
  bool observation_only{true};
  bool startup_evidence_passed{false};
  bool motion_commands_authorized{false};
  bool motion_reenable_allowed{false};
  uint32_t last_fault_notify_code{0};
  uint32_t last_job_error_code{0};
  uint64_t job_error_count{0};
  int32_t unexpected_operation_enabled_axis{-1};
  uint64_t cycles{0};
  uint64_t rx_fail_cycles{0};
  int32_t consecutive_rx_failures{0};
  uint64_t current_lateness_ns{0};
  uint64_t current_execution_ns{0};
  uint64_t max_lateness_ns{0};
  uint64_t max_execution_ns{0};
  uint64_t deadline_miss_count{0};
  uint64_t skipped_slots{0};
  bool deadline_met{false};
  bool dc_status_valid{false};
  bool dc_in_sync{false};
  int32_t dc_deviation_ns{0};
  uint64_t max_dc_deviation_ns{0};
  uint64_t dc_out_of_sync_count{0};
  uint32_t dc_out_of_sync_consecutive{0};
  uint32_t dc_out_of_sync_window{0};
};

class Master
{
public:
  explicit Master(
    unsigned int master_index = 0,
    MotionPolicy policy = MotionPolicy::ObservationOnly);
  ~Master();

  Master(const Master &) = delete;
  Master & operator=(const Master &) = delete;

  bool init(std::string & error);
  bool map_joints(const std::vector<AxisConfig> & axes, std::string & error);
  /// activate() then start hard-RT Job.
  /// Startup evidence gate and external CST command-freshness watchdog are implemented.
  bool start(std::string & error);

  /**
   * One application period: push setpoints + sample cached state.
   * PDO RX/TX runs in the IgH Job thread.
   * On comm_fault: do not inject commands; return faulted states.
   * Command injection requires motion_commands_authorized_ and !safe_output_required.
   */
  bool cycle(
    const AxisCommand * commands,
    std::size_t command_count,
    AxisState * states,
    std::size_t state_count,
    std::string & error);

  void shutdown();

  /**
   * Safety reset: clear comm_fault latch and arm healthy dwell (safe-output stays
   * asserted until release_safe_output). Does not auto re-enable.
   */
  bool request_safety_reset(std::string & error);
  /**
   * Explicit disabled-state CiA402 Fault Reset (profile control word, JMDT=0x80).
   * Requires Job OP + safe-output + all axes disabled.
   */
  bool request_fault_reset(uint8_t axis_id) noexcept;
  void request_safe_output() noexcept;
  bool release_safe_output(std::string & error);

  /**
   * When the adapter's try_to_lock fails, zero armed external CST/CSV for this
   * beat without refreshing the command deadline (contention fallback).
   */
  void apply_command_contention_fallback();

  bool is_mapped() const { return mapped_; }
  bool is_running() const { return running_; }
  /// True after healthy dwell completes post-reset (or initial allow). Gate for setEnable(true).
  bool motion_reenable_allowed() const;
  MasterHealth health() const noexcept;
  std::size_t axis_count() const { return axes_.size(); }
  const std::vector<std::string> & joint_names() const { return joint_names_; }

  ethercat_joint::EtherCATServo & servo();
  const ethercat_joint::EtherCATServo & servo() const;

private:
  std::unique_ptr<ethercat_joint::EtherCATServo> servo_;
  std::vector<AxisConfig> axes_;
  std::vector<std::string> joint_names_;
  bool initialized_{false};
  bool mapped_{false};
  bool running_{false};
  MotionPolicy motion_policy_{MotionPolicy::ObservationOnly};
  bool startup_evidence_passed_{false};
  bool motion_commands_authorized_{false};
};

}  // namespace ethercat_master_igh
