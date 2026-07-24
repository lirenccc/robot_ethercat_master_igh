#pragma once

/**
 * Stable, ROS-decoupled master surface for robot_rt_task_control adapters.
 * Maps to: init / map_joints / cycle / shutdown (see ETHERCAT_INTEGRATION.md).
 *
 * Hard-RT: Timing+Job own PDO exchange; Master::cycle() only pushes setpoints.
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ethercat_joint/servo/ethercat_servo.hpp"

namespace ethercat_master_igh
{

struct AxisConfig
{
  std::string joint_name;
  ethercat_joint::MotorConfig motor;
};

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
};

class Master
{
public:
  explicit Master(unsigned int master_index = 0);
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
   * On comm_fault / safe-output: do not inject commands; return faulted states.
   */
  bool cycle(
    const AxisCommand * commands,
    std::size_t command_count,
    AxisState * states,
    std::size_t state_count,
    std::string & error);

  void shutdown();

  /**
   * Safety reset: clear comm_fault / safe-output latch and arm healthy dwell.
   * Does not auto re-enable; call setEnable only after motion_reenable_allowed().
   */
  bool request_safety_reset(std::string & error);

  /**
   * When the adapter's try_to_lock fails, zero armed external CST/CSV for this
   * beat without refreshing the command deadline (contention fallback).
   */
  void apply_command_contention_fallback();

  bool is_mapped() const { return mapped_; }
  bool is_running() const { return running_; }
  /// True after healthy dwell completes post-reset (or initial allow). Gate for setEnable(true).
  bool motion_reenable_allowed() const;
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
};

}  // namespace ethercat_master_igh
