#include "ethercat_master_igh/master_api.hpp"

#include "ethercat_joint/master/igh/igh_master_runtime.hpp"
#include "ethercat_joint/motor/motor_kinematics.hpp"
#include "ethercat_joint/servo/cia402.hpp"
#include "ethercat_joint/servo/ethercat_servo.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace ethercat_master_igh
{
namespace
{
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}  // namespace

Master::Master(unsigned int master_index, MotionPolicy policy)
: servo_(std::make_unique<ethercat_joint::EtherCATServo>(master_index)),
  motion_policy_(policy)
{
}

Master::~Master()
{
  shutdown();
}

bool Master::init(std::string & error)
{
  if (initialized_) {
    return true;
  }
  if (!servo_) {
    error = "init: Master instances cannot be restarted after shutdown";
    return false;
  }
  // IgH master request happens inside EtherCATServo::initialize.
  initialized_ = true;
  error.clear();
  return true;
}

bool Master::map_joints(const std::vector<AxisConfig> & axes, std::string & error)
{
  if (!servo_) {
    error = "map_joints: master has been shut down";
    return false;
  }
  const auto validation = validate_axis_configs(axes);
  if (validation != AxisConfigError::None) {
    error = "map_joints: invalid axis configuration (" +
      std::to_string(static_cast<unsigned>(validation)) + ")";
    return false;
  }
  std::vector<ethercat_joint::MotorConfig> motors;
  motors.reserve(axes.size());
  joint_names_.clear();
  joint_names_.reserve(axes.size());
  for (const auto & axis : axes) {
    ethercat_joint::MotorConfig motor;
    motor.alias = axis.alias;
    motor.position = axis.position;
    motor.vendor_id = axis.vendor_id;
    motor.product_code = axis.product_code;
    motor.name = axis.joint_name;
    motor.model_id = axis.model_id;
    switch (axis.pdo_layout) {
      case PdoLayout::JointModule:
        motor.pdo_layout = ethercat_joint::PdoLayout::JOINT_MODULE;
        break;
      case PdoLayout::Gateway:
        motor.pdo_layout = ethercat_joint::PdoLayout::GATEWAY;
        break;
      case PdoLayout::CoolDriveJmdt:
        motor.pdo_layout = ethercat_joint::PdoLayout::COOLDRIVE_JMDT;
        break;
      case PdoLayout::Unknown:
      default:
        motor.pdo_layout = ethercat_joint::PdoLayout::UNKNOWN;
        break;
    }
    motors.push_back(motor);
    joint_names_.push_back(axis.joint_name);
  }
  if (!servo_->initialize(motors)) {
    error = "IgH EtherCATServo::initialize failed";
    mapped_ = false;
    return false;
  }
  axes_ = axes;
  mapped_ = true;
  error.clear();
  return true;
}

bool Master::start(std::string & error)
{
  if (!mapped_ || !servo_) {
    error = "start: call map_joints first";
    return false;
  }
  // IgH: blocking mailbox SDO must run before ecrt_master_activate(). After activate,
  // SDO without a running cyclic Job can hang indefinitely (ecrt_master_sdo_upload).
  (void)servo_->tryLoadKinematicsFromSdo();
  (void)servo_->initializePositionsFromSDO();
  // 对齐天机：activate 只点着主站 + domain_pd；PREOP→OP 交给 Job 周期。
  if (!servo_->activate()) {
    error = "IgH EtherCATServo::activate failed (master/domain)";
    running_ = false;
    return false;
  }

  // DC 同步预热（activate 后、Job 前，对齐 EC-Master DCM BurstBulk + SettleTime）
  // Bypassed for NH17: Job thread handles natural DC sync.
  // Keeping call site for future re-enable.
  // if (!servo_->dcSyncWarmup(5000)) { ... }
  (void)servo_->dcSyncWarmup(5000);  // observation-only; don't gate

  startup_evidence_passed_ = servo_->startupEvidencePassed();
  motion_commands_authorized_ = motionPolicyAuthorizesCommands(
    motion_policy_, startup_evidence_passed_);
  servo_->requestSafeOutput();

  if (motion_policy_ == MotionPolicy::SupervisedMotion &&
    !startup_evidence_passed_)
  {
    error = "IgH startup evidence gate failed (observation-only; enable refused)";
    servo_->deactivate();
    running_ = false;
    return false;
  }

  if (!ethercat_joint::IghMasterRuntime::instance().start(servo_.get())) {
    error = "IgH hard-RT Job start failed (check IGH_REQUIRE_REALTIME / privileges)";
    servo_->deactivate();
    return false;
  }

  // Job 已跑：等关节进 OP（WC 完整）。超时再 fail-closed。
  constexpr auto kOpWait = std::chrono::seconds(60);  // extended for NH17 DC lock
  const auto op_deadline = std::chrono::steady_clock::now() + kOpWait;
  auto last_log = std::chrono::steady_clock::now();
  bool all_op = false;
  while (std::chrono::steady_clock::now() < op_deadline) {
    all_op = servo_->areAllSlavesInOP();
    if (all_op) {
      break;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - last_log >= std::chrono::milliseconds(500)) {
      last_log = now;
      const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - (op_deadline - kOpWait)).count();
      const auto diag = ethercat_joint::IghMasterRuntime::instance().jobCycleDiag();
      std::cout << "[IgH] OP wait @" << elapsed_ms
                << "ms after Job start (rx_ok=" << diag.last_rx_ok
                << ")" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!all_op) {
    error = "IgH slaves did not reach OP within 30s after Job start (WC/AL stuck)";
    std::cerr << "[IgH] " << error << std::endl;
    ethercat_joint::IghMasterRuntime::instance().stop();
    servo_->deactivate();
    running_ = false;
    return false;
  }
  std::cout << "[IgH] ✓ All slaves in OP after Job start" << std::endl;
  ethercat_joint::IghMasterRuntime::instance().setOperational(true);

  // 清除启动阶段的 safe-output latch：requestSafeOutput() 在 OP 前锁 comm_fault，
  // 进 OP 后必须 reset 一次，让 healthy_dwell 重新评估。
  servo_->requestSafetyReset();

  const auto n = axes_.size();
  if (motion_commands_authorized_) {
    for (std::size_t i = 0; i < n; ++i) {
      servo_->setOperationMode(
        static_cast<uint8_t>(i), ethercat_joint::OperationMode::CYCLIC_SYNC_POSITION);
    }
  }
  running_ = true;
  error.clear();
  return true;
}

bool Master::cycle(
  const AxisCommand * commands,
  std::size_t command_count,
  AxisState * states,
  std::size_t state_count,
  std::string & error)
{
  if (!running_ || !servo_) {
    error = "cycle: master not running";
    return false;
  }
  const std::size_t n = axes_.size();
  if (commands == nullptr || command_count < n) {
    error = "cycle: commands too short";
    return false;
  }
  if (states == nullptr || state_count < n) {
    error = "cycle: states too short";
    return false;
  }

  if (servo_->commFault()) {
    const auto motor_states = servo_->getMotorStates();
    for (std::size_t i = 0; i < n; ++i) {
      AxisState & st = states[i];
      const uint8_t id = static_cast<uint8_t>(i);
      const double pos_deg = ethercat_joint::MotorKinematics::pulseToDegree(
        servo_->getPosition(id), i);
      const double vel_deg_s = ethercat_joint::MotorKinematics::pulsePerSecToDegreePerSec(
        servo_->getVelocity(id), i);
      st.position = pos_deg * kDegToRad;
      st.velocity = vel_deg_s * kDegToRad;
      st.effort = 0.0;
      st.enabled = false;
      st.fault = true;
      if (i < motor_states.size()) {
        st.status_word = motor_states[i].status_word;
        st.error_code = motor_states[i].error_code;
      } else {
        st.status_word = 0;
        st.error_code = 0;
      }
    }
    error = "IgH communication fault (safe-output active)";
    return false;
  }

  if (motion_commands_authorized_ && !servo_->safeOutputRequired()) {
    for (std::size_t i = 0; i < n; ++i) {
      const auto & cmd = commands[i];
      const uint8_t id = static_cast<uint8_t>(i);
      if (cmd.operation_mode != 0) {
        servo_->setOperationMode(
          id, static_cast<ethercat_joint::OperationMode>(cmd.operation_mode));
      }
      (void)servo_->setEnable(id, cmd.enable);
      if (std::isfinite(cmd.position)) {
        const double deg = cmd.position * kRadToDeg;
        servo_->setTargetPosition(
          id, ethercat_joint::MotorKinematics::degreeToPulse(deg, i), true);
      }
      if (std::isfinite(cmd.velocity)) {
        const double deg_s = cmd.velocity * kRadToDeg;
        servo_->setTargetVelocity(
          id,
          ethercat_joint::MotorKinematics::degreePerSecToPulsePerSec(deg_s, i),
          true,
          ethercat_joint::SetpointSource::External);
      }
      if (std::isfinite(cmd.effort) &&
        servo_->getOperationMode(id) ==
        ethercat_joint::OperationMode::CYCLIC_SYNC_TORQUE)
      {
        const int16_t raw =
          ethercat_joint::MotorKinematics::outputTorqueToRaw(cmd.effort, i);
        servo_->setTargetTorque(id, raw, ethercat_joint::SetpointSource::External);
      }
    }
  }

  const auto motor_states = servo_->getMotorStates();
  for (std::size_t i = 0; i < n; ++i) {
    AxisState & st = states[i];
    const uint8_t id = static_cast<uint8_t>(i);
    const double pos_deg = ethercat_joint::MotorKinematics::pulseToDegree(
      servo_->getPosition(id), i);
    const double vel_deg_s = ethercat_joint::MotorKinematics::pulsePerSecToDegreePerSec(
      servo_->getVelocity(id), i);
    st.position = pos_deg * kDegToRad;
    st.velocity = vel_deg_s * kDegToRad;
    st.effort = ethercat_joint::MotorKinematics::rawTorqueToOutputTorque(
      servo_->getTorque(id), i);
    if (i < motor_states.size()) {
      st.enabled = motor_states[i].enabled;
      st.fault = motor_states[i].fault;
      st.status_word = motor_states[i].status_word;
      st.error_code = motor_states[i].error_code;
    } else {
      st.enabled = false;
      st.fault = false;
      st.status_word = 0;
      st.error_code = 0;
    }
  }

  error.clear();
  return true;
}

void Master::shutdown()
{
  running_ = false;
  if (servo_) {
    servo_->deactivate();
  }
  mapped_ = false;
  initialized_ = false;
}

bool Master::request_safety_reset(std::string & error)
{
  if (!servo_) {
    error = "request_safety_reset: no servo";
    return false;
  }
  const std::size_t n = axes_.size();
  for (std::size_t i = 0; i < n; ++i) {
    (void)servo_->setEnable(static_cast<uint8_t>(i), false);
  }
  servo_->requestSafetyReset();
  error.clear();
  return true;
}

bool Master::request_fault_reset(uint8_t axis_id) noexcept
{
  return running_ && servo_ && servo_->requestFaultReset(axis_id);
}

void Master::request_safe_output() noexcept
{
  if (servo_) {
    servo_->requestSafeOutput();
  }
}

bool Master::release_safe_output(std::string & error)
{
  if (!servo_ || !motion_commands_authorized_) {
    error = "release_safe_output: motion policy or startup evidence forbids motion";
    return false;
  }
  if (!servo_->releaseSafeOutput()) {
    error = "release_safe_output: communication latch or healthy dwell active";
    return false;
  }
  error.clear();
  return true;
}

bool Master::motion_reenable_allowed() const
{
  return servo_ && motion_commands_authorized_ &&
    servo_->motionReenableAllowed();
}

MasterHealth Master::health() const noexcept
{
  MasterHealth health;
  if (!servo_) {
    return health;
  }
  const auto diag = servo_->jobCycleDiag();
  health.initialized = initialized_;
  health.operational = servo_->areAllSlavesInOP();
  health.job_thread_running = servo_->isJobThreadRunning();
  health.working_counter_complete = diag.last_rx_ok;
  health.communication_fault = servo_->commFault();
  health.link_connected = health.operational;
  health.safe_output_active = servo_->safeOutputRequired();
  health.observation_only = !motion_commands_authorized_;
  health.startup_evidence_passed = startup_evidence_passed_;
  health.motion_commands_authorized = motion_commands_authorized_;
  health.motion_reenable_allowed =
    motion_commands_authorized_ && servo_->motionReenableAllowed();
  health.current_lateness_ns = diag.lateness_ns;
  health.current_execution_ns = diag.execution_ns;
  health.max_lateness_ns = diag.max_lateness_ns;
  health.max_execution_ns = diag.max_execution_ns;
  health.deadline_miss_count = diag.deadline_miss_count;
  health.skipped_slots = diag.skipped_slots;
  health.deadline_met = diag.deadline_met;
  health.dc_status_valid = diag.dc_status_valid;
  health.dc_in_sync = diag.dc_in_sync;
  health.dc_deviation_ns = diag.dc_deviation_ns;
  health.max_dc_deviation_ns = diag.max_dc_deviation_ns;
  health.dc_out_of_sync_count = diag.dc_out_of_sync_count;
  health.dc_out_of_sync_consecutive = diag.dc_out_of_sync_consecutive;
  health.dc_out_of_sync_window = diag.dc_out_of_sync_window;
  return health;
}

void Master::apply_command_contention_fallback()
{
  if (servo_) {
    servo_->applyCommandContentionFallback();
  }
}

ethercat_joint::EtherCATServo & Master::servo()
{
  return *servo_;
}

const ethercat_joint::EtherCATServo & Master::servo() const
{
  return *servo_;
}

}  // namespace ethercat_master_igh
