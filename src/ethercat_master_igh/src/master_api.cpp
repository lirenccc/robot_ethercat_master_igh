#include "ethercat_master_igh/master_api.hpp"

#include "ethercat_joint/master/igh/igh_master_runtime.hpp"
#include "ethercat_joint/motor/motor_kinematics.hpp"
#include "ethercat_joint/servo/cia402.hpp"

#include <cmath>

namespace ethercat_master_igh
{
namespace
{
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}  // namespace

Master::Master(unsigned int master_index)
: servo_(std::make_unique<ethercat_joint::EtherCATServo>(master_index))
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
  // IgH master request happens inside EtherCATServo::initialize.
  initialized_ = true;
  error.clear();
  return true;
}

bool Master::map_joints(const std::vector<AxisConfig> & axes, std::string & error)
{
  if (axes.empty()) {
    error = "map_joints: empty axis list";
    return false;
  }
  std::vector<ethercat_joint::MotorConfig> motors;
  motors.reserve(axes.size());
  joint_names_.clear();
  joint_names_.reserve(axes.size());
  for (const auto & axis : axes) {
    if (axis.joint_name.empty()) {
      error = "map_joints: joint_name required";
      return false;
    }
    auto motor = axis.motor;
    if (motor.name.empty()) {
      motor.name = axis.joint_name;
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
  if (!mapped_) {
    error = "start: call map_joints first";
    return false;
  }
  if (!servo_->activate()) {
    error = "IgH EtherCATServo::activate failed";
    return false;
  }
  (void)servo_->tryLoadKinematicsFromSdo();
  (void)servo_->initializePositionsFromSDO();

  if (!servo_->startupEvidencePassed()) {
    error = "IgH startup evidence gate failed (observation-only; enable refused)";
    servo_->deactivate();
    return false;
  }

  if (!ethercat_joint::IghMasterRuntime::instance().start(servo_.get())) {
    error = "IgH hard-RT Job start failed (check IGH_REQUIRE_REALTIME / privileges)";
    servo_->deactivate();
    return false;
  }

  const auto n = axes_.size();
  for (std::size_t i = 0; i < n; ++i) {
    servo_->setOperationMode(
      static_cast<uint8_t>(i), ethercat_joint::OperationMode::CYCLIC_SYNC_POSITION);
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

  // Job 拥有 PDO；此处只灌 setpoint + 读缓存
  if (servo_->commFault() || servo_->safeOutputRequired()) {
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
      st.status_word = 0;
    }
    error = "IgH communication fault (safe-output active)";
    return false;
  }

  for (std::size_t i = 0; i < n; ++i) {
    const auto & cmd = commands[i];
    const uint8_t id = static_cast<uint8_t>(i);
    // operation_mode==0 → leave mode. Effort when axis is CST or this beat requests CST.
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
        id, ethercat_joint::MotorKinematics::degreePerSecToPulsePerSec(deg_s, i), true,
        ethercat_joint::SetpointSource::External);
    }
    if (std::isfinite(cmd.effort) &&
      (servo_->getOperationMode(id) ==
        ethercat_joint::OperationMode::CYCLIC_SYNC_TORQUE ||
       cmd.operation_mode ==
        static_cast<int8_t>(ethercat_joint::OperationMode::CYCLIC_SYNC_TORQUE)))
    {
      const int16_t raw =
        ethercat_joint::MotorKinematics::outputTorqueToRaw(cmd.effort, i);
      servo_->setTargetTorque(id, raw, ethercat_joint::SetpointSource::External);
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
    } else {
      st.enabled = false;
      st.fault = false;
      st.status_word = 0;
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
  servo_->clearCommFault();
  error.clear();
  return true;
}

bool Master::motion_reenable_allowed() const
{
  return servo_ ? servo_->motionReenableAllowed() : false;
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
