#include "daedalus/control/control_pipeline.hpp"

#include <stdexcept>
#include <utility>

namespace daedalus {

ControlPipeline::ControlPipeline(
    std::shared_ptr<const PinocchioModel> model, const SafetyLimits& limits,
    ComputedTorqueConfig computed_torque_config,
    JointImpedanceConfig impedance_config, const ControllerMode initial_mode,
    const JointVector& initial_tau)
    : model_(std::move(model)),
      gravity_(model_),
      computed_torque_(model_, std::move(computed_torque_config)),
      impedance_(model_, std::move(impedance_config)),
      reference_limiter_(limits),
      torque_filter_(limits.tau_max, limits.tau_rate_max),
      mode_(initial_mode) {
  if (!model_) {
    throw std::invalid_argument("model must not be null");
  }
  if (reference_limiter_.dof() != model_->nv()) {
    throw std::invalid_argument(
        "safety limits size must match the robot degrees of freedom");
  }
  torque_filter_.reset(initial_tau);
}

JointVector ControlPipeline::compute(
    const JointState& state, const JointReference& reference,
    const double dt) {
  reference_limiter_.validateState(state);
  const JointReference safe_reference = reference_limiter_.limit(reference);

  JointVector raw_tau;
  switch (mode_) {
    case ControllerMode::kGravityCompensation:
      raw_tau = gravity_.compute(state, safe_reference);
      break;
    case ControllerMode::kComputedTorque:
      raw_tau = computed_torque_.compute(state, safe_reference);
      break;
    case ControllerMode::kJointImpedance:
      raw_tau = impedance_.compute(state, safe_reference);
      break;
    default:
      throw std::logic_error("unknown controller mode");
  }
  return torque_filter_.filter(raw_tau, dt);
}

void ControlPipeline::setMode(
    const ControllerMode mode, const JointVector& current_commanded_tau) {
  torque_filter_.reset(current_commanded_tau);
  mode_ = mode;
}

ControllerMode ControlPipeline::mode() const noexcept {
  return mode_;
}

const JointVector& ControlPipeline::previousTorque() const noexcept {
  return torque_filter_.previousTorque();
}

}  // namespace daedalus
