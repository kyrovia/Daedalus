#include "daedalus/control/joint_impedance_controller.hpp"

#include <stdexcept>
#include <utility>

#include "daedalus/common/vector_require.hpp"
#include "daedalus/control/control_validation.hpp"

namespace daedalus {

JointImpedanceController::JointImpedanceController(
    std::shared_ptr<const PinocchioModel> model, JointImpedanceConfig config)
    : model_(std::move(model)),
      context_(model_ ? model_->createContext()
                      : throw std::invalid_argument("model must not be null")),
      config_(std::move(config)),
      gravity_workspace_(model_->nv()) {
  requireNonnegative(config_.stiffness, model_->nv(), "stiffness");
  requireNonnegative(config_.damping, model_->nv(), "damping");
}

ControlResult JointImpedanceController::compute(
    const JointState& state, const JointReference& reference,
    JointVector& torque) const noexcept {
  if (!model_) {
    return {ControlStatus::kNotInitialized};
  }
  if (torque.size() != model_->nv()) {
    return {ControlStatus::kInvalidDimension};
  }
  torque.setZero();
  ControlStatus status = validateJointState(state, model_->nq(), model_->nv());
  if (status == ControlStatus::kOk) {
    status = validateJointReference(reference, model_->nq(), model_->nv());
  }
  if (status != ControlStatus::kOk) {
    return {status};
  }
  const ControlResult gravity_result =
      model_->gravityRealtime(context_, state.q, gravity_workspace_);
  if (!gravity_result) {
    return gravity_result;
  }
  for (Eigen::Index index = 0; index < torque.size(); ++index) {
    torque[index] =
        config_.stiffness[index] * (reference.q[index] - state.q[index]) +
        config_.damping[index] * (reference.dq[index] - state.dq[index]) +
        gravity_workspace_[index];
  }
  return {validateTorqueOutput(torque, model_->nv())};
}

JointVector JointImpedanceController::compute(
    const JointState& state, const JointReference& reference) const {
  JointVector torque(model_->nv());
  const ControlResult result = compute(state, reference, torque);
  if (!result) {
    throw std::runtime_error(controlStatusMessage(result.status));
  }
  return torque;
}

}  // namespace daedalus
