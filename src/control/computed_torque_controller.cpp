#include "daedalus/control/computed_torque_controller.hpp"

#include <stdexcept>
#include <utility>

#include "daedalus/common/vector_require.hpp"
#include "daedalus/control/control_validation.hpp"

namespace daedalus {

ComputedTorqueController::ComputedTorqueController(
    std::shared_ptr<const PinocchioModel> model, ComputedTorqueConfig config)
    : model_(std::move(model)),
      context_(model_ ? model_->createContext()
                      : throw std::invalid_argument("model must not be null")),
      config_(std::move(config)),
      ddq_command_(model_->nv()) {
  requireNonnegative(config_.kp, model_->nv(), "kp");
  requireNonnegative(config_.kd, model_->nv(), "kd");
}

ControlResult ComputedTorqueController::compute(
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
  for (Eigen::Index index = 0; index < ddq_command_.size(); ++index) {
    ddq_command_[index] =
        reference.ddq[index] +
        config_.kp[index] * (reference.q[index] - state.q[index]) +
        config_.kd[index] * (reference.dq[index] - state.dq[index]);
  }
  return model_->inverseDynamicsRealtime(
      context_, state.q, state.dq, ddq_command_, torque);
}

JointVector ComputedTorqueController::compute(
    const JointState& state, const JointReference& reference) const {
  JointVector torque(model_->nv());
  const ControlResult result = compute(state, reference, torque);
  if (!result) {
    throw std::runtime_error(controlStatusMessage(result.status));
  }
  return torque;
}

}  // namespace daedalus
