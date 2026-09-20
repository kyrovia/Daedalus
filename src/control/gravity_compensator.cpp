#include "daedalus/control/gravity_compensator.hpp"

#include <stdexcept>
#include <utility>

#include "daedalus/control/control_validation.hpp"

namespace daedalus {

GravityCompensator::GravityCompensator(
    std::shared_ptr<const PinocchioModel> model)
    : model_(std::move(model)),
      context_(model_ ? model_->createContext()
                      : throw std::invalid_argument("model must not be null")) {}

ControlResult GravityCompensator::compute(
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
  return model_->gravityRealtime(context_, state.q, torque);
}

JointVector GravityCompensator::compute(
    const JointState& state, const JointReference& reference) const {
  JointVector torque(model_->nv());
  const ControlResult result = compute(state, reference, torque);
  if (!result) {
    throw std::runtime_error(controlStatusMessage(result.status));
  }
  return torque;
}

}  // namespace daedalus
