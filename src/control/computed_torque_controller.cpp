#include "daedalus/control/computed_torque_controller.hpp"

#include <stdexcept>
#include <utility>

#include "daedalus/common/vector_require.hpp"

namespace daedalus {

ComputedTorqueController::ComputedTorqueController(
    std::shared_ptr<const PinocchioModel> model, ComputedTorqueConfig config)
    : model_(std::move(model)), config_(std::move(config)) {
  if (!model_) {
    throw std::invalid_argument("model must not be null");
  }
  requireNonnegative(config_.kp, model_->nv(), "kp");
  requireNonnegative(config_.kd, model_->nv(), "kd");
}

//PD+加速度前馈
JointVector ComputedTorqueController::compute(
    const JointState& state, const JointReference& reference) const {
  const JointVector ddq_command =
      reference.ddq +
      config_.kp.cwiseProduct(reference.q - state.q) +
      config_.kd.cwiseProduct(reference.dq - state.dq);
  return model_->inverseDynamics(state.q, state.dq, ddq_command);
}

}  // namespace daedalus
