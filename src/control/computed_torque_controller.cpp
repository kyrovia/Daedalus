#include "daedalus/control/computed_torque_controller.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace daedalus {
namespace {

void requireGain(const JointVector& gain, const int dof, const char* name) {
  if (gain.size() != dof) {
    throw std::invalid_argument(std::string(name) + " has incorrect size");
  }
  if (!gain.allFinite() || (gain.array() < 0.0).any()) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and nonnegative");
  }
}

}  // namespace

ComputedTorqueController::ComputedTorqueController(
    std::shared_ptr<const PinocchioModel> model, ComputedTorqueConfig config)
    : model_(std::move(model)), config_(std::move(config)) {
  if (!model_) {
    throw std::invalid_argument("model must not be null");
  }
  requireGain(config_.kp, model_->nv(), "kp");
  requireGain(config_.kd, model_->nv(), "kd");
}

JointVector ComputedTorqueController::compute(
    const JointState& state, const JointReference& reference) const {
  const JointVector ddq_command =
      reference.ddq +
      config_.kp.cwiseProduct(reference.q - state.q) +
      config_.kd.cwiseProduct(reference.dq - state.dq);
  return model_->inverseDynamics(state.q, state.dq, ddq_command);
}

}  // namespace daedalus
