#include "daedalus/control/joint_impedance_controller.hpp"

#include <stdexcept>
#include <utility>

#include "daedalus/common/vector_require.hpp"

namespace daedalus {

JointImpedanceController::JointImpedanceController(
    std::shared_ptr<const PinocchioModel> model, JointImpedanceConfig config)
    : model_(std::move(model)), config_(std::move(config)) {
  if (!model_) {
    throw std::invalid_argument("model must not be null");
  }
  requireNonnegative(config_.stiffness, model_->nv(), "stiffness");
  requireNonnegative(config_.damping, model_->nv(), "damping");
}

JointVector JointImpedanceController::compute(
    const JointState& state, const JointReference& reference) const {
  return config_.stiffness.cwiseProduct(reference.q - state.q) +
         config_.damping.cwiseProduct(reference.dq - state.dq) +
         model_->gravity(state.q);
}

}  // namespace daedalus
