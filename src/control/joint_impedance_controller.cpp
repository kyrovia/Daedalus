#include "daedalus/control/joint_impedance_controller.hpp"

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

JointImpedanceController::JointImpedanceController(
    std::shared_ptr<const PinocchioModel> model, JointImpedanceConfig config)
    : model_(std::move(model)), config_(std::move(config)) {
  if (!model_) {
    throw std::invalid_argument("model must not be null");
  }
  requireGain(config_.stiffness, model_->nv(), "stiffness");
  requireGain(config_.damping, model_->nv(), "damping");
}

JointVector JointImpedanceController::compute(
    const JointState& state, const JointReference& reference) const {
  return config_.stiffness.cwiseProduct(reference.q - state.q) +
         config_.damping.cwiseProduct(reference.dq - state.dq) +
         model_->gravity(state.q);
}

}  // namespace daedalus
