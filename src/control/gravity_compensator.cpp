#include "daedalus/control/gravity_compensator.hpp"

#include <stdexcept>
#include <utility>

namespace daedalus {

GravityCompensator::GravityCompensator(
    std::shared_ptr<const PinocchioModel> model)
    : model_(std::move(model)) {
  if (!model_) {
    throw std::invalid_argument("model must not be null");
  }
}

JointVector GravityCompensator::compute(
    const JointState& state, const JointReference& reference) const {
  static_cast<void>(reference);
  return model_->gravity(state.q);
}

}  // namespace daedalus
