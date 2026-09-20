#include "daedalus/safety/reference_limiter.hpp"

#include <stdexcept>
#include <utility>

#include "daedalus/common/vector_require.hpp"

namespace daedalus {

ReferenceLimiter::ReferenceLimiter(SafetyLimits limits)
    : limits_(std::move(limits)) {
  const Eigen::Index n = limits_.q_lower.size();
  if (n == 0) {
    throw std::invalid_argument("safety limits must not be empty");
  }
  requireSizeAndFinite(limits_.q_lower, n, "q_lower");
  requireSizeAndFinite(limits_.q_upper, n, "q_upper");
  if ((limits_.q_lower.array() >= limits_.q_upper.array()).any()) {
    throw std::invalid_argument("q_lower must be smaller than q_upper");
  }
  requirePositive(limits_.dq_max, n, "dq_max");
  requirePositive(limits_.ddq_max, n, "ddq_max");
  requirePositive(limits_.tau_max, n, "tau_max");
  requirePositive(limits_.tau_rate_max, n, "tau_rate_max");
}

int ReferenceLimiter::dof() const noexcept {
  return static_cast<int>(limits_.q_lower.size());
}

ControlResult ReferenceLimiter::validateStateRealtime(
    const JointState& state) const noexcept {
  const Eigen::Index n = limits_.q_lower.size();
  if (state.q.size() != n || state.dq.size() != n) {
    return {ControlStatus::kInvalidDimension};
  }
  if (!state.q.allFinite() || !state.dq.allFinite()) {
    return {ControlStatus::kNonFiniteInput};
  }
  if ((state.q.array() < limits_.q_lower.array()).any() ||
      (state.q.array() > limits_.q_upper.array()).any() ||
      (state.dq.array().abs() > limits_.dq_max.array()).any()) {
    return {ControlStatus::kStateLimitViolation};
  }
  return {};
}

ControlResult ReferenceLimiter::limitRealtime(
    const JointReference& reference, JointReference& limited) const noexcept {
  const Eigen::Index n = limits_.q_lower.size();
  if (reference.q.size() != n || reference.dq.size() != n ||
      reference.ddq.size() != n || limited.q.size() != n ||
      limited.dq.size() != n || limited.ddq.size() != n) {
    return {ControlStatus::kInvalidDimension};
  }
  if (!reference.q.allFinite() || !reference.dq.allFinite() ||
      !reference.ddq.allFinite()) {
    return {ControlStatus::kNonFiniteInput};
  }
  limited.q = reference.q.cwiseMax(limits_.q_lower).cwiseMin(limits_.q_upper);
  limited.dq =
      reference.dq.cwiseMax(-limits_.dq_max).cwiseMin(limits_.dq_max);
  limited.ddq =
      reference.ddq.cwiseMax(-limits_.ddq_max).cwiseMin(limits_.ddq_max);
  return {};
}

void ReferenceLimiter::validateState(const JointState& state) const {
  const ControlResult result = validateStateRealtime(state);
  if (result.status == ControlStatus::kInvalidDimension ||
      result.status == ControlStatus::kNonFiniteInput) {
    throw std::invalid_argument(controlStatusMessage(result.status));
  }
  if (!result) {
    throw std::out_of_range("measured joint position exceeds safety limits");
  }
}

JointReference ReferenceLimiter::limit(
    const JointReference& reference) const {
  JointReference result{
      JointVector::Zero(limits_.q_lower.size()),
      JointVector::Zero(limits_.q_lower.size()),
      JointVector::Zero(limits_.q_lower.size())};
  const ControlResult status = limitRealtime(reference, result);
  if (!status) {
    throw std::invalid_argument(controlStatusMessage(status.status));
  }
  return result;
}

}  // namespace daedalus
