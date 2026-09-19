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

void ReferenceLimiter::validateState(const JointState& state) const {
  const Eigen::Index n = limits_.q_lower.size();
  requireSizeAndFinite(state.q, n, "state.q");
  requireSizeAndFinite(state.dq, n, "state.dq");
  if ((state.q.array() < limits_.q_lower.array()).any() ||
      (state.q.array() > limits_.q_upper.array()).any()) {
    throw std::out_of_range("measured joint position exceeds safety limits");
  }
  if ((state.dq.array().abs() > limits_.dq_max.array()).any()) {
    throw std::out_of_range("measured joint velocity exceeds safety limits");
  }
}

JointReference ReferenceLimiter::limit(
    const JointReference& reference) const {
  const Eigen::Index n = limits_.q_lower.size();
  requireSizeAndFinite(reference.q, n, "reference.q");
  requireSizeAndFinite(reference.dq, n, "reference.dq");
  requireSizeAndFinite(reference.ddq, n, "reference.ddq");

  JointReference result = reference;
  result.q = result.q.cwiseMax(limits_.q_lower).cwiseMin(limits_.q_upper);
  result.dq = result.dq.cwiseMax(-limits_.dq_max).cwiseMin(limits_.dq_max);
  result.ddq =
      result.ddq.cwiseMax(-limits_.ddq_max).cwiseMin(limits_.ddq_max);
  return result;
}

}  // namespace daedalus
