#include "daedalus/safety/torque_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "daedalus/common/vector_require.hpp"

namespace daedalus {

TorqueFilter::TorqueFilter(JointVector tau_max, JointVector tau_rate_max)
    : tau_max_(std::move(tau_max)),
      tau_rate_max_(std::move(tau_rate_max)) {
  if (tau_max_.size() == 0 || tau_rate_max_.size() != tau_max_.size()) {
    throw std::invalid_argument("torque limits have incompatible sizes");
  }
  requirePositive(tau_max_, tau_max_.size(), "tau_max");
  requirePositive(tau_rate_max_, tau_max_.size(), "tau_rate_max");
}

void TorqueFilter::reset(const JointVector& initial_tau) {
  const ControlResult result = resetRealtime(initial_tau);
  if (result.status == ControlStatus::kInvalidDimension ||
      result.status == ControlStatus::kNonFiniteInput) {
    throw std::invalid_argument(controlStatusMessage(result.status));
  }
  if (!result) {
    throw std::out_of_range("initial torque exceeds absolute limits");
  }
}

ControlResult TorqueFilter::resetRealtime(
    const JointVector& initial_tau) noexcept {
  if (initial_tau.size() != tau_max_.size()) {
    return {ControlStatus::kInvalidDimension};
  }
  if (!initial_tau.allFinite()) {
    return {ControlStatus::kNonFiniteInput};
  }
  if ((initial_tau.array().abs() > tau_max_.array()).any()) {
    return {ControlStatus::kStateLimitViolation};
  }
  previous_tau_ = initial_tau;
  initialized_ = true;
  return {};
}

ControlResult TorqueFilter::filterRealtime(
    const JointVector& raw_tau, const double dt,
    JointVector& filtered) noexcept {
  if (!initialized_) {
    return {ControlStatus::kNotInitialized};
  }
  if (raw_tau.size() != tau_max_.size() ||
      filtered.size() != tau_max_.size()) {
    return {ControlStatus::kInvalidDimension};
  }
  if (!raw_tau.allFinite()) {
    return {ControlStatus::kNonFiniteOutput};
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    return {ControlStatus::kInvalidTimeStep};
  }

  for (Eigen::Index index = 0; index < raw_tau.size(); ++index) {
    const double saturated =
        std::clamp(raw_tau[index], -tau_max_[index], tau_max_[index]);
    const double max_delta = tau_rate_max_[index] * dt;
    previous_tau_[index] =
        std::clamp(saturated, previous_tau_[index] - max_delta,
                   previous_tau_[index] + max_delta);
    filtered[index] = previous_tau_[index];
  }
  return {};
}

JointVector TorqueFilter::filter(const JointVector& raw_tau, const double dt) {
  JointVector result = JointVector::Zero(tau_max_.size());
  const ControlResult status = filterRealtime(raw_tau, dt, result);
  if (status.status == ControlStatus::kNotInitialized) {
    throw std::logic_error(controlStatusMessage(status.status));
  }
  if (!status) {
    throw std::invalid_argument(controlStatusMessage(status.status));
  }
  return result;
}

const JointVector& TorqueFilter::previousTorque() const noexcept {
  return previous_tau_;
}

}  // namespace daedalus
