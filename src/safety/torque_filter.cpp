#include "daedalus/safety/torque_filter.hpp"

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
  requireSizeAndFinite(initial_tau, tau_max_.size(), "initial_tau");
  if ((initial_tau.array().abs() > tau_max_.array()).any()) {
    throw std::out_of_range("initial torque exceeds absolute limits");
  }
  previous_tau_ = initial_tau;
  initialized_ = true;
}

JointVector TorqueFilter::filter(const JointVector& raw_tau, const double dt) {
  if (!initialized_) {
    throw std::logic_error("torque filter must be reset before use");
  }
  requireSizeAndFinite(raw_tau, tau_max_.size(), "raw_tau");
  requirePositive(dt, "dt");

  const JointVector saturated =
      raw_tau.cwiseMax(-tau_max_).cwiseMin(tau_max_);
  const JointVector max_delta = tau_rate_max_ * dt;
  previous_tau_ =
      saturated.cwiseMax(previous_tau_ - max_delta)
          .cwiseMin(previous_tau_ + max_delta)
          .cwiseMax(-tau_max_)
          .cwiseMin(tau_max_);
  return previous_tau_;
}

const JointVector& TorqueFilter::previousTorque() const noexcept {
  return previous_tau_;
}

}  // namespace daedalus
