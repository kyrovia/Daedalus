#pragma once

#include "daedalus/control/control_status.hpp"
#include "daedalus/control/torque_command.hpp"
#include "daedalus/safety/reference_limiter.hpp"
#include "daedalus/safety/torque_filter.hpp"
#include "daedalus/types/cartesian_types.hpp"
#include "daedalus/types/safety_limits.hpp"

namespace daedalus {

struct ControlStepResult final {
  ControlStatus status{ControlStatus::kOk};
  const TorqueCommand* command{nullptr};

  [[nodiscard]] constexpr bool ok() const noexcept {
    return status == ControlStatus::kOk && command != nullptr;
  }

  [[nodiscard]] constexpr explicit operator bool() const noexcept {
    return ok();
  }
};

template <typename Controller>
class ControlPipeline final {
 public:
  ControlPipeline(Controller& controller, const SafetyLimits& limits)
      : controller_(&controller),
        reference_limiter_(limits),
        torque_filter_(limits.tau_max, limits.tau_rate_max),
        limited_reference_{
            JointVector::Zero(limits.q_lower.size()),
            JointVector::Zero(limits.q_lower.size()),
            JointVector::Zero(limits.q_lower.size())},
        raw_torque_(JointVector::Zero(limits.q_lower.size())),
        command_(limits.q_lower.size()) {
    torque_filter_.reset(JointVector::Zero(limits.q_lower.size()));
  }

  [[nodiscard]] ControlStepResult step(
      const JointState& state, const JointReference& reference,
      const double dt) noexcept {
    ControlResult result = reference_limiter_.validateStateRealtime(state);
    if (!result) {
      return {result.status, nullptr};
    }

    result = reference_limiter_.limitRealtime(reference, limited_reference_);
    if (!result) {
      return {result.status, nullptr};
    }

    result = controller_->compute(state, limited_reference_, raw_torque_);
    if (!result) {
      return {result.status, nullptr};
    }

    result = torque_filter_.filterRealtime(raw_torque_, dt, command_.torque_);
    if (!result) {
      return {result.status, nullptr};
    }
    return {ControlStatus::kOk, &command_};
  }

  [[nodiscard]] ControlStepResult step(
      const JointState& state, const CartesianReference& reference,
      const double dt) noexcept {
    ControlResult result = reference_limiter_.validateStateRealtime(state);
    if (!result) {
      return {result.status, nullptr};
    }

    result = controller_->compute(state, reference, raw_torque_);
    if (!result) {
      return {result.status, nullptr};
    }

    result = torque_filter_.filterRealtime(raw_torque_, dt, command_.torque_);
    if (!result) {
      return {result.status, nullptr};
    }
    return {ControlStatus::kOk, &command_};
  }

  [[nodiscard]] ControlResult reset(
      const JointVector& initial_torque) noexcept {
    return torque_filter_.resetRealtime(initial_torque);
  }

 private:
  Controller* controller_;
  ReferenceLimiter reference_limiter_;
  TorqueFilter torque_filter_;
  JointReference limited_reference_;
  JointVector raw_torque_;
  TorqueCommand command_;
};

}  // namespace daedalus
