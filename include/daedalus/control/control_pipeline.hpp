#pragma once

#include <stdexcept>
#include <type_traits>
#include <utility>

#include "daedalus/control/control_validation.hpp"
#include "daedalus/control/torque_command.hpp"
#include "daedalus/safety/reference_limiter.hpp"
#include "daedalus/safety/torque_filter.hpp"
#include "daedalus/types/cartesian_types.hpp"
#include "daedalus/types/control_status.hpp"
#include "daedalus/types/safety_limits.hpp"

namespace daedalus {

// Software rate/amplitude limiting, not a certified safety function.
// Joint references are clipped. Cartesian references are validated but not
// clipped toward the current pose; task-error clipping belongs to the
// controller (OperationalSpaceConfig::limit_error /
// CartesianImpedanceConfig::limit_error).
enum class PipelineFailPolicy {
  kNoCommand,
  // Replay the last successful command with a non-ok status. Does not reverse
  // a command that was already driving the robot into a limit; callers must
  // inspect status, not only command != nullptr.
  kHoldLast,
};

struct ControlStepResult final {
  ControlStatus status{ControlStatus::kOk};
  // Points at pipeline-owned storage. Invalidated by the next step() or a
  // successful reset().
  const TorqueCommand* command{nullptr};

  [[nodiscard]] constexpr bool ok() const noexcept {
    return status == ControlStatus::kOk && command != nullptr;
  }

  [[nodiscard]] constexpr explicit operator bool() const noexcept {
    return ok();
  }
};

template <typename T, typename = void>
struct ControllerHasDof : std::false_type {};

template <typename T>
struct ControllerHasDof<T, std::void_t<decltype(std::declval<const T&>().dof())>>
    : std::true_type {};

template <typename T, typename = void>
struct ControllerHasReset : std::false_type {};

template <typename T>
struct ControllerHasReset<T, std::void_t<decltype(std::declval<T&>().reset())>>
    : std::true_type {};

template <typename Controller>
class ControlPipeline final {
 public:
  ControlPipeline(
      Controller& controller, const SafetyLimits& limits,
      const PipelineFailPolicy fail_policy = PipelineFailPolicy::kNoCommand)
      : controller_(&controller),
        fail_policy_(fail_policy),
        reference_limiter_(limits),
        torque_filter_(limits.tau_max, limits.tau_rate_max),
        limited_reference_{
            JointVector::Zero(limits.q_lower.size()),
            JointVector::Zero(limits.q_lower.size()),
            JointVector::Zero(limits.q_lower.size())},
        raw_torque_(JointVector::Zero(limits.q_lower.size())),
        command_(limits.q_lower.size()) {
    if constexpr (ControllerHasDof<Controller>::value) {
      if (controller.dof() != static_cast<int>(limits.q_lower.size())) {
        throw std::invalid_argument(
            "controller dof does not match safety limits");
      }
    }
    torque_filter_.reset(JointVector::Zero(limits.q_lower.size()));
  }

  [[nodiscard]] ControlStepResult step(
      const JointState& state, const JointReference& reference,
      const double dt) noexcept {
    ControlResult result = reference_limiter_.validateStateRealtime(state);
    if (!result) {
      return fail(result.status);
    }

    result = reference_limiter_.limitRealtime(reference, limited_reference_);
    if (!result) {
      return fail(result.status);
    }

    result = controller_->compute(state, limited_reference_, raw_torque_);
    if (!result) {
      return fail(result.status);
    }

    result = torque_filter_.filterRealtime(raw_torque_, dt, command_.torque_);
    if (!result) {
      return fail(result.status);
    }
    has_command_ = true;
    return {ControlStatus::kOk, &command_};
  }

  [[nodiscard]] ControlStepResult step(
      const JointState& state, const CartesianReference& reference,
      const double dt) noexcept {
    ControlResult result = reference_limiter_.validateStateRealtime(state);
    if (!result) {
      return fail(result.status);
    }

    const ControlStatus cartesian_status = validateCartesianReference(
        reference, reference_limiter_.dof());
    if (cartesian_status != ControlStatus::kOk) {
      return fail(cartesian_status);
    }

    result = controller_->compute(state, reference, raw_torque_);
    if (!result) {
      return fail(result.status);
    }

    result = torque_filter_.filterRealtime(raw_torque_, dt, command_.torque_);
    if (!result) {
      return fail(result.status);
    }
    has_command_ = true;
    return {ControlStatus::kOk, &command_};
  }

  [[nodiscard]] ControlResult reset(
      const JointVector& initial_torque) noexcept {
    if constexpr (ControllerHasReset<Controller>::value) {
      static_assert(noexcept(std::declval<Controller&>().reset()),
                    "controller reset() must be noexcept");
      controller_->reset();
    }
    const ControlResult result = torque_filter_.resetRealtime(initial_torque);
    if (result) {
      has_command_ = false;
    }
    return result;
  }

 private:
  [[nodiscard]] ControlStepResult fail(
      const ControlStatus status) const noexcept {
    if (fail_policy_ == PipelineFailPolicy::kHoldLast && has_command_) {
      return {status, &command_};
    }
    return {status, nullptr};
  }

  Controller* controller_;
  PipelineFailPolicy fail_policy_{PipelineFailPolicy::kNoCommand};
  ReferenceLimiter reference_limiter_;
  TorqueFilter torque_filter_;
  JointReference limited_reference_;
  JointVector raw_torque_;
  TorqueCommand command_;
  bool has_command_{false};
};

}  // namespace daedalus
