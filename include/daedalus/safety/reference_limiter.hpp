#pragma once

#include "daedalus/types/control_status.hpp"
#include "daedalus/types/joint_types.hpp"
#include "daedalus/types/safety_limits.hpp"

namespace daedalus {

class ReferenceLimiter final {
 public:
  explicit ReferenceLimiter(SafetyLimits limits);

  [[nodiscard]] int dof() const noexcept;
  [[nodiscard]] ControlResult validateStateRealtime(
      const JointState& state) const noexcept;
  [[nodiscard]] ControlResult limitRealtime(
      const JointReference& reference, JointReference& limited) const noexcept;
  void validateState(const JointState& state) const;
  [[nodiscard]] JointReference limit(const JointReference& reference) const;

 private:
  SafetyLimits limits_;
};

}  // namespace daedalus
