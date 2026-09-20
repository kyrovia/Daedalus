#pragma once

#include "daedalus/types/control_status.hpp"
#include "daedalus/types/joint_types.hpp"

namespace daedalus {

class TorqueFilter final {
 public:
  TorqueFilter(JointVector tau_max, JointVector tau_rate_max);

  void reset(const JointVector& initial_tau);
  [[nodiscard]] ControlResult resetRealtime(
      const JointVector& initial_tau) noexcept;
  [[nodiscard]] ControlResult filterRealtime(
      const JointVector& raw_tau, double dt, JointVector& filtered) noexcept;
  [[nodiscard]] JointVector filter(const JointVector& raw_tau, double dt);
  [[nodiscard]] const JointVector& previousTorque() const noexcept;

 private:
  JointVector tau_max_;
  JointVector tau_rate_max_;
  JointVector previous_tau_;
  bool initialized_{false};
};

}  // namespace daedalus
