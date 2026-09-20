#pragma once

#include "daedalus/types/joint_types.hpp"

namespace daedalus {

template <typename Controller>
class ControlPipeline;

class TorqueCommand final {
 public:
  [[nodiscard]] const JointVector& torque() const noexcept {
    return torque_;
  }

 private:
  template <typename Controller>
  friend class ControlPipeline;

  explicit TorqueCommand(const Eigen::Index dof)
      : torque_(JointVector::Zero(dof)) {}

  JointVector torque_;
};

}  // namespace daedalus
