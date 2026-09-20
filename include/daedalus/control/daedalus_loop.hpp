#pragma once

#include <memory>

#include "daedalus/control/cartesian_impedance_controller.hpp"
#include "daedalus/control/computed_torque_controller.hpp"
#include "daedalus/control/gravity_compensator.hpp"
#include "daedalus/control/joint_impedance_controller.hpp"
#include "daedalus/control/operational_space_controller.hpp"
#include "daedalus/safety/reference_limiter.hpp"
#include "daedalus/safety/torque_filter.hpp"
#include "daedalus/types/cartesian_types.hpp"

namespace daedalus {

enum class ControllerMode {
  kGravityCompensation,
  kComputedTorque,
  kJointImpedance,
  kCartesianImpedance,
  kOperationalSpace,
};

class DaedalusLoop final {
 public:
  DaedalusLoop(std::shared_ptr<const PinocchioModel> model,
               const SafetyLimits& limits,
               ComputedTorqueConfig computed_torque_config,
               JointImpedanceConfig impedance_config,
               CartesianImpedanceConfig cartesian_config,
               OperationalSpaceConfig operational_space_config,
               ControllerMode initial_mode,
               const JointVector& initial_tau);

  [[nodiscard]] JointVector compute(
      const JointState& state, const JointReference& reference, double dt);
  [[nodiscard]] JointVector compute(const JointState& state,
                                    const CartesianReference& reference,
                                    double dt);
  void setMode(ControllerMode mode, const JointVector& current_commanded_tau);
  [[nodiscard]] ControllerMode mode() const noexcept;
  [[nodiscard]] const JointVector& previousTorque() const noexcept;

 private:
  std::shared_ptr<const PinocchioModel> model_;
  GravityCompensator gravity_;
  ComputedTorqueController computed_torque_;
  JointImpedanceController impedance_;
  CartesianImpedanceController cartesian_impedance_;
  OperationalSpaceController operational_space_;
  ReferenceLimiter reference_limiter_;
  TorqueFilter torque_filter_;
  ControllerMode mode_;
};

}  // namespace daedalus
