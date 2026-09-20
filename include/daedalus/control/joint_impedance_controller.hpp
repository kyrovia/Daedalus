#pragma once

#include <memory>

#include "daedalus/control/control_status.hpp"
#include "daedalus/control/controller_config.hpp"
#include "daedalus/model/pinocchio_model.hpp"

namespace daedalus {

class JointImpedanceController final {
 public:
  JointImpedanceController(std::shared_ptr<const PinocchioModel> model,
                           JointImpedanceConfig config);

  [[nodiscard]] ControlResult compute(
      const JointState& state, const JointReference& reference,
      JointVector& torque) const noexcept;

  [[deprecated("use the noexcept compute overload with preallocated output")]]
  [[nodiscard]] JointVector compute(
      const JointState& state, const JointReference& reference) const;

 private:
  std::shared_ptr<const PinocchioModel> model_;
  mutable PinocchioModel::Context context_;
  JointImpedanceConfig config_;
  mutable JointVector gravity_workspace_;
};

}  // namespace daedalus
