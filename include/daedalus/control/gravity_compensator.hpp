#pragma once

#include <memory>

#include "daedalus/control/control_status.hpp"
#include "daedalus/model/pinocchio_model.hpp"

namespace daedalus {

class GravityCompensator final {
 public:
  explicit GravityCompensator(std::shared_ptr<const PinocchioModel> model);

  [[nodiscard]] ControlResult compute(
      const JointState& state, const JointReference& reference,
      JointVector& torque) const noexcept;

  [[deprecated("use the noexcept compute overload with preallocated output")]]
  [[nodiscard]] JointVector compute(
      const JointState& state, const JointReference& reference) const;

 private:
  std::shared_ptr<const PinocchioModel> model_;
  mutable PinocchioModel::Context context_;
};

}  // namespace daedalus
