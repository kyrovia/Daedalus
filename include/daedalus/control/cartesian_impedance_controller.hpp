#pragma once

#include <memory>

#include "daedalus/control/control_math.hpp"
#include "daedalus/types/control_status.hpp"
#include "daedalus/control/controller_config.hpp"
#include "daedalus/model/pinocchio_model.hpp"
#include "daedalus/types/cartesian_types.hpp"

namespace daedalus {

class CartesianImpedanceController final {
 public:
  CartesianImpedanceController(std::shared_ptr<const PinocchioModel> model,
                               CartesianImpedanceConfig config);

  [[nodiscard]] int dof() const noexcept { return model_->nv(); }

  [[nodiscard]] ControlResult compute(
      const JointState& state, const CartesianReference& reference,
      JointVector& torque) const noexcept;

  [[deprecated("use the noexcept compute overload with preallocated output")]]
  [[nodiscard]] JointVector compute(
      const JointState& state, const CartesianReference& reference) const;

 private:
  std::shared_ptr<const PinocchioModel> model_;
  mutable PinocchioModel::Context context_;
  CartesianImpedanceConfig config_;
  mutable CartesianPose pose_;
  mutable Eigen::MatrixXd jacobian_;
  mutable Eigen::MatrixXd jacobian_transpose_;
  mutable Eigen::MatrixXd jacobian_pinv_;
  mutable Eigen::MatrixXd projector_;
  mutable CartesianVector error_{CartesianVector::Zero()};
  mutable CartesianVector task_wrench_{CartesianVector::Zero()};
  mutable JointVector secondary_;
  mutable JointVector gravity_workspace_;
  mutable DampedPseudoInverseWorkspace pinv_workspace_;
};

}  // namespace daedalus
