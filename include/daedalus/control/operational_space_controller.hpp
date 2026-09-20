#pragma once

#include <memory>

#include "daedalus/types/control_status.hpp"
#include "daedalus/control/control_math.hpp"
#include "daedalus/control/controller_config.hpp"
#include "daedalus/model/pinocchio_model.hpp"
#include "daedalus/types/cartesian_types.hpp"

namespace daedalus {

class OperationalSpaceController final {
 public:
  OperationalSpaceController(std::shared_ptr<const PinocchioModel> model,
                             OperationalSpaceConfig config);

  [[nodiscard]] int dof() const noexcept { return model_->nv(); }

  [[nodiscard]] ControlResult compute(
      const JointState& state, const CartesianReference& reference,
      JointVector& torque) noexcept;

  [[deprecated("use the noexcept compute overload with preallocated output")]]
  [[nodiscard]] JointVector compute(
      const JointState& state, const CartesianReference& reference);
  void reset() noexcept;

 private:
  std::shared_ptr<const PinocchioModel> model_;
  PinocchioModel::Context context_;
  OperationalSpaceConfig config_;
  JointVector nullspace_reference_;
  CartesianPose pose_;
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd inverse_mass_;
  Eigen::MatrixXd task_matrix_;
  Eigen::MatrixXd task_inertia_;
  Eigen::MatrixXd temp_6xn_;
  Eigen::MatrixXd temp_nx6_;
  Eigen::MatrixXd j_bar_;
  Eigen::MatrixXd projector_;
  Eigen::MatrixXd jacobian_pseudoinverse_;
  CartesianVector task_velocity_{CartesianVector::Zero()};
  CartesianVector task_command_{CartesianVector::Zero()};
  JointVector secondary_;
  JointVector nullspace_torque_;
  JointVector model_term_;
  JointVector lower_limits_;
  JointVector upper_limits_;
  DampedPseudoInverseWorkspace task_inverse_workspace_;
  DampedPseudoInverseWorkspace jacobian_inverse_workspace_;
  Eigen::Vector3d desired_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond desired_orientation_{Eigen::Quaterniond::Identity()};
  int end_effector_frame_id_{0};
  bool initialized_{false};
};

}  // namespace daedalus
