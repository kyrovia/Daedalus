#pragma once

#include <memory>

#include "daedalus/control/controller_config.hpp"
#include "daedalus/model/pinocchio_model.hpp"
#include "daedalus/types/cartesian_types.hpp"

namespace daedalus {

class OperationalSpaceController final {
 public:
  OperationalSpaceController(std::shared_ptr<const PinocchioModel> model,
                             OperationalSpaceConfig config);

  [[nodiscard]] JointVector compute(
      const JointState& state, const CartesianReference& reference);
  void reset() noexcept;

 private:
  std::shared_ptr<const PinocchioModel> model_;
  OperationalSpaceConfig config_;
  JointVector nullspace_reference_;
  Eigen::Vector3d desired_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond desired_orientation_{Eigen::Quaterniond::Identity()};
  bool initialized_{false};
};

}  // namespace daedalus
