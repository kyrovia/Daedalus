#pragma once

#include <memory>

#include "daedalus/control/controller_config.hpp"
#include "daedalus/model/pinocchio_model.hpp"
#include "daedalus/types/cartesian_types.hpp"

namespace daedalus {

class CartesianImpedanceController final {
 public:
  CartesianImpedanceController(std::shared_ptr<const PinocchioModel> model,
                               CartesianImpedanceConfig config);

  [[nodiscard]] JointVector compute(
      const JointState& state, const CartesianReference& reference) const;

 private:
  std::shared_ptr<const PinocchioModel> model_;
  CartesianImpedanceConfig config_;
};

}  // namespace daedalus