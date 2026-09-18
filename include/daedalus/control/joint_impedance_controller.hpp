#pragma once

#include <memory>

#include "daedalus/control/controller_config.hpp"
#include "daedalus/model/pinocchio_model.hpp"

namespace daedalus {

class JointImpedanceController final {
 public:
  JointImpedanceController(std::shared_ptr<const PinocchioModel> model,
                           JointImpedanceConfig config);

  [[nodiscard]] JointVector compute(
      const JointState& state, const JointReference& reference) const;

 private:
  std::shared_ptr<const PinocchioModel> model_;
  JointImpedanceConfig config_;
};

}  // namespace daedalus
