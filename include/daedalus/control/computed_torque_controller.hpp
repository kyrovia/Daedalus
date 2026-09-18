#pragma once

#include <memory>

#include "daedalus/control/controller_config.hpp"
#include "daedalus/model/pinocchio_model.hpp"

namespace daedalus {

class ComputedTorqueController final {
 public:
  ComputedTorqueController(std::shared_ptr<const PinocchioModel> model,
                           ComputedTorqueConfig config);

  [[nodiscard]] JointVector compute(
      const JointState& state, const JointReference& reference) const;

 private:
  std::shared_ptr<const PinocchioModel> model_;
  ComputedTorqueConfig config_;
};

}  // namespace daedalus
