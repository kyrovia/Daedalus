#pragma once

#include <memory>

#include "daedalus/model/pinocchio_model.hpp"

namespace daedalus {

class GravityCompensator final {
 public:
  explicit GravityCompensator(std::shared_ptr<const PinocchioModel> model);

  [[nodiscard]] JointVector compute(
      const JointState& state, const JointReference& reference) const;

 private:
  std::shared_ptr<const PinocchioModel> model_;
};

}  // namespace daedalus
