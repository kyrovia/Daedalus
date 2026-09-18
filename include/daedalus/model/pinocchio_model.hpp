#pragma once

#include <memory>
#include <string>
#include <vector>

#include "daedalus/types/joint_types.hpp"

namespace daedalus {

class PinocchioModel final {
 public:
  explicit PinocchioModel(const std::string& urdf_path);
  ~PinocchioModel();

  PinocchioModel(PinocchioModel&&) noexcept;
  PinocchioModel& operator=(PinocchioModel&&) noexcept;
  PinocchioModel(const PinocchioModel&) = delete;
  PinocchioModel& operator=(const PinocchioModel&) = delete;

  [[nodiscard]] int nq() const noexcept;
  [[nodiscard]] int nv() const noexcept;
  [[nodiscard]] const std::vector<std::string>& jointNames() const noexcept;
  [[nodiscard]] JointVector effortLimits() const;

  [[nodiscard]] JointVector gravity(const JointVector& q) const;
  [[nodiscard]] JointVector inverseDynamics(
      const JointVector& q, const JointVector& dq,
      const JointVector& ddq) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace daedalus
