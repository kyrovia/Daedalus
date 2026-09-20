#pragma once

#include <memory>
#include <string>
#include <vector>

#include "daedalus/types/cartesian_types.hpp"
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
  [[nodiscard]] std::vector<std::string> bodyFrameNames() const;
  [[nodiscard]] bool hasFrame(const std::string& frame_name) const;
  [[nodiscard]] JointVector effortLimits() const;
  [[nodiscard]] JointVector lowerPositionLimits() const;
  [[nodiscard]] JointVector upperPositionLimits() const;

  [[nodiscard]] JointVector gravity(const JointVector& q) const;
  [[nodiscard]] JointVector coriolis(const JointVector& q,
                                     const JointVector& dq) const;
  [[nodiscard]] Eigen::MatrixXd inverseMassMatrix(
      const JointVector& q) const;
  [[nodiscard]] JointVector inverseDynamics(
      const JointVector& q, const JointVector& dq,
      const JointVector& ddq) const;
  [[nodiscard]] CartesianPose framePose(
      const JointVector& q, const std::string& frame_name) const;
  [[nodiscard]] Eigen::MatrixXd frameJacobian(
      const JointVector& q, const std::string& frame_name,
      JacobianReference reference =
          JacobianReference::kLocalWorldAligned) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace daedalus
