#pragma once

#include <memory>
#include <string>
#include <vector>

#include "daedalus/types/cartesian_types.hpp"
#include "daedalus/types/control_status.hpp"
#include "daedalus/types/joint_types.hpp"

namespace daedalus {

class PinocchioModel final {
 public:
  // Exclusive Pinocchio Data workspace. Not thread-safe and not shareable
  // across threads; create one Context per control instance or thread.
  class Context final {
   public:
    ~Context();

    Context(Context&&) noexcept;
    Context& operator=(Context&&) noexcept;
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;

   private:
    friend class PinocchioModel;
    struct Impl;

    explicit Context(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
  };

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

  [[nodiscard]] Context createContext() const;

  [[nodiscard]] ControlResult gravityRealtime(
      Context& context, const JointVector& q,
      Eigen::Ref<JointVector> output) const noexcept;
  [[nodiscard]] ControlResult coriolisRealtime(
      Context& context, const JointVector& q, const JointVector& dq,
      Eigen::Ref<JointVector> output) const noexcept;
  [[nodiscard]] ControlResult inverseMassMatrixRealtime(
      Context& context, const JointVector& q,
      Eigen::Ref<Eigen::MatrixXd> output) const noexcept;
  [[nodiscard]] ControlResult inverseDynamicsRealtime(
      Context& context, const JointVector& q, const JointVector& dq,
      const JointVector& ddq, Eigen::Ref<JointVector> output) const noexcept;
  [[nodiscard]] ControlResult framePoseRealtime(
      Context& context, const JointVector& q, const std::string& frame_name,
      CartesianPose& output) const noexcept;
  [[nodiscard]] ControlResult frameJacobianRealtime(
      Context& context, const JointVector& q, const std::string& frame_name,
      JacobianReference reference,
      Eigen::Ref<Eigen::MatrixXd> output) const noexcept;

  void gravity(Context& context, const JointVector& q,
               Eigen::Ref<JointVector> output) const;
  [[nodiscard]] JointVector gravity(Context& context,
                                    const JointVector& q) const;
  void coriolis(Context& context, const JointVector& q, const JointVector& dq,
                Eigen::Ref<JointVector> output) const;
  [[nodiscard]] JointVector coriolis(Context& context, const JointVector& q,
                                     const JointVector& dq) const;
  void inverseMassMatrix(Context& context, const JointVector& q,
                         Eigen::Ref<Eigen::MatrixXd> output) const;
  [[nodiscard]] Eigen::MatrixXd inverseMassMatrix(
      Context& context, const JointVector& q) const;
  void inverseDynamics(Context& context, const JointVector& q,
                       const JointVector& dq, const JointVector& ddq,
                       Eigen::Ref<JointVector> output) const;
  [[nodiscard]] JointVector inverseDynamics(
      Context& context, const JointVector& q, const JointVector& dq,
      const JointVector& ddq) const;
  [[nodiscard]] CartesianPose framePose(
      Context& context, const JointVector& q,
      const std::string& frame_name) const;
  void framePose(Context& context, const JointVector& q,
                 const std::string& frame_name, CartesianPose& output) const;
  void frameJacobian(
      Context& context, const JointVector& q, const std::string& frame_name,
      JacobianReference reference, Eigen::Ref<Eigen::MatrixXd> output) const;
  [[nodiscard]] Eigen::MatrixXd frameJacobian(
      Context& context, const JointVector& q, const std::string& frame_name,
      JacobianReference reference =
          JacobianReference::kLocalWorldAligned) const;

 private:
  [[nodiscard]] ControlResult validateContextRealtime(
      const Context& context) const noexcept;
  static void throwIfFailed(ControlResult result,
                            const std::string& unknown_frame = {});

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace daedalus
