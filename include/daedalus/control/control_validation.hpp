#pragma once

#include "daedalus/common/vector_require.hpp"
#include "daedalus/types/cartesian_types.hpp"
#include "daedalus/types/control_status.hpp"
#include "daedalus/types/joint_types.hpp"

namespace daedalus {

[[nodiscard]] inline ControlStatus validateJointState(
    const JointState& state, const int nq, const int nv) noexcept {
  const ControlStatus q_status = validateSizeAndFinite(state.q, nq);
  if (q_status != ControlStatus::kOk) {
    return q_status;
  }
  return validateSizeAndFinite(state.dq, nv);
}

[[nodiscard]] inline ControlStatus validateJointReference(
    const JointReference& reference, const int nq, const int nv) noexcept {
  ControlStatus status = validateSizeAndFinite(reference.q, nq);
  if (status != ControlStatus::kOk) {
    return status;
  }
  status = validateSizeAndFinite(reference.dq, nv);
  if (status != ControlStatus::kOk) {
    return status;
  }
  return validateSizeAndFinite(reference.ddq, nv);
}

[[nodiscard]] inline ControlStatus validateCartesianReference(
    const CartesianReference& reference, const int nv) noexcept {
  if (reference.q_nullspace.size() != 0 &&
      reference.q_nullspace.size() != nv) {
    return ControlStatus::kInvalidDimension;
  }
  const double orientation_norm =
      reference.pose.orientation.coeffs().squaredNorm();
  if (!reference.pose.position.allFinite() ||
      !reference.pose.orientation.coeffs().allFinite() ||
      !reference.wrench.allFinite() ||
      (reference.q_nullspace.size() != 0 &&
       !reference.q_nullspace.allFinite()) ||
      !std::isfinite(orientation_norm) || orientation_norm <= 1e-24) {
    return ControlStatus::kNonFiniteInput;
  }
  return ControlStatus::kOk;
}

[[nodiscard]] inline ControlStatus validateTorqueOutput(
    const JointVector& torque, const int nv) noexcept {
  if (torque.size() != nv) {
    return ControlStatus::kInvalidDimension;
  }
  return torque.allFinite() ? ControlStatus::kOk
                            : ControlStatus::kNonFiniteOutput;
}

}  // namespace daedalus
