#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "daedalus/types/cartesian_types.hpp"
#include "daedalus/types/control_status.hpp"

namespace daedalus {

// Construction-time checks. These throw and may allocate; do not call them
// from a hard-realtime control cycle.
inline void requireFinite(const double value, const char* name) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " contains NaN or Inf");
  }
}

inline void requirePositive(const double value, const char* name) {
  requireFinite(value, name);
  if (value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

inline void requireNonnegative(const double value, const char* name) {
  requireFinite(value, name);
  if (value < 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and nonnegative");
  }
}

template <typename Derived>
inline void requireFinite(const Eigen::MatrixBase<Derived>& value,
                          const char* name) {
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string(name) + " contains NaN or Inf");
  }
}

template <typename Derived>
inline void requireSizeAndFinite(const Eigen::MatrixBase<Derived>& value,
                                 const Eigen::Index size, const char* name) {
  if (value.size() != size) {
    throw std::invalid_argument(std::string(name) + " has incorrect size");
  }
  requireFinite(value, name);
}

template <typename Derived>
inline void requirePositive(const Eigen::MatrixBase<Derived>& value,
                            const Eigen::Index size, const char* name) {
  requireSizeAndFinite(value, size, name);
  if ((value.array() <= 0.0).any()) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

template <typename Derived>
inline void requireNonnegative(const Eigen::MatrixBase<Derived>& value,
                               const Eigen::Index size, const char* name) {
  requireSizeAndFinite(value, size, name);
  if ((value.array() < 0.0).any()) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and nonnegative");
  }
}

inline void requireFinitePose(const CartesianPose& pose) {
  requireFinite(pose.position, "position");
  requireFinite(pose.orientation.coeffs(), "orientation");
  if (pose.orientation.norm() < 1e-12) {
    throw std::invalid_argument("orientation must be a non-zero quaternion");
  }
}

// Realtime checks. These never throw and never allocate.
template <typename Derived>
[[nodiscard]] inline ControlStatus validateSizeAndFinite(
    const Eigen::MatrixBase<Derived>& value,
    const Eigen::Index size) noexcept {
  if (value.size() != size) {
    return ControlStatus::kInvalidDimension;
  }
  if (!value.allFinite()) {
    return ControlStatus::kNonFiniteInput;
  }
  return ControlStatus::kOk;
}

[[nodiscard]] inline ControlStatus validateMatrixShape(
    const Eigen::Index rows, const Eigen::Index cols,
    const Eigen::Index expected_rows,
    const Eigen::Index expected_cols) noexcept {
  if (rows != expected_rows || cols != expected_cols) {
    return ControlStatus::kInvalidDimension;
  }
  return ControlStatus::kOk;
}

}  // namespace daedalus
