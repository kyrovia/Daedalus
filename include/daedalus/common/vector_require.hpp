#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "daedalus/types/cartesian_types.hpp"

namespace daedalus {

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

}  // namespace daedalus
