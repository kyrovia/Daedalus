#pragma once

#include <stdexcept>
#include <string>

#include "daedalus/types/joint_types.hpp"

namespace daedalus {

inline void requireSizeAndFinite(const JointVector& value,
                                 const Eigen::Index size, const char* name) {
  if (value.size() != size) {
    throw std::invalid_argument(std::string(name) + " has incorrect size");
  }
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string(name) + " contains NaN or Inf");
  }
}

inline void requirePositive(const JointVector& value, const Eigen::Index size,
                            const char* name) {
  requireSizeAndFinite(value, size, name);
  if ((value.array() <= 0.0).any()) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

inline void requireNonnegative(const JointVector& value,
                               const Eigen::Index size, const char* name) {
  requireSizeAndFinite(value, size, name);
  if ((value.array() < 0.0).any()) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and nonnegative");
  }
}

}  // namespace daedalus
