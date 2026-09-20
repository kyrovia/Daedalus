#pragma once

#include <cstdint>

namespace daedalus {

enum class ControlStatus : std::uint8_t {
  kOk = 0,
  kInvalidDimension,
  kNonFiniteInput,
  kNotInitialized,
  kModelError,
  kNonFiniteOutput,
  kStateLimitViolation,
  kInvalidTimeStep,
  kInternalError,
};

struct ControlResult final {
  ControlStatus status{ControlStatus::kOk};

  [[nodiscard]] constexpr bool ok() const noexcept {
    return status == ControlStatus::kOk;
  }

  [[nodiscard]] constexpr explicit operator bool() const noexcept {
    return ok();
  }
};

[[nodiscard]] constexpr const char* controlStatusMessage(
    const ControlStatus status) noexcept {
  switch (status) {
    case ControlStatus::kOk:
      return "ok";
    case ControlStatus::kInvalidDimension:
      return "invalid dimension";
    case ControlStatus::kNonFiniteInput:
      return "non-finite input";
    case ControlStatus::kNotInitialized:
      return "controller not initialized";
    case ControlStatus::kModelError:
      return "model computation failed";
    case ControlStatus::kNonFiniteOutput:
      return "non-finite output";
    case ControlStatus::kStateLimitViolation:
      return "measured state exceeds safety limits";
    case ControlStatus::kInvalidTimeStep:
      return "invalid control time step";
    case ControlStatus::kInternalError:
      return "internal control error";
  }
  return "unknown control error";
}

}  // namespace daedalus
