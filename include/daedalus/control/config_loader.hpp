#pragma once

#include <string>

#include "daedalus/control/controller_config.hpp"

namespace daedalus {

[[nodiscard]] DaedalusConfig loadDaedalusConfig(const std::string& yaml_path,
                                                int nv);

}  // namespace daedalus
