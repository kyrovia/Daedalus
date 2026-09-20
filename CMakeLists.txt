cmake_minimum_required(VERSION 3.16)
project(daedalus VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(DAEDALUS_DEPENDENCY_PREFIX "" CACHE PATH
    "Prefix containing Eigen, Pinocchio, yaml-cpp and Catch2")
if(DAEDALUS_DEPENDENCY_PREFIX)
  list(PREPEND CMAKE_PREFIX_PATH "${DAEDALUS_DEPENDENCY_PREFIX}")
elseif(DEFINED ENV{CONDA_PREFIX})
  list(PREPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

set(_daedalus_yaml_hints
  "$ENV{CONDA_PREFIX}"
  "${DAEDALUS_DEPENDENCY_PREFIX}"
)
if(Eigen3_DIR)
  get_filename_component(_daedalus_eigen_prefix "${Eigen3_DIR}/../../.." ABSOLUTE)
  list(APPEND _daedalus_yaml_hints "${_daedalus_eigen_prefix}")
endif()
find_path(YAML_CPP_INCLUDE_DIR yaml-cpp/yaml.h
  HINTS ${_daedalus_yaml_hints}
  PATH_SUFFIXES include
  PATHS /usr /usr/local
  CMAKE_FIND_ROOT_PATH_BOTH
)
find_library(YAML_CPP_LIBRARY NAMES yaml-cpp
  HINTS ${_daedalus_yaml_hints}
  PATH_SUFFIXES lib
  PATHS /usr /usr/lib/x86_64-linux-gnu /usr/local
  CMAKE_FIND_ROOT_PATH_BOTH
)
if(NOT YAML_CPP_INCLUDE_DIR OR NOT YAML_CPP_LIBRARY)
  message(FATAL_ERROR
    "yaml-cpp not found. Install libyaml-cpp-dev or the yaml-cpp conda package.")
endif()
# Conda toolchains omit /usr/include; wrap system headers so CMake keeps the -I.
set(DAEDALUS_YAML_INCLUDE_DIR "${YAML_CPP_INCLUDE_DIR}")
if(YAML_CPP_INCLUDE_DIR STREQUAL "/usr/include" OR
   YAML_CPP_INCLUDE_DIR STREQUAL "/usr/local/include")
  set(DAEDALUS_YAML_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/yaml-cpp-include")
  file(COPY "${YAML_CPP_INCLUDE_DIR}/yaml-cpp"
       DESTINATION "${DAEDALUS_YAML_INCLUDE_DIR}")
endif()
add_library(daedalus_yaml_cpp UNKNOWN IMPORTED)
set_target_properties(daedalus_yaml_cpp PROPERTIES
  IMPORTED_LOCATION "${YAML_CPP_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${DAEDALUS_YAML_INCLUDE_DIR}"
)
set(DAEDALUS_YAML_TARGET daedalus_yaml_cpp)

add_library(daedalus_control
  src/model/pinocchio_model.cpp
  src/safety/reference_limiter.cpp
  src/safety/torque_filter.cpp
  src/control/control_math.cpp
  src/control/config_loader.cpp
  src/control/gravity_compensator.cpp
  src/control/computed_torque_controller.cpp
  src/control/joint_impedance_controller.cpp
  src/control/cartesian_impedance_controller.cpp
  src/control/operational_space_controller.cpp
  src/control/daedalus_loop.cpp
)
add_library(daedalus::control ALIAS daedalus_control)

target_include_directories(daedalus_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(daedalus_control
  PUBLIC
    Eigen3::Eigen
    pinocchio::pinocchio
    ${DAEDALUS_YAML_TARGET}
)
target_compile_options(daedalus_control PRIVATE -Wall -Wextra -Wpedantic)

add_executable(daedalus_control_step examples/control_step.cpp)
target_link_libraries(daedalus_control_step PRIVATE daedalus::control)

include(CTest)
if(BUILD_TESTING)
  find_package(Catch2 3 REQUIRED)
  add_executable(daedalus_tests tests/control_tests.cpp)
  target_link_libraries(daedalus_tests
    PRIVATE daedalus::control Catch2::Catch2WithMain
  )
  target_compile_definitions(daedalus_tests
    PRIVATE
      DAEDALUS_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/two_link.urdf"
      DAEDALUS_TEST_CONTROL_YAML="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/two_link_control.yaml"
      DAEDALUS_TEST_INVALID_JACOBIAN_YAML="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/invalid_jacobian_reference.yaml"
  )
  add_test(NAME daedalus_tests COMMAND daedalus_tests)
endif()
