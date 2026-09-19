cmake_minimum_required(VERSION 3.16)
project(daedalus VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(DAEDALUS_DEPENDENCY_PREFIX "" CACHE PATH
    "Prefix containing Eigen, Pinocchio and Catch2")
if(DAEDALUS_DEPENDENCY_PREFIX)
  list(PREPEND CMAKE_PREFIX_PATH "${DAEDALUS_DEPENDENCY_PREFIX}")
elseif(DEFINED ENV{CONDA_PREFIX})
  list(PREPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_library(daedalus_control
  src/model/pinocchio_model.cpp
  src/safety/reference_limiter.cpp
  src/safety/torque_filter.cpp
  src/control/gravity_compensator.cpp
  src/control/computed_torque_controller.cpp
  src/control/joint_impedance_controller.cpp
  src/control/daedalus_loop.cpp
)
add_library(daedalus::control ALIAS daedalus_control)

target_include_directories(daedalus_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(daedalus_control PUBLIC Eigen3::Eigen pinocchio::pinocchio)
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
    PRIVATE DAEDALUS_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/two_link.urdf"
  )
  add_test(NAME daedalus_tests COMMAND daedalus_tests)
endif()
