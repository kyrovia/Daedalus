cmake_minimum_required(VERSION 3.16)
project(daedalus VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DAEDALUS_ENABLE_ASAN "Enable UBSan (ASan is incompatible with prebuilt Pinocchio)" OFF)
option(DAEDALUS_ENABLE_TSAN "Enable ThreadSanitizer" OFF)

set(DAEDALUS_DEPENDENCY_PREFIX "" CACHE PATH
    "Prefix containing Eigen, Pinocchio and Catch2")
if(DAEDALUS_DEPENDENCY_PREFIX)
  list(PREPEND CMAKE_PREFIX_PATH "${DAEDALUS_DEPENDENCY_PREFIX}")
elseif(DEFINED ENV{CONDA_PREFIX})
  list(PREPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

if(DAEDALUS_ENABLE_ASAN AND DAEDALUS_ENABLE_TSAN)
  message(FATAL_ERROR "ASan and TSan cannot be enabled together")
endif()
if(DAEDALUS_ENABLE_ASAN)
  # Full AddressSanitizer false-positives on Eigen aligned_free vs Pinocchio's
  # realloc in prebuilt libpinocchio_parsers. UBSan still covers our code.
  add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=undefined)
endif()
if(DAEDALUS_ENABLE_TSAN)
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
endif()

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Threads REQUIRED)

add_library(daedalus_control
  include/daedalus/control/control_pipeline.hpp
  include/daedalus/control/torque_command.hpp
  src/model/pinocchio_model.cpp
  src/safety/reference_limiter.cpp
  src/safety/torque_filter.cpp
  src/control/control_math.cpp
  src/control/gravity_compensator.cpp
  src/control/computed_torque_controller.cpp
  src/control/joint_impedance_controller.cpp
  src/control/cartesian_impedance_controller.cpp
  src/control/operational_space_controller.cpp
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
)
target_compile_options(daedalus_control PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(daedalus_control PRIVATE EIGEN_RUNTIME_NO_MALLOC)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/control_step.cpp")
  add_executable(daedalus_control_step examples/control_step.cpp)
  target_link_libraries(daedalus_control_step PRIVATE daedalus::control)
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(Catch2 3 REQUIRED)
  add_executable(daedalus_tests
    tests/model_tests.cpp
    tests/controller_tests.cpp
    tests/safety_pipeline_tests.cpp
  )
  target_include_directories(daedalus_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/tests")
  target_link_libraries(daedalus_tests
    PRIVATE daedalus::control Catch2::Catch2WithMain Threads::Threads
  )
  target_compile_options(daedalus_tests PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_definitions(daedalus_tests
    PRIVATE
      EIGEN_RUNTIME_NO_MALLOC
      DAEDALUS_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/two_link.urdf"
  )
  add_test(NAME daedalus_tests COMMAND daedalus_tests)
endif()
