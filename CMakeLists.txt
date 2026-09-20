cmake_minimum_required(VERSION 3.16)
project(daedalus VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DAEDALUS_BUILD_EXAMPLES "Build example executables" ON)
option(DAEDALUS_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(DAEDALUS_ENABLE_ASAN "Deprecated alias for DAEDALUS_ENABLE_UBSAN" OFF)
option(DAEDALUS_ENABLE_TSAN "Enable ThreadSanitizer" OFF)

set(DAEDALUS_DEPENDENCY_PREFIX "" CACHE PATH
    "Prefix containing Eigen, Pinocchio and Catch2")
if(DAEDALUS_DEPENDENCY_PREFIX)
  list(PREPEND CMAKE_PREFIX_PATH "${DAEDALUS_DEPENDENCY_PREFIX}")
elseif(DEFINED ENV{CONDA_PREFIX})
  list(PREPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

if(DAEDALUS_ENABLE_ASAN)
  message(DEPRECATION
    "DAEDALUS_ENABLE_ASAN enables UBSan only (ASan is incompatible with prebuilt Pinocchio). Use DAEDALUS_ENABLE_UBSAN.")
  set(DAEDALUS_ENABLE_UBSAN ON)
endif()
if(DAEDALUS_ENABLE_UBSAN AND DAEDALUS_ENABLE_TSAN)
  message(FATAL_ERROR "UBSan and TSan cannot be enabled together")
endif()
if(DAEDALUS_ENABLE_UBSAN)
  add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=undefined)
endif()
if(DAEDALUS_ENABLE_TSAN)
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
endif()

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(daedalus_control
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

set_target_properties(daedalus_control PROPERTIES
  EXPORT_NAME control
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(daedalus_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(daedalus_control
  PUBLIC Eigen3::Eigen
  PRIVATE pinocchio::pinocchio
)
if(NOT BUILD_SHARED_LIBS)
  # Static consumers still need Pinocchio at link time, but not its headers.
  target_link_libraries(daedalus_control
    INTERFACE $<LINK_ONLY:pinocchio::pinocchio>)
endif()
target_compile_options(daedalus_control PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(daedalus_control PRIVATE EIGEN_RUNTIME_NO_MALLOC)

install(TARGETS daedalus_control
  EXPORT daedalusTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/daedalus
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT daedalusTargets
  FILE daedalusTargets.cmake
  NAMESPACE daedalus::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/daedalus
)

set(DAEDALUS_FIND_PINOCCHIO OFF)
if(NOT BUILD_SHARED_LIBS)
  set(DAEDALUS_FIND_PINOCCHIO ON)
endif()

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/daedalusConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/daedalusConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/daedalus
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/daedalusConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMinorVersion
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/daedalusConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/daedalusConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/daedalus
)

if(DAEDALUS_BUILD_EXAMPLES)
  add_executable(daedalus_control_step examples/control_step.cpp)
  target_link_libraries(daedalus_control_step PRIVATE daedalus::control)
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(Catch2 3 REQUIRED)
  find_package(Threads REQUIRED)
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
  if(DAEDALUS_BUILD_EXAMPLES)
    add_test(NAME daedalus_control_step
      COMMAND daedalus_control_step
              "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/two_link.urdf")
  endif()
endif()
