cmake_minimum_required(VERSION 3.20)
project(dyno_ros2_bridge)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS2 dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(std_msgs    REQUIRED)

# ── quill — low-latency async logging ─────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
    quill
    GIT_REPOSITORY https://github.com/odygrd/quill.git
    GIT_TAG        v7.5.0
    GIT_SHALLOW    ON
)
set(QUILL_NO_EXCEPTIONS ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(quill)

# ── ethercat_core ─────────────────────────────────────────────────────────────
# Point at the parent repo's built artifacts.
# Build the parent CMake project first, then colcon build this package.
set(DYNO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

# SOEM — fetched by the parent CMake into its build dir.
# Adjust DYNO_BUILD_DIR if your build folder is named differently.
set(DYNO_BUILD_DIR "${DYNO_ROOT}/build")

find_library(ETHERCAT_CORE_LIB  ethercat_core  HINTS "${DYNO_BUILD_DIR}/src/ethercat_core")
find_library(TESTBENCH_UTILS_LIB testbench_utils HINTS "${DYNO_BUILD_DIR}/src/testbench_utils")
find_library(SOEM_LIB          soem          HINTS "${DYNO_BUILD_DIR}/_deps/soem-build")

set(SOEM_INCLUDE_DIRS
    "${DYNO_BUILD_DIR}/_deps/soem-src/soem"
    "${DYNO_BUILD_DIR}/_deps/soem-src/osal"
    "${DYNO_BUILD_DIR}/_deps/soem-src/osal/linux"
    "${DYNO_BUILD_DIR}/_deps/soem-src/oshw/linux"
)

set(JSON_INCLUDE_DIR "${DYNO_BUILD_DIR}/_deps/nlohmann_json-src/include")

# ── bridge_ros2 executable ────────────────────────────────────────────────────
add_executable(bridge_ros2
    bridge_ros2.cpp
    "${DYNO_ROOT}/src/testbenches/dual_novanta_testbench/dual_novanta_testbench.cpp"
)

target_include_directories(bridge_ros2 PRIVATE
    "${DYNO_ROOT}/src"
    "${DYNO_ROOT}/src/testbenches"
    "${DYNO_ROOT}/src/interface_bridges/ros2"
    ${SOEM_INCLUDE_DIRS}
    ${JSON_INCLUDE_DIR}
)

ament_target_dependencies(bridge_ros2
    rclcpp
    std_msgs
)

target_link_libraries(bridge_ros2
    ${ETHERCAT_CORE_LIB}
    ${TESTBENCH_UTILS_LIB}
    ${SOEM_LIB}
    quill::quill
    pthread
    rt
    z
)

target_compile_options(bridge_ros2 PRIVATE -Wall -Wextra)

install(TARGETS bridge_ros2 DESTINATION lib/${PROJECT_NAME})

ament_package()
