#!/bin/bash
# Build the jipt_ros2_bridge colcon package.
# Run from the repo root: bash src/interface_bridges/ros2/joint_impact_prevention_testbench_interface/build.sh

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "$REPO_ROOT"

echo "==> Building parent CMake project (ethercat_core + SOEM)..."
cmake -S . -B build
cmake --build build -j$(nproc)

echo "==> Building jipt_ros2_bridge with colcon..."
source /opt/ros/humble/setup.bash
colcon build --packages-select jipt_ros2_bridge \
             --base-paths src/interface_bridges/ros2/joint_impact_prevention_testbench_interface

echo ""
echo "Done. To run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  sudo -E ./build/jipt_ros2_bridge/bridge_ros2"
