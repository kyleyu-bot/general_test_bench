#!/bin/bash
# Build the dyno_ros2_bridge colcon package.
# Run from the repo root: bash src/interface_bridges/ros2/build.sh

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO_ROOT"

# Pin the build type explicitly so optimization never depends on whatever is
# cached in CMakeCache.txt — the RT cycle path must not silently end up -O0.
BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"

echo "==> Building parent CMake project (ethercat_core + SOEM, ${BUILD_TYPE})..."
cmake -S . -B build -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cmake --build build -j"$(nproc)"

echo "==> Building dyno_ros2_bridge with colcon (${BUILD_TYPE})..."
source /opt/ros/humble/setup.bash

# install/ and log/ at the repo root are root-owned from historical sudo
# builds; build/dyno_ros2_bridge/ may contain root-owned cache files.
# Build into scratch bases and deploy only the binary — the launchers run
# build/dyno_ros2_bridge/bridge_ros2 directly, install/ is never used.
SCRATCH="/tmp/dyno_colcon_${USER}"
colcon --log-base "$SCRATCH/log" \
       build --packages-select dyno_ros2_bridge \
             --base-paths src/interface_bridges/ros2 \
             --build-base   "$SCRATCH/build" \
             --install-base "$SCRATCH/install" \
             --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

# Atomic deploy (cp + mv) so a running bridge doesn't cause ETXTBSY mid-copy.
mkdir -p build/dyno_ros2_bridge
cp "$SCRATCH/build/dyno_ros2_bridge/bridge_ros2" build/dyno_ros2_bridge/bridge_ros2.new
mv -f build/dyno_ros2_bridge/bridge_ros2.new build/dyno_ros2_bridge/bridge_ros2

echo ""
echo "Done. Deployed $SCRATCH/build/dyno_ros2_bridge/bridge_ros2"
echo "  -> build/dyno_ros2_bridge/bridge_ros2"
echo "To run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  sudo -E ./build/dyno_ros2_bridge/bridge_ros2"
