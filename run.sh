#!/usr/bin/env bash
# Configure, build, and launch the firmware, forwarding any arguments.
#
#   ./run.sh --setpoint 45 --kp 2.5 --ki 0.1 --kd 0.05

set -euo pipefail

BUILD_DIR="build"

cmake -S . -B "${BUILD_DIR}"
cmake --build "${BUILD_DIR}" -j"$(nproc)"

echo
"./${BUILD_DIR}/eae_firmware" "$@"
