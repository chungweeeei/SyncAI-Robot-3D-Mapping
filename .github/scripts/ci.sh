#!/usr/bin/env bash
# The in-container half of .github/workflows/ci.yml: import dependencies, build, test and report, against a colcon
# workspace at /workspace with this repo at /workspace/src/syncai_mapping.
#
# It runs under the image's entrypoint, which has already sourced ROS 2. It can also be run by
# hand inside the Dev Container to reproduce CI exactly:
#
#   bash /workspace/src/syncai_mapping/.github/scripts/ci.sh
set -euo pipefail

cd /workspace

# syncai_common and small_gicp live in other repos; vcs import puts them next to this package.
# --skip-existing so a Dev Container that already has them is left alone.
vcs import --skip-existing < src/syncai_mapping/dependencies.repos

# The whole workspace, not --packages-select: syncai_common and small_gicp have to be built before
# this package. --base-paths src so colcon only ever crawls the source tree. console_cohesion
# prints each package's full output once it ends, so a compiler error is readable in the log
# instead of interleaved.
colcon build --base-paths src --symlink-install --event-handlers console_cohesion+

# Only this package's tests (ament_lint_auto): the dependencies' own test suites are not ours.
colcon test --base-paths src --packages-select syncai_mapping --event-handlers console_cohesion+
# Exits non-zero when any test failed, and prints the failures
colcon test-result --test-result-base build/syncai_mapping --verbose
