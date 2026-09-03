#!/usr/bin/env bash
set -euo pipefail

# External ROS packages remain VCS imports, not tracked vendor copies.  Run
# this through the development image so CI and native workspace setup use the
# same vcstool/git tooling and initialize the SDK's pinned rs_driver gitlink.
docker compose run --rm ros2 bash -lc \
  'set -e; vcs import --input dependencies.repos . --skip-existing; git -C src/rslidar_sdk submodule update --init --recursive; test "$(git -C src/rslidar_sdk rev-parse HEAD)" = "7c4ea25fada93442c3d390aa4ef05e240999b851"; test "$(git -C src/rslidar_sdk rev-parse HEAD:src/rs_driver)" = "cd358851ab65bf57fc7e321837be2a425305b298"; test "$(git -C src/rslidar_msg rev-parse HEAD)" = "fe8a95cb242bd294cc3d5e3422f2093fb49a56ee"'
