#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${SALUS_REAL_IMAGE:-salus-robot:humble-real}"

docker image inspect "${image}" --format \
  'image={{.Id}} created={{.Created}} arch={{.Architecture}} os={{.Os}}'

# The source tree is read-only; build/install/log are disposable container
# filesystems.  Network and device access are deliberately absent: this gate
# validates packaging and dynamic linking, never the physical RS16.
docker run --rm --network none \
  -v "${repo_dir}/src:/input/src:ro" \
  --tmpfs /ros2_ws/build \
  --tmpfs /ros2_ws/install \
  --tmpfs /ros2_ws/log \
  "${image}" bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    set -u

    mkdir -p /ros2_ws/src
    cp -a /input/src/. /ros2_ws/src/

    test -d /ros2_ws/src/rslidar_sdk || {
      echo "missing imported source: src/rslidar_sdk" >&2
      exit 1
    }
    test -d /ros2_ws/src/rslidar_msg || {
      echo "missing imported source: src/rslidar_msg" >&2
      exit 1
    }

    # Build the workspace dependency closure needed by the target executable;
    # this keeps the runtime gate focused and fast while still compiling the
    # pinned rslidar_msg -> rslidar_sdk chain from source.
    colcon build --packages-up-to rslidar_sdk --symlink-install --event-handlers console_direct+
    set +u
    source /ros2_ws/install/setup.bash
    set -u

    prefix="$(ros2 pkg prefix rslidar_sdk)"
    target="${prefix}/lib/rslidar_sdk/rslidar_sdk_node"
    if [[ ! -e "${target}" ]]; then
      echo "installed target missing: ${target}" >&2
      ls -la "${prefix}/lib/rslidar_sdk" >&2 || true
      find /ros2_ws/build /ros2_ws/install -name rslidar_sdk_node -ls >&2 || true
      exit 1
    fi
    test -r "${target}" || {
      echo "rslidar_sdk_node is not readable under ${prefix}" >&2
      exit 1
    }

    echo "workspace_arch=$(uname -m)"
    echo "package_arch=$(dpkg --print-architecture)"
    echo "target=${target}"
    echo "--- ldd ${target} ---"
    ldd_output="$(ldd "${target}")"
    printf "%s\\n" "${ldd_output}"

    if grep -Fq "not found" <<<"${ldd_output}"; then
      echo "ERROR: unresolved shared library" >&2
      exit 1
    fi

    pcap_line="$(grep -E "^libpcap\\.so\\.0\\.8 => /" <<<"${ldd_output}" || true)"
    test -n "${pcap_line}" || {
      echo "ERROR: libpcap.so.0.8 is not resolved by the loader" >&2
      exit 1
    }
    echo "PASS: all rslidar_sdk_node shared libraries resolved"
  '
