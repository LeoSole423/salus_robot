#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

headless=false
rviz=false
adaptive_dense=false
adaptive_dense_leg_max_m="8.0"
adaptive_dense_horizon_m="35.0"
generic_world=false
for option in "$@"; do
  case "${option}" in
    --headless) headless=true ;;
    --rviz) rviz=true ;;
    --adaptive-dense) adaptive_dense=true ;;
    --adaptive-dense-leg-max-m=*)
      adaptive_dense_leg_max_m="${option#*=}"
      ;;
    --adaptive-dense-horizon-m=*)
      adaptive_dense_horizon_m="${option#*=}"
      ;;
    --generic) generic_world=true ;;
    *)
      echo "Usage: ./tools/sim_operational.sh [--headless] [--rviz] [--adaptive-dense] [--adaptive-dense-leg-max-m=<meters>] [--adaptive-dense-horizon-m=<meters>] [--generic]" >&2
      exit 2
      ;;
  esac
done

if [[ "${headless}" == "true" && "${rviz}" == "true" ]]; then
  echo "--rviz cannot be used with --headless" >&2
  exit 2
fi

route_args=""
if [[ "${adaptive_dense}" == "true" ]]; then
  route_args=" route_execution_mode:=adaptive_dense adaptive_dense_leg_max_m:=${adaptive_dense_leg_max_m} adaptive_dense_horizon_m:=${adaptive_dense_horizon_m}"
fi

georeferenced_args=""
private_route_file="${repo_dir}/artifacts/PatrullaSencillaPolo.private.json"
profile_dir="${repo_dir}/artifacts/georeferenced-patrulla-sencilla-polo-default"
if [[ "${generic_world}" == "false" && -f "${private_route_file}" ]]; then
  if [[ ! -f "${profile_dir}/launch_args_private.txt" ]]; then
    python3 "${repo_dir}/tools/prepare_georeferenced_patrol_sim.py" \
      --routes-file "${private_route_file}" \
      --route-name PatrullaSencillaPolo \
      --source-world "${repo_dir}/src/salus_simulation/worlds/free.world" \
      --output-dir "${profile_dir}" >/dev/null
  fi
  while IFS= read -r launch_arg; do
    [[ -n "${launch_arg}" ]] && georeferenced_args+=" ${launch_arg}"
  done < "${profile_dir}/launch_args_private.txt"
  echo "Starting georeferenced PatrullaSencillaPolo at waypoint 1."
elif [[ "${generic_world}" == "false" ]]; then
  echo "Private PatrullaSencillaPolo route not found; starting generic simulation."
fi

docker compose up -d --build

if [[ "${headless}" == "false" ]]; then
  xhost +local:docker >/dev/null
  trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT
fi

echo "Cockpit backend available at ws://localhost:8766"
echo "In the Cockpit repository, run: git switch migration/salus-robot-cockpit && npm run dev"

docker compose exec ros2 bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  cd /ros2_ws
  colcon build --symlink-install --packages-up-to salus_bringup
  source /ros2_ws/install/setup.bash
  exec ros2 launch salus_bringup sim_operational.launch.py headless:=${headless} rviz:=${rviz}${route_args}${georeferenced_args}
"
