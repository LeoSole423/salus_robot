#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

visual=true
cockpit=false
adaptive_dense=false
adaptive_dense_leg_max_m="8.0"
generic_world=false
for option in "$@"; do
  case "${option}" in
    --headless)
      visual=false
      ;;
    --cockpit)
      cockpit=true
      ;;
    --adaptive-dense)
      adaptive_dense=true
      ;;
    --adaptive-dense-leg-max-m=*)
      adaptive_dense_leg_max_m="${option#*=}"
      ;;
    --generic)
      generic_world=true
      ;;
    *)
      echo "Usage: ./tools/sim.sh [--headless] [--cockpit] [--adaptive-dense] [--adaptive-dense-leg-max-m=<meters>] [--generic]" >&2
      exit 2
      ;;
  esac
done

if [[ "${cockpit}" == "true" ]]; then
  operational_args=()
  [[ "${visual}" == "false" ]] && operational_args+=(--headless)
  [[ "${adaptive_dense}" == "true" ]] && operational_args+=(--adaptive-dense)
  [[ "${adaptive_dense}" == "true" ]] && operational_args+=("--adaptive-dense-leg-max-m=${adaptive_dense_leg_max_m}")
  [[ "${generic_world}" == "true" ]] && operational_args+=(--generic)
  exec "${repo_dir}/tools/sim_operational.sh" "${operational_args[@]}"
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

if [[ "${visual}" == "true" ]]; then
  xhost +local:docker >/dev/null
  trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT
  launch_args="gz_args:=-r rviz:=true"
else
  launch_args="gz_args:=-r\ -s rviz:=false"
fi
if [[ "${adaptive_dense}" == "true" ]]; then
  launch_args+=" route_execution_mode:=adaptive_dense adaptive_dense_leg_max_m:=${adaptive_dense_leg_max_m} adaptive_dense_horizon_m:=35.0"
fi
launch_args+="${georeferenced_args}"

docker compose exec ros2 bash -lc "
  set -e
  source /opt/ros/humble/setup.bash
  cd /ros2_ws
  colcon build --symlink-install --packages-up-to salus_bringup
  source /ros2_ws/install/setup.bash
  exec ros2 launch salus_bringup integration_sim.launch.py ${launch_args}
"
