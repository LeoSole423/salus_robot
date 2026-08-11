#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

case "${1:-}" in
  straight) command='{twist: {linear: {x: 1.0}, angular: {z: 0.0}}, brake_pct: 0, source: 2}' ;;
  left) command='{twist: {linear: {x: 1.0}, angular: {z: 0.25}}, brake_pct: 0, source: 2}' ;;
  right) command='{twist: {linear: {x: 1.0}, angular: {z: -0.25}}, brake_pct: 0, source: 2}' ;;
  brake) command='{twist: {linear: {x: 0.0}, angular: {z: 0.0}}, brake_pct: 100, source: 3}' ;;
  *)
    echo "Usage: ./tools/cmd_vel_sim.sh {straight|left|right|brake}" >&2
    exit 2
    ;;
esac

if ! docker compose ps --status running --services | grep -qx ros2; then
  echo "La simulacion no esta activa. Ejecuta primero: ./tools/sim.sh" >&2
  exit 1
fi

if ! docker compose exec -T ros2 bash -lc \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 node list | grep -qx /controller_server'; then
  echo "El bringup no esta activo. Ejecuta ./tools/sim.sh y espera a que Gazebo cargue." >&2
  exit 1
fi

if [[ "${1}" == "brake" ]]; then
  publish_args=(--once)
else
  publish_args=(-r 10)
fi

docker compose exec ros2 bash -lc \
  "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec ros2 topic pub ${publish_args[*]} /cmd_vel_final salus_interfaces/msg/CmdVelFinal '${command}'"
