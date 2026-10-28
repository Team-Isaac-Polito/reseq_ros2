#!/usr/bin/env bash
set -eo pipefail

task_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$task_ws/install/setup.bash"
exec "$HOME/.venvs/reseq-traction/bin/python" -m reseq_traction_autonomy.mcp_server
