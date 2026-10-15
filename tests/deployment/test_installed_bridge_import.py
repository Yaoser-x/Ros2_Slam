"""Source imports cannot establish that the installed bridge is executable."""

import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
ROS_SETUP = Path('/opt/ros/humble/setup.bash')


@pytest.mark.skipif(
    sys.platform != 'linux' or not ROS_SETUP.is_file() or shutil.which('colcon') is None,
    reason='requires a native ROS Humble colcon installation',
)
def test_bridge_imports_from_installed_overlay_without_source_pythonpath(tmp_path):
    environment = dict(os.environ)
    # pytest and the source-tree tests deliberately add package roots; the
    # installed process must obtain its Python path from generated ROS hooks.
    for name in ('PYTHONPATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        environment.pop(name, None)
    script = r'''
set -e
source /opt/ros/humble/setup.bash
colcon --log-base "$1/log" build --base-paths "$2/src/stm32_robot_bridge" \
  --build-base "$1/build" --install-base "$1/install" \
  --packages-select stm32_robot_bridge --cmake-args -DBUILD_TESTING=OFF
source "$1/install/local_setup.bash"
cd "$1"
python3 - "$1/install" <<'PY'
from pathlib import Path
import sys
import stm32_robot_bridge
from stm32_robot_bridge.bridge_core import BridgeCore
from stm32_robot_bridge.protocol_v3 import CommandStream
from stm32_robot_bridge.transport_supervisor import SerialTransportSupervisor
origin = Path(stm32_robot_bridge.__file__).resolve()
assert origin.is_relative_to(Path(sys.argv[1]).resolve()), origin
print('INSTALLED_BRIDGE_IMPORT_PASS', origin)
PY
'''
    result = subprocess.run(
        ['bash', '-c', script, 'installed-bridge-check', str(tmp_path), str(ROOT)],
        cwd=tmp_path,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=120,
    )
    assert result.returncode == 0, result.stdout
    assert 'INSTALLED_BRIDGE_IMPORT_PASS' in result.stdout
