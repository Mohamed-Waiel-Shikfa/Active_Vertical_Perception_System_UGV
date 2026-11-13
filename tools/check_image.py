"""Fail the build if the pinned Clearpath/SDF contract is not present."""
import hashlib
import json
import os
import subprocess
from pathlib import Path

from clearpath_config.clearpath_config import ClearpathConfig

ROOT = Path(__file__).resolve().parents[1]


def main():
    config = ClearpathConfig(str(ROOT / "config" / "robot.example.yaml"))
    assert config.get_platform_model() == "j100"
    assert str(config.system.namespace) == "j100_0000"
    paths = [Path("/ros_entrypoint.sh")]
    paths += [Path("/opt/ros/jazzy/share/clearpath_gz/launch") / name
              for name in ("simulation.launch.py", "gz_sim.launch.py", "robot_spawn.launch.py")]
    spawn = paths[-1].read_text()
    for token in ("'generate'", "'world'", "'setup_path'", "'use_sim_time'", "'x'", "'y'", "'z'", "'yaw'",
                  "clearpath_generator_common", "clearpath_generator_gz", "platform-service.launch.py"):
        if token not in spawn:
            raise RuntimeError(f"unsupported Clearpath robot_spawn contract: missing {token}")
    environment = dict(os.environ)
    environment["GZ_SIM_RESOURCE_PATH"] = str(ROOT / "generated" / "models")
    # The standalone sdformat CLI has no Gazebo resource-resolver callback.
    environment["SDF_PATH"] = str(ROOT / "generated" / "models")
    for path in sorted((ROOT / "generated").rglob("*.sdf")):
        result = subprocess.run(["gz", "sdf", "-k", str(path)], env=environment,
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        print(result.stdout)
        if result.returncode:
            raise RuntimeError(f"libsdformat rejected {path}")
    contract = {
        "base_image": "ghcr.io/clearpathrobotics/clearpath_docker:jazzy-sim-latest",
        "base_digest": "sha256:0355e4bb50d50dddfdb2416365c4f155e28eb7ff3a9a53de3c3d42864eba0979",
        "source_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},
    }
    (ROOT / "base-contract.json").write_text(json.dumps(contract, indent=2) + "\n")
    print("Pinned Clearpath contract and generated SDF checks passed.")


if __name__ == "__main__":
    main()
