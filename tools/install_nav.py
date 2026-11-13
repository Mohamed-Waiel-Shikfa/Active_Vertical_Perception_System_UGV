"""Install a consistent Jazzy Nav2 1.3.13 patch release; record exact apt builds."""
import json
import subprocess
from pathlib import Path

PACKAGES = Path("/opt/greenhouse/config/nav-packages.txt")


def main():
    subprocess.run(["apt-get", "update", "-qq"], check=True, timeout=180)
    installed = subprocess.run(["dpkg-query", "-W", "-f=${Package}\n", "ros-jazzy-nav2-*"],
                               check=True, text=True, capture_output=True).stdout.split()
    names = sorted(set(PACKAGES.read_text().split()) | set(installed))
    locked=json.loads(PACKAGES.with_name("nav-packages.lock.json").read_text())
    pins=locked["pins"]
    versions_by_name=dict(pin.split("=",1) for pin in pins)
    if set(names)-versions_by_name.keys():
        raise RuntimeError("Exact Nav2 dependency lock omits required/base family packages")
    for name,version in versions_by_name.items():
        result = subprocess.run(["apt-cache", "madison", name], check=True, text=True, capture_output=True)
        versions = [line.split("|")[1].strip() for line in result.stdout.splitlines() if "|" in line]
        if version not in versions:
            raise RuntimeError(f"Locked compatible binary {name}={version} is unavailable: {versions}. Update and verify the lock explicitly.")
    subprocess.run(["apt-get", "install", "-y", "--no-install-recommends", *pins], check=True, timeout=300)
    installed = subprocess.run(["dpkg-query", "-W", "-f=${Package} ${Version}\n", *versions_by_name],
                               check=True, text=True, capture_output=True).stdout
    for line in installed.splitlines():
        name,version=line.split()
        if versions_by_name[name]!=version:
            raise RuntimeError(f"Installed dependency differs from exact pin: {line}")
    library=Path("/opt/ros/jazzy/lib/libbehaviortree_cpp.so").resolve()
    symbols=subprocess.run(["nm","-D",str(library)],check=True,text=True,capture_output=True).stdout
    required=["_ZN2BT18StatusChangeLoggerC2EPNS_8TreeNodeE","_ZN2BT18StatusChangeLoggerD2Ev"]
    if any(symbol not in symbols for symbol in required):
        raise RuntimeError("BehaviorTree.CPP does not export Nav2's required runtime ABI")
    linkage=subprocess.run(["ldd","-r","/opt/ros/jazzy/lib/libnav2_navigate_to_pose_navigator.so"],
                           check=True,text=True,capture_output=True)
    if "undefined symbol:" in linkage.stdout+linkage.stderr or "not found" in linkage.stdout:
        raise RuntimeError("Nav2 runtime linkage is incomplete:\n"+linkage.stdout+linkage.stderr)
    dynamic=subprocess.run(["readelf","-d",str(library)],check=True,text=True,capture_output=True).stdout
    abi={"resolved_library":str(library),"soname":[line.strip() for line in dynamic.splitlines() if "(SONAME)" in line],
         "required_symbols":required,"ldd_unresolved":False}
    Path("/opt/greenhouse/nav-packages.lock.json").write_text(
        json.dumps({"requested_pins": pins, "installed": installed.splitlines(),"behavior_tree_abi":abi}, indent=2)+"\n")
    subprocess.run(["apt-get", "clean"], check=True)


if __name__ == "__main__":
    main()
