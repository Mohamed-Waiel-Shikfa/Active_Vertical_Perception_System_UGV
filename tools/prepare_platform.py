"""Apply config/platform_overrides.yaml to Clearpath's generated platform parameter files.

Only the overridden lines are rewritten: a YAML round trip would turn the generated
``max_jerk: nan`` into the string 'nan' and the controller would reject its parameters.
"""
import os
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
OVERRIDES = ROOT / "config" / "platform_overrides.yaml"


def flatten(values: dict, prefix: str = "") -> dict:
    """Nested ROS parameter dicts as dotted names: {"linear": {"x": 1}} -> {"linear.x": 1}."""
    flat = {}
    for key, value in values.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            flat.update(flatten(value, name + "."))
        else:
            flat[name] = value
    return flat


def user_extras(robot: dict) -> dict:
    """robot.yaml platform.extras.ros_parameters as {node: {dotted name: value}} ({} when unset)."""
    extras = ((robot.get("platform") or {}).get("extras") or {}).get("ros_parameters") or {}
    return {node: flatten(values or {}) for node, values in extras.items()}


def apply_overrides(text: str, overrides: dict, extras: dict) -> tuple[str, list[str]]:
    """Rewrite overridden ``name: value`` lines inside each node's block of one generated file.

    Keys that robot.yaml extras set are left alone. Returns the new text and the applied
    "node.name" entries. Raises KeyError when an override names a parameter the generated node
    lacks, so a Clearpath upgrade cannot silently drop one.
    """
    lines = text.splitlines(keepends=True)
    applied = []
    for node, wanted in overrides.items():
        for start, line in enumerate(lines):
            header = re.fullmatch(rf"(\s*){re.escape(node)}:\s*", line.rstrip("\n"))
            if not header:
                continue
            indent = len(header.group(1))
            end = start + 1
            while end < len(lines) and (not lines[end].strip() or
                                        len(lines[end]) - len(lines[end].lstrip()) > indent):
                end += 1
            for name, value in flatten(wanted).items():
                pattern = re.compile(rf"^(\s+){re.escape(name)}:\s.*$")
                rows = [i for i in range(start + 1, end) if pattern.match(lines[i].rstrip("\n"))]
                if len(rows) != 1:
                    raise KeyError(f"generated {node} has {len(rows)} lines for {name}, expected 1; "
                                   "check config/platform_overrides.yaml")
                if name in extras.get(node, {}):
                    continue
                row = rows[0]
                lines[row] = pattern.sub(rf"\g<1>{name}: {value}", lines[row].rstrip("\n")) + "\n"
                applied.append(f"{node}.{name}")
    return "".join(lines), applied


def main() -> None:
    setup = Path(os.environ.get("SETUP_PATH", "/etc/clearpath"))
    overrides = yaml.safe_load(OVERRIDES.read_text()) or {}
    extras = user_extras(yaml.safe_load((setup / "robot.yaml").read_text()) or {})
    applied = []
    for path in sorted((setup / "platform" / "config").glob("*.yaml")):
        text, changes = apply_overrides(path.read_text(), overrides, extras)
        if changes:
            path.write_text(text)
            applied += changes
    nodes = {entry.split(".", 1)[0] for entry in applied}
    kept = sorted(f"{node}.{name}" for node, names in extras.items() for name in names
                  if name in flatten(overrides.get(node, {})))
    missing = set(overrides) - nodes - {entry.split(".", 1)[0] for entry in kept}
    if missing:
        raise RuntimeError(f"no generated platform parameter file contains {sorted(missing)}")
    print(f"Platform overrides applied: {len(applied)}; kept from robot.yaml: {kept or 'none'}.")


if __name__ == "__main__":
    main()
