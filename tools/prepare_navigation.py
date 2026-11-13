"""Prepare known collider map and add one lower-priority navigation mux source."""
import json
import os
import sys
from pathlib import Path
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from greenhouse.navigation_map import prepare_navigation


def configure_mux(parameters):
    priority=parameters.get("topics.external.priority")
    if not isinstance(priority,int) or priority<1 or parameters.get("topics.external.topic")!="cmd_vel":
        raise RuntimeError("Unsupported Clearpath manual mux entry; cannot safely add navigation")
    if priority==1:
        parameters["topics.external.priority"]=2
    parameters.update({"topics.navigation.topic":"greenhouse/nav_cmd",
                       "topics.navigation.timeout":0.5,"topics.navigation.priority":1})


def main():
    setup=Path(os.environ.get("SETUP_PATH","/etc/clearpath"))
    directory=setup/"greenhouse"
    state=json.loads((directory/"runtime.json").read_text())
    actuator=json.loads((directory/"actuator.json").read_text())
    prepare_navigation(state["assets_dir"],directory,state["namespace"],actuator)
    mux=setup/"platform/config/twist_mux.yaml"
    data=yaml.safe_load(mux.read_text())
    found=False
    for namespace in data.values():
        for name,node in namespace.items():
            if name=="twist_mux":
                parameters=node["ros__parameters"]
                configure_mux(parameters)
                found=True
    if not found:
        raise RuntimeError("Generated Clearpath twist_mux parameters were not found")
    mux.write_text(yaml.safe_dump(data,sort_keys=False))
    print("Known collision map and lower-priority navigation source prepared.")


if __name__=="__main__":
    main()
