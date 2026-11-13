"""Explicit rollback of one observed paused, uncommitted support revision."""
import argparse
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from greenhouse.navigation_api import request
from greenhouse.terrain_update import support_points,validate_support
from greenhouse.transport import service,settings,world_status,wheel_contact
from apply_terrain import terrain,create_ground,remove_ground,update_poses,wait_status


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--observed-revision",required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    directory=Path(os.environ.get("GREENHOUSE_STATE","/etc/clearpath/greenhouse"))
    active=json.loads((directory/"terrain-state.json").read_text())
    actuator=json.loads((directory/"actuator.json").read_text())
    old_assets=Path(active["assets_dir"])
    old_manifest=json.loads((old_assets/"manifest.json").read_text())
    staged=directory/"terrain-revisions"/args.observed_revision
    expected_name="greenhouse_ground_"+args.observed_revision
    before=world_status()
    nav=request("status")
    if (not before["paused"] or len(before["grounds"])!=1
            or before["grounds"][0]["name"]!=expected_name or nav["active"]):
        raise RuntimeError("Observed paused support/navigation differs; no recovery mutation sent")
    new_assets=staged/"assets"
    old_surface,new_surface=terrain(old_assets),terrain(new_assets)
    old_config=active.get("config",old_manifest["config"])
    proposed=json.loads((new_assets/"manifest.json").read_text())["config"]
    if any(proposed[key]!=old_config[key] for key in ("ground_friction","slip_longitudinal","slip_lateral")):
        raise RuntimeError("Recovery requires separately observed contact reconciliation for changed coefficients")
    robot=before["robot"]
    validate_support(old_surface,old_config,robot)
    shift=max(new_surface.sample(x,y)-old_surface.sample(x,y) for x,y in support_points(robot))+.002
    restored_robot=dict(robot,z=robot["z"]-shift)
    if (staged/"transaction.json").is_file():
        journal=json.loads((staged/"transaction.json").read_text())
        restored_robot=journal["before"]["robot"]
        if abs(restored_robot["x"]-robot["x"])>1e-8 or abs(restored_robot["y"]-robot["y"])>1e-8:
            raise RuntimeError("Robot XY differs from the paused journal; no recovery mutation sent")
    report={"status":"FAIL","before":before,"navigation_before":nav,"inverse_z_shift_m":shift,
            "operations":[],"metadata_before":active}

    def call(name,action):
        started=time.monotonic()
        result=action()
        report["operations"].append({"name":name,"seconds":time.monotonic()-started,"acknowledged":True})
        args.output.write_text(json.dumps(report,indent=2)+"\n")
        return result

    try:
        call("hold actuator",lambda:settings(actuator["prefix"],{"blocked":True}))
        call("create last applied support",lambda:create_ground(active["name"],staged/"old.sdf"))
        wait_status(lambda s:s["paused"] and any(g["name"]==active["name"] for g in s["grounds"]))
        call("remove observed uncommitted support",lambda:remove_ground(expected_name))
        call("restore last applied contact",lambda:wheel_contact(robot["name"],old_config))
        call("restore plant and base support Z only",lambda:update_poses(old_manifest["plants"],restored_robot))
        after=wait_status(lambda s:s["paused"] and len(s["grounds"])==1 and s["grounds"][0]["name"]==active["name"])
        assert after["sim_time"]==before["sim_time"] and after["robot"]["id"]==robot["id"]
        assert abs(after["robot"]["x"]-robot["x"])<1e-8 and abs(after["robot"]["y"]-robot["y"])<1e-8
        call("restore last applied map",lambda:request("rollback_update",{
            "assets_dir":str(old_assets),"revision":active.get("revision","startup")}))
        call("release actuator hold",lambda:settings(actuator["prefix"],{"blocked":False}))
        call("resume verified single support",lambda:service("/world/greenhouse/control",
                  "gz.msgs.WorldControl",request="pause: false"))
        resumed=wait_status(lambda s:not s["paused"] and s["sim_time"]>before["sim_time"])
        assert resumed["robot"]["id"]==robot["id"]
        assert json.loads((directory/"terrain-state.json").read_text())==active
        report.update({"status":"PASS","after":resumed,"navigation_after":request("status")})
    finally:
        args.output.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))


if __name__=="__main__":
    main()
