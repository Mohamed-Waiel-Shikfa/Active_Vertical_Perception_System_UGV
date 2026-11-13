"""Bounded acceptance on an owned live simulator; Nav2, never waypoint playback."""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from greenhouse.navigation_api import request
from greenhouse.transport import world_status


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--phase",choices=("obstacle","safety","updated-goal"),required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    directory=Path(os.environ.get("GREENHOUSE_STATE","/etc/clearpath/greenhouse"))
    runtime=json.loads((directory/"runtime.json").read_text())
    rclpy.init(args=[])
    node=Node("navigation_acceptance")
    prefix=("/"+runtime["namespace"].strip("/") if runtime["namespace"].strip("/") else "")+"/greenhouse"
    override=node.create_publisher(Empty,prefix+"/manual_override",10)
    report={"status":"FAIL","phase":args.phase,"checks":[]}
    stopped_pid=None

    def spin(seconds=.1):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)

    def wait(condition,timeout,message):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            value=request("status")
            if condition(value):
                return value
            spin(.2)
        raise TimeoutError(message)

    def goal(x,y,yaw):
        start=wait(lambda s:s["ready"],30,"Native truth/lifecycle not ready")
        request("go",{"x":x,"y":y,"yaw_deg":yaw})
        end=wait(lambda s:s["phase"] in ("Succeeded","Error","Canceled"),910,"Goal deadline")
        if end["phase"]!="Succeeded":
            raise RuntimeError(end["message"])
        result=end["last_result"]
        assert result["xy_error"]<=.10 and result["yaw_error"]<=math.radians(5)
        assert result["actual"]["robot_entity"]==start["pose"]["robot_entity"]
        check={"start":start,"target":[x,y,yaw],"result":end}
        report["checks"].append(check)
        print(json.dumps(check),flush=True)
        return end

    def moving():
        return wait(lambda s:s["active"]
                    and (abs(s["last_command"][0])>.05 or abs(s["last_command"][1])>.08)
                    and (math.hypot(s["velocity"]["vx"],s["velocity"]["vy"])>.03
                         or abs(s["velocity"]["wz"])>.03),90,
                    "Nav2 did not issue a measured-feedback motion command")

    def stopped():
        deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            physical=world_status()
            if physical["robot"].get("speed",1)<.02 and physical["robot"].get("angular_speed",1)<.03:
                return physical
            spin(.2)
        raise TimeoutError("Actual base did not stop")

    def along_aisle():
        current=wait(lambda s:s["ready"],30,"Feedback did not recover")
        p=current["pose"]
        request("go",{"x":p["x"]+.8,"y":p["y"],"yaw_deg":0})
        return moving()

    try:
        wait(lambda s:s["ready"],120,"Navigation failed lifecycle/native readiness")
        # A reliable publish before discovery matches the gateway's subscription is dropped;
        # goals now start moving within a fraction of a second.
        deadline=time.monotonic()+10
        while override.get_subscription_count()==0:
            if time.monotonic()>deadline:
                raise TimeoutError("Manual override subscriber not discovered")
            spin(.1)
        if args.phase=="obstacle":
            goal(-6.7,2.0,90)
        elif args.phase=="safety":
            before=along_aisle()
            override.publish(Empty())
            canceled=wait(lambda s:s["phase"]=="Canceled",6,"Manual override did not cancel")
            physical=stopped()
            assert canceled["last_command"]==[0.0,0.0]
            report["checks"].append({"manual_cancel":canceled,"actual_stop":physical,"before":before})

            along_aisle()
            processes=subprocess.run(["ps","-eo","pid,args"],check=True,text=True,capture_output=True).stdout
            pids=[int(line.split(None,1)[0]) for line in processes.splitlines()
                  if "__node:=greenhouse_sensor_bridge" in line and "parameter_bridge" in line]
            if len(pids)!=1:
                raise RuntimeError("Cannot uniquely identify this owned simulator's physics bridge")
            stopped_pid=pids[0]
            os.kill(stopped_pid,signal.SIGSTOP)
            failed=wait(lambda s:s["phase"]=="Error",7,"Missing native feedback did not stop navigation")
            assert "Stale native physics" in failed["message"]
            assert failed["last_command"]==[0.0,0.0]
            physical=stopped()
            report["checks"].append({"stale_feedback":failed,"actual_stop":physical,"bridge_pid":stopped_pid})
            os.kill(stopped_pid,signal.SIGCONT)
            stopped_pid=None
            wait(lambda s:s["ready"],20,"Restored native physics did not become fresh")
        else:
            before=along_aisle()
            physical_before=world_status()
            process=subprocess.run(["python3","/opt/greenhouse/tools/apply_terrain.py",
                                    "--values",'{"ground_bump_rms":0.006}'],
                                   capture_output=True,text=True,timeout=180)
            (args.output/"terrain-update.log").write_text(process.stdout+process.stderr)
            if process.returncode:
                raise RuntimeError(process.stderr)
            updated=wait(lambda s:s["ready"],20,"Updated collision map did not become ready")
            applied=json.loads((directory/"terrain-state.json").read_text())
            assert updated["revision"]==applied["revision"]!=before["revision"]
            assert not updated["active"] and updated["last_command"]==[0.0,0.0]
            spin(2)
            assert not request("status")["active"]
            report["checks"].append({"terrain_cancel":updated,"applied":applied,
                                     "before":physical_before,"after":world_status()})
            p=updated["pose"]
            goal(p["x"]+.65,p["y"],15)
            process=subprocess.run(["python3","/opt/greenhouse/tools/apply_terrain.py",
                                    "--values",'{"ground_bump_rms":0.003}'],
                                   capture_output=True,text=True,timeout=180)
            (args.output/"restore-default.log").write_text(process.stdout+process.stderr)
            if process.returncode:
                raise RuntimeError(process.stderr)
        report["status"]="PASS"
    except (OSError,ValueError,RuntimeError,TimeoutError,AssertionError,subprocess.SubprocessError) as error:
        report["error"]=str(error)
        raise
    finally:
        if stopped_pid is not None:
            os.kill(stopped_pid,signal.SIGCONT)
        try:
            request("cancel")
        finally:
            (args.output/"result.json").write_text(json.dumps(report,indent=2)+"\n")
            node.destroy_node()
            rclpy.shutdown()


if __name__=="__main__":
    main()
