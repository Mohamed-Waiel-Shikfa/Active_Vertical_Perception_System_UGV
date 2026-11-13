"""One bounded native pause/settings/resume responsiveness check, with telemetry active."""
import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from greenhouse.transport import service,settings,world_status
from greenhouse.navigation_api import request


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    root=Path(os.environ.get("GREENHOUSE_STATE","/etc/clearpath/greenhouse"))
    actuator=json.loads((root/"actuator.json").read_text())
    rclpy.init(args=[])
    node=Node("paused_native_control_check")
    samples=[]
    node.create_subscription(String,actuator["prefix"]+"/lift/state",
                             lambda m:samples.append((time.monotonic(),json.loads(m.data))),10)
    def receive():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            return
    worker=threading.Thread(target=receive,daemon=True)
    worker.start()
    before=world_status()
    if before["robot"].get("speed",1)>.02:
        raise RuntimeError("Robot must already be stopped")
    results={"status":"FAIL","before":before,"settings_latency_s":[]}
    try:
        service("/world/greenhouse/control","gz.msgs.WorldControl",request="pause: true")
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            paused=world_status()
            if paused["paused"]:break
            time.sleep(.05)
        assert paused["paused"]
        start=time.monotonic()
        for blocked in (True,False):
            tick=time.monotonic()
            settings(actuator["prefix"],{"blocked":blocked})
            elapsed=time.monotonic()-tick
            results["settings_latency_s"].append(elapsed)
            assert elapsed<3,"Paused settings RPC exceeded the strict responsiveness bound"
            time.sleep(.65)
        frozen=world_status()
        duration=time.monotonic()-start
        messages=[s for stamp,s in samples if stamp>=start]
        results["telemetry_messages"]=len(messages)
        results["telemetry_duration_s"]=duration
        results["telemetry_hz"]=len(messages)/duration
        results["paused"]=frozen
        assert 5<results["telemetry_hz"]<27,"Paused telemetry is absent or flooding"
        assert frozen["sim_time"]==paused["sim_time"]
        assert frozen["robot"]["id"]==before["robot"]["id"]
        results["status"]="PAUSED_CHECKS_PASSED"
    finally:
        try:
            settings(actuator["prefix"],{"blocked":False})
            service("/world/greenhouse/control","gz.msgs.WorldControl",
                    request="pause: "+str(before["paused"]).lower())
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                observed=request("status")
                if observed["fresh"] and observed["pose"]["sim_time"]>paused["sim_time"]:
                    break
                time.sleep(.1)
            assert observed["fresh"] and observed["pose"]["sim_time"]>paused["sim_time"]
            assert observed["pose"]["robot_entity"]==before["robot"]["id"]
            results["after_native_feedback"]=observed
            if results["status"]=="PAUSED_CHECKS_PASSED":
                results["status"]="PASS"
        finally:
            args.output.write_text(json.dumps(results,indent=2)+"\n")
            rclpy.shutdown()
            worker.join(timeout=3)
            node.destroy_node()
    print(json.dumps(results,indent=2))


if __name__=="__main__":
    main()
