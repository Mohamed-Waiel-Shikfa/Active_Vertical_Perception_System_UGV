# SPDX-License-Identifier: GPL-3.0-only
"""Bounded native Gazebo calls; responses are checked, never success-shaped defaults."""
import json
import atexit
import re
import select
import subprocess
import sys
import threading
import time
import itertools
import os
from pathlib import Path
from typing import Any

_client = None
_lock = threading.Lock()
_sequence=itertools.count(1)
_seen_services=set()


def trace_request(record: dict[str, Any]) -> None:
    """Append one RPC record to $GREENHOUSE_STATE/native-rpc.jsonl; skipped if that directory is absent.

    Write failures are reported on stderr, never raised.
    """
    path=Path(os.environ.get("GREENHOUSE_STATE","/etc/clearpath/greenhouse"))/"native-rpc.jsonl"
    if not path.parent.is_dir():
        return
    try:
        with path.open("a") as stream:
            stream.write(json.dumps(record)+"\n")
    except OSError as error:
        print(f"Could not persist native RPC trace: {error}",file=sys.stderr)


def close_client() -> None:
    """Stop the shared native service session, allowing 2 s wall-clock before terminate and again before kill."""
    global _client
    if _client is None:
        return
    process,_client=_client,None
    process.stdin.close()
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    process.stdout.close()


atexit.register(close_client)


def service(name: str, request_type: str = "gz.msgs.Empty", reply_type: str = "gz.msgs.Boolean", request: str = "",
            timeout: float = 8) -> Any:
    """Call a Gazebo service through the persistent native session; `timeout` is wall-clock seconds.

    Returns True for gz.msgs.Boolean, or the decoded JSON payload for gz.msgs.StringMsg. Raises TimeoutError,
    RuntimeError on a failed or unacknowledged call, and ValueError for any other reply type.
    """
    global _client
    with _lock:
        started=time.monotonic()
        record={"id":f"{os.getpid()}:{next(_sequence)}","service":name,"request_type":request_type,
                "request":request,"first_for_service":name not in _seen_services,
                "timeout_s":timeout,"wall_time":time.time()}
        _seen_services.add(name)
        if _client is None or _client.poll() is not None:
            close_client()
            _client=subprocess.Popen(["/opt/greenhouse/native/bin/greenhouse-service","--session"],
                                     stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
        message={"service":name,"request_type":request_type,"reply_type":reply_type,
                 "timeout_ms":int((timeout-1)*1000),"text":request}
        _client.stdin.write(json.dumps(message)+"\n")
        _client.stdin.flush()
        if not select.select([_client.stdout],[],[],timeout)[0]:
            trace_request(dict(record,outcome="client_deadline",seconds=time.monotonic()-started))
            close_client()
            raise TimeoutError(f"{name}: Native service session timed out")
        line=_client.stdout.readline()
        if not line:
            trace_request(dict(record,outcome="client_exited",seconds=time.monotonic()-started))
            close_client()
            raise RuntimeError(f"{name}: Native service session exited without a response")
        result=json.loads(line)
        if result.get("ok") is not True:
            trace_request(dict(record,outcome="native_error",error=result.get("error"),
                               seconds=time.monotonic()-started))
            raise RuntimeError(f"{name}: {result.get('error','Native request failed')}")
        trace_request(dict(record,outcome="acknowledged",seconds=time.monotonic()-started))
        output=result["text"]
    if reply_type == "gz.msgs.Boolean":
        if not re.search(r"\bdata:\s*true\b", output):
            raise RuntimeError(f"{name} did not acknowledge success: {output.strip()}")
        return True
    if reply_type == "gz.msgs.StringMsg":
        match = re.search(r'data:\s*("(?:[^"\\]|\\.)*")', output)
        if not match:
            raise RuntimeError(f"{name} returned no JSON state: {output.strip()}")
        return json.loads(json.loads(match.group(1)))
    raise ValueError(f"unsupported service response type: {reply_type}")


def settings(prefix: str, values: dict[str, Any]) -> dict[str, Any]:
    """Send lift settings to `prefix`/lift/settings and return the native reply; raises ValueError if rejected."""
    response = service(prefix + "/lift/settings", "gz.msgs.StringMsg", "gz.msgs.StringMsg",
                       "data: " + json.dumps(json.dumps(values)))
    if response.get("ok") is not True:
        raise ValueError(response.get("error", "actuator settings rejected"))
    return response


def world_status() -> Any:
    """Decoded JSON from /greenhouse/world/status, retried once after 0.05 s if the first call times out."""
    try:
        return service("/greenhouse/world/status", reply_type="gz.msgs.StringMsg")
    except (subprocess.TimeoutExpired, RuntimeError, TimeoutError) as error:
        if isinstance(error, RuntimeError) and "timed out" not in str(error).lower():
            raise
        print("Native world-status read timed out; one bounded fresh-discovery retry.",file=sys.stderr)
        time.sleep(.05)
        return service("/greenhouse/world/status", reply_type="gz.msgs.StringMsg")


def wheel_contact(robot_name: str, config: dict[str, Any]) -> bool:
    """Apply config["slip_lateral"] and config["slip_longitudinal"] wheel-slip compliance to the model.

    Returns True once Gazebo acknowledges; raises as `service` does.
    """
    return service("/world/greenhouse/wheel_slip/blocking", "gz.msgs.WheelSlipParametersCmd",
                   request=f"entity {{ name: {json.dumps(robot_name)} type: MODEL }} "
                   f"slip_compliance_lateral: {config['slip_lateral']} "
                   f"slip_compliance_longitudinal: {config['slip_longitudinal']}")
