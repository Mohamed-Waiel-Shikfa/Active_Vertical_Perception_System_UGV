"""One authority for simulated truth, goal lifecycle, command arbitration and map revision."""
import json
import math
import os
import signal
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from nav_msgs.msg import Odometry, Path as RosPath
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from greenhouse.navigation_map import RADIUS, create_map
from greenhouse.atomic import replace
from greenhouse.navigation_io import NavigationJournal
from greenhouse.speed_profile import speed_profile
from greenhouse.health import snapshot as health_snapshot
from sim_checks import Observer


# Skid steering on the soil turns at only ~28% of the commanded yaw rate (measured at
# 0.2-2.0 rad/s on default firm earth); refined online from steady navigation turns.
YAW_GAIN_PRIOR=0.28
# Stock Clearpath diff_drive_controller angular limit; applied turn commands never exceed it.
TURN_COMMAND_MAX=4.0
# The base's yaw rate lags its command; feeding back the measured rate brakes and starts turns
# promptly instead of overshooting the heading.
YAW_RATE_FEEDBACK=0.8
# Commanding far above the actual speed spins the wheels on soil and removes steering grip;
# forward commands stay at most this far ahead of the measured speed (traction control).
TRACTION_LEAD=0.5
# Bounded speed feedback holds the planned speed (it recovers the ~7% slip at cruise).
SPEED_FEEDBACK=1.0
SPEED_CORRECTION_MAX=0.2
# Path speed profile: lateral acceleration and actual yaw rate allowed in curves, and the
# braking used to slow down before curves and the goal (m/s^2, rad/s, m/s^2).
CURVE_LATERAL_ACCEL=0.8
CURVE_TURN_RATE=1.0
# Pure pursuit steers at a carrot one lookahead (lookahead_time x speed) ahead and cuts the
# inside of any curve shorter than that, so curves are also slow enough that the lookahead
# stays within this fraction of their radius (v <= fraction*R/lookahead_time). Without it the
# robot cut the 0.95 m row-end hairpin by 0.15 m into the end plant's inflated footprint.
CURVE_LOOKAHEAD_FRACTION=0.7
PATH_BRAKING=1.2
PATH_MIN_SPEED=0.15
# Curvature is measured over +/- this many 0.05 m path points; shorter spans see grid kinks.
CURVATURE_STRIDE=10
# Final in-place turn deceleration (rad/s^2). Spinning in place slides this base sideways
# less the faster it turns (180 deg: 4-9 cm at the 4 rad/s command, 10-18 cm at 1 rad/s), and
# forward commands barely act mid-spin, so in-place turns run at the full panel Turn rate.
FINAL_TURN_DECEL=1.0
# Docking: when the robot is within DOCK_DISTANCE (along the path) of the goal but still more
# than DOCK_MIN_TURN rad off the goal heading, it stops, turns to the goal heading there, then
# drives onto the goal pose, forward or backward, with the polar-coordinate unicycle pose
# controller (Siegwart et al., Introduction to Autonomous Mobile Robots, 3.6):
# v = k_rho rho, w = k_alpha alpha + k_beta beta, strongly stable for k_rho > 0, k_beta < 0,
# k_alpha + 5/3 k_beta - 2/pi k_rho > 0. It removes the sideways slide of the turn (4-24 cm
# measured for 180 deg), including a purely sideways offset, with a short S-move. Forward or
# reverse is decided once when the drive starts: re-deciding it each step orbits a goal that
# lies exactly beside the robot. The gains converge from 0.3 m sideways within the turn-rate
# limit and never swing wider than the starting offset (checked on an ideal unicycle).
DOCK_DISTANCE=0.7
DOCK_MIN_TURN=0.5
# Turning to the goal heading D before the goal leaves the robot D*|sin(turn)| beside the goal
# line, and the S-move swings ~0.7x that sideways offset along the goal heading: a 90 deg goal
# docked from 0.7 m swung 0.46 m into the plants across a 1.3 m aisle. The dock point moves in
# so that the offset stays below DOCK_LATERAL_MAX (a 180 deg reverse dock keeps the full 0.7 m).
DOCK_LATERAL_MAX=0.15
DOCK_ALIGNED=0.15
DOCK_SPEED=0.3
DOCK_MIN_SPEED=0.08
DOCK_GAIN_RHO=0.5
DOCK_GAIN_ALPHA=3.0
DOCK_GAIN_BETA=-1.0
DOCK_TURN_RATE=0.8
# "At rest" for goal success, the same limits a terrain/contact Apply requires (m/s, rad/s).
STOPPED_SPEED=0.02
STOPPED_TURN=0.03
# Back-off before planning: Smac cannot plan from a start inside the planning radius ("Start
# occupied"), and RPP refuses an in-place turn whose padded footprint would touch a plant, which
# the start of most goals needs (a spin has ~0.20 m of slack off an aisle centre line, and spins
# slide 4-24 cm). On Go, a robot closer than BACKOFF_ROOM outside the planning radius but clear by
# its physical radius first creeps straight forward or backward, toward the side with more room
# on the map, until it has that room or has moved BACKOFF_DISTANCE. It then plans if it is at
# least outside the planning radius, and is refused otherwise.
BACKOFF_SPEED=0.15
BACKOFF_PROBE=0.15
BACKOFF_ROOM=0.10
BACKOFF_DISTANCE=0.3
BACKOFF_SIM_TIMEOUT=3.0


def angle(value):
    return math.atan2(math.sin(value),math.cos(value))


class Gateway(Node):
    def __init__(self):
        super().__init__("greenhouse_navigation_gateway",
                         parameter_overrides=[Parameter("use_sim_time",value=True)])
        self.directory=Path(os.environ.get("GREENHOUSE_STATE","/etc/clearpath/greenhouse"))
        self.runtime=json.loads((self.directory/"runtime.json").read_text())
        self.health_observer=Observer(self.runtime,node_name="greenhouse_persistent_health")
        self.health_timer=self.health_observer.create_timer(.2,self.health_observer.poll_controllers,
                                                            clock=rclpy.clock.Clock())
        self.actuator=json.loads((self.directory/"actuator.json").read_text())
        self.ns=self.runtime["namespace"].strip("/")
        self.prefix="/"+self.ns if self.ns else ""
        self.nav_ns=self.prefix+"/navigation"
        self.nav_odom=(self.ns+"/" if self.ns else "")+"nav_odom"
        self.nav_base=(self.ns+"/" if self.ns else "")+"nav_base"
        self.lock=threading.RLock()
        self.feedback_group=MutuallyExclusiveCallbackGroup()
        self.control_group=MutuallyExclusiveCallbackGroup()
        self.lifecycle_group=MutuallyExclusiveCallbackGroup()
        self.latest={}
        self.last_state_wall=0.0
        self.last_cmd_wall=0.0
        self.last_command=(0.0,0.0)
        self.command_history=deque(maxlen=200)
        self.yaw_gain=YAW_GAIN_PRIOR
        self.turn_limit=TURN_COMMAND_MAX
        self.goal_handle=None
        self.nav2_result=None
        self.goal_pending=False
        self.dock=None
        self.dock_direction=None
        self.goal_number=0
        self.goal=None
        self.last_result=None
        self.goal_started=0.0
        self.goal_start_sim=0.0
        self.phase="Starting"
        self.error=""
        self.update_blocked=False
        self.cancel_future=None
        self.cancel_started=0.0
        self.cancel_failed=False
        self.revision="startup"
        self.current_assets=self.runtime["assets_dir"]
        self.plan_revision=0
        self.path=[]
        self.path_caps=[]
        self.path_s=[]
        self.stop_short=0.0
        self.dock_distance=DOCK_DISTANCE
        self.last_write=0.0
        self.last_log_sim=-1.0
        self.lifecycle={}
        self.lifecycle_futures={}
        self.lifecycle_updated={}
        self.lifecycle_requested={}
        self.last_lifecycle_poll=0.0
        self.nav_ready=False
        self.map_yaml=self.directory/"nav-map/map.yaml"
        self.map_image=None
        self.map_metadata=None
        self.load_local_map()
        self.load_motion_limits()
        self.trace=NavigationJournal(self.directory)
        self.odom_pub=self.create_publisher(Odometry,"/greenhouse/nav/odom",10)
        self.output=self.create_publisher(TwistStamped,self.prefix+"/greenhouse/nav_cmd",10)
        self.tf=TransformBroadcaster(self)
        self.static_tf=StaticTransformBroadcaster(self)
        identity=TransformStamped()
        identity.header.frame_id="greenhouse"
        identity.child_frame_id=self.nav_odom
        identity.transform.rotation.w=1.0
        self.static_tf.sendTransform(identity)
        self.create_subscription(String,"/greenhouse/physics",self.physics,qos_profile_sensor_data,
                                 callback_group=self.feedback_group)
        self.create_subscription(TwistStamped,self.prefix+"/cmd_vel",self.manual,10,callback_group=self.control_group)
        self.create_subscription(Empty,self.prefix+"/greenhouse/manual_override",lambda _:self.cancel_now("Manual override"),10,
                                 callback_group=self.control_group)
        self.create_subscription(TwistStamped,self.nav_ns+"/cmd_vel",self.nav_command,10,callback_group=self.control_group)
        self.create_subscription(RosPath,self.nav_ns+"/plan",self.plan,10)
        self.client=ActionClient(self,NavigateToPose,self.nav_ns+"/navigate_to_pose")
        self.map_client=self.create_client(LoadMap,self.nav_ns+"/map_server/load_map")
        self.param_client=self.create_client(SetParameters,self.nav_ns+"/controller_server/set_parameters")
        self.clear_clients=[self.create_client(ClearEntireCostmap,self.nav_ns+"/"+name+"/clear_entirely_"+name)
                            for name in ("global_costmap","local_costmap")]
        self.life_clients={name:self.create_client(GetState,self.nav_ns+"/"+name+"/get_state")
                           for name in ("map_server","planner_server","controller_server","bt_navigator")}
        self.timer=self.create_timer(0.05,self.watchdog,clock=rclpy.clock.Clock(),callback_group=self.control_group)
        self.lifecycle_timer=self.create_timer(1.0,self.poll_lifecycle,clock=rclpy.clock.Clock(),
                                              callback_group=self.lifecycle_group)

    def load_local_map(self):
        import yaml
        self.map_metadata=yaml.safe_load(self.map_yaml.read_text())
        self.map_audit=json.loads((self.map_yaml.parent/"map-audit.json").read_text())
        self.radius=self.map_audit["circumscribed_radius_m"]
        self.physical_radius=self.map_audit.get("robot_envelope",{}).get("physical_radius_m",RADIUS)
        self.map_image=cv2.imread(str(self.map_yaml.parent/self.map_metadata["image"]),cv2.IMREAD_GRAYSCALE)
        if self.map_image is None:
            raise RuntimeError("Known collision map could not be loaded")
        free=(self.map_image>200).astype(np.uint8)
        self.map_distance=cv2.distanceTransform(free,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*self.map_metadata["resolution"]

    def load_motion_limits(self):
        import yaml
        data=yaml.safe_load((self.directory/"navigation.yaml").read_text())
        follow=next(v for k,v in data.items() if k.endswith("/controller_server"))["ros__parameters"]["FollowPath"]
        # Controller maxima: panel Drive maximum and the fastest actual turn; each goal narrows them.
        self.max_limits={key:float(follow[key]) for key in ("desired_linear_vel","rotate_to_heading_angular_vel")}
        checker=next(v for k,v in data.items() if k.endswith("/controller_server"))["ros__parameters"]["goal_checker"]
        self.goal_xy_tolerance=float(checker["xy_goal_tolerance"])
        self.goal_yaw_tolerance=float(checker["yaw_goal_tolerance"])
        self.lookahead=(float(follow["lookahead_time"]),float(follow["min_lookahead_dist"]),
                        float(follow["max_lookahead_dist"]))
        self.limits=dict(self.max_limits)

    def goal_limits(self,values):
        speed,turn=values.get("max_speed"),values.get("max_turn")
        speed=self.max_limits["desired_linear_vel"] if speed is None else speed
        turn=TURN_COMMAND_MAX if turn is None else turn
        for name,value in (("Drive speed",speed),("Turn speed",turn)):
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")
            if value<.05:
                raise ValueError(f"{name} must be at least 0.05 to navigate")
        turn=min(float(turn),TURN_COMMAND_MAX)
        # Turn is a command rate like the arrow keys; the controller gets the actual rate it produces.
        return {"desired_linear_vel":min(float(speed),self.max_limits["desired_linear_vel"]),
                "rotate_to_heading_angular_vel":round(min(turn*self.yaw_gain,
                    self.max_limits["rotate_to_heading_angular_vel"]),3)},turn

    def apply_limits(self,limits):
        if limits==self.limits:
            return
        if not self.param_client.wait_for_service(timeout_sec=3):
            raise RuntimeError("Controller parameter service unavailable")
        request=SetParameters.Request()
        request.parameters=[ParameterMsg(name="FollowPath."+key,value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,double_value=value)) for key,value in limits.items()]
        response=self.wait_future(self.param_client.call_async(request))
        if response is None or not all(result.successful for result in response.results):
            raise RuntimeError("Controller rejected the requested navigation speed")
        with self.lock:
            self.limits=dict(limits)

    def fresh(self):
        return (bool(self.latest) and time.monotonic()-self.last_state_wall<1.5
                and abs(self.get_clock().now().nanoseconds*1e-9-self.latest["sim_time"])<.12
                and not self.latest.get("paused",False))

    def physics(self,message):
        value=json.loads(message.data)
        if value.get("source")!="Gazebo PostUpdate physical base_link":
            return
        with self.lock:
            if not all(isinstance(value.get(key),(int,float)) and math.isfinite(value[key])
                       for key in ("sim_time","x","y","yaw","nav_vx","nav_vy","nav_wz")):
                self.cancel_now("Invalid native physics feedback")
                self.last_state_wall=0
                self.phase="Error"
                return
            # Qt writes 0.0 as JSON 0; rclpy aborts the process on an int in a float64 field.
            value.update((key,float(value[key])) for key in ("sim_time","x","y","yaw","nav_vx","nav_vy","nav_wz"))
            if self.latest and value["sim_time"]<=self.latest["sim_time"]:
                if value["sim_time"]<self.latest["sim_time"]:
                    self.cancel_now("Simulation clock moved backwards")
                return
            self.latest=value
            self.last_state_wall=time.monotonic()
            self.estimate_yaw_gain(value)
            stamp=rclpy.time.Time(seconds=value["sim_time"]).to_msg()
            yaw=value["yaw"]
            transform=TransformStamped()
            transform.header.stamp=stamp;transform.header.frame_id=self.nav_odom;transform.child_frame_id=self.nav_base
            transform.transform.translation.x=value["x"];transform.transform.translation.y=value["y"]
            transform.transform.rotation.z=math.sin(yaw/2);transform.transform.rotation.w=math.cos(yaw/2)
            self.tf.sendTransform(transform)
            odom=Odometry()
            odom.header.stamp=stamp;odom.header.frame_id=self.nav_odom;odom.child_frame_id=self.nav_base
            odom.pose.pose.position.x=value["x"];odom.pose.pose.position.y=value["y"]
            odom.pose.pose.orientation=transform.transform.rotation
            odom.twist.twist.linear.x=value["nav_vx"]
            odom.twist.twist.linear.y=value["nav_vy"]
            odom.twist.twist.angular.z=value["nav_wz"]
            self.odom_pub.publish(odom)
            if self.goal and value["sim_time"]-self.last_log_sim>=0.1:
                self.last_log_sim=value["sim_time"]
                self.trace.write(json.dumps({"event":"feedback","sim_time":value["sim_time"],"goal":self.goal_number,
                    "pose":[value["x"],value["y"],yaw],"velocity":[value["nav_vx"],value["nav_vy"],value["nav_wz"]],
                    "command":self.last_command,"path_revision":self.plan_revision,
                    "map_revision":self.revision,"cross_track_m":self.cross_track(),"phase":self.phase})+"\n")

    def estimate_yaw_gain(self,value):
        # Only steady turn commands, 0.2-0.7 s old to cover drive latency, refine the gain. Not
        # while docking: a 90 deg dock holds the turn command saturated while the base barely
        # pivots, which dragged the gain to its 0.15 floor and slowed every later turn.
        if self.phase!="Navigating" or self.dock:
            return
        window=[w for t,w in self.command_history if value["sim_time"]-.7<=t<=value["sim_time"]-.2]
        if len(window)<2:
            return
        mean=sum(window)/len(window)
        if abs(mean)<.3 or max(window)-min(window)>.15*abs(mean):
            return
        ratio=value["nav_wz"]/mean
        if 0<ratio<1.5:
            self.yaw_gain=min(1.0,max(.15,self.yaw_gain+.02*(ratio-self.yaw_gain)))

    def clearance(self,x,y):
        resolution=self.map_metadata["resolution"];origin=self.map_metadata["origin"]
        i=int((x-origin[0])/resolution);j=self.map_image.shape[0]-1-int((y-origin[1])/resolution)
        if not(0<=i<self.map_image.shape[1] and 0<=j<self.map_image.shape[0]):
            return 0.0
        return float(self.map_distance[j,i])

    def free_at(self,x,y):
        return self.clearance(x,y)>=self.radius

    def path_nearest(self):
        return min(range(len(self.path)),key=lambda i:math.hypot(self.path[i][0]-self.latest["x"],
                                                                  self.path[i][1]-self.latest["y"]))

    def goal_yaw(self):
        """The heading to finish at: an unoriented goal accepts whatever heading it arrives with."""
        return self.goal["yaw"] if self.goal["oriented"] else self.latest["yaw"]

    def dock_command(self):
        """Turn to the goal heading, then drive onto the goal pose; None once docked."""
        goal,yaw=self.goal,self.latest["yaw"]
        dx,dy=goal["x"]-self.latest["x"],goal["y"]-self.latest["y"]
        rho=math.hypot(dx,dy)
        eyaw=angle(yaw-self.goal_yaw())
        if self.dock=="turn":
            if abs(eyaw)>DOCK_ALIGNED:
                return 0.0,-math.copysign(min(self.limits["rotate_to_heading_angular_vel"],
                                               math.sqrt(2*FINAL_TURN_DECEL*abs(eyaw))),eyaw)
            self.dock="drive"
            self.dock_direction=None
        if rho<=self.goal_xy_tolerance:
            if abs(eyaw)<=self.goal_yaw_tolerance:
                # Reached only once at rest (the terrain/contact Apply thresholds): zero commands
                # brake actively through the speed and yaw-rate feedback meanwhile.
                if (math.hypot(self.latest["nav_vx"],self.latest["nav_vy"])>STOPPED_SPEED or
                        abs(self.latest["nav_wz"])>STOPPED_TURN):
                    return 0.0,0.0
                return None
            return 0.0,-math.copysign(min(DOCK_TURN_RATE,math.sqrt(2*FINAL_TURN_DECEL*abs(eyaw))),eyaw)
        alpha=angle(math.atan2(dy,dx)-yaw)
        if self.dock_direction is None:  # goal behind: reverse onto it
            self.dock_direction=-1.0 if abs(alpha)>math.pi/2 else 1.0
        direction=self.dock_direction
        if direction<0:
            alpha=angle(alpha+math.pi)
        beta=angle(self.goal_yaw()-yaw-alpha)
        v=direction*max(DOCK_MIN_SPEED,min(DOCK_SPEED,DOCK_GAIN_RHO*rho))
        w=max(-DOCK_TURN_RATE,min(DOCK_TURN_RATE,DOCK_GAIN_ALPHA*alpha+DOCK_GAIN_BETA*beta))
        return v,w

    def path_speed(self):
        if not self.path:
            return math.inf
        nearest=self.path_nearest()
        # Pure pursuit steers at a point one lookahead ahead (as RPP computes it), so the robot
        # must already respect every curve inside that window or it cuts across the inside.
        seconds,shortest,longest=self.lookahead
        window=min(max(seconds*abs(self.latest["nav_vx"]),shortest),longest)
        end=nearest
        while end<len(self.path)-1 and self.path_s[end]-self.path_s[nearest]<window:
            end+=1
        curve=min(self.path_caps[nearest:end+1])
        goal=math.sqrt(2*PATH_BRAKING*max(self.path_s[-1]-self.path_s[nearest]-self.stop_short,0.0))
        return max(min(curve,goal),PATH_MIN_SPEED)

    def cross_track(self):
        if not self.path or not self.latest:
            return None
        return min(math.hypot(x-self.latest["x"],y-self.latest["y"]) for x,y in self.path)

    def plan(self,message):
        with self.lock:
            self.path=[(p.pose.position.x,p.pose.position.y) for p in message.poses]
            # v <= fraction*R/lookahead_time is the same bound as an actual turn rate of
            # fraction/lookahead_time, so it tightens the curve turn-rate limit.
            turn=min(CURVE_TURN_RATE,CURVE_LOOKAHEAD_FRACTION/self.lookahead[0])
            self.path_caps=speed_profile(self.path,CURVE_LATERAL_ACCEL,turn,PATH_BRAKING,
                                         stride=CURVATURE_STRIDE,stop=False)
            self.path_s=[0.0]
            for a,b in zip(self.path,self.path[1:]):
                self.path_s.append(self.path_s[-1]+math.dist(a,b))
            # A goal facing away from the arrival direction will be docked: brake to stop at the
            # dock point rather than at the goal.
            self.stop_short=0.0
            self.dock_distance=DOCK_DISTANCE
            if self.goal and self.goal["oriented"] and len(self.path)>10:
                (x0,y0),(x1,y1)=self.path[-10],self.path[-1]
                turn=angle(self.goal["yaw"]-math.atan2(y1-y0,x1-x0))
                if abs(turn)>DOCK_MIN_TURN:
                    self.dock_distance=min(DOCK_DISTANCE,DOCK_LATERAL_MAX/abs(math.sin(turn)))
                    self.stop_short=self.dock_distance
            self.plan_revision+=1
            self.trace.write(json.dumps({"event":"replan","goal":self.goal_number,
                "sim_time":self.latest.get("sim_time"),"revision":self.plan_revision,"points":self.path})+"\n")

    def emit(self,v=0.0,w=0.0):
        message=TwistStamped();message.header.stamp=self.get_clock().now().to_msg()
        message.twist.linear.x=float(v);message.twist.angular.z=float(w)
        self.output.publish(message)
        self.last_command=(float(v),float(w))
        self.command_history.append((self.latest.get("sim_time",0.0),float(w)))

    def manual(self,message):
        # Idle manual clients suppress repeated zeros; a genuine release/Stop still cancels.
        with self.lock:
            if self.goal_handle or self.goal_pending or self.phase=="Backing off":
                self.cancel_now("Manual command")

    def cancel_now(self,reason):
        with self.lock:
            self.emit()
            if self.goal_handle and self.cancel_future is None:
                self.cancel_future=self.goal_handle.cancel_goal_async()
                self.cancel_started=time.monotonic()
            if self.goal is not None or self.goal_pending:
                self.goal_number+=1
            self.goal=None
            self.goal_pending=False
            self.phase="Canceled"
            self.error=reason

    def nav_command(self,message):
        with self.lock:
            if self.phase!="Navigating" or self.update_blocked or not self.fresh() or not self.nav_ready:
                self.emit()
                return
            v,w=message.twist.linear.x,message.twist.angular.z
            stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            if stamp < max(self.goal_start_sim,self.latest["sim_time"]-.35) or stamp>self.latest["sim_time"]+.2:
                self.emit()
                return
            if not all(math.isfinite(x) for x in (v,w)) or not -.001<=v<=self.limits["desired_linear_vel"]+.001:
                self.cancel_now(f"Planner command outside configured limits (v={v:.3f} m/s, w={w:.3f} rad/s)")
                self.phase="Error";return
            self.last_cmd_wall=time.monotonic()
            if self.dock:
                return  # the gateway is finishing the goal itself; Nav2 was cancelled
            near=math.hypot(self.goal["x"]-self.latest["x"],self.goal["y"]-self.latest["y"])<=self.goal_xy_tolerance
            if near or (self.path and self.path_s[-1]-self.path_s[self.path_nearest()]<=self.dock_distance and
                        abs(angle(self.goal_yaw()-self.latest["yaw"]))>DOCK_MIN_TURN):
                self.start_finish();return
            self.shape_and_emit(v,w)

    def start_finish(self):
        """Hand the last metre to the gateway: cancel the Nav2 goal so RPP, which plans and
        collision-checks motions the gateway no longer executes, cannot abort the docking."""
        self.dock="turn"
        self.dock_direction=None
        if self.goal_handle and self.cancel_future is None:
            self.cancel_future=self.goal_handle.cancel_goal_async()
            self.cancel_started=time.monotonic()
        self.goal_number+=1  # the cancelled Nav2 result is ignored
        self.trace.write(json.dumps({"event":"finish","goal":self.goal_number,"sim_time":self.latest["sim_time"],
                                     "pose":[self.latest["x"],self.latest["y"],self.latest["yaw"]]})+"\n")
        self.finish_step()

    def finish_step(self):
        """One 20 Hz step of the dock / final turn, from the watchdog timer."""
        if self.clearance(self.latest["x"],self.latest["y"])<self.physical_radius:
            self.cancel_now("Docking stopped: the robot is too close to a plant or wall")
            self.phase="Error";return
        command=self.dock_command()
        if command is None:
            self.succeed_now();return
        self.last_cmd_wall=time.monotonic()
        self.shape_and_emit(*command,dock=True)

    def shape_and_emit(self,v,w,dock=False):
        limited=v if dock else min(v,self.path_speed(),max(self.latest["nav_vx"],0.0)+TRACTION_LEAD)
        if v>0 and not dock:
            w*=limited/v  # keep the planned path curvature at the slower speed
        correction=max(-SPEED_CORRECTION_MAX,min(SPEED_CORRECTION_MAX,SPEED_FEEDBACK*(limited-self.latest["nav_vx"])))
        v=limited+correction
        # The controller plans actual yaw rates; command what the skid-steer base needs to
        # achieve them, never beyond the panel Turn rate.
        w=(w+YAW_RATE_FEEDBACK*(w-self.latest["nav_wz"]))/self.yaw_gain
        self.emit(v,max(-self.turn_limit,min(self.turn_limit,w)))

    def poll_lifecycle(self):
        now=time.monotonic()
        for name,client in self.life_clients.items():
            future=self.lifecycle_futures.get(name)
            if future and future.done():
                try:
                    response=future.result()
                except RuntimeError as error:
                    self.get_logger().error(f"{name} lifecycle query failed: {error}")
                    response=None
                self.lifecycle[name]=(response.current_state.id==3) if response else False
                self.lifecycle_updated[name]=now
                self.lifecycle_futures.pop(name,None)
            elif future and now-self.lifecycle_requested[name]>3:
                future.cancel()
                self.lifecycle_futures.pop(name,None)
                self.lifecycle[name]=False
            if not client.service_is_ready() or now-self.lifecycle_updated.get(name,0)>3:
                self.lifecycle[name]=False
            if name not in self.lifecycle_futures and client.service_is_ready():
                self.lifecycle_futures[name]=client.call_async(GetState.Request())
                self.lifecycle_requested[name]=now
        with self.lock:
            self.nav_ready=all(self.lifecycle.get(name,False) for name in self.life_clients)
            if self.phase=="Starting" and self.nav_ready and self.fresh():
                self.phase="Ready"

    def watchdog(self):
        with self.lock:
            now=time.monotonic()
            if self.phase=="Navigating":
                finishing=self.dock is not None
                if not self.fresh():
                    self.trace.write(json.dumps({"event":"stale_feedback","goal":self.goal_number,
                        "sim_time":self.latest.get("sim_time"),
                        "wall_age_s":time.monotonic()-self.last_state_wall,
                        "clock_minus_feedback_s":self.get_clock().now().nanoseconds*1e-9-self.latest.get("sim_time",0)})+"\n")
                    self.cancel_now("Stale native physics feedback");self.phase="Error"
                elif not self.nav_ready:
                    self.cancel_now("Navigation lifecycle unavailable");self.phase="Error"
                elif now-self.goal_started>900 or self.latest["sim_time"]-self.goal_start_sim>180:
                    self.cancel_now("Goal deadline exceeded");self.phase="Error"
                elif finishing:
                    self.finish_step()
                elif now-self.last_cmd_wall>0.5:
                    self.emit()
            if self.cancel_future:
                if self.cancel_future.done():
                    response=self.cancel_future.result()
                    if response and response.return_code not in (0,3):
                        self.error="Cancel was not acknowledged; motion remains stopped"
                        self.phase="Error"
                        self.cancel_failed=True
                    elif response is None:
                        self.error="Cancel returned no acknowledgment; motion remains stopped"
                        self.phase="Error"
                        self.cancel_failed=True
                    else:
                        self.cancel_failed=False
                        self.goal_handle=None
                    self.cancel_future=None
                elif now-self.cancel_started>5:
                    self.phase="Error";self.error="Cancel acknowledgment timed out; motion is blocked"
                    self.emit()
            if now-self.last_write>=0.25:
                self.last_write=now
                result=self.status()
                self.trace.status(result)

    def status(self):
        pose={k:self.latest.get(k) for k in ("x","y","z","yaw","sim_time","robot_entity","source",
                                          "publication_sequence","iteration")}
        result={"ok":True,"phase":self.phase,"message":self.error or self.phase,"wall_time":time.time(),
                "ready":self.nav_ready and self.fresh() and not self.update_blocked
                        and self.cancel_future is None and not self.cancel_failed,
                "active":self.phase=="Navigating","pose":pose,"goal":self.goal,
                "velocity":{key:self.latest.get("nav_"+key) for key in ("vx","vy","wz")},
                "revision":self.revision,"plan_revision":self.plan_revision,
                "fresh":self.fresh(),"last_command":self.last_command,"limits":self.limits,
                "turn_limit":self.turn_limit,"yaw_gain":self.yaw_gain}
        result["feedback_age_wall_s"]=time.monotonic()-self.last_state_wall
        result["persistence_error"]=self.trace.persistence_error
        result["clock_minus_feedback_s"]=self.get_clock().now().nanoseconds*1e-9-self.latest.get("sim_time",0)
        if self.last_result:
            result["last_result"]=self.last_result
        if self.goal and self.latest:
            result["xy_error"]=math.hypot(self.goal["x"]-self.latest["x"],self.goal["y"]-self.latest["y"])
            result["yaw_error"]=abs(angle(self.goal_yaw()-self.latest["yaw"]))
        return result

    def wait_future(self,future,timeout=8):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if future.done():
                return future.result()
            time.sleep(0.03)
        raise TimeoutError("ROS action/service acknowledgment timed out")

    def go(self,values):
        x,y,yaw=(values.get(k) for k in ("x","y","yaw_deg"))
        oriented=values.get("oriented",True)
        if not isinstance(oriented,bool):
            raise ValueError("oriented must be true or false")
        if not oriented and yaw is None:  # any heading will do; Nav2 still needs one
            with self.lock:yaw=math.degrees(self.latest.get("yaw",0.0))
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in (x,y,yaw)):
            raise ValueError("X, Y and yaw must be finite numbers (metres, degrees)")
        limits,turn=self.goal_limits(values)
        with self.lock:
            if not self.fresh() or not self.nav_ready or self.update_blocked:
                raise ValueError("Navigation is not ready: need fresh physics, active nodes and valid map")
            if (self.goal_handle or self.goal_pending or self.cancel_future or self.cancel_failed
                    or self.phase=="Backing off"):
                raise ValueError("Cancel the current goal before sending another")
            if not self.free_at(x,y):
                raise ValueError("Goal footprint intersects a collider or unsupported terrain")
            blocked=self.clearance(self.latest["x"],self.latest["y"])<self.radius+BACKOFF_ROOM
        if blocked:
            self.back_off()
        with self.lock:
            if not self.fresh() or not self.nav_ready or self.update_blocked:
                raise ValueError("Navigation is not ready: need fresh physics, active nodes and valid map")
            if self.goal_handle or self.goal_pending or self.cancel_future or self.cancel_failed:
                raise ValueError("Cancel the current goal before sending another")
            self.goal_number+=1
            number=self.goal_number
            self.goal_pending=True
            self.dock=None
            self.phase="Planning";self.error=""
            self.goal={"x":float(x),"y":float(y),"yaw":angle(math.radians(yaw)),"oriented":oriented}
        if not self.client.wait_for_server(timeout_sec=3):
            with self.lock:
                self.goal_pending=False;self.goal=None;self.phase="Error";self.error="NavigateToPose unavailable"
            raise RuntimeError(self.error)
        self.apply_limits(limits)
        previous=self.nav2_result
        if previous is not None and not previous.done():
            # A cancelled Nav2 goal stays active until its tree halts; a goal sent meanwhile is
            # taken as a preemption, finishes the old path and reports the new goal "succeeded".
            self.wait_future(previous,5)
        with self.lock:
            self.turn_limit=turn
        request=NavigateToPose.Goal()
        request.pose.header.frame_id="greenhouse";request.pose.header.stamp=self.get_clock().now().to_msg()
        request.pose.pose.position.x=float(x);request.pose.pose.position.y=float(y)
        request.pose.pose.orientation.z=math.sin(angle(math.radians(yaw))/2)
        request.pose.pose.orientation.w=math.cos(angle(math.radians(yaw))/2)
        acceptance=self.client.send_goal_async(request)
        try:
            handle=self.wait_future(acceptance)
        except TimeoutError:
            self.cancel_now("Goal acceptance timed out")
            self.phase="Error"
            self.cancel_failed=True
            def cancel_late(future):
                handle=future.result()
                with self.lock:
                    if handle and handle.accepted:
                        self.goal_handle=handle
                        self.cancel_future=handle.cancel_goal_async()
                        self.cancel_started=time.monotonic()
                    else:
                        self.cancel_failed=False
            acceptance.add_done_callback(cancel_late)
            raise
        with self.lock:
            self.goal_pending=False
            if not handle.accepted:
                self.phase="Error";self.goal=None
                raise RuntimeError("Nav2 rejected the goal")
            if number!=self.goal_number:
                handle.cancel_goal_async()
                raise RuntimeError("Goal was canceled before acceptance")
            self.goal_handle=handle
            self.phase="Navigating"
            self.goal_started=time.monotonic()
            self.goal_start_sim=self.latest["sim_time"]
            self.last_cmd_wall=time.monotonic()
            self.nav2_result=handle.get_result_async()
            self.nav2_result.add_done_callback(lambda f:self.finished(f,number))
            self.trace.write(json.dumps({"event":"goal","id":number,"target":self.goal,"limits":self.limits,
                                         "sim_time":self.goal_start_sim})+"\n")
        return self.status()

    def back_off(self):
        """Creep straight away from a plant or wall so Smac can plan and RPP can turn in place."""
        too_close="Robot is too close to a plant or wall to plan; drive it back into open space"
        with self.lock:
            x,y,yaw=self.latest["x"],self.latest["y"],self.latest["yaw"]
            here=self.clearance(x,y)
            if here<self.physical_radius:
                raise ValueError(too_close)
            ahead=self.clearance(x+BACKOFF_PROBE*math.cos(yaw),y+BACKOFF_PROBE*math.sin(yaw))
            behind=self.clearance(x-BACKOFF_PROBE*math.cos(yaw),y-BACKOFF_PROBE*math.sin(yaw))
            if max(ahead,behind)<=here:  # heading runs along the obstacle: moving gains nothing
                if here<self.radius:
                    raise ValueError(too_close)
                return
            direction=1.0 if ahead>=behind else -1.0
            start,start_sim=(x,y),self.latest["sim_time"]
            self.phase="Backing off";self.error="Moving clear of a plant or wall before planning"
            self.trace.write(json.dumps({"event":"back_off","sim_time":start_sim,"pose":[x,y,yaw],
                "direction":direction,"clearance_m":here})+"\n")
        still_since=None
        stopping=False
        wall_end=time.monotonic()+30
        while True:
            with self.lock:
                if self.phase!="Backing off":
                    raise ValueError("Back-off interrupted: "+self.error)
                if not self.fresh() or time.monotonic()>wall_end:
                    self.emit();self.phase="Error"
                    raise ValueError("Back-off stopped: physics feedback stale or too slow")
                p=(self.latest["x"],self.latest["y"])
                room=self.clearance(*p)
                if room<self.physical_radius:
                    self.emit();self.phase="Error"
                    raise ValueError(too_close)
                stopping=stopping or room>=self.radius+BACKOFF_ROOM or (
                    math.dist(p,start)>BACKOFF_DISTANCE or self.latest["sim_time"]-start_sim>BACKOFF_SIM_TIMEOUT)
                if not stopping:
                    self.emit(direction*BACKOFF_SPEED,0.0)
                else:
                    # Plan only from rest, as every goal starts.
                    self.emit()
                    if (math.hypot(self.latest["nav_vx"],self.latest["nav_vy"])<STOPPED_SPEED and
                            abs(self.latest["nav_wz"])<STOPPED_TURN):
                        still_since=self.latest["sim_time"] if still_since is None else still_since
                        if self.latest["sim_time"]-still_since>=.3:
                            if room<self.radius:
                                self.phase="Error"
                                raise ValueError(too_close)
                            self.phase="Ready";self.error=""
                            return
                    else:
                        still_since=None
            time.sleep(.05)

    def succeed_now(self):
        errors=self.status()
        self.emit()
        if self.goal_handle and self.cancel_future is None:
            self.cancel_future=self.goal_handle.cancel_goal_async()
            self.cancel_started=time.monotonic()
        self.last_result={"event":"result","id":self.goal_number,"phase":"Succeeded",
            "xy_error":errors.get("xy_error"),"yaw_error":errors.get("yaw_error"),
            "nav2_status":None,"finished_by":"gateway finish","actual":errors["pose"]}
        self.trace.write(json.dumps(self.last_result)+"\n")
        self.goal_number+=1  # the canceled Nav2 result is ignored
        self.goal=None
        self.phase="Succeeded";self.error="Goal pose reached using native feedback"

    def finished(self,future,number):
        with self.lock:
            if number!=self.goal_number:
                return
            result=future.result()
            self.emit();self.goal_handle=None
            errors=self.status()
            if (result and result.status==GoalStatus.STATUS_SUCCEEDED and self.fresh() and self.phase=="Navigating"
                    and errors.get("xy_error",1)<=DOCK_DISTANCE):
                # Nav2 judges arrival at the end of its path, which Smac may end up to 0.1 m short
                # of the goal, and while the base may still be turning. The gateway's finish docks
                # onto the true (validated) goal, settles at rest and verifies before success.
                self.goal_number+=1
                if self.dock is None:
                    self.dock="turn";self.dock_direction=None
                return
            self.phase="Error";self.error="Nav2 ended without verified full goal-pose tolerance"
            self.last_result={"event":"result","id":number,"phase":self.phase,
                "xy_error":errors.get("xy_error"),"yaw_error":errors.get("yaw_error"),
                "nav2_status":result.status if result else None,"actual":errors["pose"]}
            self.trace.write(json.dumps(self.last_result)+"\n")
            self.goal=None

    def cancel_and_wait(self,reason):
        self.cancel_now(reason)
        end=time.monotonic()+7
        while time.monotonic()<end:
            with self.lock:
                if self.cancel_future is None:
                    if self.cancel_failed:
                        raise RuntimeError("Cancel is not acknowledged; motion and updates remain blocked")
                    return
            time.sleep(0.05)
        raise RuntimeError("Cancel was not acknowledged; robot remains stopped")

    def update_map(self,assets,revision):
        target=self.directory/"nav-map-revisions"/revision
        envelope=self.map_audit["robot_envelope"]
        create_map(assets,target,envelope["height_m"],envelope)
        request=LoadMap.Request();request.map_url=str(target/"map.yaml")
        response=self.wait_future(self.map_client.call_async(request),12)
        if response is None or response.result!=0:
            raise RuntimeError("Map server rejected the new collider-derived map")
        for client in self.clear_clients:
            self.wait_future(client.call_async(ClearEntireCostmap.Request()),8)
        with self.lock:
            self.current_assets=assets;self.revision=revision;self.map_yaml=target/"map.yaml"
            self.load_local_map()
            self.update_blocked=False
            self.phase="Ready";self.error="Map updated; press Go for a new plan"

    def api(self,action,values):
        if action=="health":
            with self.lock:
                navigation=self.status()
                native=dict(self.latest)
                updating=self.update_blocked
            return health_snapshot(self.health_observer,navigation,native,updating)
        if action=="status":
            with self.lock:return self.status()
        if action=="go":
            try:
                return self.go(values)
            except (ValueError,RuntimeError,TimeoutError) as error:
                self.cancel_now(str(error))
                self.phase="Error"
                raise
        if action=="cancel":
            self.cancel_and_wait("Canceled by user");return self.status()
        if action=="prepare_update":
            self.cancel_and_wait("Canceled for terrain/contact change")
            with self.lock:self.update_blocked=True
            end=time.monotonic()+15
            still_since=None
            while time.monotonic()<end:
                with self.lock:
                    self.emit()
                    # A base braking on soil rocks through zero speed; require 0.5 s of simulated
                    # stillness so the Apply's own speed check does not catch the rebound.
                    if self.fresh() and math.hypot(self.latest["nav_vx"],self.latest["nav_vy"])<.02 and abs(self.latest["nav_wz"])<.03:
                        still_since=self.latest["sim_time"] if still_since is None else still_since
                        if self.latest["sim_time"]-still_since>=.5:
                            return {"ok":True}
                    else:
                        still_since=None
                from greenhouse.transport import world_status
                physical=world_status()
                if (physical["paused"] and physical["robot"].get("speed",1)<.02
                        and physical["robot"].get("angular_speed",1)<.03):
                    return {"ok":True,"paused":True}
                time.sleep(.1)
            raise RuntimeError("Robot did not stop for update")
        if action in ("commit_update","rollback_update"):
            self.update_map(values["assets_dir"],values["revision"]);return {"ok":True}
        raise ValueError("Unknown navigation operation")


def main():
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=Gateway()
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    executor.add_node(node.health_observer)
    def stop(signum,frame):
        executor.shutdown(timeout_sec=.5)
    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            try:
                length=int(self.headers.get("Content-Length",0))
                if length>8192:raise ValueError("Request too large")
                values=json.loads(self.rfile.read(length) or b"{}")
                result=node.api(self.path.lstrip("/"),values)
                code=503 if self.path=="/health" and result.get("ok") is not True else 200
            except (OSError,ValueError,RuntimeError,TimeoutError) as error:
                result={"ok":False,"error":str(error)};code=400
            data=json.dumps(result).encode()
            self.send_response(code);self.send_header("Content-Type","application/json");self.send_header("Content-Length",str(len(data)))
            self.end_headers();self.wfile.write(data)
        def log_message(self,*args):
            pass
    server=ThreadingHTTPServer(("127.0.0.1",8765),Handler)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_now("Navigation gateway stopped")
        for _ in range(3):
            node.emit();time.sleep(.05)
        server.shutdown();server.server_close();thread.join(timeout=3)
        node.trace.close();node.health_observer.destroy_node();node.destroy_node();rclpy.shutdown()


if __name__=="__main__":
    main()
