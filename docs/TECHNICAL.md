# Jackal greenhouse technical reference

## FPB30 actuator, camera and rear-deck mount

The selected Amazon ASIN B0CNX4RQ57 is FPB30 with B42 (42 x 47 mm NEMA17)
motor and 1500 mm stroke. Facts from the official FUYU 2026 catalog PDF p27
(printed 047-048): body length = stroke + 177 mm, nominal module mass 3.9 kg
at 1500 mm, 30 mm rail, 84 x 65 mm slider, 60 mm motion per revolution,
carriage-centre endpoints 86 mm and body length - 91 mm. The catalog's
0.9 + 2*stroke_metres nominal mass interpolation is used; component splits and
box inertias are explicitly engineering estimates, not measured dynamics.
The side-view 41 mm is the overall carriage stack dimension, not a 41 mm stroke.

B42 vertical external-payload envelope: 1 kg at 0.2 m/s, 2 kg at 0.1 m/s;
the default is 0.2 m/s (the rated maximum for the gimbal payload, see below) with a 0.5 s acceleration/deceleration ramp. The family
3 kg and 500-1000 mm/s marketing figures are not this motor's vertical operating
point. The **30 N finite effort cap is an engineering assumption**, not a
manufacturer torque/stall rating. Configuration rejects a cap inadequate for
the modeled moving mass, gravity and ramp. A native force-based servo commands
the prismatic joint; its physical hard stops remain 0..installed stroke.
No ideal actuator-position teleporting or artificial drift is used.
The powered actuator holds with limited effort; a belt drive has no assumed
passive brake/self-locking guarantee. This is not a calibrated digital twin.

Default module split: carriage 0.32 kg, motor 0.35 kg, end hardware 0.15 kg,
and the remaining catalog total assigned to the rail. The carriage now carries
two XF C-200T gimbals (0.125 kg each) with one camera each (default 0.15 kg) on a
0.06 kg crossbar plate: external payload 0.61 kg, moving carriage plus payload
0.93 kg. That is under the B42's 1 kg / 0.2 m/s vertical envelope point, so the
rated maximum lift speed is 0.2 m/s, which is the default. A heavier payload (> 1 kg) halves the rated speed and rejects this default; lower `speed_m_s` with it. Mass is
startup-only: real box inertias, fixed-joint-lumped centre of mass and gravity
feed-forward change together. There is no fake live mass slider or runtime
inertial recreation. Camera resolution, FOV and rate are configurable and shared
by both cameras. The C views show the actual gimbal sensors' frames, not a
relocated spectator camera. The separate normal Gazebo camera is restored without
changing its saved view.

The earlier soil/traction comparison below was measured with the previous single
0.8 kg camera; it has not been re-run with the 0.61 kg gimbal payload.

## Dual gimbal cameras (XF C-200T)

Vendor listing figures used ([Rotorama](https://www.rotorama.com/product/xf-c-200t-3-axis),
[RCDrone](https://rcdrone.top/fr/products/xf-c-200t-3-axis-gimbal)): 125 g with damper,
about 80 x 65-81 x 92 mm, **200 g payload**, orthogonal 3-axis mechanical stabilisation,
pitch -135..+40 deg, roll +/-50 deg, yaw +/-155 deg, 200 deg/s maximum control speed,
+/-0.01 deg stated accuracy, camera-up or camera-down mounting, horizon-lock and FPV modes.
No manufacturer drawing, joint geometry or motor torque was found; those are estimates.
The simulation limits every axis to **+/-90 deg** (`GIMBAL_LIMITS_DEG` in
`greenhouse/actuator.py`, the single source for the URDF joints, the controller, which
reads them from SDF, and the GUI, which reads them from the gimbal state).

Geometry (`greenhouse/actuator.py`): a 100 mm deep, 220 mm wide crossbar plate is fixed to
the carriage face, its top flush with the carriage top. Each gimbal hangs under it
("camera down"), its yaw axis 65 mm in front of the carriage face and +/-`gimbal_offset_m`
(0.065 m) either side of the rail centre, its mount frame turned by `gimbal_mount_yaw_deg`
(+90 deg left, -90 deg right) so that the mount's +X is the camera's outward neutral.
Chain: base/damper (fixed under the plate) -> yaw motor (+Z) -> roll motor (+X, 40 mm
behind the camera) -> pitch motor (-Y, so a positive angle tilts up) -> camera link. The
camera centre lies on the yaw axis 59 mm below the plate, upright, so images are not
inverted. The 125 g is split 28/32/40% across base, yaw stage and roll arm with pitch motor.

0.065 m is the closest spacing at which the two units cannot touch: the roll arm sweeps
about 0.06 m from its yaw axis. `tests/test_actuator.py` samples every joint over its
+/-90 deg range and the whole stroke (`swept_link_points`) and checks that each unit
stays at least 6 mm from the centre line, 18 mm in front of the carriage face, below
the plate, and 35 mm above the J100 chassis top (z 0.184 m) at the lowest lift position.
Smaller `gimbal_offset_m` values are rejected. The navigation envelope samples the same
sweep; the swept bounds stay inside the bare-body/fender minimum (maximum Z still the
1.873 m rail top), so the footprint and the 0.368 m circle are unchanged.

Keyboard (GUI, always enabled): held P / R / Y raise camera 1 pitch / roll /
yaw at **Key rotation speed** (default 5 deg/s, one value for all axes and both cameras);
Alt selects camera 2 and Shift lowers instead, both read when the key goes down. The
targets stop at the limits and are sent as normal commands on `<p>/gimbal/command`; focus
loss and Space/Esc stop steering like every other key.

Control (`greenhouse::Gimbal` in `native/Systems.cc`, kinematics in
`native/GimbalMath.hh`): every physics step the mount attitude is recovered from the
yaw link's simulated world orientation and yaw joint angle. With **Stabilise** on
(horizon lock), the desired camera attitude is roll/pitch relative to gravity and
yaw relative to the mount's horizontal heading, so the camera follows the robot's
turns but not its roll/pitch on the uneven soil. With it off (FPV) the command is
relative to the mount. Closed-form inverse kinematics of Rz(yaw) Rx(roll) Ry(-pitch)
gives the motor targets, which are clamped to the mechanical ranges (reported as
`saturated`). Each motor is driven as a DART velocity servo: 40 1/s position gain plus
target-rate feed-forward, limited to 200 deg/s and to the URDF effort limit
`gimbal_torque_nm` (0.3 N m, an **engineering assumption**). Attitude sensing is the
true simulated orientation, i.e. an ideal IMU without drift or noise; the real unit's
+/-0.01 deg is not modelled further. `native/GimbalMathTest.cc` (run during the image
build) checks the IK round trip, horizon lock on a tilted robot, FPV mode, limits and a
rate-limited closed loop (worst error 0.08 deg while the mount rocks +/-6 deg at 1 Hz;
this is a kinematic check, not a Gazebo dynamics measurement).

The URDF joint velocity limit is set to twice the 200 deg/s control limit. In DART, a
velocity limit equal to the servo command made the joint stick at that speed and run into
its hard stop. The controller still enforces 200 deg/s.

Gazebo dynamics check (gz-sim 8.15 / gz-physics 7.8 DART, 2 ms step, built from source, recorded with the
earlier upright 0.12 m mounting and the vendor ranges,
headless; `tools/gimbal_dynamics_check/`): the generated lift, carriage and both gimbals
were mounted on a 17 kg base driven to roll +/-8 deg at 0.5 Hz, pitch +/-6 deg at
0.35 Hz and yaw at 0.3 rad/s, while the lift raised 0.47 m. Both stabilised cameras held
0/0/0 with a worst attitude error of 0.006 deg (RMS 0.003 deg). A step to roll 10 /
pitch -30 / yaw 20 deg settled within 0.5 deg in under 0.2 s. A -120 deg look-down was
reached exactly. FPV mode stayed rigid to the base. Out-of-range and unknown-camera
requests were rejected. The Jackal, the soil contact and the camera sensors were not part
of this check, and the sensing is ideal, so the small errors are an upper bound on
simulation quality, not a prediction of the real gimbal.

Interfaces (`<p>` = `/<namespace>/greenhouse`): `<p>/camera1/image`, `<p>/camera2/image`
and their `camera_info`; Gazebo service `<p>/gimbal/settings` and topic
`<p>/gimbal/command` (JSON `{"camera", "roll", "pitch", "yaw", "stabilise"}`, degrees,
omitted fields kept, out-of-range values rejected); `<p>/gimbal/state` (JSON at 20 Hz
with command, measured attitude, joint angles, error and saturation). The command and
state topics are bridged to ROS as `std_msgs/String`; the six gimbal joints join the
lift on `platform/joint_states`, so `robot_state_publisher` provides
`camera1_optical`/`camera2_optical` TF frames.

`cad/rear_deck_mount.scad` is editable millimetre CAD. The generator also exports
`cad/generated/rear_deck_bracket.stl` and its dimensions to `mount.json`.
The STL is a closed fused solid with conservative 1 mm stepped gussets, exact
rectangular clearance slots and two rail attachment levels 138 mm apart.
FPB30 rail fasteners use M4 on 17 mm transverse spacing. The adapter has M5
clearance slots at the configurable Jackal deck pitch. **Standard bare lid:
120 mm square M5x0.8; optional PACS 026369 plate: 80 mm square M5x0.8.**
Deck origin, print material, component mass split, slot length and clearance are
prototype choices. Measure the installed lid and actuator interfaces before
printing; no printed fit, cantilever strength or load certification is claimed.

Print base-flat with the gussets supporting the upright; PETG/ASA or a suitably
engineered nylon, several perimeters and adequate infill are practical starting
points, not a structural rating. Use washers, correctly engaged M5 deck bolts
and suitable M4 rail hardware; never bottom bolts in blind holes. The rear-DECK
location stays inboard and must leave the rear HMI/Motion Stop accessible.
There is no rear-hanging adapter plate. Verify access and wheel/fender clearance
on real hardware before fitting any payload. Optional 80 mm mounting must match
the PACS plate actually installed.

Primary sources:
- <https://www.amazon.com/dp/B0CNX4RQ57?th=1> (selected 1500 mm FPB30).
- <https://www.fuyumotion.com/manual/> and its FUYU-Linear-Motion-System-2026.zip,
  PDF SHA256 `23145a4eacb3f452cf206584853bffc7d458b4ebfeb33d7adb911d574b5420c0`,
  p27. Manufacturer pages/images are not redistributed.
- <https://www.fuyumotion.com/high-speed-long-stroke-double-shaft-cores-thin-slide-linear-rail-guide-product/>
- <https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/jackal/integration_jackal/>
- <https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/jackal/user_manual_jackal/>

## Sensors

### SICK TiM551 2D lidar

Source: SICK data sheet TIM551-2050001, part no. 1060445 (dated 2026-08-04,
<https://www.sick.com/media/pdf/5/45/045/dataSheet_TIM551-2050001_1060445_en.pdf>): 270° aperture,
15 Hz, 1° angular resolution, working range 0.05-10 m (8 m typical at 10% remission), systematic
error ±60 mm, statistical error < 20 mm (typical, 90% remission), 250 g, 60 × 60 × 86 mm, light
emission level 62.46 mm above the housing base (dimensional drawing).

Model (`lidar` section of `config/actuator.json`, built by `add_lidar` in `greenhouse/actuator.py`):
a 250 g box housing `lidar2d_0_link` fixed to `base_link` at (0.12, 0, 0.184) m, which is
Clearpath's `front_0_mount` on the top plate; the scan frame `lidar2d_0_laser` sits 0.0625 m above
it, so the scan plane is at z = 0.2465 m in `base_link`. Gazebo `gpu_lidar`: 271 × 1 rays over
±135°, 0.05-10 m, Gaussian range noise σ = 0.02 m (the statistical error taken as 1σ). Not modelled:
the ±60 mm systematic error, the 8 m low-remission limit (Gazebo has no remission), and motion
distortion within a scan (`time_increment` is 0). A ray with no hit returns +inf. The lidar ray
sees rendered visuals, so plant leaves count as hits.

Clearances: the housing spans x 0.09-0.15 m, in front of the gimbal sweep (x ≤ 0.075 m at any lift
height or angle) and the FPB30 bracket (x ≤ 0.03 m), and inside the bare J100 footprint, so the
Nav2 footprint does not change (`tests/test_actuator.py`). The 90° blind sector faces the FPB30
rail. With the lift near 0 the left gimbal hangs in the scan plane at about +100° to +135°: the
measured minimum range was 0.069 m at +135° with the lift at 0. Raising the lift clears it. The
near clip equals `range_min_m`, so the housing (≤ 0.043 m from the scan origin) is never seen.
A 3D lidar is a config change: `vertical_samples` > 1 and `vertical_fov_deg` > 0.

In an aisle the scan plane is about 0.31 m above the soil, inside the lowest, sparse leaf crown, so
the rows are not a wall to it. At rest mid-aisle at (0, -2) m, 4 beams on one side and 8 on the
other hit the rows at 0.91-1.01 m lateral offset (stems and leaves on the row centre lines 1 m
away); most beams pass between plants and end on the greenhouse walls (+90°: 2.97 m, -90°: 7.03 m).
Lidar on vs `"enabled": false` gave RTF 0.60 vs 0.53 in single 30 s headless windows on a shared
host, so its cost is below run-to-run variation.

### Measured rates, frames and noise

Headless run, robot at rest at spawn, 18 s of sim time, RTF 0.60 (Iris Xe through `/dev/dri`, both
cameras and the lidar on). Rates come from the message header stamps, so they are in sim time.
`<ns>` is `j100_0000` in the default robot.yaml. Noise models were read from
`SETUP/greenhouse/robot.expanded.urdf`; `runtime-world.sdf` defines none.

| Topic | Type | Stated rate | Measured | frame_id | Noise model |
|---|---|---|---|---|---|
| `<ns>/sensors/lidar2d_0/scan` | `sensor_msgs/LaserScan` | 15 Hz (`actuator.json`) | 15.15 Hz | `lidar2d_0_laser` | Gaussian σ 0.02 m; measured per-beam σ 0.0197 m over 60 static scans |
| `<ns>/sensors/imu_0/data_raw` | `sensor_msgs/Imu` | 50 Hz (Clearpath `j100.urdf.xacro`) | 50.0 Hz | `imu_0_link` | none |
| `<ns>/sensors/imu_0/data` | `sensor_msgs/Imu` | 50 Hz (`imu_filter_madgwick` on `data_raw`) | 50.0 Hz | `imu_0_link` | none (filter output) |
| `<ns>/platform/odom` | `nav_msgs/Odometry` | 50 Hz (`control.yaml` `publish_rate`) | 50.0 Hz | `odom` → `base_link` | none; its only error is wheel slip on the terrain |
| `<ns>/platform/odom/filtered` | `nav_msgs/Odometry` | 50 Hz (`localization.yaml` `frequency`) | 50.0 Hz | `odom` → `base_link` | n/a (EKF of wheel odometry and IMU) |
| `<ns>/greenhouse/camera1/image` | `sensor_msgs/Image` | 10 Hz (`actuator.json`) | 8.99 Hz | `camera1_optical` | none |
| `<ns>/greenhouse/camera2/image` | `sensor_msgs/Image` | 10 Hz | 9.33 Hz | `camera2_optical` | none |
| `<ns>/greenhouse/camera{1,2}/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz | 10.0 Hz | `camera{1,2}_optical` | n/a |
| `<ns>/sensors/gps_0/fix` | `sensor_msgs/NavSatFix` | 10 Hz (Clearpath `j100.urdf.xacro`) | 10.0 Hz | `gps_0_link` | none |
| `/greenhouse/ground_truth` | `tf2_msgs/TFMessage` | 50 Hz (`PosePublisher`) | 50.0 Hz | `greenhouse` → `<ns>/robot`, plus link poses in `<ns>/robot` | none (truth) |
| `<ns>/platform/joint_states` | `sensor_msgs/JointState` | 50 Hz wheels plus the lift/gimbal `JointStatePublisher` (no rate set) | 549 Hz combined | `base_link` | none |

Both camera_info streams arrive at 10.0 Hz, so every frame is rendered; the missing images (about
10% and 7%) were most likely dropped on the way to the best-effort test subscriber (0.9 MB
messages); not investigated further.

TF today is two disconnected trees. On `<ns>/tf` and `<ns>/tf_static` the root is `odom`: the EKF
publishes `odom` → `base_link` at 50 Hz (`localization.yaml` `publish_tf: True`; the wheel
controller has `enable_odom_tf: False`), and `robot_state_publisher` publishes every fixed joint
(including `base_link` → `lidar2d_0_link` → `lidar2d_0_laser`) on `tf_static` and the wheels, lift
and gimbal joints on `tf`. On the root `/tf` and `/tf_static` the navigation gateway publishes
`greenhouse` → `<ns>/nav_odom` (static identity) and `<ns>/nav_odom` → `<ns>/nav_base` at 50 Hz
from the true pose. There is no `map` frame.


The native Qt panel replaces the stock hard-coded WASD/QE Teleop plugin while
retaining its gz.msgs.Twist -> Clearpath TwistStamped -> stock controller path.
GUI defaults are the panel maximums, 2.0 m/s and 4.0 rad/s; the terminal remains
0.2 m/s and 0.5 rad/s. Wheel radius, separation and controller limits are unchanged;
the measured WheelSlip longitudinal-compliance change is documented below.
Keyboard input is always enabled (no checkbox); the on-screen drive/lift/camera buttons
and the lift position readout were removed. Arrow keys drive, A/Q move the lift,
C cycles 3D view -> camera 1 -> camera 2 -> 3D view, Space/Esc stop. Text/spinbox focus and window
focus loss stop commands. C is edge-triggered, opposing keys cancel, and the
native lift watchdog stops stale manual commands. Terminal Esc or Ctrl-C exits;
Q now lowers the carriage. A terminal cannot sense physical key-release events,
so its retained 0.35 s repeat timeout is the deadman.
The GUI sends held-key commands from a 20 Hz native heartbeat independent of
rendering. Key release/focus loss clears it immediately; an unresponsive GUI
event loop also clears it after 1.5 seconds. The actuator's 0.35 s transport
watchdog and the unchanged wheel controller's 0.5 s command timeout remain active.

Actuator rail length is a startup setting; the greenhouse limits it to 0.1-1.8 m
stroke for roof clearance, a subset of FUYU's standard 0.1-3.5 m family.
Live speed/target/window settings cannot exceed the installed stroke or rated
vertical speed; shrinking a window below the actual carriage position is rejected.
The panel sets only target and speed (full-stroke window); a narrower window remains
available through the lift settings service.
Runtime settings are session-only; the source JSON and user robot.yaml are not
rewritten.

Ground Apply prepares a uniquely named deterministic asset revision while the
old world remains live, has navigation cancel the goal and hold the base still for 0.5 s
of simulated time (a base braking on soil rocks through zero speed), rechecks measured
stationarity, holds the actuator,
pauses physics, installs the paired collision+visual, removes old support,
resamples plant/base support Z and verifies the revision. The transaction checks
that simulation time and the robot entity ID do not change. It resumes the prior
play/pause state only after valid support is established. On an error it restores
old support and poses; failed rollback leaves the world paused with an explicit
error. No Gazebo process or world is restarted and no simulation clock is reset.
Transient renderer frames during the paused transaction are not claimed to be
visually atomic; **no physics step** may occur with missing/double support.

All five terrain fields and ground friction/two slip compliances are live.
The whole proposed configuration is checked for finite numeric values, field
ranges, actual refined-mesh budget and full rotated robot/greenhouse support.
`ground_headland` changes the physical mesh extent/seeded sampling, not row XY
or the 2.6 m crop turning space. Applied values and unique revision are read back
from `terrain-state.json`; startup JSON is never overwritten. A contact-only
compliance update uses the native blocking WheelSlip service without rebuilding
the mesh. Navigation is canceled first; its collider map is refreshed as part of
the transaction, with both revisions rolled back on failure. A new Go is required.
World Control and World Stats cards are absent; the underlying pause service remains.

## Firm-earth surrogate and measured contact choice

This is a **firm, lightly compacted earth engineering surrogate**, not a calibrated
rubber/loam model. DART resolves rigid mesh contact; there is no soil sinkage,
rutting, Bekker model, artificial drift force or engine substitution.
Defaults are 0.15 m upstream sampling, 0.03 m upstream range, 3 m ground padding,
0.003 m bump-band RMS and 0.18 m band scale. The stronger 0.016 m option remains
available. The pinned VMF algorithm and seeded irregularity are retained.

Seed 42 produces 41.295 mm whole-field peak-to-trough, 1.686 degree p95 slope
and 4.655 degree maximum. Detrended aisle profiles, measured separately, have
median RMS **0.883 mm over 1 m** (range 0.422-2.615) and **2.882 mm over 5 m**
(range 2.067-4.328). These maintained aisles are smoother than the illustrative
5-10 mm research-informed local target; they are not presented as measured
greenhouse constants. Cavity contrast was reduced from 0.38 to 0.12, preserving
soil hue and fine grain; no terrain shape was faked with normal/displacement maps.

Ground isotropic friction stays 0.9; wheels stay 0.5; restitution stays zero.
Ground friction above the wheel coefficient can have limited effect. Gazebo
8.11.0 WheelSlip and gz-physics 7.6.0 DART source confirm that the ODE-named SDF
friction/slip tags are consumed by DART's DynamicsAspect, not ignored ROS1 tags.
The native blocking WheelSlip command updates SlipComplianceCmd. Its compliance
is slip ratio divided by tangential/normal force, **not percent slip**.
The unchanged 40 N wheel load is an estimated normalization, not measured force.

One bounded comparison used actual 50 Hz simulated wheel angular velocities
and base/wheel-centre motion, the same seed, 0.8 kg camera and 10 simulated
seconds of 0.2 m/s drive. The first two seconds were excluded as acceleration.
Rolling residual compares 0.098*wheel_omega with actual wheel-centre longitudinal
velocity, including angular-motion contribution; below 0.05 m/s rolling speed
the diagnostic reports absolute residual, not undefined percentages.

| Configuration | Straight median absolute residual | p95 |
|---|---:|---:|
| Previous 0.04/0.016 m terrain, compliance 1/1 | 8.62% | 52.38% |
| Firm 0.03/0.003 m terrain, unchanged 1/1 | 3.62% | 11.49% |
| Same firm terrain, longitudinal 0.10 / lateral 1 | **2.53%** | **10.78%** |

The last is the selected engineering default. Its p95 slightly misses the 10%
aspirational threshold; no samples or coefficients were adjusted to conceal that.
The gentle 0.2 m/s, 0.2 rad/s command arc produced actual mean 0.1814 m/s and
0.0532 rad/s, with 22.83% median/35.24% p95 rolling residual and 0.00645/0.01278 m/s
median/p95 lateral wheel-centre motion. Skid-turn lateral slip and imperfect yaw
are retained, not "fixed" by changing wheel geometry or forces. There is no direct
contact-force measurement or transferable empirical rubber/loam traction curve.
Raw CSVs and comparison JSON are in `test-results/soil-navigation-20260912/`.

Research informs scale and limitations, not exact coefficients:
[Alvarez-Mozos 2009](https://pmc.ncbi.nlm.nih.gov/articles/PMC3280757/) reports
rolled silty-clay RMS 16.3 mm/correlation 236.7 mm and substantial sample variation;
[Verhoest 2008](https://pmc.ncbi.nlm.nih.gov/articles/PMC3697171/) explains dependence
on profile length/detrending (including smoother rolled profiles).
[WheelSlip 8 API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1WheelSlip.html)
defines compliance, not calibrated soil properties.
[Skid-steer kinematics](https://pmc.ncbi.nlm.nih.gov/articles/PMC4481911/) concerns
tile, not soil; [Chrono SCM](https://api.projectchrono.org/vehicle_terrain.html)
illustrates the distinct constitutive data a deformable-soil model would require.

## Autonomous navigation and exact feedback contract

**Nav2 Jazzy Regulated Pure Pursuit (RPP) + SmacPlanner2D** is selected for this
known, mild 2.5D greenhouse of straight aisles and row-end turns. Smac provides
cost-aware grid A*; RPP tracks the path at an explicit cruise speed from the
current measured pose/velocity, turns in place towards the path, regulates speed in
curves and checks the projected footprint against the local costmap. It is not a
soil-shear model. MPPI (no longer installed) was the previous controller: its critic
balance settled near 0.8-1.1 m/s whatever `vx_max` allowed, and its model assumes the
commanded yaw rate is achieved, which skid steering on this soil does not do.
DWB samples discrete local trajectories.
Maintained ROS2 MBF/MeshNav is a legitimate 3D terrain-surface alternative for
steep/multilevel scenes but needs a new mesh/map pipeline here. acados supplies
optimization, requiring calibrated dynamics and custom navigation integration.
None is dismissed as unavailable or unmaintained.

The pinned Clearpath base had partial Nav2 1.3.12; available Jazzy packages require
the coherent 1.3.13 patch family. The image installer checks that exact release
and installs the exact builds in `config/nav-packages.lock.json`, including the
directly coupled BehaviorTree.CPP 4.10.0 runtime. The lock was refreshed in
October 2026 to the September 2026 rebuilds of the same 1.3.13/4.10.0 releases
after the original builds left the apt repository. The image's
`/opt/greenhouse/nav-packages.lock.json` records installed versions, resolved
library/SONAME and verified required constructor/destructor exports (`ldd -r`,
`nm`). Missing locked binaries or unresolved ABI fail the build, never trigger
symlink/stub workarounds or silent release changes. It adds only the needed
components and aligns the already-installed Nav2 family; no host install, new
image family, AMCL, SLAM, lidar, velocity smoother or automatic recovery motion.

Native `PostUpdate` publishes `/greenhouse/physics` at 50 Hz **simulation time**:
physical base_link world X/Y/yaw and signed world velocity projected onto yaw-only
body axes, plus yaw derivative (roll/pitch angular contributions included).
Source identity, entity IDs and monotonic simulation timestamps are retained.
When slow real-time factor makes a 20 ms simulation interval exceed wall-time
feedback bounds, a 100 ms wall deadline also requests publication, **only on a
new, unpaused physical step**. That step's actual pose/twist/time are sampled;
cached data is never restamped. Sequence and trigger are reported. The original
50 Hz simulation deadline and 1.5 s wall-loss / 0.12 s simulation-age guards remain.
`/greenhouse/nav/odom` and one dedicated TF subtree use only this observation:
`greenhouse -> <namespace>/nav_odom` is identity; `nav_odom -> nav_base` is its
planar pose. No competing stock `odom -> base_link` transform is published.
The stock wheel+IMU EKF and sensor configuration remain available but are **not**
navigation's actual-motion feedback. Z is physical terrain support, not a goal.
Goal X/Y are metres in greenhouse coordinates; +X follows rows, +Y crosses them,
and yaw degrees are counterclockwise about +Z, normalized to a quaternion.

### One TF tree (`tools/tf_unify.py`)

Clearpath publishes its tree on `/<namespace>/tf(_static)` with unprefixed frames
(`odom -> base_link` from the wheel+IMU EKF, `base_link ->` sensors, lift and gimbal from
`robot_state_publisher`); navigation publishes on `/tf`. `tf_unify` (started by the launch file)
joins them on `/tf` and `/tf_static`, so one tree is rooted at `greenhouse`:

```text
greenhouse ─┬─ <ns>/nav_odom ── <ns>/nav_base          (gateway, truth; Nav2 uses only these)
            └─ map ── odom ── base_link ── sensors, lift, cameras   (tf_unify + Clearpath)
```

- Relay: `/<ns>/tf` is forwarded to `/tf` as raw bytes; `/<ns>/tf_static` messages are merged
  by child frame and republished, with `greenhouse -> map` (identity), as one latched message.
  One direction only, so there is no loop; with the root namespace there is no relay.
- `map -> odom` is a **truth localiser**, evaluation-only scaffolding for the correction an
  estimator (M4) or SLAM (M6) will publish instead: T(map←base_link, truth) · T(odom←base_link,
  EKF)⁻¹. Truth is `/greenhouse/physics` (the base_link link pose, full 6 DoF), interpolated at
  each new `platform/odom/filtered` stamp (the EKF builds its TF from that message) and stamped
  with it, so `greenhouse -> base_link` through the chain equals truth. Because the stock EKF
  runs in `two_d_mode`, terrain height, roll and pitch land in `map -> odom`.
- Nothing in navigation reads `map`, `odom` or `base_link`; the gateway, Nav2 frames and
  parameters are unchanged. The ground-truth topic `/greenhouse/ground_truth` stays off `/tf`.
- Checked 2026-10-29 (headless, `sample_use.py` driving): 35 frames, one root; chain vs truth
  < 1e-13 m and 1e-7 rad; chain vs `nav_base` x/y < 1e-13 m, yaw < 1e-6 rad (z differs by the
  base_link height, about 0.07 m, since `nav_base` is planar); `map -> odom` at the EKF rate
  (about 50 Hz sim). Clearpath's `odom_base_tf_bridge` carries nothing: no gz publisher
  exists on `/model/<ns>/robot/tf`, so the EKF is the only `odom -> base_link` authority.
- The node uses the wall clock: as a sim-time node it spent about half a core in rclpy on
  `/clock`, which publishes at the physics step rate.

The 0.05 m known occupancy map comes from actual greenhouse/plant collision
volumes, supported terrain boundary and a conservative 12-degree slope exclusion.
No rendered color or invented lidar is used; roofs above the full swept robot
height are excluded. The expanded URDF's full-stroke collision envelope measures
X +/-0.229, Y +/-0.207795 and maximum Z 1.873 m. Combining bare-body/fender minimum
with 25 mm margin gives polygon X +/-0.279, Y +/-0.240 and circumscribed radius
**0.368023 m**. Smac2D uses that conservative circle; RPP's collision check uses
the full polygon. Enlarged supported primitive-collider payloads are measured at
startup rather than silently retaining a Turtlebot footprint.
The projection is for mild single-valued terrain, not 3D clearance planning or
unmodeled moving obstacles.

The recovery-free behavior tree plans once per goal and replans only when the path
becomes invalid or the goal changes (`IsPathValid` / `GlobalUpdatedGoal`, checked at 1 Hz);
RPP runs at 20 Hz. The costmaps hold only the static map, so periodic replanning added
nothing: it restarted the path at the robot, and after a row-end corner cut each new path
hugged the end plant tighter until Smac failed with "Start occupied" (goal (5, 2, -180)
from (0, 0) aborted every time). A visual
walkthrough of the whole stack, with diagrams and the measurements behind each choice,
is in [navigation-stack.html](navigation-stack.html).

**Planner.** Smac 2D costs each step as length x (1 + m * cost/252), with cost scaling 4
and travel multiplier m = 2. The global inflation radius is derived at startup
(`aisle_inflation_radius` in `greenhouse/navigation_map.py`): one map cell short of half the
narrowest clear aisle in the generated layout, at most 0.6 m (0.6 m for the default 1.3 m
aisles). Every aisle centre line is then cost-free, so Smac picks between aisles on length
alone and plants still push paths back onto the centre line. With a 0.7 m radius each inner
aisle's centre line cost 99/252 (1.79x its length) while the wider outer aisles cost 0, so
Smac sent a goal at (6.5, 0) from the spawn 7.6 m round the outer aisle. On 14 start/goal
pairs the real planner now averages 1.02x the true shortest footprint-safe path (Dijkstra on
the same map; worst 1.09x) against 1.18x (worst 1.68x) with the 0.7 m radius. The tightest
row-end corner clears plants by 0.16 m beyond the robot body (0.23 m before). An earlier
inflation of 1.2 m / scaling 3 / m = 3 swung every row-end turn through the middle of the
headland, ~1.9 m extra each.

**Path follower.** RPP's lookahead is 1.5 s of measured speed within 0.6-3.0 m (a 1.0 m
minimum ran ~0.15 m inside the 0.95 m row-end hairpin at 0.2 m/s); it turns in
place while the path heading is more than 0.5 rad away and slows on arcs tighter than 2 m.
Its collision check steps the commanded motion one 0.025 m local-costmap cell at a time for up to 1.5 s;
with pure-pursuit curvature at most 2 sin(0.5)/0.6 = 1.598 1/m below the in-place threshold, a
footprint corner moves at most .025*(1 + .368023*1.598) = .0397 m per check, so a .020 m
footprint margin covers the half-step for the **commanded motion** (not slipping motion).
Published local footprint is .598 x .520 m (planning radius .396 m). The local costmap is
0.025 m (the map stays 0.05 m) so the padding shrank instead of growing to .040 m with the
shorter lookahead: an in-place spin now has 0.204 m of aisle slack against the map's cells
(0.184 m with the old 1.0 m / .034 m setting), and spins measured 4-24 cm of slide; the global/goal-validation radius matches.
Actual body/collisions and the stock wheel geometry/limits are unchanged.

**Per-goal limits.** On Go the GUI sends Drive and Turn. The gateway sets RPP's dynamic
`desired_linear_vel` to Drive (at most 2.0 m/s) and `rotate_to_heading_angular_vel` to the
actual rate the Turn command produces (Turn x yaw gain, at most 1.2 rad/s).

**Command shaping** (gateway, every RPP command, in order):

1. Reject stale commands; fail the goal on a speed outside [0, Drive].
2. Docking / final turn near the goal (below).
3. Path speed profile: curvature over +/-0.5 m allows 0.8 m/s^2 lateral and 0.47 rad/s
   actual yaw (1.0 rad/s, tightened so 1.5 s of speed stays within 0.7 R of the curve:
   v <= 0.7 R / 1.5); a backward pass brakes at 1.2 m/s^2 into slower points. The applied cap is
   the lowest value inside the current pure-pursuit lookahead, because RPP already steers
   at the carrot; without that the robot reached hairpins fast and cut at the row-end
   plants. A separate sqrt(2*1.2*remaining) term brakes for the goal (or for the dock
   point d <= 0.7 m short of it), floored at 0.15 m/s.
4. Traction: forward command at most 0.5 m/s above the measured speed.
5. Speed feedback (gain 1, at most +/-0.2 m/s) recovers the ~7% slip at cruise.
6. Yaw inversion: divide the wanted yaw rate by the yaw gain (0.28 prior, refined online
   from steady path-following turns, never while docking: a 90 deg dock's saturated turn
   commands dragged it to the 0.15 floor) and add 0.8 x (wanted - measured) for the ~0.5 s yaw lag.
7. Clamp to the panel Turn command rate (at most the stock 4.0 rad/s).

Stages 3-4 scale the yaw rate with the speed they remove, keeping the planned curvature.

**Turning in place** slides this base sideways, and less the faster it turns: a 180 deg spin
moved base_link 4-9 cm at the 4 rad/s command and 10-18 cm at 1 rad/s (6 runs), and up to
24 cm in navigation. Forward commands barely act mid-spin, so spins run at the full Turn
rate (slow spins with a position hold were tried and drifted more), and the final pose is
fixed by docking.

**Docking.** When the goal faces away from the arrival direction, stage 3 brakes to stop
d = min(0.7, 0.15/|sin turn|) m short, so the turn leaves at most 0.15 m of sideways offset
(a 90 deg goal docked from 0.7 m swung ~0.46 m across the 1.3 m aisle into the plants; a
180 deg reverse dock keeps 0.7 m). Within d (along the path) and more than 0.5 rad off the goal heading, the
gateway turns in place to the goal heading, then drives onto the pose, forward or reverse
(chosen once), with the polar unicycle pose controller (Siegwart et al.): v = 0.5 rho
(0.08-0.3 m/s), w = 3 alpha - 1 beta (|w| <= 0.8 rad/s). The gains satisfy the strong-stability
condition (k_alpha + 5/3 k_beta - 2/pi k_rho > 0) and, on an ideal unicycle, converge from a
0.3 m purely sideways offset without swinging wider than it. A robot arriving within
0.075 m with a small heading error finishes with a short turn instead. When either finish
starts, the gateway cancels the Nav2 goal (RPP would otherwise keep collision-checking its
own, no longer executed, commands from the drifted pose and abort the goal) and runs the
finish from its own 20 Hz timer. Without RPP's check, the finish stops with an error if the
measured clearance to any collider drops below the physical radius (0.368 m). The gateway
then declares success itself once the base is at rest (below 0.02 m/s and 0.03 rad/s, the
same limits a terrain/contact Apply requires).

**Back-off before planning.** Smac cannot plan from a start inside the planning radius
("Start occupied"), and just outside it RPP refuses the in-place turn most goals start with (a
spin has ~0.20 m of slack off an aisle centre line and slides 4-24 cm). If Go finds the robot
within 0.10 m outside the planning radius but clear of colliders by its physical radius, the
gateway first creeps straight forward or backward at 0.15 m/s, toward the side with more
clearance on the known map, until it has that 0.10 m or has moved 0.3 m (3 s simulated), then
plans from rest if it is at least outside the planning radius. It is refused below the physical
radius, on stale feedback or on any manual command.

**Success** requires the measured pose within the goal checker's 0.075 m / 0.07 rad and the
robot at rest (under 0.02 m/s and 0.03 rad/s), docking having removed the slide of the final
turn. The Nav2 goal checker
(0.075 m / 0.07 rad) stays non-stateful: a stateful checker carried "XY reached" into the
next goal after a cancelled one.

Measured (headless, simulated seconds, Drive 2.0): straight 13 m aisle 10-12 s, peak
1.8-1.9 m/s (previously 0.22 m/s cap, 19.8 s); 180 deg turn then 9.5 m 12-16 s (89 s);
cross-aisle routes with two turnarounds 25-31 s including docking (122-132 s). Drive 0.5
holds 0.50-0.52 m/s. Final 7-goal loop: all reached, 3.6-7.1 cm and <= 3.3 deg at rest.
No blind timed playback is involved.

Nav output has generated twist_mux priority 1; generated external manual is
raised from 1 to 2, leaving joystick 10, marker 8, RC 12 and safety locks 253-255
unchanged. Installed twist_mux 4.5.0 explicitly masks priority 0, so it is not
used. Idle GUI and
terminal clients do not repeatedly overwrite it with zeros. Manual arrows or
Stop cancel immediately and route normal manual commands. Invalid goals, stale
native feedback (0.12 simulated seconds or 1.5 wall seconds), lifecycle failure
or unacknowledged cancel stop/block navigation. No automatic stale-path resumption.
Goals have finite 180 simulation-second/900 wall-second deadlines. The loopback
127.0.0.1:8765 API is container-local; no port is published. Runtime status and
pose/velocity/command/replan traces are under `SETUP_DIR/greenhouse/`.
Native service calls reuse one serialized Gazebo transport node per operation
process, with checked replies, unchanged request deadlines and bounded cleanup.
`native-rpc.jsonl` records request IDs, exact operations, first-service discovery
markers and combined discovery/response timing. Mutating requests are not retried
blindly; uncertain completion is reconciled while paused before recovery.

Health probes use the same persistent live `sim_checks.Observer` in the existing
executor, not a new ROS process every 20 seconds. The lightweight loopback
`/health` probe retains named world pose, clock, IMU and odometry receipt ages
below 3 seconds, both active controllers queried within 5 seconds, observer
liveness, and native/Nav readiness. It returns 503/nonzero on stale/dead/unready
state and explicitly reports an intentional terrain/contact update. Wheel
odometry is health-only. No cached "healthy" file substitutes for observation.
Health/status I/O stays outside the feedback lock; the HTTP probe needs no ROS
environment startup.

Version-specific sources:
[MPPI](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_mppi_controller/README.md),
[Smac](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_smac_planner/README.md),
[RPP](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_regulated_pure_pursuit_controller/README.md),
[DWB](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_dwb_controller/README.md),
[MeshNav](https://github.com/naturerobots/mesh_navigation),
[MBF](https://github.com/naturerobots/move_base_flex),
[acados](https://docs.acados.org/).

## Map scoring

`greenhouse/map_score.py` scores any occupancy map (later: a SLAM-built map) against the truth map
written by `navigation_map.create_map` (`SETUP_DIR/greenhouse/nav-map/map.yaml`), offline.

- Loading follows nav2 `map_io`: occupancy p = 1 - value/255 (p = value/255 with `negate: 1`);
  p > `occupied_thresh` is occupied, p < `free_thresh` free, anything between unknown. `scale` is
  classified like `trinary`, transparent pixels are unknown, `raw` reads 0..100 as percent. With the
  nav2 default `free_thresh: 0.25` a 205 pixel loads as free; `save_map` writes 0.196 so 205 stays unknown.
- Alignment: a candidate with a different origin, resolution or size is resampled onto the truth grid;
  each truth cell takes the candidate cell containing its centre (nearest cell, world coordinates),
  cells outside the candidate are unknown. Rotated origins are rejected.
- Truth-unknown cells are ignored. Precision, recall, F1 and IoU use only cells the candidate observed
  (free or occupied), so an unexplored area lowers coverage, not precision or recall;
  `unknown_as_free` repeats recall, F1 and IoU counting candidate-unknown as free.
- Tolerance k (cells, default 0): a candidate occupied cell is a true positive if truth occupied lies
  within k cells (Euclidean); a truth occupied cell is recalled if candidate occupied lies within k.
  IoU = TP / (TP + FP + FN), the usual IoU at k = 0.
- Boundary distance (m): between boundary cells (occupied cells with a non-occupied 4-neighbour), via
  `scipy.ndimage.distance_transform_edt`; mean and 95th percentile candidate→truth and truth→candidate
  (observed truth boundary only), and the symmetric mean over both directions' samples pooled.
- Coverage: fraction of truth-free cells the candidate observed. Fractions are 0..1; undefined
  metrics (empty denominator) are `null`.
- Heatmap (truth grid, upscaled to ≥ 800 px, legend below): true positive blue, false positive orange,
  false negative reddish purple, observed free white, unobserved grey (Okabe-Ito colours).

```bash
python3 tools/score_map.py TRUTH.yaml CANDIDATE.yaml [--heatmap OUT.png] [--json OUT.json] [--tolerance-cells N]
```

The CLI prints k = 0, 1 and N; the JSON holds `coverage`, `observed_fraction`, `boundary_distance_m`
and one `results` entry per k (`precision`, `recall`, `f1`, `iou`, `unknown_as_free`, `counts`).
The heatmap uses k = N.

## Setup folder

How to start is in the [README quick start](../README.md#quick-start-linux) and `bash run.sh --help`.
The setup folder (`--setup-dir`, default `./robot.yaml`'s folder, else `runtime/clearpath/`)
must contain `robot.yaml` and any custom files it references. It is mounted **writable** at
`/etc/clearpath`, with `SETUP_PATH=/etc/clearpath`, because Clearpath generates description,
platform and sensor files there. **`robot.yaml` is never overwritten**; only a missing
`runtime/clearpath/robot.yaml` is seeded from the bundled J100 example. There is no
sensor/mast substitution.

## Drive the robot

In Gazebo, use the **Drive and parameters** panel. Its topic is generated from the
actual `robot.yaml` namespace. For the example it is `/j100_0000/cmd_vel`.
Harmonic's stock Teleop plugin reads the topic from XML, but does not read
velocity-limit XML tags; this project's native replacement uses explicit runtime
properties instead. GUI speed controls remain authoritative. The GUI publishes a
Gazebo `gz.msgs.Twist` every 50 ms while a key is held; Clearpath's **existing**
bridge turns that into ROS `geometry_msgs/msg/TwistStamped`, twist_mux forwards it
(about 1 ms) and the stock `diff_drive_controller` drives the wheels.

Response time is set by the controller's rate limits, not by the message path.
Clearpath's J100 controller file ramps at 2 m/s² and 4 rad/s², which made a key take
0.5-1 s to reach or leave the panel's 2 m/s / 4 rad/s. `tools/prepare_platform.py`
applies `config/platform_overrides.yaml` (10 m/s², 20 rad/s², same for braking) to
the generated `platform/config/control.yaml`; a value the user sets under
`platform.extras.ros_parameters` in robot.yaml is kept. Measured at real time
(sim ms after press / release, Clearpath limits → overrides):

| Command | Body at 10% | Body at 50% | Stopped to 10% |
|---|---|---|---|
| Forward 1 m/s | 82 → 46 | 484 → 326 | 348 → 276 |
| Forward 2 m/s | 170 → 84 | 1110 → 804 | 568 → 470 |
| Turn 1 rad/s | 106 → 38 | not reached | 186 → 44 |
| Turn 4 rad/s | 414 → 106 | not reached | 620 → 152 |

The remaining lag is wheel slip on the soil (`slip_longitudinal` / `slip_lateral`):
forward speed creeps from 50% to 90% over about another second, and turning in place
reaches only 40-58% of the commanded yaw rate with the stock
`wheel_separation_multiplier` of 1.5.

For a terminal keyboard inside the same running container:

```bash
docker exec -it jackal_greenhouse /opt/greenhouse/tools/teleop.sh
```

Hold **Up/Down** for forward/reverse or **Left/Right** for turning. A/Q raise/lower.
C cycles the 3D view and the two gimbal cameras. Space stops; Esc/Ctrl-C exits the terminal.
This small bundled keyboard client stamps commands with simulation time, discovers
the namespace from the running configuration, defaults to 0.2 m/s, and stops after
0.35 s without a repeated key. It requires no host ROS installation. It supplements,
not replaces, the stock Gazebo teleop bridge.

For other ROS tools, the real interfaces are:

| Interface | Example namespace | Type |
|---|---|---|
| Command input | `/j100_0000/cmd_vel` | `geometry_msgs/msg/TwistStamped` |
| Filtered odometry | `/j100_0000/platform/odom/filtered` | `nav_msgs/msg/Odometry` |
| Controller manager | `/j100_0000/controller_manager` | ROS controller-manager services |
| Clock | `/clock` | `rosgraph_msgs/msg/Clock` |
| Added physics observation | `/greenhouse/ground_truth` | `tf2_msgs/msg/TFMessage` |

For the root namespace, set `system.ros2.namespace: "/"`: use `/cmd_vel`,
`/platform/odom/filtered` and `/controller_manager`. This Clearpath version rejects
an empty-string namespace. Do not send unstamped `Twist` to this Jazzy ROS command
input. `ros2` commands used with `docker exec` should be preceded by
`/ros_entrypoint.sh`, which sources the image's ROS environment.

## Visualisation hub (`--viz`)

`run.sh --viz` adds three processes to the launch (`launch/greenhouse.launch.py`):
`foxglove_bridge` (apt package 3.4.1, sim time, port 8767, published by Docker on host
127.0.0.1 only), `tools/pose_error.py` (node `greenhouse_eval`) and `tools/ros_graph.py`
(node `greenhouse_ros_graph`, see "System graph" below). The bridge speaks the
Foxglove WebSocket protocol with subprotocol `foxglove.sdk.v1`; it rejects clients that offer
only `foxglove.websocket.v1`, so use the Foxglove app or a Lichtblick release that offers
both. Connect to `ws://127.0.0.1:8767` and import `config/foxglove-layout.json` (tabs:
robot/TF/lidar 3D view and cameras; system graph; odometry vs truth; odometry error; IMU).
The 3D panel sets `scene.meshUpAxis: "z_up"`: Lichtblick's default `y_up` turns every STL 90°
about x (three.js convention), which splits the Jackal into a standing chassis and flat wheels.
Only five frames show axes (`base_link`, `lidar2d_0_laser`, `camera{1,2}_link`,
`fpb30_carriage`); `map`, `odom` and `greenhouse` stay hidden so no line runs to the world origin. The
layout is written for namespace `j100_0000`; another namespace needs its topic names edited.
Only one `--viz` simulation can run at a time (fixed host port).

Error topics, `std_msgs/msg/Float64MultiArray`, one message per odometry message (50 Hz sim):

| Topic | Estimate |
|---|---|
| `/greenhouse/eval/ekf_error` | `<ns>/platform/odom/filtered` (Clearpath EKF) |
| `/greenhouse/eval/wheel_odom_error` | `<ns>/platform/odom` (diff-drive wheel odometry) |

`data` index (also listed in `layout.dim[0].label`): 0-5 error = estimate − truth: x, y (m,
greenhouse frame), planar norm (m), yaw (rad, wrapped to ±π), body vx (m/s), wz (rad/s);
6-10 the aligned estimate x, y, yaw, vx, wz; 11-15 truth x, y, yaw, vx, wz. Plot paths look
like `/greenhouse/eval/ekf_error.data[2]`.

Truth is the `j100_0000/robot` entry (parent `greenhouse`) of `/greenhouse/ground_truth`,
interpolated to each odometry stamp. gz publishes poses only, so truth vx (along the heading)
and wz are the central difference of the interpolated truth pose over ±0.05 s; errors are
therefore published about 0.05 s sim behind the odometry. Odometry frames start at the spawn
pose, so each source is aligned once: at its first sample the node takes the truth pose and
fixes `greenhouse ← odom` so that sample has zero pose error. Errors are drift since that
sample (restarting the node realigns). Measured during `sample_use.py`: the stock EKF's
twist follows the IMU (wz error ≈ 0.01 rad/s) but its pose yaw follows the slipping wheel
odometry (a commanded 0.6 rad truth turn read as 2.5-2.65 rad by both), so both position
errors grow to tens of metres within a minute of driving.

Greyscale views: `<ns>/greenhouse/camera{1,2}/image_mono` (`mono8`, BT.601 luma). The node
subscribes to the colour image only while the mono topic has a subscriber (checked every
0.5 s wall time), so `--viz` costs no image transport or conversion without a viewer.

### System graph

Lichtblick's built-in Topic Graph draws every node and topic at once with straight edges. The
"System graph" tab uses a panel extension instead (`viz/subsystem-graph`, plain JS + elkjs):
pick a subsystem (Drive & wheels, Cameras & gimbal, Lift, Sensors, Pose & TF, Navigation,
Evaluation, Other) and see only its ROS nodes and topics, laid out in layers with orthogonal
routed edges (left to right in wide panels, top to bottom in tall ones). Hover a box to
highlight its connections; wheel zooms, drag pans, double-click or Fit refits. Topics show
their message rate as received by Lichtblick (large images arrive slower than ROS publishes
them), `latched` for transient-local topics, or `silent`; a dashed outline means no publisher
or no subscriber. Topics nobody publishes (e-stop, RC teleop) hide behind "Show unused inputs".

Data comes from `tools/ros_graph.py`, which publishes the graph every 2 s (wall clock) as JSON
in a `std_msgs/msg/String` on `/greenhouse/viz/graph`. Subsystem membership is the regex list
`SUBSYSTEMS` there (topics may sit in several; unmatched ones go to "Other"). Plumbing is
dropped (`NOISE_TOPICS`, `NOISE_NODES`: rosout, parameter events, `/clock`, lifecycle, bond,
costmap raw/updates, controller statistics, foxglove_bridge) and each action's
`_action/feedback` + `_action/status` pair becomes one `action` edge from server to clients.
Measured cost: ~80 MB RSS, under 1 s CPU per minute.

Install the panel once on the host (Node and npm), then restart Lichtblick:

```bash
bash viz/subsystem-graph/install.sh
```

It runs `npm ci`, the panel tests and an esbuild bundle, and copies `package.json` and
`dist/extension.js` to `~/.lichtblick-suite/extensions/greenhouse.subsystem-graph-<version>/`.
The layout names the panel `Greenhouse.Subsystem graph` (Lichtblick desktop qualifies local
extension panels by `displayName`).

## Terrain, plants and clearances

The defaults are in `config/greenhouse.json`; distances are metres.

| Property | Default / measured geometry |
|---|---|
| Soil height variation | **0.04130 m peak-to-peak** over the whole greenhouse with the selected firm-earth defaults |
| Terrain mesh | 255 x 255 surface vertices, approximately 0.07441 m spacing; 132,080 closed-mesh triangles |
| Terrain extent | Approximately 18.901 x 18.901 m |
| Greenhouse shell | 18 m long x 10 m wide; 2.4 m eaves / 3.4 m ridge |
| Rows | Four 12 m rows; 2.0 m centre spacing; 0.75 m plant spacing; 68 plants |
| Tomato plant | **1.400 m measured base-to-tip**, 0.350 m maximum horizontal radius |
| Clear inter-row aisle | **1.300 m**, after both actual rotated foliage and conservative collision extents |
| Clear side aisle | At least 1.600 m |
| End-turning headland | At least 2.600 m after plant extents and frame allowance |
| End door | 2.720 m clear between posts; both ends open, no thresholds |
| Initial robot pose | x = -7.5, y = 0, yaw = 0; z sampled above its whole padded footprint |

The terrain is **not a flat plane with a rough texture**. The unchanged
`Field2DGenerator.generate_ground` method from
[virtual_maize_field](https://github.com/FieldRobotEvent/virtual_maize_field/tree/e525d70c3c73fcdfae9c77bb8e4ca03bf3377ac3)
actually runs. Its seeded NumPy random fields are resized with OpenCV, summed at
successive powers of two with weights `(n+1)^2`, normalized, and converted to the
upstream 8-bit heightmap. This source heightmap, including its plant-footprint
flat spots, is preserved unchanged as `heightmap.png`.
Setting its supported `ground_ditch_depth` to half the elevation range removes
the outdoor perimeter ditch without removing roughness.

The upstream's large-scale octaves spread its elevation range over
metres: elevation differences across Jackal's wheel track were mostly only a few
millimetres. The terrain adapter now extracts two smooth spatial bands from
**that same seeded upstream output**, creating irregular wheel-scale mounds and
depressions instead of merely scaling up the whole field. The primary Gaussian
scale is 0.18 m (with a 0.54 m background scale); a smaller band contributes 25%.
The default 3 mm component-RMS target is smoothly bounded with `tanh`, avoiding sharp
spikes. Gentle smoothing removes image quantization/flat-spot edges, and mesh
subdivision preserves the original XY footprint and mean ground elevation.
Default slopes are about 1.69 degrees at the 95th percentile, with an isolated
maximum about 4.66 degrees. The older 16 mm option is substantially rougher.

The resulting heights are converted into a local, closed triangle mesh. **The
visual and collision use the very same mesh and scale.** Its underside is at
z = -0.2 m, below every surface point; there is no flat floor under the robot that
masks the terrain. Plant bases and the spawn footprint are sampled on the exact
emitted triangles, not on an unrelated bilinear approximation.

The soil has a locally generated, non-repeating **2048 x 2048 albedo atlas**
covering the exact terrain footprint (about 9.23 mm per texel). Its restrained
cavity tint follows the actual mesh elevation relative to a 0.35 m neighborhood:
real hollows are darker and real crowns lighter. Fine seeded soil grain adds
surface scale without painting unrelated mounds. The mean linear soil RGB stays
`(0.34, 0.24, 0.135)`; sRGB encoding and a white texture multiplier avoid tinting
the brown soil twice. A matte, nonmetallic ground-only PBR material uses that
local atlas. It is registered through physical XY-derived UVs in both the
COLLADA material and the SDF albedo binding. There is no displacement/normal map;
normal-dependent lighting still uses the real mesh normals. This appearance
refinement does not change any vertex, normal, collision triangle, plant pose,
friction setting, controller, global light or other object's material.

Tomatoes are **stylized, not botanically exact**. Four resized, rotated, stacked
broad-leaf crowns adapt the upstream `unknown_weed` / **simple plant** model by
aaron_nerlich (CC BY 4.0). Each connected blade is shortened and widened, rather
than stretching grasslike blades into tall corn. Added branching stems, twelve red fruit and green
calyces make them recognizable tomatoes rather than taller corn. The original
COLLADA scene scale is baked before measuring the final generated geometry.
Each shared tomato mesh is 2,640 triangles; each plant has only a stem and a
conservative cylindrical canopy collider. Leaves/fruit are not individually
dynamic, flexible, pickable or soft-contact objects.

The transparent side/roof panels and structural frame are collidable, with open
doors and unobstructed headlands. The 0.8 x 0.64 m spawn safety envelope covers a
standard Jackal with fenders, **not arbitrary oversized custom payloads**.

## Configuration and regeneration

```bash
# Use a modified JSON without changing the baked image defaults:
bash run.sh --config config/greenhouse.json

# Export deterministic assets to your writable setup's greenhouse/generated/:
bash run.sh --generate-only --config config/greenhouse.json

# Build without opening a window:
bash run.sh --build-only

# Inspect commands only (does not prove Docker/runtime readiness):
bash run.sh --dry-run --headless
```

Relative option paths are resolved from the script's directory, not whichever
directory the caller happens to be in. Paths with spaces work. Bind-mount paths
with commas or line breaks are rejected explicitly.

Use `seed` to vary terrain and `ground_height_range` for the upstream 5-80 mm
height range. `ground_bump_rms` controls the added wheel-scale relief strength
(default 0.003 m; zero restores the original surface exactly);
`ground_bump_scale` controls patch size (default 0.18 m). These are physical mesh
changes, not texture effects or artificial vehicle drift. `ground_resolution`
controls upstream sampling; enabled relief subdivides it to at most 0.075 m
mesh spacing. The final range is reported in `manifest.json`, not assumed equal
to the upstream range. Row, shell and plant
dimensions are configurable but validation rejects less than **1.2 m clear**
inter-row/side aisles, unsafe spawn positions, undersized headlands, unsupported
terrain extents and excessive geometry budgets. Keep `house_length`/
`house_width` inside the generated soil, increasing `ground_headland` if needed.
Changing source code requires a rebuild; do not use `--no-build` after editing it.

The image contains pre-generated default assets. A custom JSON generates local
assets under the mounted setup directory. `manifest.json` records the upstream
pin, geometry bounds, clearances, every plant's sampled pose, spawn and resource
hashes. `terrain.json` stores the exact collision-surface samples.
The tracked `generated/` directory is the same default asset set, exported for
inspection. It is not used as an unverified Docker build input; the image
regenerates its own assets from the pinned source and configuration.

After rebuilding, restart to load the new image; an already-running container keeps
its old mesh. Custom configurations are regenerated
on startup; the exported `generated/` folder is not a launch-time cache.
Live terrain/contact Apply does not restart the container. Robot/wheel geometry,
controllers, manual key behavior, global lighting, other objects' materials,
plants and row XY positions remain unchanged. Terrain updates resample support Z;
ground appearance stays registered to that same physical surface.
Natural slip and imperfect heading remain physics-driven; the exact trajectory
can change when the contact surface changes.

## Scenarios and perturbation objects

A scenario JSON (`run.sh --scenario FILE`, example `config/scenario.example.json`) fixes a run.
Schema v1, validated strictly by `greenhouse/scenario.py` (unknown keys, wrong types, ids used
twice, objects outside the walls or on the robot start, and events that refer to an object
that does not exist at that time are all rejected with the offending path, e.g. `objects[2].x`):

| Key | Required | Meaning |
|---|---|---|
| `version` | yes | `1` |
| `seed` | yes | greenhouse `seed` (terrain relief, soil texture, plant yaws) |
| `greenhouse` | no | partial greenhouse config, merged over `--config` (itself over `config/greenhouse.json`); not `seed`/`spawn_*` |
| `actuator` | no | partial actuator config, merged over `--actuator-config` |
| `start` | yes | robot spawn `{x, y, rz}` (m, m, deg) -> `spawn_x`, `spawn_y`, `spawn_yaw` |
| `objects` | no | `[{id, type, x, y, rz}]` spawned once Gazebo and the Jackal exist; `rz` defaults to 0 |
| `events` | no | `[{t, action, ...}]`: `t` simulated seconds since the world started; `add` takes `id, type, x, y[, rz]`, `move` `id` plus any of `x, y, rz`, `remove` `id`. Fired in `t` order, ties in file order |

`id` matches `[A-Za-z][A-Za-z0-9_-]{0,47}`; the Gazebo model is `scenario_<id>` (never `robot`,
which the world-state system takes for the Jackal). Start-up (`docker/start.sh`): `python3 -m
greenhouse.scenario prepare` writes `scenario-greenhouse.json` and `scenario-actuator.json` into
`SETUP_DIR/greenhouse/` and points `GREENHOUSE_CONFIG`/`ACTUATOR_CONFIG` at them, so world
generation and the robot spawn use them; then `tools/scenario.py run` (background) spawns the
objects and fires the events against `/greenhouse/world/status` sim time.

Object types, all static models with a collision for every visual shape (sizes cited in
`greenhouse/scenario.py`): `fertiliser_block` 0.60 x 0.40 x 0.15 m sack; `fallen_tomato`
0.07 m sphere; `fallen_plant` 1.2 m lying stem with four leaflets; `robot` J100-sized
0.508 x 0.430 x 0.25 m body on four wheels; `person` 1.75 m standing adult (boxes, cylinders,
sphere head; an animated actor is future work because it needs Fuel assets). Each rests on the
current terrain: a plane fitted to the surface under its footprint (5 cm samples) gives roll
and pitch, and z is raised by the largest residual so no sample is above the bottom face.

Live control, the API and the runner share `tools/scenario.py` and one state file,
`SETUP_DIR/greenhouse/scenario-state.json` (objects with resting `z`, events with
`fired_sim_time`), replaced atomically under `scenario.lock` and cleared at every start.
`save FILE` / `save_scenario(path)` writes seed, the greenhouse config in use (including a live
terrain Apply), the actuator config, the robot's Gazebo truth pose as `start`, and the current
objects; it carries no events (a snapshot replays from sim time 0) and refuses a robot pose
that would fail spawn validation. Gazebo calls: `/world/greenhouse/create/blocking`
(`EntityFactory` with inline SDF), `set_pose/blocking`, `remove/blocking`.

Limits (v1): objects are not in the navigation map (Nav2 plans through them; obstacle reaction
is M5); a live terrain Apply does not re-rest existing objects; no GUI panel yet.

## Why this is a small custom launch

The base is still
`ghcr.io/clearpathrobotics/clearpath_docker:jazzy-sim-latest`, frozen at:

```text
sha256:0355e4bb50d50dddfdb2416365c4f155e28eb7ff3a9a53de3c3d42864eba0979
```

The image's real entrypoint is `/ros_entrypoint.sh`, user `robot`, and its original
CMD is `/usr/local/bin/cpr-sim-launch`. The entrypoint is **inherited unchanged**.
Only the warehouse-specific CMD is replaced.

The installed Jazzy `simulation.launch.py` restricts `world` to a fixed list,
while `gz_sim.launch.py` overwrites resource paths and has no headless argument.
Rather than invent `WORLD`/`HEADLESS` environment variables or edit installed
Clearpath files, `launch/greenhouse.launch.py` composes `ros_gz_sim` with the
**stock `clearpath_gz/robot_spawn.launch.py`**:

- The same four Clearpath description/semantic/launch/parameter generators run
  sequentially with checked exit codes against the writable setup directory.
- Stock robot spawning then uses `generate:=false`, the same namespace,
  generated services/controllers/sensor bridges, and `world:=greenhouse`.
- Gazebo, all sensor prefixes, clock, pause services and spawn consistently
  select `greenhouse`. Resource paths append local assets and sourced ROS shares.

The observation bridge uses the **existing stock model PosePublisher** at
`/model/<namespace>/robot/pose`, retaining its parent/child frame names. The smoke
selects only the exact robot model pose relative to `greenhouse`; it does not
mistake link-relative transforms or unnamed SceneBroadcaster poses for world pose.

No ROS 1 / Gazebo Classic plugins are imported. Physics, UserCommands,
SceneBroadcaster, Sensors, Imu and NavSat are explicit Harmonic systems.
All greenhouse/plant resources are local; the example Jackal uses image-local
Clearpath resources. Custom robot extensions must also have their meshes/packages
available in the image/setup; remote model URIs in generated descriptions fail.
There are no launch-time Fuel downloads for the supplied setup.

Generator-only dependencies are pinned in `requirements-generator.txt` and
installed into an isolated image directory; ROS's own NumPy/OpenCV are left alone.
The digest freezes the base ROS/Gazebo/OS dependencies. Only the explicitly
documented compatible Nav2-family packages are added/patch-aligned in the image;
no general OS upgrade or global host install is performed.

## Checks and bounded headless smoke

```bash
# Unit/geometry/resource checks plus Bash argument/lifecycle checks:
docker run --rm --memory 3g jackal-greenhouse:jazzy bash -c \
  'PYTHONPATH=/opt/greenhouse/python:/opt/greenhouse:$PYTHONPATH python3 -m unittest discover -s tests'

# Real production entrypoint: generate, spawn Jackal, check controllers/bridges,
# drive at 0.18 m/s for eight simulation seconds, command zero, then shut down:
bash run.sh --smoke-test --name jackal_greenhouse_smoke
```

The image build also checks the vendored source hashes, terrain-function AST,
determinism, actual COLLADA bounds/scales, clearances, triangle sampling,
watertight collision, local URIs and **`gz sdf -k`** on every generated SDF.
Launcher unit tests use an explicitly fake Docker only for argument/error
contracts; they are **not** physics evidence.

The live smoke requires advancing ROS time, real named world-pose data, IMU,
filtered odometry and both active controllers, followed by plausible motion,
small nonzero vertical response and a measured stop. The live checker has a
180-second wall-time limit; the entire in-container smoke, including all
generators and shutdown, has a 240-second deadline plus at most 15 seconds of
termination grace. Its result and full launch log are persisted at
`SETUP_DIR/greenhouse/smoke-result.json` and `smoke-launch.log`.
Only its own container/launch process is stopped. No global prune, name-based
process kill, or removal of `jackal_warehouse` is performed.

Two optional checks operate on an already-running container:

```bash
# Exercise the actual Gazebo GUI message type and existing GZ -> ROS bridge:
docker exec jackal_greenhouse /ros_entrypoint.sh python3 \
  /opt/greenhouse/tools/smoke_gazebo_teleop.py \
  --output /etc/clearpath/greenhouse/gazebo-teleop-result.json

# Render the real scene with a temporary native Gazebo camera (EGL in headless mode):
docker exec jackal_greenhouse /ros_entrypoint.sh python3 \
  /opt/greenhouse/tools/render_check.py \
  --output-dir /etc/clearpath/greenhouse/render-check
```

The first moves the simulated robot a short distance, then stops it. The second
saves overview, aisle and tomato PNGs, and removes its own camera/bridge afterward.
Neither opens a desktop window.

Evidence from the September 2026 runs (soil, actuator and terrain studies) is kept under
`test-results/`; each run's own evidence goes to the setup's `greenhouse/` folder. These checks
are not a guarantee for other host graphics drivers or arbitrary custom robot configurations.

## Graphics and operational limits

- X11/Xwayland use the existing local X11 socket; a symlinked socket directory is
  resolved before mounting. Qt uses XCB; direct Wayland and SSH/TCP X11 are not
  configured automatically.
- `XAUTHORITY`, or readable `~/.Xauthority`, is mounted read-only. Matching the
  container hostname preserves local X11 cookie lookup. The container explicitly
  checks `XOpenDisplay`; missing/invalid authorization is an error. Never use
  unrestricted `xhost +`.
- Existing `/dev/dri` devices and groups are passed through in every mode, so both the
  window and `--headless` (server-only Gazebo with EGL) render on the GPU. Measured on an
  Intel Iris Xe: real-time factor 1.0 with the GPU, 0.5 headless and 0.23 with the window
  on Mesa llvmpipe (`--software-rendering`, which skips the passthrough). NVIDIA-specific
  runtime setup is not included; **real-time speed is not promised on every driver**.
- There is no privileged container, published ROS port, or host-network
  requirement. ROS discovery is limited to the container's localhost. This
  launcher is for simulation, not controlling a physical robot.
- An existing container with the chosen name causes an explicit error, even if
  stopped. Use `--name` rather than deleting someone else's container.
- The stock example enables its PS4 joystick node. Without a passed-through
  gamepad, its `/dev/input/ps4` warning is expected; GUI/keyboard teleop still work.
  The stock model can also report IMU free-fall warnings during its brief spawn
  drop, `gz_frame_id` extension warnings and occasional EKF timing warnings under
  load. Missing mesh/plugin errors are not expected and must not be ignored.
- The motion/geometry checks are for a normal J100 on gentle terrain, not
  calibrated soil deformation, tire models, crop contact forces or prior
  research-project acceptance results.

See [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md) and `licenses/` for attribution.
