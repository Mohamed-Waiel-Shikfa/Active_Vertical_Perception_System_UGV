# Changelog

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Versioning: [Semantic Versioning](https://semver.org/).
Below 1.0.0, config keys, ROS topics and the scripting API may change in any minor release.

## [0.1.0] - 2026-11-14

First public release: a Gazebo simulation of a Clearpath Jackal in a greenhouse, a test bed for active vertical perception. It is research code in progress; state estimation from sensors and mapping are not implemented yet (navigation uses the simulator's true pose).

### Added
- Clearpath Jackal J100 with an FPB30 vertical lift on the rear deck and two cameras on 3-axis gimbals under the carriage, all in one Docker container (`run.sh`; the host needs no ROS).
- Greenhouse on a physically uneven heightfield (terrain from Virtual Maize Field, GPL-3, see `THIRD_PARTY_NOTICES.md`), with live terrain, friction and wheel-slip settings.
- Gazebo window with a drive panel, keyboard driving, lift and gimbal controls.
- Navigation with Nav2 (Smac 2D planner, Regulated Pure Pursuit) using the simulator's true pose.
- 2D lidar (SICK TiM551) on `sensor_msgs/LaserScan`.
- Scripting API (`self_driving/`) and replayable scenario files with live perturbation objects (person, robot, fallen plant, fertiliser block).
- Visualisation hub for Foxglove/Lichtblick, presentation demos, and a printable rear-deck mount (`cad/`).
- Headless, smoke-test and dry-run modes, a unit-test suite, and technical documentation in `docs/`.
