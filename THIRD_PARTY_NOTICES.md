# Licenses, provenance and adaptations

## Virtual Maize Field terrain

Repository: <https://github.com/FieldRobotEvent/virtual_maize_field>  
Pinned revision: `e525d70c3c73fcdfae9c77bb8e4ca03bf3377ac3` (`ros2-gz`, version 6.0.0).  
Copyright (C) 2021 Farm Technology Group of Wageningen University & Research
and Kamaro Engineering e.V. License: **GPL version 3**.

`third_party/virtual_maize_field/upstream/` preserves the original README,
CITATION, full GPL license and
`virtual_maize_field/world_generator/field_2d_generator.py`.
`provenance.json` records source/asset SHA256 checksums.

`third_party/virtual_maize_field/terrain.py` is an explicitly modified subset:
the imports and constructor were narrowed to NumPy/OpenCV and supplied state.
**The `generate_ground` method is unchanged**, and a maintained AST comparison
checks this. It is invoked by `greenhouse/terrain.py`; it is not an unused
download. ROS/plotting/model/row-generation interfaces and templates are not
imported. In upstream the wider generator also depends on ROS package discovery,
Jinja2, Shapely, Matplotlib and YAML; those unrelated interfaces are unnecessary
for this standalone terrain subset.

The greenhouse supplies straight rows, explicit NumPy seed and supported ground
parameters. Setting ditch depth to half the requested elevation makes upstream's
field-height offset zero. The downstream adapter decodes the upstream uint8
heightmap and reverses the image Y convention. The terrain-only refinement
extracts smooth wheel-scale bands from that same heightmap, softly limits their
relief, and subdivides the mesh without changing its XY footprint. It retains
the original heightmap separately, closes the mesh underside below ground, and
uses triangle-exact placement on the final surface. The vendored upstream
function and revision remain unchanged.

This project's new source is distributed under **GPL-3.0-only**, unless noted
otherwise. The complete license is in `LICENSE` and in the upstream directory.
Source code and build instructions for the adapter are included in the image.

## Broad-leaf asset adapted into tomatoes

**simple plant**, by **aaron_nerlich**, 2016, distributed upstream as
`models/unknown_weed/`.

Original source:
<https://sketchfab.com/3d-models/simple-plant-8983e5d47f12435bbd1189168b75abeb>  
License: **Creative Commons Attribution 4.0 International**,
<https://creativecommons.org/licenses/by/4.0/>  
Full text: `licenses/CC-BY-4.0.txt`.

The original mesh, texture, `model.config` and `model.sdf` are preserved verbatim
under `third_party/virtual_maize_field/upstream/models/unknown_weed/`.
The upstream model config additionally credits Thijs Ruigrok for its packaging.

Changes by this project: bake the original metre/Z-up scene's 0.10223 scale;
shorten each connected leaf blade radially to 0.75x and widen it tangentially to
2.2x for broad, non-grasslike leaves; resize/rotate/stack four low-rise crowns;
render both sides of alpha-masked leaf geometry; add original
branching stems, twelve red tomato fruits and green calyces; normalize the final
mesh to its measured height/radius; replace the original physics with two stable
conservative primitive colliders. The resulting **stylized tomato asset and its
copied leaf texture remain available under CC BY 4.0**. Retain this attribution
when redistributing generated assets. No endorsement by the original author is
implied.

No maize, noncommercial, restricted dandelion, bottle or third-party ground
texture assets are included. Soil and greenhouse-frame/panel materials are
generated locally.

## Clearpath Robotics

The project extends the published Clearpath Jazzy simulator image without
vendoring or modifying its ROS packages or installed launch files.

Image tag: `ghcr.io/clearpathrobotics/clearpath_docker:jazzy-sim-latest`  
Pinned OCI index digest:
`sha256:0355e4bb50d50dddfdb2416365c4f155e28eb7ff3a9a53de3c3d42864eba0979`  
Image's Docker-source revision:
`3e15b020476cc7180213791f2e0d244042b9e0ab`.

Clearpath Docker source is Apache-2.0; full text is in
`licenses/Apache-2.0.txt`. Installed packages/resources retain their respective
upstream notices inside the base image.

`config/robot.example.yaml` is adapted from
`clearpath_config/sample/j100/j100_default.yaml` at Clearpath Config revision
`b2a64ba3b12d20bfeeb1127c3869accc7704e634`, with only an attribution comment
added. Copyright (C) Clearpath Robotics, Inc.; BSD-3-Clause license text is
preserved at `licenses/Clearpath-Config-BSD-3-Clause.txt`. The example is checked
with `ClearpathConfig` from the actual pinned image.

`tests/fixtures/clearpath-control.yaml` is the unmodified `platform/config/control.yaml`
that `clearpath_generator_common` 2.9.16 generates in the pinned image from
`clearpath_control/config/j100/control/diff_4wd.yaml` (BSD, Copyright (C) Clearpath
Robotics, Inc.; license text at `licenses/Clearpath-Config-BSD-3-Clause.txt`). It is test
input for `tools/prepare_platform.py` only.

Precise interfaces were checked against installed image files and these primary
sources (the image's hashes are also recorded by `tools/check_image.py`):

- <https://github.com/clearpathrobotics/clearpath_docker/tree/3e15b020476cc7180213791f2e0d244042b9e0ab>
- <https://github.com/clearpathrobotics/clearpath_simulator/tree/590a45118c599c08abdf3bd3d02d5fa7a8af9a49>
- <https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/simulate/>
- <https://docs.clearpathrobotics.com/docs/ros/tutorials/driving/>
- <https://docs.clearpathrobotics.com/docs/ros/config/yaml/system/>

## Gazebo / ROS / generator dependencies

ROS 2 Jazzy, Gazebo Harmonic, ros_gz and Clearpath retain their own licenses in
the base image. Modern plugin/resource-path guidance was verified using
<https://gazebosim.org/docs/harmonic/> and the official Gazebo migration docs.

NumPy 1.26.4 and opencv-python-headless 4.11.0.86 are pinned generator-only wheel
dependencies. Their upstream license notices ship with their installed wheel
metadata in `/opt/greenhouse/python`. They do not replace the base image's ROS
Python dependencies. There is no GPL/Classic ROS 1 Gazebo plugin port.
