"""Gimbal dynamics test world: generated wrapper on a heavy rocking/lifting base (no Jackal model)."""
import sys, tempfile
from pathlib import Path
from xml.etree import ElementTree as ET
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from greenhouse.actuator import load_actuator, generate_wrapper

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
setup = out / "setup"; setup.mkdir(exist_ok=True)
(setup / "robot.urdf.xacro").write_text("stub")
generate_wrapper(load_actuator(str(Path(__file__).resolve().parents[2] / "config" / "actuator.json")), setup, "")
robot = ET.parse(setup / "greenhouse" / "robot-with-actuator.urdf.xacro").getroot()
for include in robot.findall("{http://www.ros.org/wiki/xacro}include"):
    robot.remove(include)
for link in robot.findall("link"):           # drop the STL visual (not needed headless)
    for visual in link.findall("visual"):
        geometry = visual.find("geometry")
        if geometry.find("mesh") is not None:
            link.remove(visual)
robot.set("name", "robot")
def link(name, mass, size):
    l = ET.SubElement(robot, "link", name=name)
    i = ET.SubElement(l, "inertial"); ET.SubElement(i, "mass", value=str(mass))
    x, y, z = size
    ET.SubElement(i, "inertia", ixx=str(mass*(y*y+z*z)/12), iyy=str(mass*(x*x+z*z)/12), izz=str(mass*(x*x+y*y)/12),
                  ixy="0", ixz="0", iyz="0")
    c = ET.SubElement(l, "collision"); g = ET.SubElement(c, "geometry"); ET.SubElement(g, "box", size=f"{x} {y} {z}")
def joint(name, kind, parent, child, xyz="0 0 0", axis=None):
    j = ET.SubElement(robot, "joint", name=name, type=kind)
    ET.SubElement(j, "parent", link=parent); ET.SubElement(j, "child", link=child)
    ET.SubElement(j, "origin", xyz=xyz, rpy="0 0 0")
    if axis:
        ET.SubElement(j, "axis", xyz=axis)
        ET.SubElement(j, "limit", lower="-3", upper="3", effort="5000", velocity="10")
ET.SubElement(robot, "link", name="world")
link("yaw_stage", 5, (0.1, 0.1, 0.1)); link("roll_stage", 5, (0.1, 0.1, 0.1)); link("base_link", 17, (0.42, 0.31, 0.184))
joint("rock_yaw", "revolute", "world", "yaw_stage", "0 0 0.5", "0 0 1")
joint("rock_roll", "revolute", "yaw_stage", "roll_stage", "0 0 0", "1 0 0")
joint("rock_pitch", "revolute", "roll_stage", "base_link", "0 0 0", "0 1 0")
for name in ("rock_yaw", "rock_roll", "rock_pitch"):
    g = ET.SubElement(robot, "gazebo"); p = ET.SubElement(g, "plugin", filename="gz-sim-joint-controller-system",
                                                          name="gz::sim::systems::JointController")
    ET.SubElement(p, "joint_name").text = name
    ET.SubElement(p, "topic").text = f"/test/{name}"
(out / "robot.urdf").write_text(ET.tostring(robot, encoding="unicode"))
world = f"""<?xml version="1.0"?>
<sdf version="1.9"><world name="test">
  <physics name="2ms" type="dart"><max_step_size>0.002</max_step_size><real_time_factor>0</real_time_factor></physics>
  <gravity>0 0 -9.81</gravity>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <include><uri>file://{out / 'robot.urdf'}</uri><name>robot</name></include>
</world></sdf>"""
(out / "world.sdf").write_text(world)
print(out / "world.sdf")
