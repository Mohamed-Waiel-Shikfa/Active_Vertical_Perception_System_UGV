# SPDX-License-Identifier: GPL-3.0-only
"""Known-world occupancy from actual SDF collision volumes, never from rendered colors."""
import argparse
import json
import math
from collections.abc import Iterable
from pathlib import Path
from typing import Any
from xml.etree import ElementTree as ET

import cv2
import numpy as np
import yaml

HALF_LENGTH = 0.279  # max(URDF collision half-length .229, Jackal body .254) + .025m margin
HALF_WIDTH = 0.240   # max(wheel collision edge .207795, body .215) + .025m margin
RADIUS = math.hypot(HALF_LENGTH, HALF_WIDTH)
RESOLUTION = 0.05


def transform(text: str | None) -> np.ndarray:
    """4x4 homogeneous transform from a pose string "x y z roll pitch yaw" (metres, radians).

    Rotation is Rz(yaw) @ Ry(pitch) @ Rx(roll), as in SDF and URDF; None or "" is identity.
    Raises ValueError unless the string holds exactly six numbers.
    """
    values = list(map(float, (text or "0 0 0 0 0 0").split()))
    x,y,z,r,p,w = values
    cr,sr,cp,sp,cw,sw = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(w),math.sin(w)
    matrix = np.eye(4)
    matrix[:3,:3] = ((cw*cp,cw*sp*sr-sw*cr,cw*sp*cr+sw*sr),
                      (sw*cp,sw*sp*sr+cw*cr,sw*sp*cr-cw*sr),(-sp,cp*sr,cp*cr))
    matrix[:3,3] = (x,y,z)
    return matrix


def collision_vertices(geometry: ET.Element) -> np.ndarray:
    """Homogeneous (N, 4) points in metres, in the geometry's own frame: a box's 8 corners or 2 x 32 cylinder rim points.

    Raises ValueError for any other primitive.
    """
    box, cylinder = geometry.find("box"), geometry.find("cylinder")
    if box is not None:
        size=np.fromstring(box.get("size") or box.findtext("size"),sep=" ")/2
        return np.array([(x,y,z,1) for x in (-size[0],size[0])
                         for y in (-size[1],size[1]) for z in (-size[2],size[2])])
    if cylinder is not None:
        radius=float(cylinder.get("radius") or cylinder.findtext("radius"))
        half=float(cylinder.get("length") or cylinder.findtext("length"))/2
        return np.array([(radius*math.cos(a),radius*math.sin(a),z,1)
                         for z in (-half,half) for a in np.linspace(0,2*math.pi,32,endpoint=False)])
    raise ValueError("Known collision projection requires a box or cylinder primitive")


def swept_link_points(urdf: Path | str) -> dict[str, np.ndarray]:
    """Collision-box vertices of every link over the full prismatic stroke and sampled revolute ranges.

    Returns link name -> (N, 3) points in metres in the base_link frame; raises ValueError if the
    joint tree is not rooted at base_link.
    """
    robot=ET.parse(urdf).getroot()
    poses={"base_link":[np.eye(4)]}
    joints=list(robot.findall("joint"))
    while joints:
        pending=[]
        for joint in joints:
            parent,child=joint.find("parent").get("link"),joint.find("child").get("link")
            if parent not in poses:
                pending.append(joint)
                continue
            origin=joint.find("origin")
            local=transform(" ".join((origin.get("xyz","0 0 0"),origin.get("rpy","0 0 0"))) if origin is not None else None)
            poses[child]=[pose@local for pose in poses[parent]]
            if joint.get("type")=="prismatic":
                axis=np.fromstring(joint.find("axis").get("xyz"),sep=" ")
                limit=joint.find("limit")
                endpoints=[]
                for pose in poses[child]:
                    for value in (float(limit.get("lower")),float(limit.get("upper"))):
                        offset=np.eye(4);offset[:3,3]=axis*value
                        endpoints.append(pose@offset)
                poses[child]=endpoints
            elif joint.get("type")=="revolute":
                # Gimbal motors: sample the whole mechanical range so the swept volume is covered.
                axis=np.fromstring(joint.find("axis").get("xyz"),sep=" ")
                axis=axis/np.linalg.norm(axis)
                limit=joint.find("limit")
                angles=set(np.linspace(float(limit.get("lower")),float(limit.get("upper")),7).tolist())|{0.0}
                cross=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
                sampled=[]
                for angle in sorted(angles):
                    offset=np.eye(4)
                    offset[:3,:3]=np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*cross@cross
                    sampled.extend(pose@offset for pose in poses[child])
                poses[child]=sampled
        if len(pending)==len(joints):
            raise ValueError("Robot collision tree is not rooted at physical base_link")
        joints=pending
    links={}
    for link in robot.findall("link"):
        points=[]
        for collision in link.findall("collision"):
            origin=collision.find("origin")
            local=transform(" ".join((origin.get("xyz","0 0 0"),origin.get("rpy","0 0 0"))) if origin is not None else None)
            vertices=collision_vertices(collision.find("geometry"))
            for pose in poses[link.get("name")]:
                points.extend((pose@local@vertices.T).T[:,:3])
        if points:
            links[link.get("name")]=np.array(points)
    return links


def robot_envelope(urdf: Path | str) -> dict[str, Any]:
    """Rectangular planar footprint in base_link (x forward, y left, metres) covering every swept collision point.

    Half-sizes are at least the bare Jackal body (0.254, 0.215) m, plus a 0.025 m margin; height_m is the
    top of the envelope plus 0.04 m. Raises ValueError if the URDF has no collision primitives.
    """
    links=swept_link_points(urdf)
    bounds=np.concatenate(list(links.values())) if links else np.empty((0,3))
    if not len(bounds):
        raise ValueError("Expanded robot description has no measured collision envelope")
    minimum,maximum=bounds.min(axis=0),bounds.max(axis=0)
    half=np.maximum(np.maximum(-minimum[:2],maximum[:2]),[.254,.215])+.025
    return {"source":"expanded URDF primitive collisions, full prismatic stroke and sampled revolute "
                      "gimbal ranges; bare-body/fender minimum",
            "bounds_m":[minimum.tolist(),maximum.tolist()],"margin_m":.025,
            "footprint":[[-half[0],-half[1]],[half[0],-half[1]],[half[0],half[1]],[-half[0],half[1]]],
            "radius_m":float(np.hypot(*half)),"height_m":float(maximum[2]+.04)}


def create_map(assets: Path | str, output: Path | str, robot_height: float = 2.05,
               envelope: dict[str, Any] | None = None) -> dict[str, Any]:
    """Rasterise SDF collisions, unsupported cells and >12 deg slopes into output/map.pgm, map.yaml, map-audit.json.

    Square grid of RESOLUTION (0.05 m) cells in the greenhouse world frame: map.yaml origin is the lower-left
    corner, image row 0 is the +y edge, 254 = free and 0 = occupied. Colliders more than `robot_height` (m)
    above the highest terrain are skipped. Returns the audit; raises ValueError or OSError.
    """
    assets, output = Path(assets), Path(output)
    info = json.loads((assets / "manifest.json").read_text())
    terrain = json.loads((assets / "terrain.json").read_text())
    heights = np.asarray(terrain["heights"])
    origin = float(terrain["origin"])
    extent = (len(heights)-1)*terrain["resolution"]
    count = math.ceil(extent/RESOLUTION)+3
    map_origin = origin-RESOLUTION
    image = np.full((count,count), 254, dtype=np.uint8)
    image[:2,:]=image[-2:,:]=image[:,:2]=image[:,-2:]=0
    centers=map_origin+(np.arange(count)+.5)*RESOLUTION
    supported=(centers>=origin)&(centers<=origin+extent)
    image[:,~supported]=0
    image[~supported[::-1],:]=0
    def pixel(points):
        uv=np.asarray(points)/RESOLUTION - map_origin/RESOLUTION
        uv[:,1]=count-1-uv[:,1]
        return np.rint(uv).astype(np.int32)
    polygons=[]
    world=ET.parse(assets/"worlds/greenhouse.sdf").find("world")
    models=[(m,transform(m.findtext("pose"))) for m in world.findall("model")]
    for include in world.findall("include"):
        uri=include.findtext("uri")
        if uri=="model://greenhouse_ground":
            continue
        if not uri or not uri.startswith("model://"):
            raise ValueError(f"Unsupported known-world include: {uri}")
        model=ET.parse(assets/"models"/uri.removeprefix("model://")/"model.sdf").find("model")
        models.append((model,transform(include.findtext("pose"))@transform(model.findtext("pose"))))
    for model, model_pose in models:
        for link in model.findall("link"):
            link_pose=model_pose@transform(link.findtext("pose"))
            for collision in link.findall("collision"):
                pose=link_pose@transform(collision.findtext("pose"))
                points=collision_vertices(collision.find("geometry"))
                points=(pose@points.T).T[:,:3]
                if points[:,2].min() > robot_height+float(heights.max()) or points[:,2].max() < float(heights.min())-.1:
                    continue
                polygon=cv2.convexHull(pixel(points[:,:2]))
                cv2.fillConvexPoly(image,polygon,0)
                polygons.append({"model":model.get("name"),"collision":collision.get("name"),
                                 "xy":points[:,:2].tolist()})
    # Cells lacking physical support or exceeding this 2.5D navigation slope limit are blocked.
    gy,gx=np.gradient(heights,terrain["resolution"])
    steep=(np.hypot(gx,gy)>math.tan(math.radians(12))).astype(np.uint8)
    for j,i in np.argwhere(steep):
        x=origin+i*terrain["resolution"]; y=origin+j*terrain["resolution"]
        cv2.circle(image,tuple(pixel([[x,y]])[0]),max(1,math.ceil(terrain["resolution"]/RESOLUTION)),0,-1)
    output.mkdir(parents=True,exist_ok=True)
    if not cv2.imwrite(str(output/"map.pgm"),image):
        raise OSError("Cannot write occupancy map")
    # Server yaml uses the exact supported image frame/origin, no AMCL origin shift.
    metadata={"image":"map.pgm","mode":"trinary","resolution":RESOLUTION,"origin":[map_origin,map_origin,0.0],
              "negate":0,"occupied_thresh":0.65,"free_thresh":0.25}
    (output/"map.yaml").write_text(yaml.safe_dump(metadata,sort_keys=False))
    audit={"frame":"greenhouse","colliders":len(polygons),"source":"actual SDF collisions",
           "footprint":[[-HALF_LENGTH,-HALF_WIDTH],[HALF_LENGTH,-HALF_WIDTH],
                        [HALF_LENGTH,HALF_WIDTH],[-HALF_LENGTH,HALF_WIDTH]],
           "circumscribed_radius_m":RADIUS,"map_origin":map_origin,"resolution":RESOLUTION,
           "excluded_above_height_m":robot_height,"grid_cells":count,"polygons":polygons}
    if envelope:
        audit.update({"robot_envelope":envelope,"footprint":envelope["footprint"],
                      "circumscribed_radius_m":envelope["radius_m"]})
    (output/"map-audit.json").write_text(json.dumps(audit,indent=2)+"\n")
    return audit


def aisle_inflation_radius(clear_aisles: Iterable[float], maximum: float, resolution: float = RESOLUTION) -> float:
    """Largest inflation radius (at most `maximum`) that leaves every aisle centreline cost-free.

    Nav2 inflates cells up to ceil(radius/resolution) away; the centreline sits half the clear
    width from the plants, so stopping one cell short of it keeps all aisles equally cheap.
    All values are in metres.
    """
    cells=math.floor(min(clear_aisles)/2/resolution+1e-6)-1
    return round(min(maximum,cells*resolution),6)  # 0.6, not 0.6000000000000001 (13 cells)


def prepare_navigation(assets: Path | str, directory: Path | str, namespace: str, actuator: dict[str, Any]) -> str:
    """Write the occupancy map and the namespaced Nav2 navigation.yaml into `directory`; return "<ns>/navigation".

    The footprint comes from directory/robot.expanded.urdf plus RPP swept padding (metres, base_link frame).
    `actuator` (the actuator manifest) is accepted but unused. Raises ValueError if aisles are too narrow.
    """
    directory=Path(directory)
    envelope=robot_envelope(directory/"robot.expanded.urdf")
    ns=(namespace.strip("/")+"/" if namespace.strip("/") else "")+"navigation"
    frame=(namespace.strip("/")+"/" if namespace.strip("/") else "")
    text=(Path(__file__).resolve().parents[1]/"config/navigation.yaml").read_text()
    for key,value in {"@MAP@":str(directory/"nav-map/map.yaml"),"@NAV_BASE@":frame+"nav_base",
                      "@NAV_ODOM@":frame+"nav_odom","@MAP_TOPIC@":"/"+ns+"/map"}.items():
        text=text.replace(key,value)
    data=yaml.safe_load(text)
    follow=data["controller_server"]["ros__parameters"]["FollowPath"]
    resolution=data["local_costmap/local_costmap"]["ros__parameters"]["resolution"]
    # RPP's collision projection advances one costmap cell per check (in-place rotation: the
    # circumscribed radius sweeps one cell). Along a pure-pursuit arc, curvature is at most
    # 2*sin(rotate_to_heading_min_angle)/min_lookahead_dist before it rotates in place instead,
    # so a footprint corner moves at most resolution*(1+radius*curvature) between checks.
    curvature=2*math.sin(follow["rotate_to_heading_min_angle"])/follow["min_lookahead_dist"]
    padding=math.ceil(resolution*(1+envelope["radius_m"]*curvature)*.5*1000)/1000
    footprint=envelope["footprint"]
    expanded=[[x+math.copysign(padding,x),y+math.copysign(padding,y)] for x,y in footprint]
    envelope=dict(envelope,physical_footprint=footprint,physical_radius_m=envelope["radius_m"],
                  swept_padding_m=padding,footprint=expanded,
                  radius_m=max(math.hypot(x,y) for x,y in expanded))
    create_map(assets,directory/"nav-map",envelope["height_m"],envelope)
    data["global_costmap/global_costmap"]["ros__parameters"]["robot_radius"]=envelope["radius_m"]
    inflation=data["global_costmap/global_costmap"]["ros__parameters"]["inflation_layer"]
    aisles=json.loads((Path(assets)/"manifest.json").read_text())["layout"]["clear_aisles_m"]
    inflation["inflation_radius"]=aisle_inflation_radius(aisles,inflation["inflation_radius"])
    if inflation["inflation_radius"]<=envelope["radius_m"]:
        raise ValueError("aisles are too narrow to centre the padded robot footprint")
    data["local_costmap/local_costmap"]["ros__parameters"]["footprint"]=json.dumps(footprint)
    data["local_costmap/local_costmap"]["ros__parameters"]["footprint_padding"]=padding
    scoped={"/"+ns+"/"+key:value for key,value in data.items()}
    (directory/"navigation.yaml").write_text(yaml.safe_dump(scoped,sort_keys=False))
    return ns


def main() -> None:
    """CLI: prepare navigation files for --assets into --setup/greenhouse."""
    parser=argparse.ArgumentParser()
    parser.add_argument("--assets",required=True)
    parser.add_argument("--setup",required=True)
    args=parser.parse_args()
    directory=Path(args.setup)/"greenhouse"
    actuator=json.loads((directory/"actuator.json").read_text())
    prepare_navigation(args.assets,directory,actuator["namespace"],actuator)


if __name__=="__main__":
    main()
