from self_driving_api import pose, apply_terrain

terrains = [
    {"sample_spacing": 0.2, "base_height_range": 0.4, "ground_padding": 2.5, "bump_band": 2, "bump_scale": 0.2, "ground_friction": 0.9, "slip_long": 0.08,  "slip_lat": 1.1},
    {"sample_spacing": 0.3}
]

poses1 = [
    {"x": 0, "y": 0, "rz": 0, "lift": 0, "p1": 0, "y1": 0, "r1": 0, "p2": 0, "y2": 0, "r2": 0, "cam1_rot_speed": 5, "cam2_rot_speed": 5, "lift_speed": 0.2, "drive_speed": 2, "turn_speed": 4},
    {"x": 5, "y": 2, "rz": -180},
    {"x": -5, "rz": 0, "lift": 500},
    {"y": -2, "p1": 45, "r1": 45},
    {"x": 0, "y": 0, "rz": 90},
    {"lift": 0,"p2": -45, "r2": -30},
]

poses2 = [
    {"x": -6.5, "y": 0, "oriented": False},
    {"x": -6.5, "y": 4, "oriented": False},
    {"x": 6.5, "y": 4, "oriented": False},
    {"x": 6.5, "y": 2, "oriented": False},
    {"x": -6.5, "y": 2, "oriented": False},
    {"x": -6.5, "y": 0, "oriented": False},
    {"x": 6.5, "y": 0, "oriented": False},
    {"x": 6.5, "y": -2, "oriented": False},
    {"x": -6.5, "y": -2, "oriented": False},
    {"x": -6.5, "y": -4, "oriented": False},
    {"x": 6.5, "y": -4, "oriented": False},
]

# apply_terrain(terrains[0])

for p in poses2:
    pose(p)

# apply_terrain(terrains[1])

# for p in poses2:
#     pose(p)
