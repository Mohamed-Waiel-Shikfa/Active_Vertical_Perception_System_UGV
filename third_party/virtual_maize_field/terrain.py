# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2021 Farm Technology Group of Wageningen University & Research
# and Kamaro Engineering e.V.
#
# Extracted from Field2DGenerator at revision
# e525d70c3c73fcdfae9c77bb8e4ca03bf3377ac3.
# generate_ground is unchanged. Only imports and the class constructor were
# narrowed to avoid importing ROS, plotting, row-generation and model plugins.
# The exact original file and license are in upstream/. Tests compare the AST.

import cv2
import numpy as np


class Field2DGenerator:
    def __init__(self, world_description):
        self.wd = world_description

    def generate_ground(self) -> None:
        ditch_depth = self.wd.structure["params"]["ground_ditch_depth"]
        ditch_distance = self.wd.structure["params"]["ground_headland"]

        self.crop_placements = np.vstack(self.rows)

        if self.object_placements.ndim == 2:
            self.placements = np.concatenate(
                (self.crop_placements, self.object_placements), axis=0
            )
        else:
            self.placements = self.crop_placements
        # Calculate image resolution
        metric_x_min = self.placements[:, 0].min()
        metric_x_max = self.placements[:, 0].max()
        metric_y_min = self.placements[:, 1].min()
        metric_y_max = self.placements[:, 1].max()

        metric_width = metric_x_max - metric_x_min + 2 * ditch_distance + 1
        metric_height = metric_y_max - metric_y_min + 2 * ditch_distance + 1

        min_resolution = self.wd.structure["params"][
            "ground_resolution"
        ]  # min resolution
        min_image_size = int(
            np.ceil(max(metric_width / min_resolution, metric_height / min_resolution))
        )
        # gazebo expects heightmap in format 2**n
        image_size = int(2 ** np.ceil(np.log2(min_image_size)))

        self.resolution = min_resolution * (min_image_size / image_size)

        # Generate noise
        heightmap = np.zeros((image_size, image_size))

        n = 0
        while 2**n < image_size:
            heightmap += (
                cv2.resize(
                    self.wd.rng.random((image_size // 2**n, image_size // 2**n)),
                    (image_size, image_size),
                )
                * (n + 1) ** 2
            )
            n += 1

        # Normalize heightmap
        heightmap -= heightmap.min()
        heightmap /= heightmap.max()

        max_elevation = self.wd.structure["params"]["ground_elevation_max"]

        self.heightmap_elevation = ditch_depth + (max_elevation / 2)

        heightmap *= (max_elevation) / self.heightmap_elevation
        field_height = (ditch_depth - (max_elevation / 2)) / self.heightmap_elevation

        field_mask = np.zeros((image_size, image_size))

        offset = image_size // 2

        def metric_to_pixel(pos: int) -> int:
            return int(pos // self.resolution) + offset

        # Make plant placements flat and save the heights for the sdf renderer
        PLANT_FOOTPRINT = (2 * 0.02**2) ** 0.5
        flatspot_radius = int((PLANT_FOOTPRINT / 2) // self.resolution) + 2

        self.placements_ground_height = []
        for mx, my in self.placements:
            px = metric_to_pixel(mx)
            py = metric_to_pixel(my)

            height = heightmap[py, px]
            heightmap = cv2.circle(heightmap, (px, py), flatspot_radius, height, -1)
            self.placements_ground_height.append(
                (field_height + height) * self.heightmap_elevation
            )

        # create ditch around the crop field
        for mx, my in self.crop_placements:
            px = metric_to_pixel(mx)
            py = metric_to_pixel(my)

            field_mask = cv2.circle(
                field_mask, (px, py), int((ditch_distance) / self.resolution), 1, -1
            )

        blur_size = (int(0.2 / self.resolution) // 2) * 2 + 1
        field_mask = cv2.GaussianBlur(field_mask, (blur_size, blur_size), 0)

        heightmap += field_height * field_mask

        assert heightmap.max() <= 1
        assert heightmap.min() >= 0

        # Convert to grayscale
        self.heightmap = (255 * heightmap[::-1, :]).astype(np.uint8)

        self.metric_size = image_size * self.resolution
        # Calc heightmap position. Currently unused, overwritten in @ref fix_gazebo
        self.heightmap_position = [
            metric_x_min - 2 + 0.5 * self.metric_size,
            metric_y_min - 2 + 0.5 * self.metric_size,
        ]
