#!/usr/bin/env python3
import sys
import os
import math
import yaml
import numpy as np
from PIL import Image, ImageDraw
import xml.etree.ElementTree as ET


def parse_pose(pose_str):
    parts = [float(x) for x in pose_str.split()]
    return parts  # x, y, z, r, p, y


def parse_size(size_str):
    parts = [float(x) for x in size_str.split()]
    return parts  # x, y, z


def create_pgm_from_sdf(sdf_path, output_name, resolution=0.05, map_size_x=25.0, map_size_y=25.0):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    world = root.find('world')

    # Map parameters
    width_px = int(map_size_x / resolution)
    height_px = int(map_size_y / resolution)
    origin_x = -map_size_x / 2.0
    origin_y = -map_size_y / 2.0

    # Create white image (free space) - 255 is free, 0 is occupied
    # In PGM: 255 is white (free), 0 is black (occupied).
    # However, standard ROS map interpretation:
    # 0 (black) -> Occupied
    # 254 (white) -> Free
    # 205 (gray) -> Unknown

    # We will initialize with 205 (unknown) or 254 (free)?
    # Usually a generated map assumes free space for the ground plane.
    # The ground plane in the SDF is 20x20.
    # Let's initialize with 255 (Free) because it's a warehouse with a floor.

    img = Image.new('L', (width_px, height_px), 255)
    draw = ImageDraw.Draw(img)

    # Check for models
    for model in world.findall('model'):
        model_name = model.get('name')

        # Skip ground plane
        if 'ground' in model_name or 'plane' in model_name:
            continue

        pose_elem = model.find('pose')
        if pose_elem is None:
            continue

        mx, my, mz, mr, mp, myaw = parse_pose(pose_elem.text)

        # Check links for collisions
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                if geometry is not None:
                    box = geometry.find('box')
                    if box is not None:
                        size_elem = box.find('size')
                        sx, sy, sz = parse_size(size_elem.text)

                        # Calculate corners of the box in world coordinates
                        # We assume the box is centered at model pose (link pose is ignored usually or 0)
                        # We need to rotate the 4 corners of the footprint

                        # Half dimensions
                        dx = sx / 2.0
                        dy = sy / 2.0

                        corners = [
                            (dx, dy),
                            (dx, -dy),
                            (-dx, -dy),
                            (-dx, dy)
                        ]

                        rotated_corners = []
                        cos_yaw = math.cos(myaw)
                        sin_yaw = math.sin(myaw)

                        for cx, cy in corners:
                            rx = cx * cos_yaw - cy * sin_yaw + mx
                            ry = cx * sin_yaw + cy * cos_yaw + my

                            # Convert to pixel coordinates
                            # Pixel (0,0) is top-left in PIL?
                            # Map origin is bottom-left usually in ROS.
                            # But PIL image: (0,0) is top-left.
                            # We need to transform world (x,y) to pixel (u,v)
                            # u = (x - origin_x) / res
                            # v = height - (y - origin_y) / res  <-- Flip Y for image

                            px = (rx - origin_x) / resolution
                            py = height_px - (ry - origin_y) / resolution
                            rotated_corners.append((px, py))

                        # Draw polygon (black = 0)
                        draw.polygon(rotated_corners, fill=0)

    # Save Image
    pgm_filename = output_name + ".pgm"
    img.save(pgm_filename)
    print(f"Generated {pgm_filename}")

    # Save YAML
    yaml_filename = output_name + ".yaml"
    yaml_data = {
        'image': os.path.basename(pgm_filename),
        'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }

    with open(yaml_filename, 'w') as f:
        yaml.dump(yaml_data, f)
    print(f"Generated {yaml_filename}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 sdf_to_pgm.py <sdf_file> [output_name_prefix]")
        sys.exit(1)

    sdf_file = sys.argv[1]
    output_prefix = sys.argv[2] if len(sys.argv) > 2 else "map"

    create_pgm_from_sdf(sdf_file, output_prefix)
