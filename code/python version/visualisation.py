# visualize_data.py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import json
from scipy.spatial.distance import cdist
from vector_based_corner_detection import (
    detect_corners_polar,
    apply_radial_mask,
    reduce_corner_blobs_by_angle,
    remove_close_corners_by_distance,
)

# Load configuration
with open('config.json', 'r') as f:
    config = json.load(f)

min_radius = config["min_radius"]
max_radius = config["max_radius"]
angle_range = config["angle_range_degrees"]
span = config["span"]
match_threshold = config["match_threshold_mm"]
angle_target_rad = np.deg2rad(config["angle_target_degrees"])
angle_grouping_threshold_rad = np.deg2rad(config["angle_grouping_threshold_degrees"])
min_corner_distance_mm = config["min_corner_distance_mm"]
distance_threshold = span*30


# Load data
file_path = 'LidarData.csv'
data = pd.read_csv(file_path, header=None, skiprows=3)

angles_deg = np.linspace(0, 240, data.shape[0])
angles_rad = np.deg2rad(angles_deg)

plt.ion()
fig, ax = plt.subplots()

memory_length = config.get("memory_length", 3)
history = []  # store last N frames of corners and ids
next_id = 1

for col in data.columns[1:]:
    distances = pd.to_numeric(data[col], errors='coerce').fillna(0).values
    polar_points_real = np.column_stack((angles_rad, distances))
    polar_points_real = apply_radial_mask(polar_points_real, min_radius, max_radius)

    polar_points = apply_radial_mask(polar_points_real, 0, max_radius)
    angle_lower = np.deg2rad(90 - angle_range)
    angle_higher = np.deg2rad(90 + angle_range)

    corners = detect_corners_polar(polar_points_real, angle_lower, angle_higher, span, distance_threshold)
    if corners.any():
        corners = apply_radial_mask(corners, min_radius, max_radius)
        print('corners before reduction: ', len(corners))
        corners = reduce_corner_blobs_by_angle(corners, angle_target=angle_target_rad, angle_threshold=angle_grouping_threshold_rad)
        print('after reduction: ', len(corners))
        corners = remove_close_corners_by_distance(corners, min_distance_mm=min_corner_distance_mm)

    if len(corners) > 0:
        cx = corners[:, 1] * np.cos(corners[:, 0])
        cy = corners[:, 1] * np.sin(corners[:, 0])
        curr_corners_cartesian = np.column_stack((cx, cy))
    else:
        curr_corners_cartesian = np.empty((0, 2))

    curr_corner_ids = [-1] * len(curr_corners_cartesian)
    if len(curr_corners_cartesian) > 0:
        matched = set()
        for h_corners, h_ids in reversed(history):
            if len(h_corners) == 0:
                continue
            dists = cdist(curr_corners_cartesian, h_corners)
            for i, row in enumerate(dists):
                if curr_corner_ids[i] != -1:
                    continue
                min_idx = np.argmin(row)
                if row[min_idx] < match_threshold and h_ids[min_idx] not in matched:
                    curr_corner_ids[i] = h_ids[min_idx]
                    matched.add(h_ids[min_idx])

    for i in range(len(curr_corner_ids)):
        if curr_corner_ids[i] == -1:
            curr_corner_ids[i] = next_id
            next_id += 1

    ax.clear()
    x = polar_points[:, 1] * np.cos(polar_points[:, 0])
    y = polar_points[:, 1] * np.sin(polar_points[:, 0])
    ax.scatter(x, y, color='blue', s=2, label='Scan points')

    if len(curr_corners_cartesian) > 0:
        ax.scatter(curr_corners_cartesian[:, 0], curr_corners_cartesian[:, 1], color='red', s=20, label='Detected corners')
        for (pt, fid) in zip(curr_corners_cartesian, curr_corner_ids):
            ax.text(pt[0] + 10, pt[1], str(fid), fontsize=8, color='black')

    ax.set_title(f'Scan {col}')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.axis('equal')
    ax.grid(True)
    ax.legend()

    plt.draw()
    print("Press any key or click to show the next scan...")
    plt.waitforbuttonpress()

    # Update history
    history.append((curr_corners_cartesian, curr_corner_ids))
    if len(history) > memory_length:
        history.pop(0)

plt.ioff()
plt.close()

