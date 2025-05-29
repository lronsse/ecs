import numpy as np

def polar_to_cartesian(p):
    return np.array([p[1] * np.cos(p[0]), p[1] * np.sin(p[0])])

def apply_radial_mask(polar_points, min_radius, max_radius):
    r = polar_points[:, 1]
    mask = (r >= min_radius) & (r <= max_radius)
    return polar_points[mask]

def calculate_angle_between_polar_vectors(p1, p2, p3):
    c1 = polar_to_cartesian(p1)
    c2 = polar_to_cartesian(p2)
    c3 = polar_to_cartesian(p3)

    vec1 = c2 - c1
    vec2 = c3 - c2

    vec1_norm = vec1 / np.linalg.norm(vec1)
    vec2_norm = vec2 / np.linalg.norm(vec2)

    cos_theta = np.dot(vec1_norm, vec2_norm)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    angle = np.arccos(cos_theta)

    return angle

def detect_corners_polar(polar_points, angle_lower, angle_higher, span, distance_threshold):
    corner_points = []

    for i in range(span, len(polar_points) - span):
        p1 = polar_points[i - span]
        p2 = polar_points[i]
        p3 = polar_points[i + span]

        dist1 = np.linalg.norm(polar_to_cartesian(p1) - polar_to_cartesian(p2))
        dist2 = np.linalg.norm(polar_to_cartesian(p3) - polar_to_cartesian(p2))

        if dist1 > distance_threshold or dist2 > distance_threshold:
            continue

        angle = calculate_angle_between_polar_vectors(p1, p2, p3)
        if angle_lower < angle < angle_higher:
            if prev_corner:
                corner_points.append(p2)
            prev_corner = True
        else:
            prev_corner = False

    return np.array(corner_points)

def reduce_corner_blobs_by_angle(corners, angle_target=np.deg2rad(90), angle_threshold=np.deg2rad(0.5)):
    if len(corners) == 0:
        return corners

    sorted_indices = np.argsort(corners[:, 0])
    sorted_corners = corners[sorted_indices]

    groups = []
    current_group = [sorted_corners[0]]

    for i in range(1, len(sorted_corners)):
        if abs(sorted_corners[i][0] - current_group[-1][0]) <= angle_threshold:
            current_group.append(sorted_corners[i])
        else:
            groups.append(current_group)
            current_group = [sorted_corners[i]]
    groups.append(current_group)

    reduced_corners = []
    for group in groups:
        group = np.array(group)
        best_index = np.argmin(np.abs(group[:, 0] - angle_target))
        reduced_corners.append(group[best_index])

    return np.array(reduced_corners)

def remove_close_corners_by_distance(corners, min_distance_mm=15):
    if len(corners) <= 1:
        return corners

    cart = np.array([[r * np.cos(theta), r * np.sin(theta)] for theta, r in corners])
    keep = np.ones(len(corners), dtype=bool)

    for i in range(len(cart)):
        if not keep[i]:
            continue
        for j in range(i + 1, len(cart)):
            if not keep[j]:
                continue
            dist = np.linalg.norm(cart[i] - cart[j])
            if dist < min_distance_mm:
                avg_polar = np.mean([corners[i], corners[j]], axis=0)
                corners[i] = avg_polar
                keep[j] = False

    return corners[keep]
