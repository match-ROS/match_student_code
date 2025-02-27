import cv2
import numpy as np
import csv
import time

# --------------------------------------------------------------------
# 1) Transformation Functions
# --------------------------------------------------------------------
def build_base_to_camera_transform():
    """Returns the transformation matrix from camera frame to base frame."""
    tx, ty, tz = -0.04633616, 0.03997007, 0.69179826
    R = np.asarray([
        [0.00359873,  -0.7084172,   0.70578475],
        [-0.99988093,  0.00804181,  0.01317009],
        [-0.0150057,  -0.70574811, -0.70830391]
    ])
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3]  = [tx, ty, tz]
    return T

def transform_point_cam_to_base(T_cam_to_base, cam_point_3):
    """Transforms a 3D point from camera coordinates to base frame coordinates."""
    pt_hom = np.append(cam_point_3, 1.0)  # Convert to homogeneous coordinates [x, y, z, 1]
    base_pt_hom = T_cam_to_base @ pt_hom
    return base_pt_hom[:3]

#############################
# Filtering Functions
#############################
def sample_points_by_distance(points_3d, min_distance=0.05):
    """
    Filters 3D points by keeping only points that are at least `min_distance` apart.
    """
    if not points_3d:
        return []
    
    sampled = [points_3d[0]]
    last = points_3d[0]
    
    for i in range(1, len(points_3d)):
        dist = np.linalg.norm(np.array(points_3d[i]) - np.array(last))
        if dist >= min_distance:
            sampled.append(points_3d[i])
            last = points_3d[i]
    
    return sampled

def main():
    # Original Camera Intrinsics (for 1920x1080)
    original_width = 1920
    original_height = 1080
    fx = 1385.3656
    fy = 1386.4231
    cx = 959.9992
    cy = 535.1730

    # Resize parameters (for alignment)
    target_width = 960
    target_height = 540  # Maintain 16:9 aspect ratio

    # Adjust intrinsics for resized image
    scale_x = target_width / original_width
    scale_y = target_height / original_height
    fx *= scale_x
    fy *= scale_y
    cx *= scale_x
    cy *= scale_y

    # Load aligned segmentation mask + depth images
    segmentation_mask_path = r"C:\Users\uie65011\OneDrive - Continental AG\Studienarbeit\realsense_camera\generated_mask.png"
    depth_img_path = r"C:\Users\uie65011\OneDrive - Continental AG\Studienarbeit\realsense_camera\data_capture_cable_1m\depth_001.png"

    # Load images
    mask_image = cv2.imread(segmentation_mask_path, cv2.IMREAD_GRAYSCALE)  # Load mask in grayscale
    depth_image = cv2.imread(depth_img_path, cv2.IMREAD_ANYDEPTH)  # Load depth in raw format

    if mask_image is None or depth_image is None:
        print("Error: Could not load segmentation mask or depth image.")
        return

    # Resize both images to the same resolution as YOLO input
    mask_image = cv2.resize(mask_image, (target_width, target_height), interpolation=cv2.INTER_NEAREST)
    depth_image = cv2.resize(depth_image, (target_width, target_height), interpolation=cv2.INTER_NEAREST)

    print(f"Sample raw depth values: {depth_image[100, 100]}, {depth_image[200, 200]}")
    print(f"Converted meters: {depth_image[100, 100] * 0.001}, {depth_image[200, 200] * 0.001}")

    ####**Start measuring detection + pose estimation time** ####
    detection_start = time.time()

    # 1) Convert Mask to Binary (Thresholding)
    _, binary_mask = cv2.threshold(mask_image, 127, 255, cv2.THRESH_BINARY)

    # 2) Get all pixel coordinates in the mask
    pixels = np.argwhere(binary_mask > 0)  # Get all white (cable) pixels

    # 3) Convert selected pixels to camera coordinates
    camera_points = []
    for (row, col) in pixels:
        depth_raw = depth_image[row, col]
        if depth_raw == 0:
            continue  # Ignore invalid depth points

        depth_m = depth_raw * 0.001  # Convert mm to meters
        X_cam = (col - cx) * depth_m / fx
        Y_cam = (row - cy) * depth_m / fy
        Z_cam = depth_m

        camera_points.append([X_cam, Y_cam, Z_cam])

    # 4) Apply **distance-based filtering** (remove close points)
    filtered_camera_points = sample_points_by_distance(camera_points, min_distance=0.05)

    # 5) Transform to base frame
    T_cam_to_base = build_base_to_camera_transform()
    base_points = [transform_point_cam_to_base(T_cam_to_base, pt) for pt in filtered_camera_points]

    #### **Stop timing after all points are transformed to base frame** ####
    detection_end = time.time()
    detection_time = detection_end - detection_start

    # 6) Save segmentation mask (for visualization)
    segmented_output_path = "segmentation_mask_output.png"
    cv2.imwrite(segmented_output_path, binary_mask)
    print(f"Saved segmented mask to {segmented_output_path}.")

    # 7) Save to CSV
    output_csv = "labeled_filtered_cable_positions_base_frame_5cm.csv"
    with open(output_csv, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Index", "Base_X", "Base_Y", "Base_Z"])
        for i, (bx, by, bz) in enumerate(base_points):
            writer.writerow([i, bx, by, bz])

    print(f"\n Detection + Pose Estimation Time: {detection_time:.3f} seconds")
    print(f" Saved {len(base_points)} filtered points to {output_csv}.")

if __name__ == "__main__":
    main()
