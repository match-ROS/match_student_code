import os
import csv
import pyrealsense2 as rs
import numpy as np
import cv2

def capture_data(
    output_dir="data_capture_ArUcoMarker_cable_1m",
    num_images=50,
    resolution_depth=(1280, 720),  # Updated resolution for depth
    resolution_rgb=(1920, 1080),  # Updated resolution for color
    fps=30  # Frames per second for both streams
):
    """
    Captures RGB and aligned depth images from the RealSense camera at the specified resolution and frame rate.

    Parameters:
    -----------
    output_dir : str
        Directory to save the captured images and metadata.
    num_images : int
        Number of frames to capture (color + depth).
    resolution_depth : tuple
        (width, height) for the depth stream.
    resolution_rgb : tuple
        (width, height) for the RGB stream.
    fps : int
        Frames per second for both color and depth streams.
    """
    os.makedirs(output_dir, exist_ok=True)

    # Prepare CSV for metadata
    csv_path = os.path.join(output_dir, "metadata.csv")
    with open(csv_path, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["frame_index", "color_resolution", "depth_resolution", "depth_scale", "valid_depth_check"])

        pipeline = rs.pipeline()
        config = rs.config()

        # Enable depth and color streams with the specified resolution and FPS
        config.enable_stream(rs.stream.depth, resolution_depth[0], resolution_depth[1], rs.format.z16, fps)
        config.enable_stream(rs.stream.color, resolution_rgb[0], resolution_rgb[1], rs.format.bgr8, fps)

        profile = pipeline.start(config)

        # Align depth to color
        align = rs.align(rs.stream.color)

        # Get depth scale
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print(f"Depth scale: {depth_scale} meters/unit")

        try:
            for i in range(num_images):
                # Wait for frames and align depth to color
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)

                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    print(f"[WARNING] Frame {i} is invalid. Retrying...")
                    continue

                # Convert frames to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())

                # Debug: Visualize depth as 8-bit for inspection
                depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)

                # Save color and depth images
                color_filename = os.path.join(output_dir, f"color_{i:03d}.png")
                depth_filename = os.path.join(output_dir, f"depth_{i:03d}.png")
                debug_filename = os.path.join(output_dir, f"depth_debug_{i:03d}.png")

                cv2.imwrite(color_filename, color_image)
                cv2.imwrite(depth_filename, depth_image)  # Save raw depth
                cv2.imwrite(debug_filename, depth_image_8bit)  # Save normalized depth for visualization

                # Validate depth values (ensure at least some non-zero values)
                valid_depth = np.count_nonzero(depth_image)
                if valid_depth == 0:
                    print(f"[WARNING] No valid depth values found in Frame {i}.")

                # Log metadata
                csv_writer.writerow([
                    i,
                    color_image.shape,
                    depth_image.shape,
                    depth_scale,
                    valid_depth > 0  # True if valid depth exists
                ])

                print(f"[INFO] Saved {color_filename}, {depth_filename}, {debug_filename} | Valid Depth: {valid_depth > 0}")

                # Optionally display the images during capture
                cv2.imshow("Color Image", color_image)
                cv2.imshow("Depth (Debug)", depth_image_8bit)
                if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit early
                    break

        finally:
            pipeline.stop()
            print("[INFO] Data capture complete. Pipeline stopped.")
            cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_data(
        output_dir="data_capture_ArUcoMarker_cable_1m",
        num_images=50,
        resolution_depth=(1280, 720),  # Updated depth resolution
        resolution_rgb=(1920, 1080),  # Updated RGB resolution
        fps=30  # FPS
    )

