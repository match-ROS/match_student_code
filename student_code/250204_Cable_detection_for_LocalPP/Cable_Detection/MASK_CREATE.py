import json
import cv2
import numpy as np

# Path to LabelMe JSON file
json_path = r"C:\Users\uie65011\OneDrive - Continental AG\Studienarbeit\realsense_camera\data_capture_cable_1m\color_001.json"
image_width = 1920   # Set to your actual image width
image_height = 1080  # Set to your actual image height

# Load JSON file
with open(json_path, "r") as f:
    data = json.load(f)

# Create an empty mask
mask = np.zeros((image_height, image_width), dtype=np.uint8)

# Process annotations
for shape in data["shapes"]:
    if shape["label"] == "cable":  # Only extract cables
        polygon = np.array(shape["points"], dtype=np.int32)
        cv2.fillPoly(mask, [polygon], 255)  # Fill the cable area with white (255)

# Save the generated mask
output_mask_path = "generated_mask.png"
cv2.imwrite(output_mask_path, mask)
print(f" Saved generated mask to {output_mask_path}")
