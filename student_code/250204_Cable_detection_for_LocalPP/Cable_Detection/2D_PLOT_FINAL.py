import os
import pandas as pd
import matplotlib.pyplot as plt

# Define paths
ground_truth_path = r"C:\Users\uie65011\OneDrive - Continental AG\Studienarbeit\realsense_camera\labeled_filtered_cable_positions_base_frame_5cm.csv"
predicted_path = r"C:\Users\uie65011\OneDrive - Continental AG\Studienarbeit\realsense_camera\filtered_cable_positions_base_frame_5cm.csv"

# Check if files exist before proceeding
if not os.path.exists(ground_truth_path):
    raise FileNotFoundError(f" Error: Ground truth file not found at {ground_truth_path}")

if not os.path.exists(predicted_path):
    raise FileNotFoundError(f" Error: Predicted mask file not found at {predicted_path}")

# Load CSV files
df_ground_truth = pd.read_csv(ground_truth_path)
df_predicted = pd.read_csv(predicted_path)

# Extract X and Y coordinates for both datasets (Swapping X and Y)
ground_truth_x = df_ground_truth["Base_Y"]  # Swapped
ground_truth_y = df_ground_truth["Base_X"]  # Swapped
predicted_x = df_predicted["Base_Y"]  # Swapped
predicted_y = df_predicted["Base_X"]  # Swapped


# Error threshold (2.5 cm = 0.025 m)
radius = 0.025

# Create plot
fig, ax = plt.subplots(figsize=(6,6))
ax.scatter(ground_truth_x, ground_truth_y, color='red', label='Ground Truth', marker='o', alpha=0.6)
ax.scatter(predicted_x, predicted_y, color='blue', label='Predicted Mask', marker='x', alpha=0.6)

# Draw tolerance circles around predicted mask positions
for x, y in zip(ground_truth_x, ground_truth_y):
    circle = plt.Circle((x, y), radius, color='gray', fill=False, linestyle='dashed', alpha=0.5)
    ax.add_patch(circle)

# Labels and legend (Updated after swapping X and Y)
ax.set_xlabel("Y Position (m)")
ax.set_ylabel("X Position (m)")
ax.set_title("Predicted vs. Ground Truth Positions with Tolerance Radius")

# Reverse Y-axis (so higher values appear at the top)
ax.invert_xaxis()

ax.legend()
ax.set_aspect('equal', adjustable='datalim')
plt.grid()

# Show plot
plt.show()
