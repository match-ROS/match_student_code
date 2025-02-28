## Overview
This project focuses on detecting cables and estimating their 2D poses using an **Intel RealSense D435** depth camera. The method leverages YOLOv11-nSeg for segmentation and transforms detected cable positions into the robot’s base frame. The output is formatted for potential costmap integration in mobile robots.

We already have:

 - A dataset with labeled images used for training the model.
 - A testing dataset for evaluating pose estimation.
 - All trained YOLO models are saved in the YOLO_Background folder on the server.

However, if additional images are required or real-time streaming is necessary, this repository provides the necessary scripts.

**Author:** Qendresa Berisha

**E-Mail:** qendresa.berisha@stud.uni-hannover.de

## Scripts and Usage
### 1. Capturing images: ```capture_images.py ```
This script is used for capturing **RGB and depth images** from the **Intel RealSense D435** camera.

 - It ensures alignment between **RGB and depth images**.

 - If the dataset that is already collected is sufficient, this script is not required.

 - If new images need to be captured or streamed live, use this script as a reference on how to capture images so that they are aligned properly.


### 2. Creating Ground Truth Cable Positions
To create a ground truth from an image, follow these steps:

#### Step 1: Label the Image (Using LabelME) 
 - Install LabelME using:  ```pip install labelme ```
 - Open LabelME by running ```labelme``` in the terminal.
 - Click **"Open"** and select the image you want to annotate and use **"Create Polygons"** to draw arounf the cable and enter the label name **"cable"**.
 - Click **"Save"** and the annotation will be saved as a **JSON file** in the same directory as the image.

#### Step 2: Generate Segmented Mask (```MASK_CREATE.py ```)
  - Use the labeled image in ```MASK_CREATE.py ```.
  - This script generates a segmented mask from the labeled data.

#### Step 3: Extract Ground Truth Positions (```Labeled_Image.py ```)
  - This script takes the generated mask and extracts all cable positions.
  - It saves all labeled positions in: ```labeled_filtered_cable_positions_base_frame_5cm.csv```

### 3. Running the Model for Cable Detection and Pose Estimation (Main Algorithm)
#### Step 4: Pose Estimation with YOLO (```MODEL_POSE.py ```) 
  - This script loads the trained YOLOv11-nSeg model (model size: nano, image size: 960) with the path ```YOLO_Background\runs\segment\train5\weights\best.pt```.
  - It resizes the image to 960x540 for detection.
  - It detects cables, generates a segmented mask, and estimates their 2D pose in the robot’s base frame.
  - Outputs are saved in: ```filtered_cable_positions_base_frame_5cm.csv```

Important: If the camera position is changed, the transformation matrix needs to be recalculated and updated in the code.

### 4. Visualizing and Comparing Results
#### Step 5: Compare Ground Truth and Model Pose (```2D_FINAL_PLOT.py ```) 
  - Loads the two CSV files:
         ```labeled_filtered_cable_positions_base_frame_5cm.csv``` (***Ground truth***) and ```filtered_cable_positions_base_frame_5cm.csv``` (***Estimated poses from YOLO***)
- Generates a graph comparing actual vs. detected cable positions

## Live Streaming Consideration
Currently, the scripts work only with pre-saved images. To run real-time detection:
  - Modify the pipeline to stream images directly from the camera.
  - Capture and align RGB and depth images.
  - Process the images through the pipeline as done with pre-saved images.

## Folders saved in the server
### YOLO_Background
All trained YOLO models are saved in the YOLO_Background folder on the server, the train5 is then picked to be deployed.
### DATASET
All datasets are saved on the server in the following folders:
  - **Dataset_with_BG**: Includes all images used for training the model together with their labels.
  - **data_capture_ArUco_marker**: Used for validating the transformation matrix in the pose estimation.
  - **data_capture_ArUco_marker_cable_1m**: Used for testing three poses of the cable relative to ArUco markers to check accuracy.
  - **data_capture_cable_1m**: Contains the images used for the last overall system evaluation. The label for the image color_001.png is also available and ready to be used to replicate the test.

### Pose_Validation
This folder contains two scripts designed to replicate tests for pose estimation validation. These scripts have not been uploaded here, as they were used as intermediate tests to isolate errors. 

The tests include:
  - **Transformation matrix validation**: Ensures the accuracy of transformations in pose estimation. ```ArUco_marker.py```
  - **Cable position validation**: Tests three different poses of the cable relative to ArUco markers to check detection accuracy. ```CABLE_POSE_ARUCA.py```
### Graph_Creation
This folder contains all scripts used to create the graphs included in the thesis.
