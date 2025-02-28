# Cable Detection and Pose Estimation
## Overview

This folder contains all code for cable detection and pose estimation using an **Intel RealSense D435** depth camera. The method uses **YOLOv11-Seg** for segmentation to detect cables in the scene and estimate their **2D poses**. The final output is formatted for potential integration into a **costmap for obstacle avoidance** in mobile robots.

The project includes:
- **Capture scripts** to collect real-world RGB and depth images.
- **Segmentation and Pose estimation pipeline** using YOLOv11-Seg to detect cables and transforming the positions of the detected cables into the robot’s base frame.
- **Visualization tools** for evaluating the pose estimation quality.

**Author:** Qendresa Berisha

**E-Mail:** qendresa.berisha@stud.uni-hannover.de

Due to large file sizes, raw dataset images and pre-trained YOLO models are stored separately.  

---

## **Installation**
This project requires **Python 3.8+**, and various dependencies for computer vision and deep learning.

### **1. Install Dependencies**
To install all required Python packages, run:
```bash
pip install numpy opencv-python torch torchvision ultralytics matplotlib pandas json labelme
```

---
Please read https://github.com/qendre/match_student_code/blob/main/student_code/250204_Cable_detection_for_LocalPP/Cable_Detection/README.md for further details.
