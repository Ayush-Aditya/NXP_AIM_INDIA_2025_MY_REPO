



# NXP\_AIM\_INDIA\_2025\_MY\_REPO 🚀

This repository tracks my development on the NXP Autonomous Inventory Management (AIM) Challenge — building a B3RB rover that autonomously navigates warehouse shelves, decodes QR codes, and identifies objects using YOLOv5 + ROS 2.

---

## 📁 Repository Structure

```
NXP_AIM_INDIA_2025_MY_REPO/
├── b3rb_ros_aim_india/           # Your modified ROS nodes for AIM India
│   ├── b3rb_ros_warehouse.py
│   ├── b3rb_ros_object_recog.py
│   └── ... others
├── resource/                     # YAML, TFLite, model files
│   ├── coco.yaml
│   ├── yolov5n-int8.tflite
│   └── ...
├── docs/                         # Notes, screenshots, test logs
├── LICENSE
└── README.md                     # (This file)
```

---

## 🎯 Project Overview

* **Goal:** Implement an autonomous inventory-tracking rover in simulation using ROS 2 and Gazebo.
* **Key Features:**

  * Navigate to a sequence of warehouse shelves using heuristic angles encoded in QR codes.
  * Decode QR codes positioned on both sides of each shelf.
  * Recognize objects on shelves using a quantized YOLOv5 model.
  * Publish object counts and QR metadata reliably to `/shelf_data`.
  * Reveal hidden shelves via curtain mechanics (driven by QR publication order).

---

## 🔧 Development Status & Progress

| Component                            | Status        | Notes                           |
| ------------------------------------ | ------------- | ------------------------------- |
| ROS Node: `b3rb_ros_warehouse.py`    | ✅ In progress | QR decoding + navigation        |
| ROS Node: `b3rb_ros_object_recog.py` | ✅ In progress | YOLO detection tuned            |
| GUI Progress Table                   | ⚙ Partially   | Populates objects + QR code     |
| ROS2 Nav2 tuning                     | ⚙ In progress | nav2.yaml and BT config         |
| Frontier-based exploration logic     | ⚙ In design   | For exploring unknown map       |
| Submission-ready packaging           | ❌ Pending     | Final cleanup before submission |

---

## 🧠 Getting Started

### Prerequisites

* Ubuntu 22.04 (Jammy)
* ROS 2 Humble
* CogniPilot (AIRY) installed per official AIM India instructions
* Gazebo + Nav2 + SLAM configured via `cranium`

### Installation

```bash
cd ~/cognipilot/cranium/src/
git clone https://github.com/Ayush-Aditya/NXP_AIM_INDIA_2025_MY_REPO.git
# Move your forked package into the expected workspace structure
mv NXP_AIM_INDIA_2025_MY_REPO b3rb_ros_aim_india

# Clean and rebuild cranium workspace
cd ~/cognipilot/cranium
colcon build --packages-select b3rb_ros_aim_india
source install/setup.bash
```

### Simulation Launch

```bash
ros2 launch b3rb_gz_bringup sil.launch.py \
  world:=nxp_aim_india_2025/warehouse_X \
  warehouse_id:=<N> shelf_count:=<M> \
  initial_angle:=<initial_deg> x:=0.0 y:=0.0 yaw:=0.0
```

---

## 🧪 How to Test

1. **Run the warehouse node:**

   ```bash
   ros2 run b3rb_ros_aim_india explore
   ```
2. **Run the detection node (object recognition):**

   ```bash
   ros2 run b3rb_ros_aim_india detect
   ```
3. **Verify Topics:**

   ```bash
   ros2 topic echo /shelf_objects
   ros2 topic echo /shelf_data
   ```
4. **Visual Debugging:**

   * Launch Foxglove with `/debug_images/qr_code` and `/debug_images/object_recog`
   * Use OpenCV windows if supported

---

## 🛠️ Features & Improvements

### Developed:

* ✅ QR code detection using `pyzbar`.
* ✅ Object detection using quantized YOLOv5 TFLite.
* ✅ Publishing of `synapse_msgs/WarehouseShelf` on `/shelf_data`.
* ✅ GUI progress table (optional, toggle via `PROGRESS_TABLE_GUI` flag).

### Planned Enhancements:

* 🛣️ Better shelf localization strategy (vision-based + map-based).
* 🔄 Navigation recovery logic to handle Nav2 failures.
* 🔁 State machine to track shelf visits and commands.
* 📉 Optimize confidence thresholds and NMS parameters.
* 🧰 Tune Nav2 behaviors and SLAM for robust movement.

---
b3rb_ros_warehouse.py SCRIPT FUNCTIONALITY OVERVIEW
The explore node provides a foundational structure.

SLAM MAP (INTRODUCTION & UNDERSTANDING)
The maps are created using LIDAR, IMU and odometry data.
Coordinate Frames:
Occupancy Grid Frame: SLAM maps are shared as nav_msgs/OccupancyGrid which is a 2-D grid.
Each cell represents the probability (expressed as 0-100) of being occupied by an object.
Thus, 0 = free space, 100 = occupied by obstacle; -1 (special case) = unexplored space.
For conversion to world coordinate frame:
Origin: Position of the bottom-left corner of the map in the world frame in meters.
Resolution: Length of each grid cell in meters.
(0, 0) is the bottom-most and left-most cell in the map in the below diagrams.
World Coordinate Frame: Real world coordinates in meters.
Origin: The starting point of the robot.
Types of Occupancy Grid Frames:
Static costmap (/map): Simple map used for quick decisions​
Global costmap (/global_costmap/costmap): Static costmap + inflation
Inflation: The expanding of obstacles in the costmap to create a buffer zone around them.
It's used because the robot needs to account for its size and potential localization errors.
Axes: The axes for static and global maps are given as follows. (NOTE: x-axis is robot's front direction at starting.)
<img width="1328" height="562" alt="image" src="https://github.com/user-attachments/assets/127ebfbf-7057-402b-8bea-f40adf76dcd7" />


NAVIGATION FRAMEWORK (BASE FUNCTIONALITY)
Core Navigation Logic: The script offers a framework for navigation using a Nav2 action client.
Participants provide a goal pose, and the Nav2 stack manages robot movement.
Goal: Consists of x-y coordinates (in world coordinate frame) and yaw (angle about the z-axis).
NOTE: YAW is the angle (between 0 to 2π) from the Positive x-axis in CCW direction.
The Nav2 stack provides feedback on the current goal's status via a feedback callback.
A mechanism to cancel an ongoing navigation goal is included.
This can be used if a goal repeatedly fails or if a more optimal goal is found.
Autonomous Exploration Example (Frontier-Based):
The script includes a demo frontier-based exploration approach for space exploration.
This demonstrates Nav2 usage and how the warehouse might be initially explored.
Participants have full autonomy to modify or replace this exploration logic.
Frontier Detection (get_frontiers_for_space_exploration):
Identify the boundaries between explored and unknown space.
Select the next exploration goal intelligently, considering proximity to obstacles.
Robot Arming: Monitors /cerebri/out/status and attempts to arm via /cerebri/in/joy.
WAREHOUSE INTERACTION (FRAMEWORK FOR CHALLENGE)
Shelf Object Handling:
Subscribes to /shelf_objects (synapse_msgs/WarehouseShelf): self.shelf_objects_callback.
Participants must process self.shelf_objects_curr for task-specific object identification.
Publishes to /shelf_data (synapse_msgs/WarehouseShelf): self.publisher_shelf_data.
Participants must construct and send messages per challenge rules.
QR Code Detection (Framework):
Subscribes to /camera/image_raw/compressed: self.camera_image_callback.
Participants must implement QR decoding logic here.
Participants may store the the last decoded QR string in self.qr_code_str.
Optionally publish debug images for QR to /debug_images/qr_code.
GUI - PROGRESS TABLE (OPTIONAL UTILITY)
WindowProgressTable Class: A Tkinter-based GUI.
Functionality:
Displays a 2 x n grid, mapping to 2 rows and n shelves.
Can be enabled/disabled using the PROGRESS_TABLE_GUI flag.
This GUI is provided for participant convenience to track progress. It's use is entirely optional.
Participants choosing to use it should integrate updates to reflect their challenge progress.
It's usage is described in shelf_objects_callback.

## 📚 References

* [NXP AIM India 2025 challenge repository](https://github.com/NXPHoverGames/NXP_AIM_INDIA_2025)
* CogniPilot AIRY Dev Guide
* ROS 2 Humble + Nav2 documentation
* `synapse_msgs/WarehouseShelf` message specs

---

## 📌 License

This project builds upon the NXP AIM India 2025 open-source base. Licensed under Apache 2.0 (unless otherwise stated). Check individual package files for licensing details.

---

## 🙌 Contact

For feedback or collaboration, feel free to reach out via GitHub!

---


