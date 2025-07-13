# NXP\_AIM\_INDIA\_2025\_MY\_REPO 🚀

This repository tracks my development and custom enhancements for the **NXP Autonomous Inventory Management (AIM) Challenge 2025**. The goal is to build an autonomous B3RB rover that:

* Navigates warehouse environments
* Decodes QR codes
* Detects shelf objects using YOLOv5 (quantized TFLite)
* Publishes inventory and shelf metadata using ROS 2

---

## 📁 Repository Structure

```
NXP_AIM_INDIA_2025_MY_REPO/
├── b3rb_ros_aim_india/           # Modified ROS nodes for AIM India
│   ├── b3rb_ros_warehouse.py     # QR code + navigation logic
│   ├── b3rb_ros_object_recog.py  # Object detection using YOLOv5n
│   └── ...
├── resource/                     # Model + label files
│   ├── coco.yaml
│   ├── yolov5n-int8.tflite
│   └── ...
├── docs/                         # Notes, screenshots, debug logs
├── LICENSE
└── README.md                     # This file
```

---

## 🎯 Project Overview

* **Challenge:** Develop an autonomous robot in simulation using ROS 2 & Gazebo.
* **Objectives:**

  * Navigate to all warehouse shelves
  * Decode left and right QR codes per shelf
  * Detect and count objects using an onboard YOLOv5 model
  * Publish structured data to `/shelf_data` (custom `WarehouseShelf` message)
  * Enable curtain mechanics by visiting shelves in a specific order

---

## 🔧 Development Status

| Component                  | Status        | Notes                           |
| -------------------------- | ------------- | ------------------------------- |
| `b3rb_ros_warehouse.py`    | ✅ In progress | QR code reading + nav client    |
| `b3rb_ros_object_recog.py` | ✅ In progress | YOLO TFLite + object publishing |
| GUI Progress Table         | ⚙ Partial     | Tkinter GUI optional module     |
| Nav2 + SLAM Configuration  | ⚙ In progress | BT navigator + tuning ongoing   |
| Frontier Exploration       | ⚙ Planned     | For autonomous map discovery    |
| Submission Packaging       | ❌ Pending     | Final clean-up before deadline  |

---

## 🧠 Getting Started

### Prerequisites

* Ubuntu 22.04 LTS
* ROS 2 Humble
* CogniPilot `cranium` setup (AIRY base)
* Gazebo, Nav2, SLAM Toolbox

### Installation

```bash
cd ~/cognipilot/cranium/src/
git clone https://github.com/Ayush-Aditya/NXP_AIM_INDIA_2025_MY_REPO.git
mv NXP_AIM_INDIA_2025_MY_REPO b3rb_ros_aim_india

cd ~/cognipilot/cranium
colcon build --packages-select b3rb_ros_aim_india
source install/setup.bash
```

---

## 🚀 Launch Instructions

### Run Simulation

```bash
ros2 launch b3rb_gz_bringup sil.launch.py \
  world:=nxp_aim_india_2025/warehouse_X \
  warehouse_id:=<N> shelf_count:=<M> \
  initial_angle:=<deg> x:=0.0 y:=0.0 yaw:=0.0
```

### Run Nodes

```bash
# Warehouse navigation + QR logic
ros2 run b3rb_ros_aim_india explore

# Object detection node
ros2 run b3rb_ros_aim_india detect
```

### Test Output

```bash
ros2 topic echo /shelf_objects
ros2 topic echo /shelf_data
```

Use `/debug_images/qr_code` and `/debug_images/object_recog` in **Foxglove** or **rqt\_image\_view** for visual debug.

---

## 🌐 SLAM & Navigation Overview

* **Occupancy Grid Map:**

  * `/map` = Static 2D costmap
  * `/global_costmap/costmap` = Inflated map

* **Coordinate Frames:**

  * Origin = Starting robot pose
  * Resolution = meters per cell
  * Cells: 0 = free, 100 = occupied, -1 = unknown

* **Frontier Detection:**

  * Uses occupancy grid to detect unexplored regions
  * Selects goals for autonomous exploration

* **Navigation Goals:**

  * Sent via Nav2 action client with pose + yaw
  * Feedback via callbacks, with cancel-on-failure logic

---

## 🔧 QR Code + Object Detection

### QR Code (in `b3rb_ros_warehouse.py`)

* Decodes left and right QR codes on each shelf
* Stores content in `self.qr_code_str`
* Optionally publishes `/debug_images/qr_code`

### Object Detection (in `b3rb_ros_object_recog.py`)

* Uses `yolov5n-int8.tflite` + `coco.yaml`
* Inference using TFLite interpreter
* Publishes object counts to `/shelf_objects` (type: `WarehouseShelf`)
* Optionally visualizes with `/debug_images/object_recog`

---

## 📊 GUI - Progress Table

* Optional **Tkinter GUI** for visualizing shelf progress
* Controlled using `PROGRESS_TABLE_GUI` flag
* Automatically updates when `/shelf_objects` are published

---

## ✨ Features & Enhancements

### Completed:

* ✅ QR decoding with `pyzbar`
* ✅ TFLite YOLOv5 object recognition
* ✅ Publishing to `/shelf_data` and `/shelf_objects`
* ✅ Debug image support for QR and object view
* ✅ Modular design for easy upgrades

### Planned:

* 🚜 Better shelf localization (map + vision)
* ⚡ Recovery logic for failed goals
* 🔄 Visit tracker & state machine
* 📊 Confidence + NMS optimization
* ⚖ Nav2 + SLAM parameter tuning

---

## 📊 How to Contribute (Pull Request Guide)

If you'd like to contribute via Pull Request (PR), here's how:

### 1. Fork the Repo

Go to: [https://github.com/Ayush-Aditya/NXP\_AIM\_INDIA\_2025\_MY\_REPO](https://github.com/Ayush-Aditya/NXP_AIM_INDIA_2025_MY_REPO)
Click on **"Fork"** to create your own copy.

### 2. Clone Your Fork

```bash
git clone https://github.com/<your-username>/NXP_AIM_INDIA_2025_MY_REPO.git
cd NXP_AIM_INDIA_2025_MY_REPO
git remote add upstream https://github.com/Ayush-Aditya/NXP_AIM_INDIA_2025_MY_REPO.git
```

### 3. Create a Feature Branch

```bash
git checkout -b feature/my-contribution
```

### 4. Make Changes, Commit, Push

```bash
git add .
git commit -m "Add: Fixed QR bug and improved nav params"
git push origin feature/my-contribution
```

### 5. Open PR

* Go to your GitHub fork.
* Click **"Compare & Pull Request"**.
* Fill out description and submit PR!

---

## 📚 References

* [Official NXP AIM India 2025 Repository](https://github.com/NXPHoverGames/NXP_AIM_INDIA_2025)
* [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
* [Nav2 Tutorials](https://navigation.ros.org/tutorials/docs/)
* [TFLite Model Zoo](https://www.tensorflow.org/lite/models)

---

## 📌 License

Based on the NXP AIM India 2025 challenge code.
Licensed under the **Apache License 2.0** (see `LICENSE` file for details).

---

## 🙌 Contact

If you want to collaborate, report bugs, or suggest improvements, open an [Issue](https://github.com/Ayush-Aditya/NXP_AIM_INDIA_2025_MY_REPO/issues) or reach out via GitHub!

---
