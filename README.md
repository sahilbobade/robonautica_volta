Sure! Here's the full markdown text you can copy and paste into your `README.md` file:

```markdown
# Vision-Based Leader-Follower System using ArUco Markers

This ROS package implements a **vision-based leader-follower system** using ArUco marker tracking and a Proportional-Derivative (PD) controller. The system uses camera input to detect an ArUco marker and control a robot to follow the marker by adjusting its linear and angular velocities.

---

## 🧠 Features

- ArUco marker detection using OpenCV
- Pose estimation with `cv2.solvePnP` or `cv2.aruco.estimatePoseSingleMarkers`
- PD controller for smooth and stable following behavior
- Real-time command publishing to `/cmd_vel`
- Live visualization of camera feed with marker and pose axes

---

## 📦 Dependencies

- Python 3
- ROS (tested with ROS Noetic)
- OpenCV (with `cv2.aruco` module)
- `cv_bridge`
- `sensor_msgs`
- `geometry_msgs`

---

## 🔧 Installation

1. Clone the repository into your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/your-username/vision-based-leader-follower.git
cd ~/catkin_ws
catkin_make
```

2. Install necessary dependencies:

```bash
sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport python3-opencv
```

---

## 🚀 Usage

1. Launch your camera node (e.g., `usb_cam`):

```bash
rosrun usb_cam usb_cam_node
```

2. Run the ArUco follower node:

```bash
rosrun vision_based_leader_follower aruco_follower.py
```

3. A window will display the live camera feed with detected ArUco markers and pose estimation.

---

## ⚙️ Configuration

- **Camera Calibration**:
  Update the `camera_matrix` and `dist_coeffs` in the script according to your camera.

- **PD Controller Parameters**:
  Tune the following gains in the code as needed:
  - `Kp_z`, `Kd_z`: Linear (distance) control
  - `Kp_ang`, `Kd_ang`: Angular (orientation) control

- **Marker Dictionary**:
  The script uses `DICT_5X5_50` by default. You can change this in the `ARUCO_DICTS` dictionary.

---

## 🖨️ ArUco Marker Generation

To generate your own ArUco markers for printing:

```python
import cv2
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
marker = cv2.aruco.drawMarker(aruco_dict, id=0, sidePixels=200)
cv2.imwrite("marker0.png", marker)
```


## 📋 License

This project is licensed under the MIT License.

---
