<div align="center">

# Vision-Guided Robotic Arm — 6-DOF, ROS 2 & MoveIt

**Autonomous object detection, 3D localization, and collision-free motion planning in ROS 2 & Gazebo, driven by a depth camera and MoveIt.**

A 6-DOF arm finds a colored object it has never been told the location of, computes the object's 3D position from a fixed overhead RGB-D camera, checks it against the arm's reachable workspace, and plans a **collision-free** trajectory to hover directly above it — fully automatically.

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)
![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Motion_Planning-0A7BBB)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-FF6C2C?logo=gazebo&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-Perception-5C3EE8?logo=opencv&logoColor=white)
![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)

</div>

---

## Demo

<table>
<tr>
<td width="50%" align="center">

### Arm detecting and reaching the object

<!-- Paste your GIF's <img ... /> tag here (drag-drop the file into GitHub's editor to get one). -->
<img width="100%" alt="Arm detecting the object and moving to hover above it" src="" />

</td>
<td width="50%" align="center">

### Robot model in RViz

<!-- Paste your RViz URDF screenshot's <img ... /> tag here (drag-drop the file into GitHub's editor to get one). -->
<img width="100%" alt="6-DOF arm URDF in RViz" src="" />

</td>
</tr>
</table>

---

## Highlights

- **No hard-coded target position** — the arm discovers where the object is purely from camera data at runtime.
- **Closed perception → planning → action loop** — vision detects the object, deprojects it to a 3D pose, publishes it, and the controller plans and drives the arm to hover above it.
- **Reachability-aware** — every detected point is checked against a model of the arm's workspace; out-of-reach points are clamped onto the nearest reachable shell so the arm always gets something it can attempt.
- **Real collision checking** — a `compute_ik` pre-check confirms a collision-free IK solution exists before MoveIt plans and executes, so no unreachable or colliding target is ever sent to the robot.
- **Written in modern C++** across two ROS 2 nodes, with a URDF/Xacro robot model and a MoveIt Setup Assistant configuration.

---

## How It Works

```
                 Gazebo (sim + RGB-D camera)
                          │
        /camera/image_raw │ /camera/depth/image_raw │ /camera/camera_info
                          ▼
                   ┌─────────────┐
                   │ vision_node │   detect blob → deproject with depth →
                   │ (OpenCV)    │   transform to robot_base frame →
                   └─────────────┘   reachability check → hover height
                          │
                          │  /object_position  (geometry_msgs/PointStamped)
                          ▼
                  ┌────────────────┐
                  │ arm_controller │   compute_ik pre-check →
                  │ (MoveGroup)    │   MoveIt plan → execute
                  └────────────────┘
                          │  FollowJointTrajectory action
                          ▼
              gazebo_ros2_control controllers (in sim)
```

**1. Perception —** `vision_node` subscribes to the simulated RGB-D camera. It segments the colored object in HSV space (high-saturation pixels against the gray ground), takes the largest blob, and finds its pixel centroid.

**2. 3D localization —** using the depth image and the camera intrinsics (`camera_info`), it deprojects that pixel to a 3D point in the camera optical frame with a pinhole model, then transforms it into the arm's `robot_base` frame via **tf2**.

**3. Reachability gate —** the point is checked against a coarse spherical-shell model of the arm's workspace. In-reach points are published as-is; out-of-reach points are clamped onto the nearest reachable shell so the arm always gets something it can try. The published Z is overridden to a fixed **hover height** above the object (so the arm doesn't dive into the floor).

**4. Motion —** `arm_controller` receives the latest target on `/object_position`, runs a `compute_ik` pre-check to confirm a collision-free solution exists, and if so asks MoveIt to plan and execute a trajectory. Execution is sent to the `gazebo_ros2_control` `JointTrajectoryController` running inside the simulation.

---

## Tech Stack

| Layer | Tools / Libraries |
|---|---|
| **OS / Middleware** | ROS 2 **Humble**, Ubuntu (tested under WSL2) |
| **Motion planning** | **MoveIt 2** (`move_group`, `MoveGroupInterface`, `compute_ik`) |
| **Simulation** | **Gazebo Classic**, `gazebo_ros`, `gazebo_ros2_control` |
| **Control** | `ros2_control`, `joint_state_broadcaster`, `joint_trajectory_controller` |
| **Perception** | **OpenCV**, `cv_bridge`, `image_geometry` (pinhole model), `tf2` / `tf2_ros` |
| **Robot model** | URDF / **Xacro** (6-DOF arm + fixed RGB-D camera), SRDF from MoveIt Setup Assistant |
| **Language / build** | **C++** (rclcpp nodes), **Python** (launch files), `colcon`, `ament_cmake` |
| **Messages** | `geometry_msgs`, `sensor_msgs`, `moveit_msgs` |

---

## Repository Layout

```
MoveItArm/
├── docs/
│   └── moveit_gazebo_integration.md   # deep-dive on the MoveIt↔Gazebo wiring
├── src/
│   ├── moveit_arm_package/            # robot description + the two custom nodes
│   │   ├── src/
│   │   │   ├── ArmController.cpp      # MoveGroupInterface motion node (+ IK pre-check)
│   │   │   └── VisionNode.cpp         # OpenCV perception → 3D target publisher
│   │   ├── launch/
│   │   │   ├── gazebo.launch.py       # Gazebo + robot + controllers
│   │   │   ├── moveit.launch.py       # move_group + RViz
│   │   │   ├── bringup.launch.py      # gazebo + moveit in one shot
│   │   │   ├── run_controller.launch.py  # the custom arm_controller node
│   │   │   └── display.launch.py      # RViz-only description viewer
│   │   ├── urdf/
│   │   │   ├── arm.urdf.xacro         # the arm + ros2_control/Gazebo system
│   │   │   └── camera.xacro           # fixed RGB-D camera on a stand
│   │   ├── worlds/blocks.world        # Gazebo scene with the target block
│   │   ├── models/aruco_marker/       # ArUco marker model (textured)
│   │   └── config/controllers.yaml    # controllers gazebo_ros2_control spawns
│   └── moveit_config/                 # MoveIt Setup Assistant output
│       ├── config/                    # SRDF, kinematics, joint limits, RViz, etc.
│       └── launch/                    # generated MoveIt launch files
├── ik_probe.py / ik_probe_xy.py       # helper scripts to probe IK reachability
└── README.md
```

The arm is a 6-DOF manipulator. The MoveIt planning group is named **`arm`**, with named poses `pose_1`, `pose_2`, `pose_3` defined in the SRDF.

---

## Getting Started

### Prerequisites
- **ROS 2 Humble** (desktop install)
- **Gazebo Classic**, MoveIt 2, and the ROS 2 control bridges

```bash
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-cv-bridge \
  ros-humble-image-geometry \
  ros-humble-tf2-geometry-msgs \
  ros-humble-ros2controlcli   # optional, for `ros2 control list_controllers`
```

OpenCV (`libopencv-dev`) is pulled in as a package dependency.

### Build

```bash
cd ~/MoveItArm
colcon build --packages-select moveit_arm_package moveit_config
source install/setup.bash
```

Re-`source install/setup.bash` in every new terminal.

### Run

**Option A — everything, then drive it (recommended)**

```bash
# Terminal 1 — Gazebo simulation + MoveIt move_group + RViz
ros2 launch moveit_arm_package bringup.launch.py

# Terminal 2 — the custom vision-guided controller (after RViz/Gazebo are up)
ros2 launch moveit_arm_package run_controller.launch.py

# Terminal 3 — the perception node
ros2 run moveit_arm_package vision_node
```

Once all three are running, the camera detects the block, `vision_node` publishes a target on `/object_position`, and the arm plans and moves to hover above it. An OpenCV window ("raw frame") shows the live camera feed with the detected object outlined.

**Option B — split launch (more control / easier debugging)**

```bash
# Terminal 1
ros2 launch moveit_arm_package gazebo.launch.py
# Terminal 2 (after Gazebo is up)
ros2 launch moveit_arm_package moveit.launch.py
# Terminal 3
ros2 launch moveit_arm_package run_controller.launch.py
# Terminal 4
ros2 run moveit_arm_package vision_node
```

**Verify the controllers came up (optional)**

```bash
ros2 control list_controllers
# joint_state_broadcaster  ... active
# arm_controller           ... active
# gripper_controller       ... active
```

---

## How the Nodes Work

### `vision_node` (`src/VisionNode.cpp`)
- Subscribes to `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info`.
- Segments the object by saturation in HSV, picks the largest contour, computes its centroid.
- Deprojects the centroid to 3D with `image_geometry::PinholeCameraModel` + the depth value, then `tf2`-transforms it into the `robot_base` frame.
- Gates the point against an approximate reachable workspace (spherical shell around the shoulder); clamps out-of-reach points to the nearest reachable point.
- Publishes a rate-limited (1 Hz) `geometry_msgs/PointStamped` on `/object_position`, with Z set to a fixed hover height.

### `arm_controller` (`src/ArmController.cpp`)
- Subscribes to `/object_position` (keep-last depth 1: only the newest target matters).
- A dedicated worker thread consumes the latest target one motion at a time (`MoveGroupInterface` is not thread-safe).
- For each target: calls the `compute_ik` service to confirm a **collision-free** IK solution exists (and logs the precise reason if not), then plans and executes with MoveIt.
- The executor spins in a background thread so the async `compute_ik` future is actually fulfilled while the worker waits on it.

---

## Key ROS Topics & Services

| Name | Type | Direction |
|------|------|-----------|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → vision_node |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1) | Gazebo → vision_node |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → vision_node |
| `/object_position` | `geometry_msgs/PointStamped` | vision_node → arm_controller |
| `compute_ik` | `moveit_msgs/srv/GetPositionIK` | arm_controller → move_group |

---

## Notes & Gotchas

These are the non-obvious things that make the integration work (full detail in
[docs/moveit_gazebo_integration.md](docs/moveit_gazebo_integration.md)):

- **Never** `ros2 run moveit_arm_package arm_controller` directly — it needs `robot_description_semantic` (the SRDF) injected, which only the launch file provides. Always start it via `run_controller.launch.py`.
- **Don't** run `moveit_config demo.launch.py` alongside Gazebo — it spins up a *mock* `ros2_control` node that fights Gazebo's controller_manager.
- Only **one** `ros2_control` block should be loaded for Gazebo (`GazeboSystem` in `arm.urdf.xacro`). XML comments inside `robot_description` can silently break the controller_manager, so the launch file strips comments and the XML declaration before publishing.
- `use_sim_time: True` everywhere so MoveIt and the Gazebo controllers share `/clock`.
- On a slow machine, increase the `TimerAction` delay in `bringup.launch.py` if `move_group` starts before the controllers exist.

---

## License

Apache-2.0

<div align="center">

*Built by [Ibrahim Zantout](https://github.com/IbrahimZantoutt) — robotics, perception & motion planning in ROS 2.*

</div>
