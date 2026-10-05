<sub>🌐 **English** · [Español](README.es.md)</sub>

# RB1 Warehouse Autonomy — Nav2 + Behavior Trees (ROS 2 Humble)

Autonomous shelf-handling mission for the **RB1 mobile robot** in a warehouse, built on
**ROS 2 Humble**, **Nav2** and **BehaviorTree.CPP**. Tested in Gazebo simulation and **on the real robot**.

▶️ **Real-robot demo:** [YouTube video](https://www.youtube.com/watch?v=rZ5ojMnCDvw)

<img src="images/gifs/approach_and_pick_shelf.gif" width="500"/>

## What the robot does

A complex behavior is composed from simpler ones using behavior trees:

1. **Find the charging station and localize** — the robot detects the station with the LiDAR and uses its known position to initialize localization on the map.
2. **Find a shelf** anywhere in the warehouse, patrolling a set of Nav2 waypoints.
3. **Approach the shelf**, drive underneath it and use the elevator to **pick it up**.
4. **Carry and place** the shelf at the requested location.

**Key techniques:** LiDAR intensity-based object detection (reflective tapes), TF2 frame publishing for docking,
proportional control for the approach, Nav2 `NavigateToPose` clients, dynamic footprint change while carrying,
and custom action/condition nodes for BehaviorTree.CPP.

## 1. Setup

### 1.1 Prepare the workspace
```bash
mkdir -p ~/rb1_ws/src
cd ~/rb1_ws/src
git clone https://github.com/morg1207/rb1_autonomy.git
```

### 1.2 Install dependencies and build
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/rb1_ws
vcs import src < ~/rb1_ws/src/rb1_autonomy/rb1_simulation.repos
sudo apt update
rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install
```

## 2. Architecture

<img src="./architecture_docs/architecture.png" alt="System architecture" width="500"/>

### 2.1 Servers

#### 2.1.1 Find Object Server
Detects objects through the **intensity values of the laser**, looking for reflective tapes. The object must have
two legs, each with a piece of tape. It currently detects two object types: `shelf` and `station`.

<img src="./images/servers/find_object.jpg" alt="Find object server" width="400"/>

It returns a `geometry_msgs::msg::Pose` where `x` = d, `y` = θ and `z` = β.

**Parameters**
- `use_sim_time` — `true` for simulation, `false` for the real robot.
- `limit_intensity_laser_detect` — minimum intensity required to detect a reflective tape.
- `limit_min_detection_distance_legs_shelf` / `limit_max_detection_distance_legs_shelf` — leg-separation range that identifies a shelf.
- `limit_min_detection_distance_legs_charge_station` / `limit_max_detection_distance_legs_charge_station` — leg-separation range that identifies the charging station (**real robot only**; the simulation has no charging station).

#### 2.1.2 Approach Shelf Server
Controls the approach to and exit from the shelf using TF frames. Three control types are available (see the
image); only the proportional controller is used.

<img src="./images/servers/approach_shelf.jpg" alt="Approach shelf server" width="600"/>

**Parameters**
- `use_sim_time` — `true` for simulation, `false` for the real robot.
- `vel_min_linear_x`, `vel_max_linear_x` — linear velocity limits along x.
- `vel_min_angular_z`, `vel_max_angular_z` — angular velocity limits around z.
- `kp_lineal`, `kp_angular` — proportional gains for linear and angular velocity.
- `distance_approach_target_error` — acceptable distance error when approaching the target.
- `distance_approach_target_error_back` — acceptable distance error when reversing to the target.
- `angle_approach_target_error` — acceptable angular error when approaching the target.
- `laser_min_range` — minimum laser range used to detect objects.
- `distance_for_back_frame_publish` — distance threshold to publish the back frame.

#### 2.1.3 Init Localization Server (real robot only)
Computes the robot's pose on the map from a static object with a known location — the charging station —
and publishes it on `/initial_pose`.

<img src="./images/servers/init_localization.png" alt="Init localization server" width="600"/>

**Parameters**
- `use_sim_time` — `true` for simulation, `false` for the real robot.
- `pose_map_to_station_charge_x`, `pose_map_to_station_charge_y`, `pose_map_to_station_charge_yaw` — pose of the map relative to the charging station.

### 2.2 Behavior tree

<img src="./architecture_docs/behavior_tree_nodes.png" alt="Behavior tree nodes" width="600"/>

<img src="./architecture_docs/behavior_tree_nodes_auxiliary.png" alt="Auxiliary behavior tree nodes" width="600"/>

#### 2.2.1 Action nodes

| Node | Description |
|---|---|
| **ClientFindObject** | Client wrapper for the Find Object server: sends the search request and returns the result. |
| **ClientApproachShelf** | Client wrapper for the Approach Shelf server: drives the robot to a detected shelf. |
| **ClientInitLocalization** | Client wrapper for the Init Localization server: sets the starting pose from the charging station. |
| **ClientNav** | Client wrapper for Nav2's `NavigateToPose` action. |
| **PublishTransform** | Publishes the transform between reference frames. |
| **HandlerPlatform** | Raises or lowers the elevator platform to load or unload the shelf. |
| **ChangeFootprint** | Changes the robot footprint (e.g. while carrying a shelf). |
| **TurnRobot** | Rotates the robot in place towards a target or to adjust its pose. |
| **NavPoses** | Sends a list of waypoints, defined in a YAML file, to Nav2 to patrol the warehouse looking for the shelf. |
| **WaitForGoalNav** | Waits until the robot reaches its navigation goal — used to wait for the drop-off position. |

#### 2.2.2 Condition nodes

| Node | Description |
|---|---|
| **CheckApproach** | Checks whether the approach controller has finished while the Approach Shelf server is active. |

#### 2.2.3 Behavior trees

1. **Find station and init localization** (real robot only) — finds the station and initializes localization.

   <img src="./images/bt/find_station_and_init_localization.png" alt="Find station BT" width="600"/>

2. **Find shelf** — sends navigation waypoints while the Find Object server searches for the shelf.

   <img src="./images/bt/find_shelf.png" alt="Find shelf BT" width="600"/>

3. **Approach and pick shelf** — guides the robot underneath the shelf and positions it correctly before lifting.

   <img src="./images/bt/approach_and_pick_shelf.png" alt="Approach and pick BT" width="600"/>

4. **Carry and discharge shelf** — waits until the robot reaches the drop-off pose, lowers the platform and drives out from under the shelf.

   <img src="./images/bt/carry_and_discharge_shelf.png" alt="Carry and discharge BT" width="600"/>

## 3. Running the project

Every terminal starts with:
```bash
cd ~/rb1_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

### 3.1 Simulation

| Terminal | Command |
|---|---|
| 1 — Gazebo | `ros2 launch the_construct_office_gazebo warehouse_rb1_rviz.launch.xml` |
| 2 — Nav2 | `ros2 launch path_planner_server navigation.launch.py type_simulation:=sim_robot use_sim_time:=True map_file:=warehouse_map_sim_edit.yaml` |
| 3 — Servers | `ros2 launch rb1_autonomy servers.launch.py robot_mode:=sim_robot` |
| 4 — Autonomy | `ros2 launch rb1_autonomy autonomy.launch.py robot_mode:=sim_robot` |

#### Select a behavior tree (terminal 5)

**Find shelf**
```bash
ros2 topic pub -t 3 /bt_selector std_msgs/msg/String "{data: 'find_shelf'}"
```
![Find shelf](images/gifs/find_shelf.gif)

**Approach and pick shelf**
```bash
ros2 topic pub -t 3 /bt_selector std_msgs/msg/String "{data: 'approach_and_pick_shelf'}"
```

**Carry and discharge shelf**

<img src="images/gifs/carry_and_dischargge_shelf.gif" width="500"/>

> **Known limitation:** after picking up the shelf, the robot is often too close to nearby objects and the
> enlarged footprint puts it in collision. Before sending a new goal, move it back manually:
> ```bash
> ros2 run teleop_twist_keyboard teleop_twist_keyboard
> ```

Then run the tree and publish the drop-off pose:
```bash
ros2 topic pub -t 3 /bt_selector std_msgs/msg/String "{data: 'carry_and_discharge_shelf'}"
ros2 topic pub -t 3 /nav_goal_for_discharge geometry_msgs/msg/Pose "{position: {x: 0.53, y: 0.62, z: 2.0}, orientation: {x: 0.0, y: 0.0, z: 0.7, w: 0.71}}"
```

**Entire mission**

Place the shelf where it will not lead to collisions. When terminal 4 prints `waiting for nav goal`
(the shelf is loaded), publish the drop-off pose:
```bash
ros2 topic pub -t 3 /bt_selector std_msgs/msg/String "{data: 'entire_simulation'}"
ros2 topic pub -t 3 /nav_goal_for_discharge geometry_msgs/msg/Pose "{position: {x: 4.56, y: 0.0, z: 0.58}, orientation: {x: 0.0, y: 0.0, z: 0.68, w: 0.72}}"
```

### 3.2 Real robot

| Terminal | Command |
|---|---|
| 1 — Nav2 | `ros2 launch path_planner_server navigation.launch.py type_simulation:=real_robot use_sim_time:=False map_file:=warehouse_map_real.yaml` |
| 2 — Servers | `ros2 launch rb1_autonomy servers.launch.py robot_mode:=real_robot` |
| 3 — Autonomy | `ros2 launch rb1_autonomy autonomy.launch.py robot_mode:=real_robot` |

▶️ [Real-robot test video](https://www.youtube.com/watch?v=rZ5ojMnCDvw)

## Notes

This repository searches for the shelf by sending patrol waypoints. The version shown in the video uses a
different method: it detects the shelf legs by **clustering the laser readings**. I am still improving and
documenting the code and fixing some bugs related to that approach.

## Acknowledgements

Thanks to [The Construct](https://www.theconstruct.ai/) for the RB1 simulation and the training behind this
project, and to [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2) for the ROS 2 wrappers
for behavior trees.
