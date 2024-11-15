## RB1 tools

## Install

### Prepare environment
```bash
mkdir -p /rb1_ws/src
cd ros2_ws
git clone https://github.com/morg1207/rb1_autonomy.git


```
### Install dependencies and build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ros2_ws
vcs import src < src/rb1_autonomy/rb1_

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install
```



## Sim Robot
### Launch servers

Terminal 1
```bash
ros2 launch rb1_autonomy servers.launch.py robot_mode:=sim_robot
```
Terminal 2

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### Launch
```bash
ros2 topic pub /nav_goal_for_discharge geometry_msgs/msg/Pose "{position: {x: 2.0, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```
### Launch only PathPlanner 
```bash
ros2 launch path_planner_server path_planner_server.launch.py type_simulation:=sim_robot use_sim_time:=True 
```

### Launch entire navigation
```bash
ros2 launch path_planner_server navigation.launch.py type_simulation:=sim_robot use_sim_time:=True map_file:=warehouse_map_sim_edit.yaml 
```

## Real Robots
### Launch servers
Terminal 1
```bash
ros2 launch rb1_autonomy servers.launch.py robot_mode:=sim_robot
```
Terminal 2

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Launch only localization 
```bash
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml use_sim_time:=True
```
### Launch only PathPlanner 

```bash
ros2 launch path_planner_server navigation.launch.py type_simulation:=real_robot use_sim_time:=False 