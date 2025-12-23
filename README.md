# Turtlebot3 Move Base Wandering - Standalone Repository

This repository contains a ROS Noetic catkin workspace with a single package for controlling Turtlebot3 in Gazebo with obstacle avoidance using the standard `move_base` navigation stack. The provided node continuously sends random navigation goals so the robot wanders while avoiding obstacles. A short demonstration of the system running is included in the repository as a video.

## 1. Install dependencies

You can install all required ROS packages with:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-noetic-turtlebot3-gazebo \
  ros-noetic-turtlebot3-navigation \
  ros-noetic-turtlebot3-description \
  ros-noetic-turtlebot3-msgs \
  ros-noetic-move-base-msgs \
  ros-noetic-dwa-local-planner \
  ros-noetic-gmapping
```

These provide:
- Gazebo simulation and Turtlebot3 models
- Navigation stack (`move_base`, costmaps, AMCL, map_server, DWA local planner)
- Messages used by the wandering node and move_base

## 2. Quick Start

### 2.1 Using SLAM (Recommended - Map Built On-The-Fly)

**First, install gmapping:**
```bash
sudo apt-get install ros-noetic-gmapping
```

**Then launch:**
```bash
cd /data2/pruthvi/Personal/turtlebot
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_move_base_wander custom_world_slam_wander.launch model:=burger
```

This will:
- Launch Gazebo with your custom world (`worlds/custom_tb3_world.world`)
- Build the map on-the-fly using SLAM (gmapping) as the robot moves
- Start `move_base` for navigation
- Run the wandering node that continuously sends random goals
- **RViz will show the map being built in real-time, matching your Gazebo world**

### 2.2 Using Pre-Built Map

If you have a pre-built map file:

```bash
cd /data2/pruthvi/Personal/turtlebot
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_move_base_wander custom_world_wander.launch model:=burger
```

**Note:** The default `custom_world_map.pgm` is a simple empty map. To generate a proper map:
1. Use the SLAM option above
2. Once the map looks good in RViz, save it:
   ```bash
   rosrun map_server map_saver -f $(rospack find turtlebot3_move_base_wander)/maps/custom_world_map
   ```

### 2.3 Simple Wander in Default Turtlebot3 World

If you only want the robot to wander in the **default Turtlebot3 Gazebo world** using the standard Turtlebot3 navigation map:

```bash
cd /data2/pruthvi/Personal/turtlebot
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_move_base_wander wander_with_move_base.launch model:=burger
```

This launch file:
- Starts Gazebo with the default `turtlebot3_world.world`
- Starts Turtlebot3 navigation (`turtlebot3_navigation.launch`, including `move_base` with DWA)
- Runs the wandering node that sends random `move_base` goals

## 3. Customizing Your World

Edit the world file:
```
src/turtlebot3_move_base_wander/worlds/custom_tb3_world.world
```

You can add obstacles, walls, or any Gazebo models. After editing, just relaunch.

## 4. Package Structure

```
turtlebot3_move_base_wander/
├── launch/
│   ├── custom_world_wander.launch      # Pre-built map version
│   ├── custom_world_slam_wander.launch # SLAM version (recommended)
│   └── wander_with_move_base.launch    # Uses default Turtlebot3 world
├── worlds/
│   └── custom_tb3_world.world         # Your custom Gazebo world
├── maps/
│   ├── custom_world_map.yaml          # Map metadata
│   └── custom_world_map.pgm           # Map image (placeholder)
└── src/
    └── random_goal_wander.py           # Node that sends random goals to move_base
```

## 5. How It Works

1. **Gazebo**: Simulates the Turtlebot3 robot in your custom world
2. **SLAM (gmapping)**: Builds a map from laser scan data as the robot moves
3. **AMCL**: Localizes the robot within the map
4. **move_base**: Plans paths and avoids obstacles
5. **random_goal_wander**: Continuously sends random navigation goals, keeping the robot moving

### Main node implementing wandering / avoidance

- `src/turtlebot3_move_base_wander/src/random_goal_wander.py`
  - Connects to the `move_base` action server.
  - Samples random 2D goals within a configurable area in the `map` frame.
  - Sends a new goal whenever the previous one succeeds, is aborted/rejected, or times out.
  - All **obstacle avoidance** is handled by the standard `move_base` stack (global/local planners and costmaps).

## 6. Building

```bash
- Clone the src and build like any other ROS1 repo with catkin_make
cd /data2/pruthvi/Personal/turtlebot
catkin_make
source devel/setup.bash
```


