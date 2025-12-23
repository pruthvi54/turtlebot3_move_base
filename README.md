# Turtlebot3 Move Base Wandering - Standalone Repository

This repository contains a complete ROS Noetic package for controlling Turtlebot3 in Gazebo with obstacle avoidance using `move_base`. Everything needed (world files, launch files, nodes) is contained within this repo.

## Quick Start

### Option 1: Using SLAM (Recommended - Map Built On-The-Fly)

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

### Option 2: Using Pre-Built Map

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

## Customizing Your World

Edit the world file:
```
src/turtlebot3_move_base_wander/worlds/custom_tb3_world.world
```

You can add obstacles, walls, or any Gazebo models. After editing, just relaunch.

## Package Structure

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

## How It Works

1. **Gazebo**: Simulates the Turtlebot3 robot in your custom world
2. **SLAM (gmapping)**: Builds a map from laser scan data as the robot moves
3. **AMCL**: Localizes the robot within the map
4. **move_base**: Plans paths and avoids obstacles
5. **random_goal_wander**: Continuously sends random navigation goals, keeping the robot moving

## Dependencies

Required ROS packages (install if missing):
- `ros-noetic-turtlebot3-gazebo`
- `ros-noetic-turtlebot3-navigation`
- `ros-noetic-turtlebot3-description`
- `ros-noetic-move-base-msgs`
- `ros-noetic-gmapping` (for SLAM option)

## Building

```bash
cd /data2/pruthvi/Personal/turtlebot
catkin_make
source devel/setup.bash
```


