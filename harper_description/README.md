# HARPER - Robot Description

ROS 2 description package for **HARPER**: URDF/xacro, meshes, and launch files for visualization and control.

HARPER is a dual-arm upper-body platform with:

- An aluminum torso
- A chest-mounted OAK-D Pro camera (`camera_chest_oak_d_pro`); head and chest camera mounts are reserved as swappable dummy assemblies
- Left and right arms, each with **6 actuated arm joints** (3× shoulder, elbow, forearm roll, wrist) and a **5-finger hand**

## Quick Start

```bash
# 1. Copy package to your ROS 2 workspace
cp -r harper_description ~/ros2_ws/src/

# 2. Build
cd ~/ros2_ws
colcon build --packages-select harper_description
source install/setup.bash

# 3. Visualize in RViz2
ros2 launch harper_description display.launch.py

# 4. Validate URDF structure
check_urdf install/harper_description/share/harper_description/urdf/harper.urdf

# 5. Print kinematic tree
urdf_to_graphviz install/harper_description/share/harper_description/urdf/harper.urdf
```

**Joint control**: The launch file includes `joint_state_publisher_gui` —
use the sliders to move revolute/prismatic joints in RViz2.

**Topic inspection**:
```bash
# See published joint states
ros2 topic echo /joint_states

# See robot description parameter
ros2 param get /robot_state_publisher robot_description
```