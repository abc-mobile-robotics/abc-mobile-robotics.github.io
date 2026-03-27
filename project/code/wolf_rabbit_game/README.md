# wolf_rabbit_game

A single-package ROS 2 starter project for a TurtleBot4 predator-prey game.

## Nodes
- `yolo_detector`: detects wolf, rabbit, and carrot targets.
- `geofence_node`: checks arena and wolf-territory boundaries.
- `rabbit_fsm`: handles rabbit search and escape behavior.
- `wolf_fsm`: handles wolf patrol and chase behavior.
- `referee_node`: determines capture and escape game states.

## Build
```bash
cd ~/colcon_ws/src
cp -r wolf_rabbit_game ./
cd ..
colcon build --packages-select wolf_rabbit_game
source install/setup.bash
```

## Run
```bash
ros2 launch wolf_rabbit_game game.launch.py
```

## Topics used in this starter
- `/rabbit/odom`
- `/wolf/odom`
- `/camera/image_raw`
- `/rabbit/cmd_vel`
- `/wolf/cmd_vel`
- `/rabbit/geofence`
- `/wolf/geofence`
- `/rabbit/vision`
- `/wolf/vision`
- `/game/state`

## Notes
This starter keeps everything in one ROS 2 package for easier development.
The current detection and coordination messages use JSON strings to avoid creating a separate interface package.
You can later upgrade them into custom ROS messages if needed.
