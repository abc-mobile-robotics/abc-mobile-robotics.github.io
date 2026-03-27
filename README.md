# RAS 598 Mobile-Robotics: Team Palantir 

This repository is for the final project of the team **Palantir** for the course RAS598 Mobile Robotics. 
our mission is to build a two-robot predator-prey game using TurtleBot4 in a bounded indoor arena. 
One robot acts as a **wolf** patrolling a designated territory, the other as a 
**rabbit** searching the arena for randomly generated AR carrots. Both robots 
use **YOLO-based visual detection** to recognize each other and carrot targets.

When the rabbit detects the wolf, it turns 180° and flees — changing direction 
at the territory boundary until it escapes. The wolf chases the rabbit when 
detected but stops at its boundary. The game ends when the wolf closes within 
**0.4 m** of the rabbit.

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

## Team Members
- Aldrick Peter Thomas 
- Shao-Chi Cheng (Brian)
- Chach Chaimongkol