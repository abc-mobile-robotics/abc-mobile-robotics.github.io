---
nav_exclude: true
---
# turtlebot4_geofence

A ROS2 geofencing engine for TurtleBot4.  
Subscribes to `/amcl_pose`, classifies the robot's state against a polygon
boundary, and publishes corrective `/cmd_vel` commands to keep it inside.

---

## Package structure

```
turtlebot4_geofence/
├── config/
│   └── geofence_params.yaml   ← edit this to set your polygon + tuning
├── launch/
│   └── geofence.launch.py
├── turtlebot4_geofence/
│   ├── __init__.py
│   └── geofence_node.py       ← the node
├── package.xml
├── setup.py
└── setup.cfg
```

---

## What /amcl_pose gives you

```
geometry_msgs/PoseWithCovarianceStamped
  header
    stamp.sec / stamp.nanosec   ← timestamp
    frame_id                    ← always "map"

  pose
    pose
      position
        x   ← robot X in map frame  (metres)
        y   ← robot Y in map frame
        z   ← always ~0 for ground robots

      orientation               ← quaternion (x, y, z, w)
                                   convert to yaw: atan2(2(wz+xy), 1-2(y²+z²))

    covariance[36]              ← 6×6 flat matrix
      [0]  variance in X
      [7]  variance in Y
      [35] variance in yaw
      (if these are large the localisation is uncertain — worth checking)
```

---

## Zone states

| State   | Condition                                | Action                              |
|---------|------------------------------------------|-------------------------------------|
| SAFE    | Inside polygon, > warning_distance away  | No override — Nav2 drives normally  |
| WARNING | Inside polygon, ≤ warning_distance away  | Scale speed down + steer inward     |
| BREACH  | Outside polygon                          | Full override — drive back inside   |

---

## How to get your polygon coordinates

The easiest way is to use RViz while Nav2 is running:

```bash
# In RViz, add a tool:  Panels → Tool Properties → Add → Publish Point
# Then click corners of your desired zone on the map.
# Echo the topic to read coordinates:
ros2 topic echo /clicked_point
```

Copy the `x` and `y` values into `config/geofence_params.yaml` as a flat list.

---

## Build & run

```bash
# 1. Clone into your workspace
cd ~/ros2_ws/src
# (copy this package here)

# 2. Build
cd ~/ros2_ws
colcon build --packages-select turtlebot4_geofence
source install/setup.bash

# 3. On the robot: start localization + Nav2
#    (run on the Pi or via SSH)
ros2 launch turtlebot4_navigation localization.launch.py map:=your_map.yaml
ros2 launch turtlebot4_navigation nav2.launch.py

# 4. On your laptop: set domain ID to match robot (default 0)
export ROS_DOMAIN_ID=0

# 5. Launch the geofence node
ros2 launch turtlebot4_geofence geofence.launch.py

# Or with a custom params file:
ros2 launch turtlebot4_geofence geofence.launch.py \
  params_file:=/path/to/my_zone.yaml
```

---

## Visualise in RViz

Add a **MarkerArray** display, topic: `/geofence/boundary`

- Green line  = SAFE
- Orange line = WARNING zone active
- Red line    = BREACH

---

## Tuning tips

| Parameter         | Effect                                                        |
|-------------------|---------------------------------------------------------------|
| `warning_distance`| Larger → earlier slow-down. Start at 0.5 m.                 |
| `max_linear_speed`| Cap on corrective forward speed. Keep ≤ Nav2 max speed.     |
| `correction_gain` | Higher = harder push-back. Too high → oscillation at edge.   |
| `publish_rate`    | 10 Hz is fine. Match to your AMCL update rate.               |

---

## Notes

- The node publishes to `/cmd_vel` **only** in WARNING or BREACH state.  
  In SAFE state it stays silent — Nav2 drives as normal.
- If you use `twist_mux`, set the geofence source to a **higher priority**
  than Nav2 so it can override navigation commands.
- The polygon is defined in the **map frame**.  
  Make sure your map's origin matches what Nav2 is using.
