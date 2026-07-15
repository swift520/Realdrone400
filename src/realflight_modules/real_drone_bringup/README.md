# real_drone_bringup

This package contains the two hardware bringup functions migrated from the
`LIO-Drone-250` workspace for use by `REAL_DRONE_400`.

- `takeoff_px4.launch` starts MAVROS on `/dev/ttyTHS0:921600`. Despite its
  historical name, it does not arm the vehicle or send a takeoff command.
- `takeoff_vrpn.launch` keeps its historical name for script compatibility,
  but the active implementation does not use VRPN. It forwards FAST-LIO
  `/Odometry` poses to `/mavros/vision_pose/pose` at 30 Hz and sets the output
  frame to `map`.

Do not run either launch file alongside another MAVROS instance or another
node publishing the same external-vision pose topic.
