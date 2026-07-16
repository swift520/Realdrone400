# real_drone_bringup

This package contains the two hardware bringup functions migrated from the
`LIO-Drone-250` workspace for use by `REAL_DRONE_400`.

- `takeoff_px4.launch` starts MAVROS on `/dev/ttyTHS0:921600`. Despite its
  historical name, it does not arm the vehicle or send a takeoff command.
- `takeoff_vrpn.launch` keeps its historical name for script compatibility,
  but it does not use VRPN. It forwards fresh FAST-LIO high-rate poses to
  MAVROS and independently requires fresh LiDAR-corrected mapping odometry.

## External-vision safety gate

The bridge uses two FAST-LIO topics for different purposes:

- `/Odom_high_freq`: high-rate IMU-propagated pose used as the output data.
- `/Odometry`: LiDAR-corrected mapping pose used only as a correction-health
  watchdog.

The node publishes each high-rate measurement at most once, preserves its
measurement timestamp, and limits output to 50 Hz by default. It never creates
new timestamps for a cached pose. A separate 100 Hz safety timer checks both
streams, so lowering the output rate does not weaken timeout detection.

Both input timestamps must advance and both streams must remain fresh. The
bridge also checks finite values, quaternion norms, expected frame IDs,
timestamp epoch/skew, per-sample pose continuity, recovery continuity, and
agreement between a fresh high-rate pose and each LiDAR-corrected pose. A
failure stops `/mavros/vision_pose/pose` immediately, publishes unhealthy, and
latches the fault. The publisher queue is one message to minimize data already
in flight when a fault is detected.

The default input frame contract matches this FAST-LIO fork:

- high-rate: `world -> odom_imu`
- LiDAR-corrected: `camera_init -> body`

Although the frame names differ, the producer currently fills both messages
from the same FAST-LIO state coordinates. The frame and cross-stream checks are
intended to reject an accidental remap to an unrelated odometry topic.

Health is published as a latched `std_msgs/Bool` on
`/localization/healthy`. A fault does not automatically recover when data
returns. Once the cause is understood and both streams are healthy, request a
manual recovery check with:

```bash
rosservice call /vision_pose_node/reset_fault
```

This bridge does not arm, change flight mode, or issue a land command. Stopping
fresh external-vision measurements lets PX4 EKF detect aiding loss; the
aircraft's subsequent position-mode/nav failsafe action is determined by its
PX4 estimator and commander parameters and must be verified on the actual FCU.

The service starts a validation interval; it does not immediately resume PX4
output. Every high-rate recovery pose sample is checked against the last
trusted pose and the previous recovery sample. If an input timestamp moves
backwards after trusted output has begun, online reset is permanently disabled
for that bridge process: land/disarm and restart the localization
bridge/FAST-LIO chain so PX4 can never receive a backwards time epoch. During
initial startup, before anything has reached PX4, a backwards timestamp instead
restarts the validation sequence. The same landed restart procedure applies if
FAST-LIO starts with a new origin.

Default watchdog values assume approximately 50+ Hz high-rate odometry and
10 Hz mapping odometry:

- high-rate timeout: `0.15 s`
- LiDAR-correction timeout: `0.50 s`
- continuous recovery validation: `1.0 s`
- maximum high-rate/corrected position disagreement: `1.0 m`

Tune these only after measuring worst-case timing under real onboard load.
FAST-LIO source stamps must use the same time base as the ROS host clock;
otherwise the age check intentionally rejects them.

The FAST-LIO high-rate predictor also rejects non-positive IMU intervals and
intervals above `0.1 s`, and re-seeds its trapezoidal integration after every
LiDAR correction. A rejected predictor interval stops `/Odom_high_freq` until
the next corrected state instead of publishing a dubious extrapolation. If the
gap exceeds the bridge's `0.15 s` high-rate timeout, the bridge latches it as a
localization fault; a shorter gap can be safely re-anchored by the next LiDAR
correction without sending a fabricated sample.

## MID360-to-airframe lever arm

FAST-LIO reports the pose of the MID360 internal IMU, not the flight-controller
body origin expected by PX4. This aircraft has the MID360 approximately 0.10 m
directly above the FCU, with the forward/up axes aligned. The launch defaults
therefore set the body-origin-to-sensor vector, in aligned ROS body axes, to:

```text
body_to_sensor_x = 0.0    # forward, metres
body_to_sensor_y = 0.0    # left, metres
body_to_sensor_z = 0.10   # up, metres
```

The launch also rejects lever-arm lengths above 0.50 m, which catches a likely
metres-versus-centimetres configuration error before any pose is published.

For every normalized input pose, the bridge computes
`p_world_body = p_world_sensor - R_world_sensor * p_body_sensor`. Both the
high-rate and LiDAR-corrected poses receive the same transformation before
continuity and cross-stream checks, and only the transformed body pose is sent
to MAVROS. The rotation is deliberately not configurable here: this setup
assumes the MID360 and airframe axes are physically aligned. Correct the mount
or extend/calibrate the full rigid transform before use if that assumption is
not true.

These parameters are separate from FAST-LIO's `extrinsic_T`/`extrinsic_R`,
which describe the LiDAR relative to the MID360's own IMU. Do not put the 0.10 m
airframe offset in the FAST-LIO config. The same lever arm must also not be
configured a second time in PX4 `EKF2_EV_POS_X/Y/Z`; this deployment performs
the compensation in the ROS bridge and leaves those PX4 values at zero.

`output_frame_id=map` preserves the previous MAVROS interface. Changing that
label does not rotate the FAST-LIO world frame; MAVROS still performs its normal
ROS ENU/FLU to PX4 NED/FRD conversion.

Do not run either launch file alongside another MAVROS instance or another
node publishing the same external-vision pose topic.
