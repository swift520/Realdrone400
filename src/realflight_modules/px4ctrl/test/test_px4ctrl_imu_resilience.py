#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from mavros_msgs.msg import AttitudeTarget, ExtendedState, State
from mavros_msgs.srv import (CommandBool, CommandBoolResponse, CommandLong,
                             CommandLongResponse, SetMode, SetModeResponse)
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TakeoffLand
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


class Px4ctrlImuResilienceTest(unittest.TestCase):
    IMU_TIMEOUT = 0.25
    PREFIX = "/test_imu"

    def setUp(self):
        self._lock = threading.Lock()
        self._running = True
        self._mode = "STABILIZED"
        self._armed = False
        self._landed_state = ExtendedState.LANDED_STATE_ON_GROUND

        self._imu_mode = "normal"
        self._imu_fixed_stamp = rospy.Time()
        self._imu_pause_ack = threading.Event()
        self._imu_frozen_publish = threading.Event()
        self._last_advancing_imu_stamp = rospy.Time()
        self._last_advancing_imu_publish_time = 0.0
        self._first_frozen_publish_time = 0.0

        self._setpoints = []
        self._mode_requests = []
        self._arm_requests = []
        self._logs = []

        prefix = self.PREFIX
        self._state_pub = rospy.Publisher(prefix + "/mavros/state", State,
                                          queue_size=1, latch=True)
        self._extended_pub = rospy.Publisher(
            prefix + "/mavros/extended_state", ExtendedState,
            queue_size=1, latch=True)
        self._odom_pub = rospy.Publisher(
            prefix + "/localization/validated_odom", Odometry, queue_size=1)
        self._imu_pub = rospy.Publisher(prefix + "/mavros/imu", Imu,
                                        queue_size=10)
        self._health_pub = rospy.Publisher(
            prefix + "/localization/healthy", Bool, queue_size=1, latch=True)
        self._takeoff_pub = rospy.Publisher(prefix + "/takeoff_land",
                                            TakeoffLand, queue_size=1)
        self._setpoint_sub = rospy.Subscriber(
            prefix + "/mavros/setpoint_raw/attitude", AttitudeTarget,
            self._setpoint_callback, queue_size=200)
        self._rosout_sub = rospy.Subscriber("/rosout", Log,
                                            self._rosout_callback,
                                            queue_size=300)

        self._mode_service = rospy.Service(prefix + "/mavros/set_mode",
                                           SetMode, self._set_mode)
        self._arm_service = rospy.Service(prefix + "/mavros/cmd/arming",
                                          CommandBool, self._set_arm)
        self._command_service = rospy.Service(prefix + "/mavros/cmd/command",
                                              CommandLong,
                                              self._command_long)

        self._publisher_thread = threading.Thread(target=self._publish_inputs)
        self._publisher_thread.daemon = True
        self._publisher_thread.start()
        self._wait_for_connections()

    def tearDown(self):
        self._running = False
        self._publisher_thread.join(timeout=1.0)

    @staticmethod
    def _imu_message(stamp):
        msg = Imu()
        msg.header.stamp = stamp
        msg.orientation.w = 1.0
        msg.linear_acceleration.z = 9.81
        return msg

    def _wait_for_connections(self):
        deadline = time.monotonic() + 10.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if (self._state_pub.get_num_connections() and
                    self._extended_pub.get_num_connections() and
                    self._odom_pub.get_num_connections() and
                    self._imu_pub.get_num_connections() and
                    self._health_pub.get_num_connections() and
                    self._takeoff_pub.get_num_connections() and
                    self._setpoint_sub.get_num_connections()):
                return
            rospy.sleep(0.02)
        self.fail("px4ctrl IMU-resilience test connections were not established")

    def _publish_inputs(self):
        rate = rospy.Rate(100)
        while self._running and not rospy.is_shutdown():
            now = rospy.Time.now()
            with self._lock:
                mode = self._mode
                armed = self._armed
                landed_state = self._landed_state
                imu_mode = self._imu_mode
                fixed_stamp = self._imu_fixed_stamp

            state = State()
            state.header.stamp = now
            state.connected = True
            state.mode = mode
            state.armed = armed
            state.system_status = 4 if armed else 3
            self._state_pub.publish(state)

            extended = ExtendedState()
            extended.header.stamp = now
            extended.landed_state = landed_state
            self._extended_pub.publish(extended)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "world"
            odom.child_frame_id = "body"
            odom.pose.pose.orientation.w = 1.0
            self._odom_pub.publish(odom)

            self._health_pub.publish(Bool(data=True))

            if imu_mode == "paused":
                self._imu_pause_ack.set()
            else:
                imu_stamp = now if imu_mode == "normal" else fixed_stamp
                publish_time = time.monotonic()
                self._imu_pub.publish(self._imu_message(imu_stamp))
                with self._lock:
                    if imu_mode == "normal":
                        self._last_advancing_imu_stamp = imu_stamp
                        self._last_advancing_imu_publish_time = publish_time
                    elif self._first_frozen_publish_time == 0.0:
                        self._first_frozen_publish_time = publish_time
                        self._imu_frozen_publish.set()

            rate.sleep()

    def _setpoint_callback(self, msg):
        with self._lock:
            self._setpoints.append((time.monotonic(), msg))

    def _rosout_callback(self, msg):
        if msg.name == "/px4ctrl":
            with self._lock:
                self._logs.append((time.monotonic(), msg.level, msg.msg))

    def _set_mode(self, request):
        with self._lock:
            self._mode_requests.append(
                (time.monotonic(), request.custom_mode))
        # A service acknowledgement is deliberately separate from state
        # feedback, as it is on a real MAVROS connection.
        return SetModeResponse(mode_sent=True)

    def _set_arm(self, request):
        with self._lock:
            self._arm_requests.append((time.monotonic(), request.value))
        return CommandBoolResponse(success=True, result=0)

    @staticmethod
    def _command_long(_request):
        return CommandLongResponse(success=True, result=0)

    def _snapshot(self):
        with self._lock:
            return (list(self._setpoints), list(self._mode_requests),
                    list(self._arm_requests), list(self._logs))

    def _set_vehicle_state(self, mode, armed):
        with self._lock:
            self._mode = mode
            self._armed = armed

    def _set_landed_state(self, landed_state):
        with self._lock:
            self._landed_state = landed_state

    def _publish_takeoff(self):
        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.TAKEOFF
        self._takeoff_pub.publish(msg)

    def _wait_until(self, predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail(message)

    def _pause_imu_stream(self):
        self._imu_pause_ack.clear()
        with self._lock:
            self._imu_mode = "paused"
        self.assertTrue(self._imu_pause_ack.wait(timeout=0.5),
                        "IMU publisher did not acknowledge pause")
        # Drain the one message that may already have been queued when the
        # publisher observed the requested state change.
        rospy.sleep(0.03)
        with self._lock:
            return (self._last_advancing_imu_stamp,
                    self._last_advancing_imu_publish_time)

    def _resume_imu_stream(self):
        with self._lock:
            self._imu_mode = "normal"
        self._imu_pause_ack.clear()

    def _freeze_imu_source_stamp(self):
        source_stamp, last_publish_time = self._pause_imu_stream()
        self.assertNotEqual(source_stamp, rospy.Time(),
                         "no advancing IMU source stamp was available to freeze")
        self._imu_frozen_publish.clear()
        with self._lock:
            self._imu_fixed_stamp = source_stamp
            self._first_frozen_publish_time = 0.0
            self._imu_mode = "frozen"
        self.assertTrue(self._imu_frozen_publish.wait(timeout=0.5),
                        "frozen IMU publisher did not resume")
        with self._lock:
            first_frozen_publish = self._first_frozen_publish_time
        return last_publish_time, first_frozen_publish

    def _has_rollback_warning(self, since):
        for stamp, level, message in self._snapshot()[3]:
            text = message.lower()
            if (stamp >= since and level == Log.WARN and
                    "fcu imu" in text and "out-of-order" in text and
                    "ignored" in text and "retaining" in text):
                return True
        return False

    def _has_failsafe_reason(self, reason, since):
        return any(stamp >= since and level >= Log.ERROR and
                   "Enter FAILSAFE_EXIT" in message and reason in message
                   for stamp, level, message in self._snapshot()[3])

    def _enter_active_offboard(self):
        # Subscribers are advertised before px4ctrl's final 0.5 s startup
        # sleep.  Connection alone therefore does not mean the main loop has
        # begun consuming the health heartbeat and qualifying its 0.2 s
        # recovery window.
        rospy.sleep(0.85)
        entry_start = time.monotonic()
        initial_setpoint_count = len(self._snapshot()[0])
        self._publish_takeoff()

        self._wait_until(
            lambda: len(self._snapshot()[0]) > initial_setpoint_count,
            0.5, "takeoff did not start OFFBOARD prestream")
        self._wait_until(
            lambda: any(stamp >= entry_start and mode == "OFFBOARD"
                        for stamp, mode in self._snapshot()[1]),
            1.6, "OFFBOARD was not requested after prestream")

        offboard_feedback_time = time.monotonic()
        self._set_vehicle_state("OFFBOARD", False)
        self._wait_until(
            lambda: any(stamp >= offboard_feedback_time and armed
                        for stamp, armed in self._snapshot()[2]),
            0.5, "arming was not requested after OFFBOARD confirmation")

        self._set_landed_state(ExtendedState.LANDED_STATE_IN_AIR)
        armed_feedback_time = time.monotonic()
        self._set_vehicle_state("OFFBOARD", True)
        self._wait_until(
            lambda: any(stamp >= armed_feedback_time and msg.thrust > 0.05
                        for stamp, msg in self._snapshot()[0]),
            0.5, "active OFFBOARD control did not start after armed feedback")

    def test_imu_source_stamp_resilience(self):
        self._enter_active_offboard()

        # One out-of-order sample is rejected, but the last accepted attitude
        # remains usable until a fresh, advancing sample arrives. Pause the
        # automatic stream so exactly one regressed frame is observable for
        # several 100 Hz control cycles.
        continuity_start = time.monotonic()
        last_stamp, _ = self._pause_imu_stream()
        rollback_stamp = last_stamp - rospy.Duration.from_sec(0.002)
        rollback_publish_time = time.monotonic()
        self._imu_pub.publish(self._imu_message(rollback_stamp))
        rospy.sleep(0.06)

        recovery_stamp = rospy.Time.now()
        self.assertGreater(recovery_stamp, last_stamp)
        self._imu_pub.publish(self._imu_message(recovery_stamp))
        self._resume_imu_stream()

        self._wait_until(
            lambda: self._has_rollback_warning(rollback_publish_time),
            0.5, "out-of-order IMU sample did not emit the expected warning")
        rospy.sleep(self.IMU_TIMEOUT + 0.10)
        continuity_end = time.monotonic()

        setpoints, modes, _, logs = self._snapshot()
        self.assertFalse(
            any(stamp >= continuity_start and mode != "OFFBOARD"
                for stamp, mode in modes),
            "one out-of-order IMU sample requested an OFFBOARD exit")
        self.assertFalse(
            any(stamp >= continuity_start and
                "Enter FAILSAFE_EXIT" in message
                for stamp, _, message in logs),
            "one out-of-order IMU sample entered FAILSAFE_EXIT")

        continuous_setpoints = [
            stamp for stamp, _ in setpoints
            if continuity_start <= stamp <= continuity_end
        ]
        self.assertGreaterEqual(
            len(continuous_setpoints), 15,
            "too few setpoints were published across isolated IMU rollback")
        self.assertLessEqual(
            max(later - earlier for earlier, later in
                zip(continuous_setpoints, continuous_setpoints[1:])),
            0.10, "setpoint stream was interrupted by isolated IMU rollback")
        self.assertGreaterEqual(
            continuous_setpoints[-1], continuity_end - 0.05,
            "setpoints did not continue after the advancing IMU sample recovered")

        # Messages continue to arrive at 100 Hz, but their source timestamp no
        # longer advances. This must not look like a transport heartbeat loss:
        # control exits only after the last accepted sample reaches imu timeout.
        last_valid_publish, frozen_start = self._freeze_imu_source_stamp()
        early_check_time = (
            last_valid_publish + self.IMU_TIMEOUT - 0.06)
        if time.monotonic() < early_check_time:
            rospy.sleep(early_check_time - time.monotonic())
        self.assertFalse(
            any(stamp >= frozen_start and mode == "AUTO.LAND"
                for stamp, mode in self._snapshot()[1]),
            "repeated IMU source stamp triggered FAILSAFE before its timeout")

        latest_expected_request = (
            last_valid_publish + self.IMU_TIMEOUT + 0.20)
        self._wait_until(
            lambda: any(stamp >= frozen_start and mode == "AUTO.LAND"
                        for stamp, mode in self._snapshot()[1]),
            max(0.05, latest_expected_request - time.monotonic()),
            "stalled IMU source timestamp did not request AUTO.LAND")
        request_time = next(
            stamp for stamp, mode in self._snapshot()[1]
            if stamp >= frozen_start and mode == "AUTO.LAND")
        self.assertGreaterEqual(
            request_time, last_valid_publish + self.IMU_TIMEOUT - 0.07,
            "stalled IMU source timestamp failed before the configured timeout")
        self.assertLessEqual(
            request_time, latest_expected_request,
            "stalled IMU source timestamp exceeded its bounded detection time")

        expected_reason = "FCU IMU source timestamp stopped advancing"
        self._wait_until(
            lambda: self._has_failsafe_reason(expected_reason, frozen_start),
            0.5, "FAILSAFE log did not identify the stalled IMU source stamp")
        failsafe_logs = [
            message for stamp, level, message in self._snapshot()[3]
            if stamp >= frozen_start and level >= Log.ERROR and
            "Enter FAILSAFE_EXIT" in message
        ]
        self.assertTrue(failsafe_logs,
                        "stalled source timestamp produced no FAILSAFE log")
        for message in failsafe_logs:
            self.assertIn(expected_reason, message)
            self.assertNotIn("heartbeat timed out", message)
            self.assertNotIn("non-finite", message)
            self.assertNotIn("invalid quaternion", message)

        # Complete the mocked handover before the test process tears its
        # services down, avoiding an unrelated retry/rejection during shutdown.
        handover_start = time.monotonic()
        self._set_vehicle_state("AUTO.LAND", True)
        self._wait_until(
            lambda: any(stamp >= handover_start and
                        "PX4 confirmed mode 'AUTO.LAND'" in message
                        for stamp, _, message in self._snapshot()[3]),
            0.5, "PX4 mode feedback did not complete the mocked handover")


if __name__ == "__main__":
    rospy.init_node("test_px4ctrl_imu_resilience")
    rostest.rosrun("px4ctrl", "px4ctrl_imu_resilience",
                   Px4ctrlImuResilienceTest)
