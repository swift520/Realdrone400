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


class Px4ctrlNormalLandingExitTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._running = True
        self._health = True
        self._mode = "STABILIZED"
        self._armed = False
        self._landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        self._odom_z = 0.0
        self._setpoints = []
        self._mode_requests = []
        self._arm_requests = []
        self._logs = []

        prefix = "/test_landing"
        self._state_pub = rospy.Publisher(prefix + "/mavros/state", State,
                                          queue_size=1, latch=True)
        self._extended_pub = rospy.Publisher(
            prefix + "/mavros/extended_state", ExtendedState,
            queue_size=1, latch=True)
        self._odom_pub = rospy.Publisher(
            prefix + "/localization/validated_odom", Odometry, queue_size=1)
        self._imu_pub = rospy.Publisher(prefix + "/mavros/imu", Imu,
                                        queue_size=1)
        self._health_pub = rospy.Publisher(
            prefix + "/localization/healthy", Bool, queue_size=1, latch=True)
        self._takeoff_pub = rospy.Publisher(prefix + "/takeoff_land",
                                            TakeoffLand, queue_size=1)
        self._setpoint_sub = rospy.Subscriber(
            prefix + "/mavros/setpoint_raw/attitude", AttitudeTarget,
            self._setpoint_callback, queue_size=200)
        self._rosout_sub = rospy.Subscriber("/rosout", Log,
                                            self._rosout_callback,
                                            queue_size=200)

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
        self.fail("px4ctrl normal-landing test connections were not established")

    def _publish_inputs(self):
        rate = rospy.Rate(100)
        while self._running and not rospy.is_shutdown():
            now = rospy.Time.now()
            with self._lock:
                mode = self._mode
                armed = self._armed
                landed_state = self._landed_state
                odom_z = self._odom_z
                health = self._health

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
            odom.pose.pose.position.z = odom_z
            odom.pose.pose.orientation.w = 1.0
            self._odom_pub.publish(odom)

            imu = Imu()
            imu.header.stamp = now
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = 9.81
            self._imu_pub.publish(imu)

            self._health_pub.publish(Bool(data=health))
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
        # Service acceptance is intentionally independent of state feedback.
        return SetModeResponse(mode_sent=True)

    def _set_arm(self, request):
        with self._lock:
            self._arm_requests.append((time.monotonic(), request.value))
        return CommandBoolResponse(success=True, result=0)

    @staticmethod
    def _command_long(_request):
        return CommandLongResponse(success=True, result=0)

    def _set_vehicle_state(self, mode, armed):
        with self._lock:
            self._mode = mode
            self._armed = armed

    def _set_landed_state(self, value):
        with self._lock:
            self._landed_state = value

    def _set_odom_z(self, value):
        with self._lock:
            self._odom_z = value

    def _publish_takeoff_land(self, command):
        msg = TakeoffLand()
        msg.takeoff_land_cmd = command
        self._takeoff_pub.publish(msg)

    def _snapshot(self):
        with self._lock:
            return (list(self._setpoints), list(self._mode_requests),
                    list(self._arm_requests), list(self._logs))

    def _wait_until(self, predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail(message)

    def _has_log(self, text, since=0.0):
        return any(stamp >= since and text in message
                   for stamp, _, message in self._snapshot()[3])

    def test_normal_landing_stops_setpoints_without_failsafe(self):
        # px4ctrl intentionally sleeps during startup; allow its main spin loop
        # to observe a complete health-recovery interval before the command.
        rospy.sleep(0.75)

        self._publish_takeoff_land(TakeoffLand.TAKEOFF)
        self._wait_until(lambda: len(self._snapshot()[0]) > 0, 0.5,
                         "takeoff did not start OFFBOARD prestream")
        self._wait_until(
            lambda: any(mode == "OFFBOARD"
                        for _, mode in self._snapshot()[1]),
            1.6, "OFFBOARD was not requested")

        self._set_vehicle_state("OFFBOARD", False)
        self._wait_until(
            lambda: any(value for _, value in self._snapshot()[2]),
            0.5, "arming was not requested")

        self._set_landed_state(ExtendedState.LANDED_STATE_IN_AIR)
        self._set_vehicle_state("OFFBOARD", True)
        rospy.sleep(0.10)
        self._set_odom_z(0.60)
        rospy.sleep(3.25)

        landing_start = time.monotonic()
        self._publish_takeoff_land(TakeoffLand.LAND)
        # At 0.3 m/s the desired point needs about 1.67 s to move 0.5 m
        # below the fixed odometry, followed by the detector's 3.0 s dwell.
        rospy.sleep(5.10)
        self.assertFalse(
            any(stamp >= landing_start and not value
                for stamp, value in self._snapshot()[2]),
            "vehicle was disarmed before PX4 reported ON_GROUND")

        self._set_landed_state(ExtendedState.LANDED_STATE_ON_GROUND)
        self._wait_until(
            lambda: any(stamp >= landing_start and not value
                        for stamp, value in self._snapshot()[2]),
            0.5, "agreed land detectors did not request disarm")

        normal_exit_start = time.monotonic()
        self._set_vehicle_state("OFFBOARD", False)
        self._wait_until(
            lambda: any(stamp >= normal_exit_start and mode == "STABILIZED"
                        for stamp, mode in self._snapshot()[1]),
            0.5, "normal landing did not request the pre-OFFBOARD mode")
        self._wait_until(
            lambda: self._has_log(
                "Landing complete and PX4 disarmed", normal_exit_start),
            0.5, "normal landing exit did not emit its INFO transition")

        rospy.sleep(0.15)
        setpoints, _, _, logs = self._snapshot()
        self.assertTrue(setpoints)
        self.assertLessEqual(
            setpoints[-1][0], normal_exit_start + 0.10,
            "normal landing continued external setpoints after disarm feedback")
        stopped_count = len(setpoints)

        # Keep publishing fresh OFFBOARD/disarmed state longer than the real
        # failsafe's 0.30 s grace. Normal landing must remain stopped without
        # producing the failsafe timeout seen in flight logs. An input edge
        # received during handover must also be consumed rather than causing
        # an automatic re-entry after mode confirmation.
        self._publish_takeoff_land(TakeoffLand.TAKEOFF)
        rospy.sleep(0.55)
        self.assertEqual(
            len(self._snapshot()[0]), stopped_count,
            "normal landing restarted setpoints while awaiting mode feedback")
        errors = [
            message for stamp, level, message in self._snapshot()[3]
            if stamp >= normal_exit_start and level >= Log.ERROR
        ]
        self.assertFalse(
            any("FAILSAFE_EXIT" in message or
                "did not confirm OFFBOARD exit" in message
                for message in errors),
            "normal landing emitted a failsafe error")

        self._set_vehicle_state("STABILIZED", False)
        rospy.sleep(0.20)
        self.assertEqual(
            len(self._snapshot()[0]), stopped_count,
            "setpoints resumed after normal landing exit confirmation")


if __name__ == "__main__":
    rospy.init_node("test_px4ctrl_normal_landing_exit")
    rostest.rosrun("px4ctrl", "px4ctrl_normal_landing_exit",
                   Px4ctrlNormalLandingExitTest)
