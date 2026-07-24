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
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


class Px4ctrlHealthGateTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._running = True
        self._health = None
        self._mode = "MANUAL"
        self._armed = False
        self._landed_state = ExtendedState.LANDED_STATE_ON_GROUND
        self._accept_offboard_requests = True
        self._setpoints = []
        self._mode_requests = []
        self._arm_requests = []

        self._state_pub = rospy.Publisher("/test/mavros/state", State,
                                          queue_size=1, latch=True)
        self._extended_pub = rospy.Publisher("/test/mavros/extended_state", ExtendedState,
                                             queue_size=1, latch=True)
        self._odom_pub = rospy.Publisher("/test/localization/validated_odom", Odometry,
                                         queue_size=1)
        self._imu_pub = rospy.Publisher("/test/mavros/imu", Imu, queue_size=1)
        self._health_pub = rospy.Publisher("/test/localization/healthy", Bool,
                                           queue_size=1, latch=True)
        self._takeoff_pub = rospy.Publisher("/test/takeoff_land", TakeoffLand, queue_size=1)
        self._setpoint_sub = rospy.Subscriber("/test/mavros/setpoint_raw/attitude",
                                              AttitudeTarget, self._setpoint_callback,
                                              queue_size=200)

        self._mode_service = rospy.Service("/test/mavros/set_mode", SetMode,
                                           self._set_mode)
        self._arm_service = rospy.Service("/test/mavros/cmd/arming", CommandBool,
                                          self._set_arm)
        self._command_service = rospy.Service("/test/mavros/cmd/command", CommandLong,
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
        self.fail("px4ctrl test connections were not established")

    def _publish_inputs(self):
        rate = rospy.Rate(50)
        while self._running and not rospy.is_shutdown():
            now = rospy.Time.now()
            with self._lock:
                mode = self._mode
                armed = self._armed
                health = self._health
                landed_state = self._landed_state

            state = State()
            state.header.stamp = now
            state.connected = True
            state.mode = mode
            state.armed = armed
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

            imu = Imu()
            imu.header.stamp = now
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = 9.81
            self._imu_pub.publish(imu)

            if health is not None:
                self._health_pub.publish(Bool(data=health))
            rate.sleep()

    def _setpoint_callback(self, msg):
        with self._lock:
            self._setpoints.append((time.monotonic(), msg))

    def _set_mode(self, request):
        with self._lock:
            self._mode_requests.append((time.monotonic(), request.custom_mode))
            accepted = (request.custom_mode != "OFFBOARD" or
                        self._accept_offboard_requests)
        # Acceptance is deliberately decoupled from state feedback.
        return SetModeResponse(mode_sent=accepted)

    def _set_arm(self, request):
        with self._lock:
            self._arm_requests.append((time.monotonic(), request.value))
        # The test updates /mavros/state separately to exercise confirmation.
        return CommandBoolResponse(success=True, result=0)

    @staticmethod
    def _command_long(_request):
        return CommandLongResponse(success=True, result=0)

    def _set_vehicle_state(self, mode, armed):
        with self._lock:
            self._mode = mode
            self._armed = armed

    def _set_health(self, value):
        with self._lock:
            self._health = value

    def _set_landed_state(self, value):
        with self._lock:
            self._landed_state = value

    def _set_offboard_request_acceptance(self, accepted):
        with self._lock:
            self._accept_offboard_requests = accepted

    def _publish_takeoff(self):
        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.TAKEOFF
        self._takeoff_pub.publish(msg)

    def _snapshot(self):
        with self._lock:
            return (list(self._setpoints), list(self._mode_requests),
                    list(self._arm_requests))

    def _wait_until(self, predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            rospy.sleep(0.01)
        self.fail(message)

    def test_health_gate_entry_confirmation_and_bounded_exit(self):
        rospy.sleep(0.25)

        # No heartbeat: takeoff is consumed and rejected without prestreaming.
        self._publish_takeoff()
        rospy.sleep(0.25)
        setpoints, modes, arms = self._snapshot()
        self.assertEqual(len(setpoints), 0)
        self.assertEqual(len(modes), 0)
        self.assertEqual(len(arms), 0)

        # A continuous healthy interval permits explicit setpoint prestreaming.
        self._set_health(True)
        rospy.sleep(0.35)

        # An external/early OFFBOARD transition must not be accepted as this
        # controller's own confirmation before its one-second prestream and
        # mode request. It is treated as a fault and handed back to PX4.
        early_setpoint_count = len(self._snapshot()[0])
        self._publish_takeoff()
        self._wait_until(lambda: len(self._snapshot()[0]) > early_setpoint_count,
                         0.5, "early-entry scenario did not start prestream")
        rospy.sleep(0.10)
        early_fault_time = time.monotonic()
        self._set_vehicle_state("OFFBOARD", False)
        self._wait_until(
            lambda: any(t >= early_fault_time and mode == "AUTO.LAND"
                        for t, mode in self._snapshot()[1]),
            0.5, "unrequested OFFBOARD entry was incorrectly accepted")
        self.assertEqual(len(self._snapshot()[2]), 0,
                         "unrequested OFFBOARD entry attempted to arm")
        self._set_vehicle_state("MANUAL", False)
        rospy.sleep(0.30)

        # A rejected SetMode response is an attempt, not authorization. Even
        # if state later changes to OFFBOARD, px4ctrl must not arm or activate
        # from a request that MAVROS/PX4 explicitly rejected.
        self._set_offboard_request_acceptance(False)
        rejected_setpoint_count = len(self._snapshot()[0])
        rejected_entry_start = time.monotonic()
        self._publish_takeoff()
        self._wait_until(lambda: len(self._snapshot()[0]) > rejected_setpoint_count,
                         0.5, "rejected-request scenario did not start prestream")
        self._wait_until(
            lambda: any(t >= rejected_entry_start and mode == "OFFBOARD"
                        for t, mode in self._snapshot()[1]),
            1.6, "rejected-request scenario did not call SetMode")
        rejected_fault_time = time.monotonic()
        self._set_vehicle_state("OFFBOARD", False)
        self._wait_until(
            lambda: any(t >= rejected_fault_time and mode == "AUTO.LAND"
                        for t, mode in self._snapshot()[1]),
            0.5, "rejected OFFBOARD request was incorrectly treated as confirmed")
        self.assertEqual(len(self._snapshot()[2]), 0,
                         "rejected OFFBOARD request attempted to arm")
        self._set_vehicle_state("MANUAL", False)
        self._set_offboard_request_acceptance(True)
        rospy.sleep(0.30)

        # A new explicit command now starts the normal entry sequence.
        normal_setpoint_count = len(self._snapshot()[0])
        self._publish_takeoff()
        prestream_start = time.monotonic()
        self._wait_until(lambda: len(self._snapshot()[0]) > normal_setpoint_count, 0.5,
                         "healthy takeoff did not start OFFBOARD prestream")
        rospy.sleep(0.65)
        self.assertFalse(any(t >= prestream_start and mode == "OFFBOARD"
                             for t, mode in self._snapshot()[1]),
                         "OFFBOARD was requested before one second of prestream")

        self._wait_until(
            lambda: any(t >= prestream_start and mode == "OFFBOARD"
                        for t, mode in self._snapshot()[1]),
            max(0.5, 1.6 - (time.monotonic() - prestream_start)),
            "OFFBOARD was not requested after prestream")
        self.assertEqual(len(self._snapshot()[2]), 0,
                         "arming was attempted before /mavros/state confirmed OFFBOARD")

        # mode_sent=true is only an acknowledgement. Arming starts after fresh
        # state feedback says OFFBOARD, and takeoff starts after armed feedback.
        self._set_vehicle_state("OFFBOARD", False)
        self._wait_until(lambda: any(value for _, value in self._snapshot()[2]),
                         0.5, "arming was not requested after OFFBOARD confirmation")

        # PX4's land detector can leave ON_GROUND as soon as it accepts ARM,
        # before the lower-rate /mavros/state heartbeat reports armed=true.
        # This transient must not abort entry, and the preparation stream must
        # remain at a zero-thrust request throughout the feedback gap.
        in_air_time = time.monotonic()
        setpoint_count_before_arm_feedback = len(self._snapshot()[0])
        self._set_landed_state(ExtendedState.LANDED_STATE_IN_AIR)
        self._wait_until(
            lambda: len(self._snapshot()[0]) >=
            setpoint_count_before_arm_feedback + 3,
            0.4, "setpoints stopped during the arming feedback gap")
        rospy.sleep(0.10)
        setpoints, modes, _ = self._snapshot()
        pending_arm_setpoints = [
            msg for stamp, msg in setpoints if stamp >= in_air_time
        ]
        self.assertTrue(pending_arm_setpoints,
                        "no setpoints were captured during arming confirmation")
        self.assertLessEqual(
            max(abs(msg.thrust) for msg in pending_arm_setpoints), 1.0e-6,
            "takeoff preparation published nonzero thrust before armed feedback")
        self.assertFalse(
            any(stamp >= in_air_time and mode == "AUTO.LAND"
                for stamp, mode in modes),
            "PX4's expected post-ARM landed-state transition aborted entry")

        armed_feedback_time = time.monotonic()
        self._set_vehicle_state("OFFBOARD", True)
        self._wait_until(
            lambda: any(stamp >= armed_feedback_time and msg.thrust > 0.05
                        for stamp, msg in self._snapshot()[0]),
            0.4, "takeoff did not start after armed feedback")
        rospy.sleep(0.10)
        self.assertFalse(
            any(stamp >= armed_feedback_time and mode == "AUTO.LAND"
                for stamp, mode in self._snapshot()[1]),
            "IN_AIR landed state aborted entry when armed feedback arrived")

        # Simulate a bridge crash: leave its final latched true in ROS but stop
        # heartbeat publication. The steady-time timeout must enter failsafe.
        health_fault_time = time.monotonic()
        self._set_health(None)
        self._wait_until(
            lambda: any(t >= health_fault_time and mode == "AUTO.LAND"
                        for t, mode in self._snapshot()[1]),
            0.7, "stale localization heartbeat did not request failsafe mode")
        fault_request_time = next(t for t, mode in self._snapshot()[1]
                                  if t >= health_fault_time and mode == "AUTO.LAND")

        rospy.sleep(0.55)
        setpoints_after_grace = self._snapshot()[0]
        self.assertTrue(setpoints_after_grace)
        self.assertLessEqual(setpoints_after_grace[-1][0], fault_request_time + 0.42,
                             "setpoints continued beyond the bounded exit grace")
        stopped_count = len(setpoints_after_grace)
        rospy.sleep(0.20)
        self.assertEqual(len(self._snapshot()[0]), stopped_count,
                         "setpoints restarted while PX4 still reported OFFBOARD")

        # A post-request state update confirms the handover. Health recovery
        # while still armed must not automatically regain OFFBOARD control.
        self._set_vehicle_state("AUTO.LAND", True)
        rospy.sleep(0.15)
        self._set_health(True)
        rospy.sleep(0.35)
        self.assertEqual(len(self._snapshot()[0]), stopped_count,
                         "armed health recovery automatically resumed control")

        # Disarm + a healthy interval clears the local latch. A new explicit
        # takeoff command is then required and starts a fresh prestream.
        self._set_vehicle_state("MANUAL", False)
        self._set_landed_state(ExtendedState.LANDED_STATE_ON_GROUND)
        rospy.sleep(0.30)
        reentry_start = time.monotonic()
        self._publish_takeoff()
        self._wait_until(lambda: len(self._snapshot()[0]) > stopped_count,
                         0.5, "interlock did not re-arm after disarm and recovery")

        # Complete one more entry and then report an unexpected disarm while
        # PX4 still says OFFBOARD. The controller must hand back to the
        # pre-entry mode instead of leaving stale thrust ready for a re-arm.
        self._wait_until(
            lambda: any(t >= reentry_start and mode == "OFFBOARD"
                        for t, mode in self._snapshot()[1]),
            1.6, "second OFFBOARD request was not sent")
        arm_phase_start = time.monotonic()
        self._set_vehicle_state("OFFBOARD", False)
        self._wait_until(
            lambda: any(t >= arm_phase_start and value
                        for t, value in self._snapshot()[2]),
            0.5, "second entry did not request arming")
        self._set_vehicle_state("OFFBOARD", True)
        rospy.sleep(0.15)
        unexpected_disarm_time = time.monotonic()
        self._set_vehicle_state("OFFBOARD", False)
        self._wait_until(
            lambda: any(t >= unexpected_disarm_time and mode == "MANUAL"
                        for t, mode in self._snapshot()[1]),
            0.5, "unexpected active-state disarm did not trigger mode handback")


if __name__ == "__main__":
    rospy.init_node("test_px4ctrl_health_gate")
    rostest.rosrun("px4ctrl", "px4ctrl_health_gate", Px4ctrlHealthGateTest)
