#!/usr/bin/env python3

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class VisionPoseWatchdogTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._outputs = []
        self._health = None
        self._sent_high_stamps = set()

        self._high_pub = rospy.Publisher("/test/odom_high_rate", Odometry, queue_size=1)
        self._correction_pub = rospy.Publisher("/test/odom_correction", Odometry, queue_size=1)
        self._pose_sub = rospy.Subscriber("/test/vision_pose", PoseStamped, self._pose_callback,
                                          queue_size=100)
        self._health_sub = rospy.Subscriber("/test/localization_healthy", Bool,
                                            self._health_callback, queue_size=10)

        rospy.wait_for_service("/vision_pose_node/reset_fault", timeout=10.0)
        self._reset_fault = rospy.ServiceProxy("/vision_pose_node/reset_fault", Trigger)
        self._wait_for_publishers()

    def _wait_for_publishers(self):
        deadline = time.monotonic() + 10.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if (self._high_pub.get_num_connections() and
                    self._correction_pub.get_num_connections() and
                    self._pose_sub.get_num_connections() and
                    self._health_sub.get_num_connections()):
                return
            rospy.sleep(0.02)
        self.fail("test and bridge topic connections were not all established")

    def _pose_callback(self, msg):
        with self._lock:
            self._outputs.append((time.monotonic(), msg))

    def _health_callback(self, msg):
        with self._lock:
            self._health = msg.data

    def _output_count(self):
        with self._lock:
            return len(self._outputs)

    def _latest_output(self):
        with self._lock:
            return self._outputs[-1][1] if self._outputs else None

    def _wait_for_output_stamp(self, stamp, timeout=0.15):
        deadline = time.monotonic() + timeout
        stamp_nsec = stamp.to_nsec()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                for _, msg in self._outputs:
                    if msg.header.stamp.to_nsec() == stamp_nsec:
                        return msg
            rospy.sleep(0.005)
        return None

    def _outputs_since(self, index):
        with self._lock:
            return [entry[1] for entry in self._outputs[index:]]

    def _wait_for_output_quiet(self, quiet_duration=0.08, timeout=0.60):
        """Return the output count after callbacks have stopped arriving."""
        deadline = time.monotonic() + timeout
        last_count = self._output_count()
        quiet_since = time.monotonic()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            current_count = self._output_count()
            if current_count != last_count:
                last_count = current_count
                quiet_since = time.monotonic()
            elif time.monotonic() - quiet_since >= quiet_duration:
                return current_count
            rospy.sleep(0.005)
        self.fail("vision output did not become quiet")

    def _is_healthy(self):
        with self._lock:
            return self._health is True

    @staticmethod
    def _make_odom(stamp, x, frame_id, child_frame_id):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.w = 1.0
        return msg

    def _publish_for(self, duration, publish_high=True, publish_correction=True,
                     fixed_high_stamp=None, fixed_correction_stamp=None,
                     high_x=1.25, correction_x=None):
        if correction_x is None:
            correction_x = high_x + 0.25
        end = time.monotonic() + duration
        next_high = 0.0
        next_correction = 0.0
        while not rospy.is_shutdown() and time.monotonic() < end:
            now_monotonic = time.monotonic()
            if publish_high and now_monotonic >= next_high:
                stamp = fixed_high_stamp if fixed_high_stamp is not None else rospy.Time.now()
                self._high_pub.publish(self._make_odom(stamp, high_x, "world", "odom_imu"))
                self._sent_high_stamps.add(stamp.to_nsec())
                next_high = now_monotonic + 0.02
            if publish_correction and now_monotonic >= next_correction:
                stamp = (fixed_correction_stamp if fixed_correction_stamp is not None
                         else rospy.Time.now())
                self._correction_pub.publish(
                    self._make_odom(stamp, correction_x, "camera_init", "body"))
                next_correction = now_monotonic + 0.10
            rospy.sleep(0.005)

    def _wait_for_health(self, expected, timeout=2.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if self._health is expected:
                    return True
            rospy.sleep(0.01)
        return False

    def _recover(self, high_x=1.25):
        self._publish_for(0.12, True, True, high_x=high_x)
        response = self._reset_fault()
        self.assertTrue(response.success, response.message)
        self._publish_for(0.60, True, True, high_x=high_x)
        self.assertTrue(self._wait_for_health(True), "bridge did not become healthy after reset")

    def test_dual_watchdog_and_latched_recovery(self):
        self.assertTrue(self._wait_for_health(False), "initial health was not false")
        self.assertEqual(self._output_count(), 0)

        # Both streams are required. Output data must come from the high-rate stream.
        self._publish_for(0.15, True, True)
        self.assertEqual(self._output_count(), 0,
                         "bridge published during startup validation")
        self._publish_for(0.55, True, True)
        self.assertTrue(self._wait_for_health(True), "bridge did not become healthy")
        self._publish_for(0.15, True, True)
        latest = self._latest_output()
        self.assertIsNotNone(latest)
        self.assertAlmostEqual(latest.pose.position.x, 1.25)
        self.assertAlmostEqual(latest.pose.position.y, 0.0)
        self.assertAlmostEqual(latest.pose.position.z, -0.10)
        self.assertEqual(latest.header.frame_id, "map")
        self.assertIn(latest.header.stamp.to_nsec(), self._sent_high_stamps,
                      "output timestamp was not preserved from high-rate odometry")
        output_stamps = [msg.header.stamp.to_nsec() for msg in self._outputs_since(0)]
        self.assertEqual(len(output_stamps), len(set(output_stamps)),
                         "a cached high-rate sample was published more than once")

        # The body origin is 0.10 m below the aligned tracking sensor.  The
        # offset must rotate with attitude; subtracting a fixed world-frame Z
        # value would be wrong as soon as the aircraft pitches or rolls.
        pitch = 0.50
        pitched_stamp = rospy.Time.now()
        pitched = self._make_odom(pitched_stamp, 1.25, "world", "odom_imu")
        pitched.pose.pose.orientation.y = math.sin(0.5 * pitch)
        pitched.pose.pose.orientation.w = math.cos(0.5 * pitch)
        self._high_pub.publish(pitched)
        pitched_output = self._wait_for_output_stamp(pitched_stamp)
        self.assertIsNotNone(pitched_output, "pitched lever-arm sample was not forwarded")
        self.assertAlmostEqual(pitched_output.pose.position.x,
                               1.25 - 0.10 * math.sin(pitch), places=8)
        self.assertAlmostEqual(pitched_output.pose.position.y, 0.0, places=8)
        self.assertAlmostEqual(pitched_output.pose.position.z,
                               -0.10 * math.cos(pitch), places=8)
        self._publish_for(0.10, True, True)
        self.assertTrue(self._is_healthy(), "valid rotated lever arm caused a fault")

        # Quaternion magnitude within the configured tolerance is normalized
        # before jump detection; it must not look like an attitude jump.
        scaled_stamp = rospy.Time.now()
        scaled_quaternion = self._make_odom(scaled_stamp, 1.25, "world", "odom_imu")
        scaled_quaternion.pose.pose.orientation.w = 0.91
        self._high_pub.publish(scaled_quaternion)
        normalized_output = self._wait_for_output_stamp(scaled_stamp)
        self.assertIsNotNone(normalized_output,
                             "accepted scaled quaternion sample was not forwarded")
        q = normalized_output.pose.orientation
        q_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        self.assertAlmostEqual(q_norm, 1.0, places=9,
                               msg="forwarded quaternion was not normalized")
        self._publish_for(0.10, True, True)
        self.assertTrue(self._is_healthy(),
                        "equivalent non-unit quaternion caused a false pose-jump fault")

        # Losing only high-rate odometry must stop output and latch the fault.
        self._publish_for(0.40, False, True)
        self.assertTrue(self._wait_for_health(False), "high-rate timeout did not mark unhealthy")
        stopped_count = self._wait_for_output_quiet()
        self._publish_for(0.20, False, True)
        self.assertEqual(self._wait_for_output_quiet(), stopped_count,
                         "bridge published after high-rate watchdog expired")
        self.assertFalse(self._reset_fault().success,
                         "reset succeeded while high-rate odometry was stale")

        # Fresh data returning alone must not clear a latched fault.
        self._publish_for(0.30, True, True)
        self.assertFalse(self._is_healthy())
        self.assertEqual(self._wait_for_output_quiet(), stopped_count,
                         "latched fault recovered without an explicit reset")
        self._recover()
        self.assertGreater(self._output_count(), stopped_count)

        # Losing only LiDAR corrections must also stop high-rate output.
        self._publish_for(0.55, True, False)
        self.assertTrue(self._wait_for_health(False), "correction timeout did not mark unhealthy")
        stopped_count = self._wait_for_output_quiet()
        self._publish_for(0.20, True, False)
        self.assertEqual(self._wait_for_output_quiet(), stopped_count,
                         "bridge published after correction watchdog expired")

        # A reset must not reconnect a restarted estimator with a different origin.
        self._publish_for(0.12, True, True, high_x=2.0)
        response = self._reset_fault()
        self.assertTrue(response.success, response.message)
        # A short excursion must break the recovery streak even if the pose
        # returns to the old value before the validation interval ends.
        self._publish_for(0.06, True, True, high_x=2.0)
        self._publish_for(0.60, True, True, high_x=1.25)
        self.assertFalse(self._is_healthy(),
                         "discontinuous recovery incorrectly became healthy")
        self.assertEqual(self._wait_for_output_quiet(), stopped_count,
                         "discontinuous recovery pose was forwarded")
        # _recover's reset can only succeed if the continuity check re-latched the fault.
        self._recover()

        # A fresh but unrelated odometry topic is not a valid LiDAR correction gate.
        output_start = self._output_count()
        self._correction_pub.publish(
            self._make_odom(rospy.Time.now(), 10.0, "camera_init", "body"))
        self.assertTrue(self._wait_for_health(False, timeout=0.12),
                        "cross-stream pose disagreement did not latch immediately")
        rospy.sleep(0.05)
        self.assertFalse(any(abs(msg.pose.position.x - 10.0) < 1.0e-9
                             for msg in self._outputs_since(output_start)),
                         "correction pose was forwarded as high-rate output")

        # A topic with traffic but a frozen timestamp is stale and must latch.
        self._recover()
        frozen_stamp = rospy.Time.now()
        self._publish_for(0.40, True, True, fixed_high_stamp=frozen_stamp)
        self.assertTrue(self._wait_for_health(False), "repeated timestamp did not expire watchdog")
        stopped_count = self._wait_for_output_quiet()
        self._publish_for(0.15, True, True, fixed_high_stamp=frozen_stamp)
        self.assertEqual(self._wait_for_output_quiet(), stopped_count,
                         "repeated high-rate sample was forwarded")

        # Correction traffic with a frozen timestamp must not hide lost LiDAR corrections.
        self._recover()
        frozen_correction_stamp = rospy.Time.now()
        self._publish_for(0.55, True, True,
                          fixed_correction_stamp=frozen_correction_stamp)
        self.assertTrue(self._wait_for_health(False),
                        "repeated correction timestamp did not expire watchdog")
        stopped_count = self._wait_for_output_quiet()
        self._publish_for(0.15, True, True,
                          fixed_correction_stamp=frozen_correction_stamp)
        self.assertEqual(self._wait_for_output_quiet(), stopped_count,
                         "high-rate pose was forwarded with frozen LiDAR corrections")

        # One invalid sample while ACTIVE must latch immediately, not wait for a timeout.
        self._recover()
        output_start = self._output_count()
        invalid = self._make_odom(rospy.Time.now(), float("nan"), "world", "odom_imu")
        self._high_pub.publish(invalid)
        self.assertTrue(self._wait_for_health(False, timeout=0.12),
                        "invalid sample did not latch immediately")
        rospy.sleep(0.05)
        self.assertFalse(any(not math.isfinite(msg.pose.position.x)
                             for msg in self._outputs_since(output_start)),
                         "invalid high-rate pose was forwarded")

        # A valid-looking but discontinuous pose must latch before publication.
        self._recover()
        output_start = self._output_count()
        jump = self._make_odom(rospy.Time.now(), 3.0, "world", "odom_imu")
        self._high_pub.publish(jump)
        self.assertTrue(self._wait_for_health(False, timeout=0.12),
                        "active position jump did not latch immediately")
        rospy.sleep(0.05)
        self.assertFalse(any(abs(msg.pose.position.x - 3.0) < 1.0e-9
                             for msg in self._outputs_since(output_start)),
                         "discontinuous active pose was forwarded")

        # Frame contracts prevent an accidentally remapped odometry topic from
        # being accepted merely because it has fresh timestamps.
        self._recover()
        wrong_frame = self._make_odom(rospy.Time.now(), 1.25, "camera_init", "odom_imu")
        self._high_pub.publish(wrong_frame)
        self.assertTrue(self._wait_for_health(False, timeout=0.12),
                        "wrong high-rate frame did not latch immediately")

        # A backwards epoch can never be reconnected online: PX4 must not see
        # time move backwards, and the old publish watermark must not be reused.
        self._recover()
        trusted_output = self._latest_output()
        self.assertIsNotNone(trusted_output)
        output_start = self._output_count()
        backwards_stamp = trusted_output.header.stamp - rospy.Duration.from_sec(0.001)
        fault_start = time.monotonic()
        self._high_pub.publish(self._make_odom(backwards_stamp, 99.0, "world", "odom_imu"))
        # This is deliberately shorter than the test's high-rate watchdog, so a
        # normal input timeout cannot make the backwards-stamp assertion pass.
        self.assertTrue(self._wait_for_health(False, timeout=0.12),
                        "backwards timestamp did not latch immediately")
        self.assertLess(time.monotonic() - fault_start, 0.20)
        rospy.sleep(0.05)
        self.assertFalse(any(abs(msg.pose.position.x - 99.0) < 1.0e-9
                             for msg in self._outputs_since(output_start)),
                         "backwards-timestamp pose was forwarded")
        self._publish_for(0.20, True, True)
        response = self._reset_fault()
        self.assertFalse(response.success,
                         "timestamp epoch rollback was incorrectly resettable online")
        self.assertIn("restart", response.message)
        self.assertFalse(self._is_healthy())


if __name__ == "__main__":
    rospy.init_node("test_vision_pose_watchdog")
    rostest.rosrun("real_drone_bringup", "vision_pose_watchdog", VisionPoseWatchdogTest)
