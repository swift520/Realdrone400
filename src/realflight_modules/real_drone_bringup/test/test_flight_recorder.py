#!/usr/bin/env python3

import csv
import datetime as dt
import importlib.util
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "flight_recorder.py"
SPEC = importlib.util.spec_from_file_location("flight_recorder", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class EventSink:
    def __init__(self):
        self.records = []

    def write(self, level, event, detail=""):
        self.records.append((level, event, detail))


class FlightRecorderUnitTest(unittest.TestCase):
    def test_topic_profiles_are_explicit_and_unique(self):
        compact = MODULE.build_topics()
        raw = MODULE.build_topics(with_lidar=True)
        self.assertEqual(len(compact), len(set(compact)))
        self.assertNotIn("/livox/lidar", compact)
        self.assertIn("/livox/imu", compact)
        self.assertIn("/livox/lidar", raw)
        self.assertNotIn("/cloud_registered", raw)

    def test_extra_topics_are_normalized_and_deduplicated(self):
        topics = MODULE.build_topics(extra_topics=["custom/topic", "/custom/topic"])
        self.assertEqual(topics.count("/custom/topic"), 1)

    def test_realsense_profile_records_rgb_depth_and_calibration(self):
        topics = MODULE.build_topics(
            with_realsense=True,
            realsense_namespace="front/camera/",
        )
        self.assertIn("/front/camera/color/image_raw", topics)
        self.assertIn("/front/camera/color/camera_info", topics)
        self.assertIn("/front/camera/depth/image_rect_raw", topics)
        self.assertIn("/front/camera/depth/camera_info", topics)
        self.assertIn("/front/camera/extrinsics/depth_to_color", topics)

    def test_realsense_emitter_and_laser_are_verified(self):
        class FakeClient:
            def __init__(self, name, timeout):
                self.name = name
                self.timeout = timeout
                self.closed = False

            def get_parameter_descriptions(self, timeout):
                return [
                    {"name": "emitter_enabled"},
                    {"name": "laser_power"},
                ]

            def update_configuration(self, changes):
                self.changes = changes
                return dict(changes)

            def close(self):
                self.closed = True

        result = MODULE.enable_realsense_emitter(
            "/camera/stereo_module",
            laser_power=150.0,
            timeout=2.0,
            client_factory=FakeClient,
        )
        self.assertEqual(result, {"emitter_enabled": 1, "laser_power": 150.0})

    def test_unavailable_realsense_is_removed_but_core_topics_remain(self):
        recorder = MODULE.FlightRecorder.__new__(MODULE.FlightRecorder)
        recorder.topics = MODULE.build_topics(
            with_realsense=True,
            extra_topics=["/custom/debug"],
        )
        recorder.realsense_topics = set(MODULE.build_realsense_topics("/camera"))
        recorder.realsense_active = True
        recorder.stream_timeouts = dict(MODULE.STREAM_TIMEOUTS)
        recorder.stream_timeouts.update({
            "realsense_color": 0.50,
            "realsense_depth": 0.50,
        })
        recorder.metadata = {
            "topics": list(recorder.topics),
            "with_realsense": True,
        }
        recorder.events = EventSink()
        recorder._write_metadata = lambda: None

        recorder._continue_without_realsense("camera absent")

        self.assertIn("/Odometry", recorder.topics)
        self.assertIn("/mavros/state", recorder.topics)
        self.assertIn("/custom/debug", recorder.topics)
        self.assertNotIn("/camera/color/image_raw", recorder.topics)
        self.assertNotIn("/camera/depth/image_rect_raw", recorder.topics)
        self.assertFalse(recorder.metadata["with_realsense"])
        self.assertFalse(recorder.realsense_active)
        self.assertNotIn("realsense_color", recorder.stream_timeouts)
        self.assertEqual(recorder.events.records[-1][0:2],
                         ("WARN", "realsense_unavailable"))

    def test_tag_and_session_names_are_safe(self):
        self.assertEqual(MODULE.sanitize_tag(" hover test / 01 "), "hover_test_01")
        with tempfile.TemporaryDirectory() as root:
            now = dt.datetime(2026, 7, 17, 12, 30, 45)
            first = MODULE.create_session_dir(root, "hover test", now)
            second = MODULE.create_session_dir(root, "hover test", now)
            self.assertEqual(first.name, "20260717_123045_hover_test")
            self.assertEqual(second.name, "20260717_123045_hover_test_1")

    def test_event_writer_emits_a_stable_csv_schema(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "events.csv"
            writer = MODULE.EventWriter(path)
            writer.write("WARN", "test_event", "detail,with,commas")
            writer.close()
            with path.open(newline="") as handle:
                rows = list(csv.reader(handle))
            self.assertEqual(tuple(rows[0]), MODULE.EventWriter.HEADER)
            self.assertEqual(rows[1][3:], ["WARN", "test_event", "detail,with,commas"])

    def test_edge_tracker_writes_only_transitions(self):
        tracker = MODULE.EdgeTracker()
        self.assertTrue(tracker.changed("mode", "MANUAL"))
        self.assertFalse(tracker.changed("mode", "MANUAL"))
        self.assertTrue(tracker.changed("mode", "OFFBOARD"))

    def test_unknown_battery_is_not_reported_as_ok(self):
        recorder = MODULE.FlightRecorder.__new__(MODULE.FlightRecorder)
        recorder.args = SimpleNamespace(low_voltage=19.8)
        recorder.edges = MODULE.EdgeTracker()
        recorder.events = EventSink()
        message = MODULE.BatteryState()
        message.voltage = float("nan")
        message.percentage = float("nan")

        recorder._battery_callback(message)
        recorder._battery_callback(message)
        message.voltage = 22.2
        recorder._battery_callback(message)

        self.assertEqual([record[1] for record in recorder.events.records],
                         ["battery_unknown", "battery_ok"])
        self.assertEqual(recorder.events.records[0][0], "WARN")

    def test_estimator_const_position_mode_is_a_warning(self):
        recorder = MODULE.FlightRecorder.__new__(MODULE.FlightRecorder)
        recorder.edges = MODULE.EdgeTracker()
        recorder.events = EventSink()
        message = MODULE.EstimatorStatus()
        message.attitude_status_flag = True
        message.velocity_horiz_status_flag = True
        message.velocity_vert_status_flag = True
        message.pos_horiz_rel_status_flag = True
        message.const_pos_mode_status_flag = True

        recorder._estimator_callback(message)

        self.assertEqual(recorder.events.records[0][0:2],
                         ("WARN", "estimator_status"))


if __name__ == "__main__":
    unittest.main()
