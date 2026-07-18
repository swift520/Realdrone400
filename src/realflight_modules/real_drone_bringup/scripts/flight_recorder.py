#!/usr/bin/env python3

"""Record a compact, self-describing real-flight debugging session."""

import argparse
import csv
import datetime as dt
import fcntl
import json
import math
import os
import platform
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rospy
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State, StatusText
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool


CORE_TOPICS = (
    "/livox/imu",
    "/Odometry",
    "/Odom_high_freq",
    "/localization/validated_odom",
    "/localization/healthy",
    "/mavros/vision_pose/pose",
    "/mavros/state",
    "/mavros/extended_state",
    "/mavros/imu/data",
    "/mavros/local_position/odom",
    "/mavros/estimator_status",
    "/mavros/timesync_status",
    "/mavros/sys_status",
    "/mavros/statustext/recv",
    "/mavros/battery",
    "/mavros/rc/in",
    "/mavros/setpoint_raw/attitude",
    "/mavros/setpoint_raw/target_attitude",
    "/position_cmd",
    "/debugPx4ctrl",
    "/px4ctrl/takeoff_land",
    "/traj_start_trigger",
    "/move_base_simple/goal",
    "/drone_0_planning/bspline",
    "/tf",
    "/tf_static",
    "/diagnostics",
    "/rosout_agg",
)

LIDAR_TOPICS = ("/livox/lidar",)
REGISTERED_CLOUD_TOPICS = ("/cloud_registered",)

STREAM_TIMEOUTS = {
    "localization_health": 0.25,
    "validated_odom": 0.50,
    "corrected_odom": 0.50,
    "high_rate_odom": 0.15,
    "fcu_imu": 0.50,
    "mavros_state": 2.00,
    "livox_imu": 0.20,
}

IMPORTANT_LOGGERS = ("px4ctrl", "vision_pose", "laserMapping", "mavros")


def sanitize_tag(tag):
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", tag.strip())
    return cleaned.strip("._-")[:48]


def build_topics(with_lidar=False, with_registered_cloud=False, extra_topics=None):
    topics = list(CORE_TOPICS)
    if with_lidar:
        topics.extend(LIDAR_TOPICS)
    if with_registered_cloud:
        topics.extend(REGISTERED_CLOUD_TOPICS)
    topics.extend(extra_topics or [])

    unique = []
    seen = set()
    for topic in topics:
        normalized = topic.strip()
        if normalized and not normalized.startswith("/"):
            normalized = "/" + normalized
        if normalized and normalized not in seen:
            seen.add(normalized)
            unique.append(normalized)
    return unique


def create_session_dir(root, tag, now=None):
    root = Path(root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    stamp = (now or dt.datetime.now()).strftime("%Y%m%d_%H%M%S")
    suffix = sanitize_tag(tag)
    base = stamp + ("_" + suffix if suffix else "")
    for index in range(100):
        name = base if index == 0 else "{}_{}".format(base, index)
        candidate = root / name
        try:
            candidate.mkdir(mode=0o750)
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError("unable to allocate a unique flight-log session directory")


class EdgeTracker:
    def __init__(self):
        self._values = {}

    def changed(self, key, value):
        previous = self._values.get(key, object())
        if previous == value:
            return False
        self._values[key] = value
        return True


class EventWriter:
    HEADER = ("wall_time_utc", "ros_time", "elapsed_s", "level", "event", "detail")

    def __init__(self, path):
        self._path = Path(path)
        self._handle = self._path.open("w", newline="", buffering=1)
        self._writer = csv.writer(self._handle)
        self._writer.writerow(self.HEADER)
        self._lock = threading.Lock()
        self._start = time.monotonic()
        self._last_fsync = 0.0
        self._closed = False

    def write(self, level, event, detail=""):
        with self._lock:
            if self._closed:
                return
            monotonic_now = time.monotonic()
            try:
                ros_time = rospy.Time.now().to_sec()
            except rospy.exceptions.ROSException:
                ros_time = 0.0
            self._writer.writerow((
                dt.datetime.now(dt.timezone.utc).isoformat(),
                "{:.9f}".format(ros_time),
                "{:.3f}".format(monotonic_now - self._start),
                level,
                event,
                detail,
            ))
            self._handle.flush()
            if monotonic_now - self._last_fsync >= 1.0:
                os.fsync(self._handle.fileno())
                self._last_fsync = monotonic_now

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._handle.flush()
            os.fsync(self._handle.fileno())
            self._handle.close()
            self._closed = True


class FlightRecorder:
    def __init__(self, args, session_dir, topics, workspace):
        self.args = args
        self.session_dir = Path(session_dir)
        self.topics = topics
        self.workspace = workspace
        self.events = EventWriter(self.session_dir / "events.csv")
        self.edges = EdgeTracker()
        self.stream_receive_times = {}
        self.stream_stale = {}
        self.start_monotonic = time.monotonic()
        self.last_disk_check = 0.0
        self.last_rosout = {}
        self.bag_process = None
        self.bag_log_handle = None
        self.bag_failed = False
        self.bag_failure_return_code = None
        self.metadata_thread = None
        self.shutdown_lock = threading.Lock()
        self.shutdown_complete = False
        self.subscribers = []
        self.metadata = {
            "session_dir": str(self.session_dir),
            "start_time_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "hostname": platform.node(),
            "platform": platform.platform(),
            "workspace": str(workspace) if workspace else "",
            "topics": topics,
            "with_lidar": args.with_lidar,
            "with_registered_cloud": args.with_registered_cloud,
            "compression": not args.no_compression,
            "split_duration": args.split_duration,
            "split_size_mb": args.split_size_mb,
            "runtime_min_space": args.min_space,
        }
        self._write_metadata()

    def start(self):
        self.events.write("INFO", "session_start", str(self.session_dir))
        self._start_bag()
        self._subscribe_events()
        self.metadata_thread = threading.Thread(target=self._capture_metadata, daemon=True)
        self.metadata_thread.start()

    def _start_bag(self):
        command = [
            "rosbag", "record",
            "--split",
            "--duration={}".format(self.args.split_duration),
            "--size={}".format(self.args.split_size_mb),
            "--buffsize={}".format(self.args.buffer_mb),
            "--min-space={}".format(self.args.min_space),
            "--tcpnodelay",
            "--repeat-latched",
        ]
        if not self.args.no_compression:
            command.append("--lz4")
        command.extend(("-O", str(self.session_dir / "flight.bag")))
        command.extend(self.topics)
        command.append("__name:=flight_bag")

        (self.session_dir / "rosbag_command.txt").write_text(
            " ".join(command) + "\n", encoding="utf-8")
        self.bag_log_handle = (self.session_dir / "rosbag.log").open(
            "w", buffering=1, encoding="utf-8")
        try:
            self.bag_process = subprocess.Popen(
                command,
                stdout=self.bag_log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError:
            self.bag_failed = True
            raise
        self.events.write("INFO", "rosbag_started", "pid={}".format(self.bag_process.pid))
        time.sleep(0.2)
        return_code = self.bag_process.poll()
        if return_code is not None:
            self.bag_failed = True
            self.bag_failure_return_code = return_code
            self.events.write("ERROR", "rosbag_start_failed",
                              "return_code={}".format(return_code))
            raise RuntimeError(
                "rosbag exited during startup with code {}; see {}".format(
                    return_code, self.session_dir / "rosbag.log"))

    def _subscribe_events(self):
        hints = {"queue_size": 20, "tcp_nodelay": True}
        self.subscribers.extend((
            rospy.Subscriber("/localization/healthy", Bool, self._health_callback, **hints),
            rospy.Subscriber("/mavros/state", State, self._state_callback, **hints),
            rospy.Subscriber("/mavros/extended_state", ExtendedState,
                             self._extended_state_callback, **hints),
            rospy.Subscriber("/mavros/estimator_status", EstimatorStatus,
                             self._estimator_callback, **hints),
            rospy.Subscriber("/mavros/statustext/recv", StatusText,
                             self._status_text_callback, **hints),
            rospy.Subscriber("/mavros/battery", BatteryState,
                             self._battery_callback, **hints),
            rospy.Subscriber("/rosout_agg", Log, self._rosout_callback,
                             queue_size=100, tcp_nodelay=True),
            rospy.Subscriber("/localization/validated_odom", Odometry,
                             lambda _msg: self._mark_stream("validated_odom"), **hints),
            rospy.Subscriber("/Odometry", Odometry,
                             lambda _msg: self._mark_stream("corrected_odom"), **hints),
            rospy.Subscriber("/Odom_high_freq", Odometry,
                             lambda _msg: self._mark_stream("high_rate_odom"), **hints),
            rospy.Subscriber("/mavros/imu/data", Imu,
                             lambda _msg: self._mark_stream("fcu_imu"), **hints),
            rospy.Subscriber("/livox/imu", Imu,
                             lambda _msg: self._mark_stream("livox_imu"), **hints),
        ))

    def _mark_stream(self, name):
        self.stream_receive_times[name] = time.monotonic()

    def _health_callback(self, msg):
        self._mark_stream("localization_health")
        if self.edges.changed("localization_health_value", msg.data):
            level = "INFO" if msg.data else "ERROR"
            self.events.write(level, "localization_health", str(bool(msg.data)))

    def _state_callback(self, msg):
        self._mark_stream("mavros_state")
        value = (bool(msg.connected), bool(msg.armed), msg.mode, int(msg.system_status))
        if self.edges.changed("mavros_state", value):
            detail = "connected={} armed={} mode={} system_status={}".format(*value)
            level = "INFO" if msg.connected else "ERROR"
            self.events.write(level, "mavros_state", detail)

    def _extended_state_callback(self, msg):
        value = int(msg.landed_state)
        if self.edges.changed("landed_state", value):
            self.events.write("INFO", "landed_state", str(value))

    def _estimator_callback(self, msg):
        value = (
            bool(msg.attitude_status_flag),
            bool(msg.velocity_horiz_status_flag),
            bool(msg.velocity_vert_status_flag),
            bool(msg.pos_horiz_rel_status_flag),
            bool(msg.pos_horiz_abs_status_flag),
            bool(msg.pos_vert_abs_status_flag),
            bool(msg.const_pos_mode_status_flag),
            bool(msg.gps_glitch_status_flag),
            bool(msg.accel_error_status_flag),
        )
        if self.edges.changed("estimator_status", value):
            names = ("att", "vel_h", "vel_v", "pos_rel", "pos_abs",
                     "pos_z", "const_pos", "gps_glitch", "accel_error")
            detail = " ".join("{}={}".format(name, int(flag))
                              for name, flag in zip(names, value))
            unhealthy = not all(value[:4]) or any(value[6:])
            self.events.write("WARN" if unhealthy else "INFO", "estimator_status", detail)

    def _status_text_callback(self, msg):
        if msg.severity <= StatusText.WARNING:
            level = "ERROR" if msg.severity <= StatusText.ERROR else "WARN"
            self.events.write(level, "px4_statustext",
                              "severity={} {}".format(msg.severity, msg.text))

    def _battery_callback(self, msg):
        voltage = msg.voltage
        if not math.isfinite(voltage) or voltage <= 0.0:
            cells = [value for value in msg.cell_voltage if math.isfinite(value) and value > 0.0]
            voltage = sum(cells) if cells else float("nan")
        if not math.isfinite(voltage) or voltage <= 0.0:
            state = "unknown"
        elif voltage < self.args.low_voltage:
            state = "low"
        else:
            state = "ok"
        if self.edges.changed("battery_state", state):
            level = "WARN" if state != "ok" else "INFO"
            detail = "voltage={:.2f} percentage={:.3f}".format(voltage, msg.percentage)
            self.events.write(level, "battery_{}".format(state), detail)

    def _rosout_callback(self, msg):
        if msg.level < Log.WARN or not any(name in msg.name for name in IMPORTANT_LOGGERS):
            return
        key = (msg.name, msg.msg)
        now = time.monotonic()
        if now - self.last_rosout.get(key, -1.0e9) < 2.0:
            return
        self.last_rosout[key] = now
        level = "ERROR" if msg.level >= Log.ERROR else "WARN"
        self.events.write(level, "rosout", "{}: {}".format(msg.name, msg.msg))

    def poll(self):
        if self.bag_process is not None:
            return_code = self.bag_process.poll()
            if return_code is not None:
                self.bag_failed = True
                self.bag_failure_return_code = return_code
                self.events.write("ERROR", "rosbag_exited",
                                  "return_code={}".format(return_code))
                rospy.logerr("rosbag exited unexpectedly with code %s; see %s",
                             return_code, self.session_dir / "rosbag.log")
                rospy.signal_shutdown("rosbag exited unexpectedly")
                return

        now = time.monotonic()
        if now - self.start_monotonic >= self.args.startup_grace:
            for name, timeout in STREAM_TIMEOUTS.items():
                received = self.stream_receive_times.get(name)
                stale = received is None or now - received >= timeout
                if self.stream_stale.get(name) != stale:
                    self.stream_stale[name] = stale
                    if stale:
                        detail = "missing" if received is None else "age={:.3f}s".format(now - received)
                        self.events.write("ERROR", "stream_stale", "{} {}".format(name, detail))
                    else:
                        self.events.write("INFO", "stream_recovered", name)

        if now - self.last_disk_check >= 5.0:
            self.last_disk_check = now
            free_gb = shutil.disk_usage(str(self.session_dir)).free / (1024.0 ** 3)
            low = free_gb < self.args.runtime_warn_free_gb
            if self.edges.changed("runtime_disk_low", low):
                self.events.write("WARN" if low else "INFO", "disk_space",
                                  "free_gb={:.2f}".format(free_gb))

    def shutdown(self):
        with self.shutdown_lock:
            if self.shutdown_complete:
                return
            self.shutdown_complete = True

        self.events.write("INFO", "session_stop_requested")
        return_code = None
        forced_shutdown = False
        if self.bag_process is not None and self.bag_process.poll() is None:
            try:
                self._signal_bag_group(signal.SIGINT)
                return_code = self.bag_process.wait(timeout=30.0)
            except subprocess.TimeoutExpired:
                forced_shutdown = True
                self.events.write("ERROR", "rosbag_sigint_timeout", "sending SIGTERM")
                self._signal_bag_group(signal.SIGTERM)
                try:
                    return_code = self.bag_process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.events.write("ERROR", "rosbag_sigterm_timeout", "sending SIGKILL")
                    self._signal_bag_group(signal.SIGKILL)
                    try:
                        return_code = self.bag_process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        self.events.write("ERROR", "rosbag_sigkill_timeout")
        elif self.bag_process is not None:
            if self.bag_failed:
                self._signal_bag_group(signal.SIGINT)
            return_code = self.bag_process.returncode

        metadata_complete = False
        if self.metadata_thread is not None:
            self.metadata_thread.join(timeout=2.0)
            metadata_complete = not self.metadata_thread.is_alive()
            if not metadata_complete:
                self.events.write("WARN", "metadata_capture_incomplete")

        bag_files = [
            {"name": path.name, "bytes": path.stat().st_size}
            for path in sorted(self.session_dir.glob("*.bag"))
        ]
        active_files = [path.name for path in self.session_dir.glob("*.active")]
        if forced_shutdown or not bag_files or active_files or (
                self.bag_process is not None and return_code != 0):
            self.bag_failed = True
            if self.bag_failure_return_code is None:
                self.bag_failure_return_code = return_code

        self.metadata["stop_time_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        self.metadata["duration_s"] = round(time.monotonic() - self.start_monotonic, 3)
        self.metadata["rosbag_return_code"] = return_code
        self.metadata["rosbag_failed"] = self.bag_failed
        self.metadata["rosbag_failure_return_code"] = self.bag_failure_return_code
        self.metadata["flight_log_complete"] = not self.bag_failed
        self.metadata["metadata_capture_complete"] = metadata_complete
        self.metadata["bag_files"] = bag_files
        self.metadata["active_files"] = active_files
        self._write_metadata()
        self.events.write("INFO" if not self.bag_failed and return_code in (None, 0) else "ERROR",
                          "session_stopped", "rosbag_return_code={}".format(return_code))
        self.events.close()
        if self.bag_log_handle is not None:
            self.bag_log_handle.close()
        if self.bag_failed:
            print("Flight log INCOMPLETE: {} (check rosbag.log and *.active)".format(
                self.session_dir), file=sys.stderr, flush=True)
        else:
            print("Flight log saved to: {}".format(self.session_dir), flush=True)

    def _signal_bag_group(self, requested_signal):
        if self.bag_process is None:
            return
        try:
            # start_new_session=True makes the rosbag PID its process-group ID.
            os.killpg(self.bag_process.pid, requested_signal)
        except ProcessLookupError:
            # The recorder can exit between poll() and killpg(); wait() below
            # still reaps it and normal session finalization must continue.
            pass
        except OSError as error:
            self.bag_failed = True
            self.events.write("ERROR", "rosbag_signal_failed",
                              "signal={} error={}".format(requested_signal, error))

    def _write_metadata(self):
        target = self.session_dir / "metadata.json"
        temporary = self.session_dir / "metadata.json.tmp"
        temporary.write_text(json.dumps(self.metadata, indent=2, sort_keys=True) + "\n",
                             encoding="utf-8")
        os.replace(str(temporary), str(target))

    def _capture_metadata(self):
        commands = (
            (("rosnode", "list"), "rosnodes.txt"),
            (("rostopic", "list", "-v"), "rostopics.txt"),
            (("rosservice", "list"), "rosservices.txt"),
            (("df", "-h", str(self.session_dir)), "disk_start.txt"),
        )
        for command, filename in commands:
            self._capture_text(command, filename, timeout=8.0)

        self._capture_to_file(("rosparam", "dump", str(self.session_dir / "rosparams.yaml")),
                              "rosparam_dump.log", timeout=20.0)
        self._capture_to_file(("rosrun", "mavros", "mavparam", "dump",
                               str(self.session_dir / "px4_params.txt")),
                              "px4_param_dump.log", timeout=30.0)

        if self.workspace:
            self._capture_text(("git", "-C", str(self.workspace), "rev-parse", "HEAD"),
                               "git_head.txt", timeout=5.0)
            self._capture_text(("git", "-C", str(self.workspace), "status", "--short"),
                               "git_status.txt", timeout=8.0)

    def _capture_text(self, command, filename, timeout):
        try:
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, timeout=timeout, check=False)
            content = "$ {}\n{}\nreturn_code={}\n".format(
                " ".join(command), result.stdout, result.returncode)
        except (OSError, subprocess.TimeoutExpired) as error:
            content = "$ {}\nERROR: {}\n".format(" ".join(command), error)
        (self.session_dir / filename).write_text(content, encoding="utf-8")

    def _capture_to_file(self, command, log_filename, timeout):
        self._capture_text(command, log_filename, timeout)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Record a compact REAL_DRONE_400 flight-debugging session.")
    parser.add_argument("tag", nargs="?", default="", help="short label such as hover_01")
    parser.add_argument("--output-root", default=os.environ.get(
        "FLIGHT_LOG_ROOT", str(Path.home() / "flight_logs")))
    parser.add_argument("--with-lidar", action="store_true",
                        help="also record /livox/lidar (roughly 14+ GB/hour)")
    parser.add_argument("--with-registered-cloud", action="store_true",
                        help="also record /cloud_registered (large and adds planner CPU load)")
    parser.add_argument("--extra-topic", action="append", default=[])
    parser.add_argument("--split-duration", default="10m")
    parser.add_argument("--split-size-mb", type=int, default=2048)
    parser.add_argument("--buffer-mb", type=int, default=256)
    parser.add_argument("--min-space", default="5G",
                        help="rosbag stops before the recording filesystem falls below this")
    parser.add_argument("--min-start-free-gb", type=float, default=8.0)
    parser.add_argument("--runtime-warn-free-gb", type=float, default=10.0)
    parser.add_argument("--low-voltage", type=float, default=19.8)
    parser.add_argument("--startup-grace", type=float, default=5.0)
    parser.add_argument("--no-compression", action="store_true")
    parser.add_argument("--dry-run", action="store_true",
                        help="print configuration without creating files or contacting ROS")
    args = parser.parse_args(argv)
    if args.split_size_mb <= 0 or args.buffer_mb <= 0:
        parser.error("split size and buffer size must be positive")
    if args.min_start_free_gb < 0.0 or args.runtime_warn_free_gb < 0.0:
        parser.error("disk thresholds must be non-negative")
    return args


def discover_workspace():
    configured = os.environ.get("REAL_DRONE_WORKSPACE")
    if configured:
        path = Path(configured).expanduser().resolve()
        return path if (path / ".git").exists() else None
    try:
        result = subprocess.run(("git", "rev-parse", "--show-toplevel"),
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                text=True, timeout=3.0, check=False)
        if result.returncode == 0:
            return Path(result.stdout.strip()).resolve()
    except (OSError, subprocess.TimeoutExpired):
        pass
    return None


def acquire_instance_lock(root):
    root = Path(root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    handle = (root / ".flight_recorder.lock").open("a+")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        handle.close()
        raise RuntimeError("another flight recorder is already running") from error
    return handle


def main(argv=None):
    cli_args = rospy.myargv(argv=sys.argv if argv is None else argv)[1:]
    args = parse_args(cli_args)
    topics = build_topics(args.with_lidar, args.with_registered_cloud, args.extra_topic)
    if args.dry_run:
        print(json.dumps({
            "output_root": str(Path(args.output_root).expanduser()),
            "tag": sanitize_tag(args.tag),
            "topics": topics,
            "estimated_profile": "raw_lidar" if args.with_lidar else "compact",
        }, indent=2))
        return 0

    lock_handle = None
    recorder = None
    try:
        lock_handle = acquire_instance_lock(args.output_root)
        free_gb = shutil.disk_usage(str(Path(args.output_root).expanduser())).free / (1024.0 ** 3)
        if free_gb < args.min_start_free_gb:
            raise RuntimeError("only {:.2f} GiB free; {:.2f} GiB required to start".format(
                free_gb, args.min_start_free_gb))

        session_dir = create_session_dir(args.output_root, args.tag)
        rospy.init_node("flight_recorder", anonymous=False)
        recorder = FlightRecorder(args, session_dir, topics, discover_workspace())
        rospy.on_shutdown(recorder.shutdown)
        recorder.start()

        profile = "compact + raw LiDAR" if args.with_lidar else "compact"
        rospy.loginfo("Flight recorder ACTIVE (%s). Output: %s", profile, session_dir)
        rospy.loginfo("Land and disarm before Ctrl-C; wait for rosbag indexing to finish.")
        while not rospy.is_shutdown():
            recorder.poll()
            time.sleep(0.1)
        return 1 if recorder.bag_failed else 0
    except (OSError, RuntimeError, rospy.ROSException) as error:
        print("flight_recorder: {}".format(error), file=sys.stderr)
        if recorder is not None:
            recorder.bag_failed = True
            recorder.metadata["supervisor_error"] = str(error)
            recorder.shutdown()
        return 1
    finally:
        if lock_handle is not None:
            lock_handle.close()


if __name__ == "__main__":
    sys.exit(main())
