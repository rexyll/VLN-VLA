#!/usr/bin/env python3
"""Read-only ROS 2 + Gazebo telemetry recorder for local DrinkTask diagnosis."""
import argparse
import json
import math
import os
import platform
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from tf2_msgs.msg import TFMessage
import yaml

try:
    import gz.transport13 as gz_transport
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.model_pb2 import Model
    from gz.msgs10.stringmsg_pb2 import StringMsg
    from gz.msgs10.double_pb2 import Double
except Exception as exc:
    raise RuntimeError(
        "Gazebo Harmonic Python bindings unavailable (gz-transport13 / gz-msgs10): %s" % exc
    )


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9 if stamp is not None else None


def safe_float(value):
    value = float(value)
    return value if math.isfinite(value) else None


def run_text(args):
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception as exc:
        return "unavailable: %s" % exc


class Recorder(Node):
    def __init__(self, output_dir, run_id, bridge_config):
        super().__init__("drinktask_local_telemetry_recorder")
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.telemetry_path = self.output_dir / "telemetry.jsonl"
        self.stream = self.telemetry_path.open("w", encoding="utf-8", buffering=1)
        self.lock = threading.RLock()
        self.latest = {}
        self.command_bridges = self._load_command_bridges(bridge_config)
        self.command_stats = {
            entry["ros_topic"]: {
                "gz_topic": entry["gz_topic"],
                "ros": {"count": 0, "last_value": None, "last_received_monotonic": None, "last_received_utc": None},
                "gazebo": {"count": 0, "last_value": None, "last_received_monotonic": None, "last_received_utc": None},
            }
            for entry in self.command_bridges
        }
        self.last_state = None
        self.started_mono = time.monotonic()
        self.seq = 0
        self.gz_node = gz_transport.Node()
        self.gz_ready = {}
        self.ros_command_subscriptions = {}
        self.gz_command_subscriptions = {}
        self._subscribe_ros()
        self._subscribe_command_bridges()
        self._subscribe_gz()
        metadata = {
            "run_id": run_id,
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
            "host": platform.node(),
            "platform": platform.platform(),
            "python": platform.python_version(),
            "ros_distro": os.environ.get("ROS_DISTRO"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
            "drink_command_bridges": [
                {
                    "ros_topic": entry["ros_topic"],
                    "gz_topic": entry["gz_topic"],
                    "ros_subscribed": self.ros_command_subscriptions.get(entry["ros_topic"], False),
                    "gazebo_subscribed": self.gz_command_subscriptions.get(entry["gz_topic"], False),
                }
                for entry in self.command_bridges
            ],
            "gazebo_topics": [
                "/world/follow_world/dynamic_pose/info",
                "/world/follow_world/model/robot/joint_state",
                "/world/follow_world/chassis_lock/state"
            ],
            "ros_topics": [
                "/drink/state", "/drink/joint_state", "/robot/odom",
                "/drink/cmd_vel", "/robot/cmd_vel", "/drink/base_lock_state",
                "/follower/status", "/nav2_active", "/tf", "/tf_static"
            ],
            "git_head": run_text(["git", "-C", str(Path.cwd()), "rev-parse", "HEAD"]),
            "git_branch": run_text(["git", "-C", str(Path.cwd()), "branch", "--show-current"]),
            "output_dir": str(self.output_dir),
            "notes": "Read-only telemetry. No ROS/Gazebo command topics are published."
        }
        (self.output_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.timer = self.create_timer(0.1, self.write_sample)
        self.get_logger().info("LOCAL_TELEMETRY_START output=%s" % self.output_dir)

    def _put(self, key, value):
        with self.lock:
            self.latest[key] = {"received_monotonic": time.monotonic(), "value": value}

    def _subscribe_ros(self):
        self.create_subscription(String, "/drink/state", lambda m: self._put("drink_state", m.data), 50)
        self.create_subscription(JointState, "/drink/joint_state", self._joint_cb, 50)
        self.create_subscription(Odometry, "/robot/odom", self._odom_cb, 50)
        self.create_subscription(Twist, "/drink/cmd_vel", lambda m: self._put("drink_cmd_vel", self._twist(m)), 50)
        self.create_subscription(Twist, "/robot/cmd_vel", lambda m: self._put("robot_cmd_vel", self._twist(m)), 50)
        self.create_subscription(String, "/drink/base_lock_state", lambda m: self._put("base_lock_state", m.data), 50)
        self.create_subscription(String, "/follower/status", lambda m: self._put("follower_status", m.data), 20)
        self.create_subscription(Bool, "/nav2_active", lambda m: self._put("nav2_active", bool(m.data)), 20)
        self.create_subscription(TFMessage, "/tf", lambda m: self._put("tf", [
            {"parent": t.header.frame_id, "child": t.child_frame_id, "stamp": stamp_to_sec(t.header.stamp)}
            for t in m.transforms]), 50)
        self.create_subscription(TFMessage, "/tf_static", lambda m: self._put("tf_static", [
            {"parent": t.header.frame_id, "child": t.child_frame_id}
            for t in m.transforms]), 10)

    @staticmethod
    def _load_command_bridges(config_path):
        path = Path(config_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError("Bridge config not found: %s" % path)
        config = yaml.safe_load(path.read_text(encoding="utf-8")) or []
        entries = [
            {
                "ros_topic": item["ros_topic_name"],
                "gz_topic": item["gz_topic_name"],
            }
            for item in config
            if item.get("direction") == "ROS_TO_GZ"
            and item.get("ros_type_name") == "std_msgs/msg/Float64"
            and item.get("gz_type_name") == "gz.msgs.Double"
            and item.get("ros_topic_name", "").startswith("/drink/cmd/")
        ]
        ros_topics = [entry["ros_topic"] for entry in entries]
        if len(entries) != 39 or len(set(ros_topics)) != 39:
            raise RuntimeError(
                "Expected exactly 39 unique /drink/cmd Float64 -> gz.msgs.Double bridges; found %d" % len(entries)
            )
        return entries

    def _command_cb(self, ros_topic, side, value):
        now = time.monotonic()
        with self.lock:
            stats = self.command_stats[ros_topic][side]
            stats["count"] += 1
            stats["last_value"] = safe_float(value)
            stats["last_received_monotonic"] = now
            stats["last_received_utc"] = datetime.now(timezone.utc).isoformat()

    def _subscribe_command_bridges(self):
        for entry in self.command_bridges:
            ros_topic = entry["ros_topic"]
            gz_topic = entry["gz_topic"]
            ros_sub = self.create_subscription(
                Float64,
                ros_topic,
                lambda msg, topic=ros_topic: self._command_cb(topic, "ros", msg.data),
                50,
            )
            self.ros_command_subscriptions[ros_topic] = ros_sub is not None
            gz_ok = self.gz_node.subscribe(
                Double,
                gz_topic,
                lambda msg, topic=ros_topic: self._command_cb(topic, "gazebo", msg.data),
            )
            self.gz_command_subscriptions[gz_topic] = bool(gz_ok)
            if not gz_ok:
                self.get_logger().error("GAZEBO_COMMAND_SUBSCRIBE_FAILED ros=%s gz=%s" % (ros_topic, gz_topic))

    def _subscribe_gz(self):
        subscriptions = (
            (Pose_V, "/world/follow_world/dynamic_pose/info", self._gz_pose_cb),
            (Model, "/world/follow_world/model/robot/joint_state", self._gz_joint_cb),
            (StringMsg, "/world/follow_world/chassis_lock/state",
             lambda m: self._put("gz_physical_lock_state", str(m.data))),
        )
        for msg_type, topic, callback in subscriptions:
            ok = self.gz_node.subscribe(msg_type, topic, callback)
            self.gz_ready[topic] = bool(ok)
            if not ok:
                self.get_logger().error("GAZEBO_SUBSCRIBE_FAILED topic=%s" % topic)

    @staticmethod
    def _twist(msg):
        return {
            "linear": [safe_float(msg.linear.x), safe_float(msg.linear.y), safe_float(msg.linear.z)],
            "angular": [safe_float(msg.angular.x), safe_float(msg.angular.y), safe_float(msg.angular.z)]
        }

    def _joint_cb(self, msg):
        self._put("ros_joint_state", {
            "stamp": stamp_to_sec(msg.header.stamp),
            "name": list(msg.name),
            "position": [safe_float(v) for v in msg.position],
            "velocity": [safe_float(v) for v in msg.velocity],
            "effort": [safe_float(v) for v in msg.effort]
        })

    def _odom_cb(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        self._put("odom", {
            "stamp": stamp_to_sec(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "position": [safe_float(p.x), safe_float(p.y), safe_float(p.z)],
            "orientation_xyzw": [safe_float(q.x), safe_float(q.y), safe_float(q.z), safe_float(q.w)],
            "twist": self._twist(t)
        })

    def _gz_pose_cb(self, msg):
        poses = {}
        for item in msg.pose:
            p, q = item.position, item.orientation
            if item.name == "robot" or item.name.endswith("::chassis"):
                yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                                 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
                poses[item.name] = {
                    "position": [safe_float(p.x), safe_float(p.y), safe_float(p.z)],
                    "quaternion_xyzw": [safe_float(q.x), safe_float(q.y), safe_float(q.z), safe_float(q.w)],
                    "yaw_rad": safe_float(yaw)
                }
        self._put("gz_truth_pose", poses)

    def _gz_joint_cb(self, msg):
        joints = {}
        for joint in msg.joint:
            if not joint.HasField("axis1"):
                continue
            axis = joint.axis1
            joints[joint.name] = {
                "position_rad": safe_float(axis.position),
                "velocity_rad_s": safe_float(axis.velocity),
                "effort_nm": safe_float(axis.force)
            }
        self._put("gz_joint_state", joints)

    def write_sample(self):
        now = time.monotonic()
        with self.lock:
            snapshot = {k: dict(v) for k, v in self.latest.items()}
        values, ages = {}, {}
        for key, item in snapshot.items():
            values[key] = item["value"]
            ages[key] = round(now - item["received_monotonic"], 4)
        state = values.get("drink_state")
        event = "drink_state_changed" if state != self.last_state else None
        self.last_state = state
        self.seq += 1
        record = {
            "seq": self.seq,
            "t_monotonic_s": round(now - self.started_mono, 4),
            "wall_time_utc": datetime.now(timezone.utc).isoformat(),
            "event": event,
            "topic_age_s": ages,
            "gazebo_subscriptions": self.gz_ready,
            "drink_command_telemetry": {
                ros_topic: {
                    "gz_topic": stats["gz_topic"],
                    "ros": {
                        "count": stats["ros"]["count"],
                        "last_value": stats["ros"]["last_value"],
                        "age_s": round(now - stats["ros"]["last_received_monotonic"], 4)
                        if stats["ros"]["last_received_monotonic"] is not None else None,
                        "last_received_monotonic_s": round(stats["ros"]["last_received_monotonic"] - self.started_mono, 4)
                        if stats["ros"]["last_received_monotonic"] is not None else None,
                        "last_received_utc": stats["ros"]["last_received_utc"],
                    },
                    "gazebo": {
                        "count": stats["gazebo"]["count"],
                        "last_value": stats["gazebo"]["last_value"],
                        "age_s": round(now - stats["gazebo"]["last_received_monotonic"], 4)
                        if stats["gazebo"]["last_received_monotonic"] is not None else None,
                        "last_received_monotonic_s": round(stats["gazebo"]["last_received_monotonic"] - self.started_mono, 4)
                        if stats["gazebo"]["last_received_monotonic"] is not None else None,
                        "last_received_utc": stats["gazebo"]["last_received_utc"],
                    },
                }
                for ros_topic, stats in self.command_stats.items()
            },
            "data": values
        }
        self.stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")

    def close(self):
        try:
            self.write_sample()
        finally:
            self.stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--run-id", default="local-drinktask")
    parser.add_argument(
        "--bridge-config",
        default=str(Path(__file__).resolve().parents[2] / "vla_ws/src/vla_follow/config/bridge.yml"),
        help="ROS-Gazebo bridge YAML used to derive the 39 read-only drink command subscriptions",
    )
    args = parser.parse_args()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output_dir or ("tests/out/drinktask_%s_%s" % (args.run_id, stamp))
    rclpy.init()
    node = Recorder(output, args.run_id, args.bridge_config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.get_logger().info("LOCAL_TELEMETRY_STOP records=%d file=%s" % (node.seq, node.telemetry_path))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
