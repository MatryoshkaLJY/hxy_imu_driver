#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Real ROS publisher/subscriber and POSIX PTY test; no sensor hardware needed."""
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from sensor_msgs.msg import Imu

BINARY, GENERATOR, CONFIG = map(lambda s: str(Path(s).resolve()), sys.argv[1:4])
sys.argv[1:] = []


class Ros2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wire = subprocess.check_output([GENERATOR], timeout=5)
        cls.frame = wire[:wire[1] + 12]
        cls.next_frame = wire[len(cls.frame):2 * len(cls.frame)]
        cls.warmup_frames = [wire[i * len(cls.frame):(i + 1) * len(cls.frame)]
                             for i in range(100, 260)]
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def wait_for(self, node, predicate, seconds=6):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(node, timeout_sec=0.05)
        self.fail("timed out waiting for ROS data or discovery")

    def test_publish_and_reconnect(self):
        with tempfile.TemporaryDirectory() as directory:
            link = Path(directory) / "imu"
            master, slave = pty.openpty()
            link.symlink_to(os.ttyname(slave))
            node = rclpy.create_node("imu_test_subscriber")
            messages = []
            subscription = node.create_subscription(
                Imu, "/test_imu", messages.append, qos_profile_sensor_data)
            with tempfile.TemporaryFile(mode="w+") as log:
                child = subprocess.Popen([
                    BINARY, "--ros-args", "--params-file", CONFIG,
                    "-p", f"port:={link}", "-p", "topic:=/test_imu",
                    "-p", "frame_id:=test_imu_link", "-p", "reconnect_interval:=0.1",
                    "-p", "data_timeout:=10.0",
                    "-p", "linear_acceleration_variance:=[0.1, 0.2, 0.3]",
                    "-p", "angular_velocity_variance:=[0.4, 0.5, 0.6]",
                ], stdout=log, stderr=subprocess.STDOUT)
                try:
                    self.wait_for(node, lambda: subscription.get_publisher_count() == 1)
                    # Discovery and opening the device are separate operations.
                    def connected():
                        log.seek(0)
                        return "Reading " in log.read()
                    self.wait_for(node, connected)
                    # Best-effort discovery is asynchronous on both endpoints.
                    # Stream unique warmup samples until a delivery proves they
                    # are matched; then drain before the controlled assertions.
                    for warmup in self.warmup_frames:
                        os.write(master, warmup)
                        rclpy.spin_once(node, timeout_sec=0.02)
                        if messages:
                            break
                    self.assertTrue(messages, "publisher and subscriber did not match")
                    deadline = time.monotonic() + 0.2
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.02)
                    messages.clear()
                    qos = node.get_publishers_info_by_topic("/test_imu")[0].qos_profile
                    self.assertEqual(qos.reliability, ReliabilityPolicy.BEST_EFFORT)
                    corrupt = bytearray(self.frame)
                    corrupt[-1] ^= 0x40
                    os.write(master, corrupt)
                    for part in (self.frame[:1], self.frame[1:9], self.frame[9:]):
                        os.write(master, part)
                    self.wait_for(node, lambda: len(messages) == 1)
                    message = messages[0]
                    self.assertEqual(message.header.frame_id, "test_imu_link")
                    self.assertGreater(message.header.stamp.sec, 1_000_000_000)
                    self.assertAlmostEqual(message.linear_acceleration.z, 9.80665, places=5)
                    self.assertAlmostEqual(message.angular_velocity.x, 0.01, places=6)
                    self.assertAlmostEqual(message.angular_velocity.y, 0.02, places=6)
                    self.assertAlmostEqual(message.angular_velocity.z, -0.03, places=6)
                    self.assertEqual(message.orientation_covariance[0], -1.0)
                    self.assertEqual(list(message.linear_acceleration_covariance),
                                     [0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.3])
                    self.assertEqual(list(message.angular_velocity_covariance),
                                     [0.4, 0, 0, 0, 0.5, 0, 0, 0, 0.6])
                    os.write(master, self.frame + self.next_frame)
                    self.wait_for(node, lambda: len(messages) >= 2)
                    self.assertEqual(len(messages), 2, "duplicate timestamp was published")

                    os.close(master)
                    master = None
                    os.close(slave)
                    slave = None
                    new_master, new_slave = pty.openpty()
                    master, slave = new_master, new_slave
                    link.unlink()
                    link.symlink_to(os.ttyname(slave))
                    def reconnected():
                        log.seek(0)
                        return log.read().count("Reading ") >= 2
                    self.wait_for(node, reconnected)
                    os.write(master, self.frame)
                    self.wait_for(node, lambda: len(messages) >= 3)
                    self.assertIsNone(child.poll())
                finally:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
                    if master is not None:
                        os.close(master)
                    if slave is not None:
                        os.close(slave)
                    node.destroy_node()
                    log.seek(0)
                    print(log.read())
                self.assertEqual(child.returncode, 0)

    def test_invalid_parameters(self):
        for parameter in ("system_id:=256", "component_id:=-1", "sensor_id:=256",
                          "data_timeout:=0.0", "linear_acceleration_variance:=[1.0, 2.0]",
                          "angular_velocity_variance:=[-1.0, 0.0, 0.0]"):
            with self.subTest(parameter=parameter):
                result = subprocess.run([BINARY, "--ros-args", "-p", parameter],
                                        capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
