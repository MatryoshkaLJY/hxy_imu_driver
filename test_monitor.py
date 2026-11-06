#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Exercise the built C++ client through binary files and a real POSIX pseudo tty."""
import csv
import io
import os
from pathlib import Path
import pty
import selectors
import subprocess
import sys
import tempfile
import unittest

BINARY, GENERATOR = map(lambda s: str(Path(s).resolve()), sys.argv[1:3])
sys.argv[1:] = []


class MonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wire = subprocess.check_output([GENERATOR], timeout=5)

    def test_replay_csv_units_sequence_wrap(self):
        with tempfile.TemporaryDirectory() as directory:
            raw, csv_path = Path(directory) / "raw.bin", Path(directory) / "samples.csv"
            raw.write_bytes(self.wire)
            result = subprocess.run([BINARY, "--file", str(raw), "--csv", str(csv_path)],
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("frames=260 imu=260 rate_hz=100.000", result.stdout)
            self.assertIn("crc_errors=0 field_errors=0 timestamp_errors=0 seq_gaps=0", result.stdout)
            records = list(csv.DictReader(io.StringIO(csv_path.read_text())))
            self.assertEqual(len(records), 260)
            for i, record in enumerate(records):
                self.assertEqual(int(record["time_usec"]), 1000000 + i * 10000)
                self.assertEqual(int(record["seq"]), i % 256)
                self.assertEqual((record["sysid"], record["compid"], record["id"]), ("1", "200", "0"))
                self.assertAlmostEqual(float(record["zacc"]), -9.80665, places=5)
                self.assertAlmostEqual(float(record["ygyro"]), -0.02, places=6)

    def test_crc_corruption_and_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            raw = Path(directory) / "corrupt.bin"
            length = self.wire[1] + 12
            corrupt = bytearray(self.wire[:length])
            corrupt[-1] ^= 0x40
            raw.write_bytes(corrupt + self.wire)
            result = subprocess.run([BINARY, "--file", str(raw)], capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertIn("frames=260 imu=260", result.stdout)
            self.assertIn("crc_errors=1", result.stdout)

    def test_no_frames_and_invalid_arguments(self):
        with tempfile.NamedTemporaryFile() as raw:
            result = subprocess.run([BINARY, "--file", raw.name], capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 1)
            for value in ("nan", "-1", "0", "1oops"):
                result = subprocess.run([BINARY, "--duration", value], capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 2)

    def start_pty(self):
        master, slave = pty.openpty()
        child = subprocess.Popen([BINARY, "--port", os.ttyname(slave), "--duration", "0.6"],
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(child.stdout, selectors.EVENT_READ)
                self.assertTrue(selector.select(3), "client did not open pseudo tty")
            first = child.stdout.readline()
            self.assertIn("Reading /dev/pts/", first)
        except BaseException:
            child.kill()
            child.communicate()
            os.close(master)
            raise
        finally:
            os.close(slave)
        return master, child

    def test_serial_split_and_coalesced_frames(self):
        master, child = self.start_pty()
        try:
            offset = 0
            for size in (1, 2, 7, 64, 73, 128, 4096):
                block = self.wire[offset:offset + size]
                offset += os.write(master, block)
            while offset < len(self.wire):
                offset += os.write(master, self.wire[offset:offset + 256])
            stdout, stderr = child.communicate(timeout=4)
            self.assertEqual(child.returncode, 0, stderr)
            self.assertIn("frames=260 imu=260 rate_hz=100.000", stdout)
        finally:
            if child.poll() is None:
                child.kill()
                child.communicate()
            os.close(master)

    def test_serial_disconnect(self):
        master, child = self.start_pty()
        os.close(master)
        stdout, stderr = child.communicate(timeout=4)
        self.assertEqual(child.returncode, 2, stdout)
        self.assertIn("disconnected", stderr)


if __name__ == "__main__":
    unittest.main()
