#!/usr/bin/env python3
"""Smoke test: feed NMEA into a PTY, verify the UM982 driver publishes ~/fix."""
import os
import pty
import subprocess
import sys
import time
import threading


def main():
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    print(f"PTY slave={slave_name}", flush=True)

    def feed():
        time.sleep(3.0)
        body = (b"GNGGA,012345.00,3723.2475,N,12158.3416,W,4,12,0.7,18.20,"
                b"M,-25.6,M,1.2,0102")
        cs = 0
        for c in body:
            cs ^= c
        line = b"$" + body + ("*%02X\r\n" % cs).encode()
        for _ in range(10):
            os.write(master, line)
            time.sleep(0.5)

    threading.Thread(target=feed, daemon=True).start()

    driver = subprocess.Popen([
        "ros2", "run", "um982_driver", "um982_driver_node",
        "--ros-args",
        "-p", f"port:={slave_name}",
        "-p", "baudrate:=115200",
        "-p", "unlogall_on_configure:=false",
        "-p", "mode:=skip",
    ])

    time.sleep(1.5)
    subprocess.run(["ros2", "lifecycle", "set", "/um982_driver", "configure"],
                   check=False)
    time.sleep(0.5)
    subprocess.run(["ros2", "lifecycle", "set", "/um982_driver", "activate"],
                   check=False)

    echoer = subprocess.run(
        ["ros2", "topic", "echo", "--once", "/um982_driver/fix"],
        capture_output=True, text=True, timeout=12)
    print("--- ros2 topic echo /um982_driver/fix ---")
    print(echoer.stdout)
    if echoer.returncode != 0:
        print("STDERR:", echoer.stderr, file=sys.stderr)

    driver.terminate()
    try:
        driver.wait(timeout=5)
    except subprocess.TimeoutExpired:
        driver.kill()
    ok = "latitude: 37.387458" in echoer.stdout or "STATUS_GBAS_FIX" in echoer.stdout
    print("OK" if ok else "MISSING_FIX")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
