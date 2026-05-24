#!/usr/bin/env python3
"""
vesc_tune.py — host-side driver for the ESP32-S3 tuning CLI.

Talks to the firmware over a line-oriented byte stream. The default
transport is the USB-UART bridge at /dev/ttyUSB0 @ 115200 (the firmware
exposes the CLI on UART0). A WiFi TCP transport is planned firmware-side
and will work here unchanged once a `--host`/`--port` flag is wired in.

Wire protocol (matches firmware/main/tune_cli.h):

  ──> commands (one per line, '\\n'):
      help / ping / status / enable / disable / stop
      rpm <L|R|B> <erpm>
      step <L|R|B> <e0> <e1> <ms>
      chirp <L|R|B> <amp> <f0_hz> <f1_hz> <ms>
      log <hz>     (0..100)

  <── responses (prefix-filtered):
      OK / ERR,<msg> / R,<text> / E,<event>
      T,<t_us>,<tgtL>,<tgtR>,<erpmL>,<erpmR>,
        <iL>,<iR>,<dutyL>,<dutyR>,<vin>

Quick start (ground the wheels first!):

  # Interactive REPL (anything you type is sent verbatim):
  python3 vesc_tune.py --repl

  # Scripted step response, save CSV:
  python3 vesc_tune.py step --side L --e0 0 --e1 300 --ms 800 \\
      --log-hz 100 --out step_L_300.csv

  # Chirp on both wheels:
  python3 vesc_tune.py chirp --side B --amp 200 --f0 0.2 --f1 5 \\
      --ms 8000 --log-hz 100 --out chirp_B.csv

Safety:
  - Wheels off the ground for first runs.
  - Ctrl-C sends `stop` before exiting.
  - The firmware's override watchdog (150 ms) stops motors on
    disconnect.
  - RC failsafe and VESC health watchdog remain authoritative on
    the robot side.
"""

from __future__ import annotations

import argparse
import csv
import queue
import signal
import socket
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Optional, TextIO

try:
    import serial   # type: ignore[import-not-found]  # pyserial; UART transport only
except ImportError:
    serial = None   # type: ignore[assignment]


# ── Transport ──────────────────────────────────────────────────────

class _LinkBase:
    """Line-oriented duplex link. Subclasses provide _read_chunk / _write_raw."""

    def __init__(self) -> None:
        self._rx_buf = bytearray()
        self._lines: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._reader, daemon=True)

    def _start(self) -> None:
        self._thr.start()

    # Subclass hooks ------------------------------------------------
    def _read_chunk(self) -> bytes:
        raise NotImplementedError

    def _write_raw(self, data: bytes) -> None:
        raise NotImplementedError

    def close(self) -> None:
        self._stop.set()

    # Reader thread -------------------------------------------------
    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self._read_chunk()
            except OSError:
                break
            if not chunk:
                continue
            self._rx_buf.extend(chunk)
            while True:
                idx = self._rx_buf.find(b"\n")
                if idx < 0:
                    break
                raw = bytes(self._rx_buf[:idx]).rstrip(b"\r")
                del self._rx_buf[: idx + 1]
                try:
                    line = raw.decode("utf-8", errors="replace")
                except Exception:
                    continue
                self._lines.put(line)

    # Public API ----------------------------------------------------
    def send(self, line: str) -> None:
        if not line.endswith("\n"):
            line += "\n"
        self._write_raw(line.encode("utf-8"))

    def recv_line(self, timeout: float = 1.0) -> Optional[str]:
        try:
            return self._lines.get(timeout=timeout)
        except queue.Empty:
            return None

    def lines(self, timeout: float) -> Iterable[str]:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            line = self.recv_line(timeout=remaining)
            if line is None:
                return
            yield line


class SerialLink(_LinkBase):
    """Line-oriented duplex link over a pyserial Serial."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1):
        if serial is None:
            raise RuntimeError("pyserial not installed; pip install pyserial")
        super().__init__()
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self._start()

    def _read_chunk(self) -> bytes:
        try:
            return self.ser.read(256)
        except Exception as e:  # serial.SerialException, etc.
            raise OSError(str(e)) from e

    def _write_raw(self, data: bytes) -> None:
        self.ser.write(data)
        self.ser.flush()

    def close(self) -> None:
        super().close()
        try:
            self.ser.close()
        except Exception:
            pass


class TcpLink(_LinkBase):
    """Line-oriented duplex link over a TCP socket to the firmware."""

    def __init__(self, host: str, port: int = 3334, timeout: float = 5.0):
        super().__init__()
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(0.2)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._start()

    def _read_chunk(self) -> bytes:
        try:
            return self.sock.recv(512)
        except socket.timeout:
            return b""
        except OSError as e:
            raise OSError(str(e)) from e

    def _write_raw(self, data: bytes) -> None:
        self.sock.sendall(data)

    def close(self) -> None:
        super().close()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass


# Backwards-compatible alias used by older callers / notebooks.
LineLink = SerialLink


# ── Helpers ────────────────────────────────────────────────────────

@dataclass
class TelemetryRow:
    t_us: int
    tgt_l: int
    tgt_r: int
    erpm_l: int
    erpm_r: int
    i_l: float
    i_r: float
    duty_l: float
    duty_r: float
    vin: float

    @classmethod
    def parse(cls, line: str) -> Optional["TelemetryRow"]:
        if not line.startswith("T,"):
            return None
        parts = line[2:].split(",")
        if len(parts) != 10:
            return None
        try:
            return cls(
                t_us=int(parts[0]),
                tgt_l=int(parts[1]),
                tgt_r=int(parts[2]),
                erpm_l=int(parts[3]),
                erpm_r=int(parts[4]),
                i_l=float(parts[5]),
                i_r=float(parts[6]),
                duty_l=float(parts[7]),
                duty_r=float(parts[8]),
                vin=float(parts[9]),
            )
        except ValueError:
            return None


def wait_ok(link: _LinkBase, timeout: float = 2.0) -> None:
    """Read response lines until OK / ERR. Raise on ERR."""
    for line in link.lines(timeout):
        if line.startswith("T,") or line.startswith("R,") or line.startswith("E,"):
            continue
        if line == "OK":
            return
        if line.startswith("ERR,"):
            raise RuntimeError(f"firmware rejected: {line}")
        # ignore ESP_LOG / boot noise
    raise TimeoutError("no OK/ERR within timeout")


def run_experiment(
    link: _LinkBase,
    command: str,
    duration_s: float,
    log_hz: int,
    out_path: Optional[str],
) -> int:
    """Send `enable`, configure log, send the experiment command, capture telemetry."""
    link.send("enable");  wait_ok(link)
    link.send(f"log {log_hz}"); wait_ok(link)
    link.send(command);   wait_ok(link)

    rows: list[TelemetryRow] = []
    deadline = time.monotonic() + duration_s + 0.5
    try:
        while time.monotonic() < deadline:
            line = link.recv_line(timeout=0.2)
            if line is None:
                continue
            row = TelemetryRow.parse(line)
            if row is not None:
                rows.append(row)
            elif line.startswith("E,exp_done"):
                # let a little tail-data come in
                deadline = min(deadline, time.monotonic() + 0.3)
    finally:
        link.send("stop")
        try:
            wait_ok(link, timeout=1.0)
        except Exception:
            pass

    if out_path:
        parent = os.path.dirname(os.path.abspath(out_path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "t_us", "tgt_l", "tgt_r", "erpm_l", "erpm_r",
                "i_l", "i_r", "duty_l", "duty_r", "vin",
            ])
            for r in rows:
                w.writerow([
                    r.t_us, r.tgt_l, r.tgt_r, r.erpm_l, r.erpm_r,
                    f"{r.i_l:.3f}", f"{r.i_r:.3f}",
                    f"{r.duty_l:.4f}", f"{r.duty_r:.4f}",
                    f"{r.vin:.2f}",
                ])
        print(f"wrote {len(rows)} samples to {out_path}", file=sys.stderr)
    else:
        print(f"captured {len(rows)} samples (no --out, discarded)", file=sys.stderr)
    return 0


def cmd_repl(link: _LinkBase) -> int:
    """Interactive: stdin → device, device → stdout."""
    stop = threading.Event()

    def printer() -> None:
        while not stop.is_set():
            line = link.recv_line(timeout=0.2)
            if line is not None:
                print(line)

    t = threading.Thread(target=printer, daemon=True)
    t.start()
    try:
        for line in sys.stdin:
            line = line.rstrip("\r\n")
            if not line:
                continue
            link.send(line)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        try:
            link.send("stop")
        except Exception:
            pass
    return 0


# ── CLI ────────────────────────────────────────────────────────────

def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(
        description=(__doc__ or "").splitlines()[1] if __doc__ else "")
    # Transport selection: --host implies TCP, else UART on --port.
    p.add_argument("--host",
                   help="WiFi transport: ESP32 IP/hostname. Implies TCP.")
    p.add_argument("--tcp-port", type=int, default=3334,
                   help="TCP port for WiFi transport (default 3334)")
    p.add_argument("--port", default="/dev/ttyUSB0",
                   help="UART transport: serial device (default /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=115200)

    sub = p.add_subparsers(dest="cmd", required=True)

    pr = sub.add_parser("repl", help="interactive line passthrough")

    pr2 = sub.add_parser("status", help="query status once and exit")

    ps = sub.add_parser("step", help="step-response experiment")
    ps.add_argument("--side", choices=["L", "R", "B"], default="L")
    ps.add_argument("--e0", type=int, default=0)
    ps.add_argument("--e1", type=int, required=True)
    ps.add_argument("--ms", type=int, default=800)
    ps.add_argument("--log-hz", type=int, default=100)
    ps.add_argument("--out", help="CSV output path")

    pc = sub.add_parser("chirp", help="frequency-sweep experiment")
    pc.add_argument("--side", choices=["L", "R", "B"], default="L")
    pc.add_argument("--amp", type=int, required=True)
    pc.add_argument("--f0", type=float, default=0.2)
    pc.add_argument("--f1", type=float, default=5.0)
    pc.add_argument("--ms", type=int, default=8000)
    pc.add_argument("--log-hz", type=int, default=100)
    pc.add_argument("--out", help="CSV output path")

    args = p.parse_args(argv)

    link: _LinkBase
    if args.host:
        link = TcpLink(args.host, args.tcp_port)
    else:
        link = SerialLink(args.port, args.baud)

    # Ctrl-C → stop motors then exit
    def on_sigint(_sig, _frm):
        try:
            link.send("stop")
        except Exception:
            pass
        link.close()
        sys.exit(130)
    signal.signal(signal.SIGINT, on_sigint)

    try:
        if args.cmd == "repl":
            return cmd_repl(link)

        if args.cmd == "status":
            link.send("status")
            for line in link.lines(timeout=1.0):
                if line.startswith("R,"):
                    print(line)
                    return 0
            print("no response", file=sys.stderr)
            return 1

        if args.cmd == "step":
            duration = (2 * args.ms) / 1000.0
            cmd = f"step {args.side} {args.e0} {args.e1} {args.ms}"
            return run_experiment(link, cmd, duration, args.log_hz, args.out)

        if args.cmd == "chirp":
            duration = args.ms / 1000.0
            cmd = (f"chirp {args.side} {args.amp} "
                   f"{args.f0} {args.f1} {args.ms}")
            return run_experiment(link, cmd, duration, args.log_hz, args.out)

        return 2
    finally:
        try:
            link.send("stop")
        except Exception:
            pass
        link.close()


if __name__ == "__main__":
    sys.exit(main())
