"""Low-latency serial driver for the Roomba Open Interface.

Design notes (measured on a Roomba 690, see docs/RESEARCH.md):

  * A blocking Sensors(142) round-trip costs ~15.7 ms, because the robot
    refreshes its sensor bank every 15 ms. Doing that on the control path
    would add ~16 ms of jitter to every keypress.
  * A DriveDirect(145) write costs ~1.17 ms.

So: telemetry uses Stream(148), which the robot pushes at ~64 Hz with no
request needed, drained by a background thread. The control path only ever
writes. The two never block each other.
"""
from __future__ import annotations

import math
import threading
import time

import serial

from . import protocol as P


class RoombaError(RuntimeError):
    pass


class RoombaOI:
    def __init__(self, port: str, baud: int = 115200, watchdog_s: float = 0.35,
                 passive_only: bool = False):
        self.port = port
        self.baud = baud
        self.watchdog_s = watchdog_s
        # Monitor mode: never leave Passive. Safe/Full terminate charging and
        # never sleep, so a docked robot must stay in Passive to keep charging.
        self.passive_only = passive_only

        self._ser: serial.Serial | None = None
        self._wlock = threading.Lock()      # serialises writes across threads
        self._slock = threading.Lock()      # guards _state
        self._state: dict = {}
        self._reader: threading.Thread | None = None
        self._running = False

        # This 690 reports OI mode one higher than the published spec, so the
        # offset is calibrated at connect time instead of being hardcoded.
        self._mode_offset = 0

        # Odometry integrated from the wheel encoders.
        self._enc_prev: tuple[int, int] | None = None
        self._heading_rad = 0.0
        self._dist_mm = 0.0

        self.last_drive: tuple[int, int] = (0, 0)
        self._last_drive_sent = 0.0
        self.frames_ok = 0
        self.frames_bad = 0
        self.connected_at = 0.0

    # ------------------------------------------------------------ plumbing --
    def _write(self, data: bytes) -> None:
        if self._ser is None:
            raise RoombaError("not connected")
        with self._wlock:
            self._ser.write(data)

    def connect(self) -> None:
        self._ser = serial.Serial(
            self.port, self.baud, timeout=0.05, write_timeout=1.0,
            bytesize=8, parity="N", stopbits=1,
            rtscts=False, dsrdtr=False, xonxoff=False,
        )
        time.sleep(0.15)
        self._ser.reset_input_buffer()

        self._write(bytes([P.PAUSE_STREAM, 0]))   # clear any stale stream
        time.sleep(0.1)
        self._ser.reset_input_buffer()

        self._write(bytes([P.START]))             # -> Passive
        time.sleep(0.25)
        if not self.passive_only:
            self._write(bytes([P.SAFE]))          # -> Safe
            time.sleep(0.25)

        self._calibrate_mode_offset()

        self._ser.reset_input_buffer()
        self._write(bytes([P.STREAM, len(P.STREAM_PACKETS)]) + bytes(P.STREAM_PACKETS))

        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True,
                                        name="roomba-reader")
        self._reader.start()
        self.connected_at = time.time()

    def wait_awake(self, timeout: float = 2.0) -> bool:
        """True once sensor frames are actually arriving.

        A sleeping Roomba accepts bytes silently and answers nothing, so an
        open port proves nothing -- the stream is the only real liveness test.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.frames_ok > 0:
                return True
            time.sleep(0.05)
        return False

    def rewake(self) -> None:
        """Re-issue the OI handshake, e.g. after the robot was woken by hand."""
        if self._ser is None:
            raise RoombaError("not connected")
        self._ser.reset_input_buffer()
        self._write(bytes([P.START]))
        time.sleep(0.25)
        if not self.passive_only:
            self._write(bytes([P.SAFE]))
            time.sleep(0.25)
        self._calibrate_mode_offset()
        self._write(bytes([P.STREAM, len(P.STREAM_PACKETS)]) + bytes(P.STREAM_PACKETS))

    def _calibrate_mode_offset(self) -> None:
        """We just commanded Safe, so whatever mode byte comes back must mean
        Safe. Deriving the offset here makes the driver correct on both
        spec-compliant firmware and this 690's off-by-one variant."""
        # Sensor packets are readable in Passive, so calibration works there too;
        # the reference mode is whichever one we actually commanded.
        expected = P.MODE_PASSIVE if self.passive_only else P.MODE_SAFE
        raw = None
        for _ in range(3):
            self._ser.reset_input_buffer()
            self._write(bytes([P.SENSORS, P.NAME_TO_ID["oi_mode"]]))
            d = self._ser.read(1)
            if d:
                raw = d[0]
                break
            time.sleep(0.05)
        if raw is not None and 0 <= raw <= 8:
            self._mode_offset = raw - expected

    def normalise_mode(self, raw: int | None) -> int | None:
        if raw is None:
            return None
        return raw - self._mode_offset

    # -------------------------------------------------------- reader thread --
    def _read_loop(self) -> None:
        buf = bytearray()
        while self._running:
            try:
                chunk = self._ser.read(1024)
            except Exception:
                break
            if chunk:
                buf += chunk
                self._consume(buf)
            if len(buf) > 8192:
                del buf[:-1024]

    def _consume(self, buf: bytearray) -> None:
        """Resynchronising frame parser: [19][n][payload...][checksum]."""
        while True:
            start = buf.find(P.STREAM_HEADER)
            if start < 0:
                buf.clear()
                return
            if start:
                del buf[:start]
            if len(buf) < 2:
                return
            n = buf[1]
            total = 2 + n + 1
            if len(buf) < total:
                return
            frame = bytes(buf[:total])
            if P.checksum_ok(frame):
                values = P.parse_frame(frame)
                values["_t"] = time.time()
                with self._slock:
                    self._state.update(values)
                    self._integrate(values)
                self.frames_ok += 1
                del buf[:total]
            else:
                # Bad checksum: drop one byte and hunt for the next header.
                self.frames_bad += 1
                del buf[:1]

    def _integrate(self, values: dict) -> None:
        """Dead-reckon heading and distance from encoder deltas.

        Encoders are signed 16-bit and roll over (~14.5 m), so deltas are taken
        modulo 65536. Caveat from the spec: these are square-wave, not
        quadrature -- they count using the *commanded* direction, so shoving the
        robot by hand corrupts the estimate. Zero the heading to recover."""
        l, r = values.get("encoder_left"), values.get("encoder_right")
        if l is None or r is None:
            return
        if self._enc_prev is None:
            self._enc_prev = (l, r)
            return
        pl, pr = self._enc_prev
        dl = ((l - pl + 32768) % 65536) - 32768
        dr = ((r - pr + 32768) % 65536) - 32768
        self._enc_prev = (l, r)

        # Ignore implausible jumps (a dropped frame or a rollover artefact).
        if abs(dl) > 2000 or abs(dr) > 2000:
            return

        dl_mm, dr_mm = dl * P.MM_PER_COUNT, dr * P.MM_PER_COUNT
        self._heading_rad += (dr_mm - dl_mm) / P.WHEELBASE_MM   # CCW positive
        self._dist_mm += (dl_mm + dr_mm) / 2.0

    def reset_odometry(self) -> None:
        with self._slock:
            self._heading_rad = 0.0
            self._dist_mm = 0.0

    # ------------------------------------------------------------ telemetry --
    def sensors(self) -> dict:
        with self._slock:
            s = dict(self._state)
        raw = s.get("oi_mode")
        s["oi_mode_raw"] = raw
        s["oi_mode"] = self.normalise_mode(raw)
        s["oi_mode_name"] = P.OI_MODES.get(s["oi_mode"], "unknown")
        s["frames_ok"] = self.frames_ok
        s["frames_bad"] = self.frames_bad

        bd = s.get("bumps_wheeldrops", 0) or 0
        s["bump_left"]  = bool(bd & P.BUMP_LEFT)
        s["bump_right"] = bool(bd & P.BUMP_RIGHT)
        s["wheeldrop_left"]  = bool(bd & P.WHEELDROP_LEFT)
        s["wheeldrop_right"] = bool(bd & P.WHEELDROP_RIGHT)

        lb = s.get("light_bumper", 0) or 0
        s["light_bump"] = {name: bool(lb & bit) for name, bit in P.LIGHT_BUMPER_BITS}

        # Compass-style: 0 = orientation at start/reset, increasing clockwise.
        ccw_deg = math.degrees(self._heading_rad)
        s["heading_deg"] = round((-ccw_deg) % 360.0, 1)
        s["heading_ccw_deg"] = round(ccw_deg, 1)
        s["distance_mm"] = round(self._dist_mm, 1)

        cs = s.get("charging_state")
        s["charging_state_name"] = P.CHARGING_STATES.get(cs, "unknown")
        chg, cap = s.get("charge_mah"), s.get("capacity_mah")
        s["battery_pct"] = round(100 * chg / cap) if chg and cap else None
        return s

    # -------------------------------------------------------------- control --
    def drive(self, left: int, right: int, force: bool = False) -> bool:
        """Set wheel velocities in mm/s. Skips redundant writes unless forced."""
        if self.passive_only:
            return False
        pair = (int(left), int(right))
        now = time.time()
        if not force and pair == self.last_drive and (now - self._last_drive_sent) < 0.2:
            return False
        self._write(P.drive_direct(pair[1], pair[0]))   # wire order: right, left
        self.last_drive = pair
        self._last_drive_sent = now
        return True

    def stop(self) -> None:
        self.drive(0, 0, force=True)

    def set_mode(self, mode: str) -> None:
        if self.passive_only and mode != "passive":
            raise RoombaError("monitor mode: refusing to leave Passive "
                              "(Safe/Full would stop charging)")
        op = {"safe": P.SAFE, "full": P.FULL, "passive": P.START}.get(mode)
        if op is None:
            raise RoombaError(f"unknown mode {mode!r}")
        self._write(bytes([op]))

    def power_off(self) -> None:
        """Opcode 133. Powers the robot down.

        This is the only reliable way to guarantee it will not resume a
        cleaning cycle: a paused cycle survives in Passive mode and restarts
        the moment the robot leaves the dock."""
        if self._ser is None:
            raise RoombaError("not connected")
        self._write(P.drive_direct(0, 0))
        time.sleep(0.05)
        self._write(bytes([P.POWER]))

    def command(self, opcode: int, data: bytes = b"") -> None:
        if self.passive_only:
            raise RoombaError("monitor mode: actuator commands are disabled")
        self._write(bytes([opcode]) + data)

    def close(self) -> None:
        """Always leave the robot safe: motors off, stream off, Passive mode.

        Safe/Full never sleep and terminate charging, so parking in Passive is
        what protects the battery."""
        self._running = False
        if self._reader:
            self._reader.join(timeout=1.0)
        if self._ser is None:
            return
        try:
            if not self.passive_only:
                self._write(P.drive_direct(0, 0))
                time.sleep(0.05)
            self._write(bytes([P.PAUSE_STREAM, 0]))
            time.sleep(0.05)
            self._write(bytes([P.START]))          # -> Passive
            time.sleep(0.05)
        except Exception:
            pass
        finally:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
