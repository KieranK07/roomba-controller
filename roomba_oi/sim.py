"""A simulated Roomba for running the server with no robot attached.

SimRoomba is a RoombaOI that never opens a serial port. The bytes the driver
would write are interpreted here instead, and a background thread plays the
robot's side: it integrates the commanded wheel speeds, keeps a pose inside a
walled 4 x 3 m room, and encodes real Stream(148) frames that go through the
driver's own parser, checksum and odometry. So everything above the serial
port -- server.py, the UI, the heading dead-reckoning -- runs unchanged.

What it models: OI modes (drive commands only move the wheels in Safe/Full),
wheel encoders, bumpers and light bumpers against the walls, battery drain.
What it does not: cliffs, wheel drops, docking, the cleaning behaviours.
"""
from __future__ import annotations

import math
import random
import threading
import time

from . import protocol as P
from .driver import RoombaOI

STREAM_HZ = 64                  # what a real 690 pushes
ROOM_W, ROOM_H = 4000.0, 3000.0  # mm, centred on the start position
RADIUS = 170.0                  # mm, half the 690's 340 mm body
LIGHT_RANGE = 150.0             # mm, how far ahead the light bumpers see
CAPACITY = 2050                 # mAh


def _enc(value: int, n: int, signed: bool) -> bytes:
    if signed:
        lo, hi = -(1 << (8 * n - 1)), (1 << (8 * n - 1)) - 1
    else:
        lo, hi = 0, (1 << (8 * n)) - 1
    return max(lo, min(hi, int(value))).to_bytes(n, "big", signed=signed)


def encode_frame(values: dict) -> bytes:
    """A Stream(148) frame carrying STREAM_PACKETS: [19][n][id][data]...[checksum]."""
    body = bytearray()
    for pid in P.STREAM_PACKETS:
        name, n, signed = P.PACKETS[pid]
        body.append(pid)
        body += _enc(values.get(name, 0), n, signed)
    frame = bytearray([P.STREAM_HEADER, len(body)]) + body
    frame.append((-sum(frame)) & 0xFF)
    return bytes(frame)


class SimRoomba(RoombaOI):
    def __init__(self, watchdog_s: float = 0.35, passive_only: bool = False,
                 seed: int | None = None):
        super().__init__("sim", watchdog_s=watchdog_s, passive_only=passive_only)
        self._rng = random.Random(seed)
        self._mode = P.MODE_OFF
        self._powered = True
        self._wheels = (0, 0)           # left, right in mm/s, as commanded
        self._x = self._y = 0.0         # mm, room frame
        self._theta = math.pi / 2       # facing +y
        self._counts = [0.0, 0.0]       # left, right encoder counts
        self._charge = CAPACITY * 0.82
        self._bumps = 0
        self._light = 0
        self._current = -180

    # --------------------------------------------------- the "serial" link --
    def _write(self, data: bytes) -> None:
        with self._wlock:
            self._interpret(data)

    def _interpret(self, data: bytes) -> None:
        if not data:
            return
        op = data[0]
        if op == P.DRIVE_DIRECT and len(data) == 5:
            right = int.from_bytes(data[1:3], "big", signed=True)
            left = int.from_bytes(data[3:5], "big", signed=True)
            # The real robot ignores drive commands outside Safe/Full.
            if self._mode in (P.MODE_SAFE, P.MODE_FULL):
                self._wheels = (left, right)
        elif op == P.START:
            self._powered = True
            self._mode = P.MODE_PASSIVE
            self._wheels = (0, 0)
        elif op in (P.SAFE, P.CONTROL):
            self._mode = P.MODE_SAFE
        elif op == P.FULL:
            self._mode = P.MODE_FULL
        elif op == P.POWER:
            self._powered = False
            self._mode = P.MODE_OFF
            self._wheels = (0, 0)
        elif op in (P.SEEK_DOCK, P.CLEAN, P.SPOT, P.MAX):
            # The robot takes over and drops to Passive. Its behaviours are
            # not simulated, so it just sits there.
            self._mode = P.MODE_PASSIVE
            self._wheels = (0, 0)
        # STREAM, PAUSE_STREAM, SENSORS, MOTORS: nothing to model.

    # ------------------------------------------------------- driver hooks --
    def connect(self) -> None:
        self._interpret(bytes([P.START]))
        if not self.passive_only:
            self._interpret(bytes([P.SAFE]))
        self._mode_offset = 0           # the sim reports modes per the spec
        self._running = True
        self._reader = threading.Thread(target=self._sim_loop, daemon=True,
                                        name="roomba-sim")
        self._reader.start()
        self.connected_at = time.time()

    def rewake(self) -> None:
        """Stands in for pressing CLEAN: a powered-off sim comes back."""
        self._write(bytes([P.START]))
        if not self.passive_only:
            self._write(bytes([P.SAFE]))

    def power_off(self) -> None:
        self._write(P.drive_direct(0, 0))
        self._write(bytes([P.POWER]))

    # -------------------------------------------------------- the robot ----
    def _step(self, dt: float) -> None:
        left, right = self._wheels
        v = (left + right) / 2.0
        w = (right - left) / P.WHEELBASE_MM
        theta = self._theta + w * dt
        x = self._x + v * math.cos(theta) * dt
        y = self._y + v * math.sin(theta) * dt

        xmax, ymax = ROOM_W / 2 - RADIUS, ROOM_H / 2 - RADIUS
        blocked = not (-xmax <= x <= xmax and -ymax <= y <= ymax)
        self._theta = theta
        if not blocked:
            self._x, self._y = x, y
            self._counts[0] += left * dt / P.MM_PER_COUNT
            self._counts[1] += right * dt / P.MM_PER_COUNT
        else:
            # Spinning in place still turns the wheels; pushing into a wall
            # does not.
            self._counts[0] += (left - v) * dt / P.MM_PER_COUNT
            self._counts[1] += (right - v) * dt / P.MM_PER_COUNT

        # The bumper closes when the robot drives into a wall; which side
        # depends on which half of the bumper meets it first.
        self._bumps = 0
        if blocked:
            dl = self._wall_distance(math.radians(30))
            dr = self._wall_distance(math.radians(-30))
            if dl <= dr + 10:
                self._bumps |= P.BUMP_LEFT
            if dr <= dl + 10:
                self._bumps |= P.BUMP_RIGHT
        self._light = 0
        for name, bit in P.LIGHT_BUMPER_BITS:
            bearing = {"left": 70, "front_left": 40, "center_left": 12,
                       "center_right": -12, "front_right": -40, "right": -70}[name]
            if self._wall_distance(math.radians(bearing)) < LIGHT_RANGE:
                self._light |= bit

        draw = 180 + 0.9 * (abs(left) + abs(right))       # mA
        self._current = -int(draw + self._rng.uniform(-8, 8))
        self._charge = max(0.0, self._charge - draw * dt / 3600.0)

    def _wall_distance(self, bearing: float) -> float:
        """Gap between the body and the nearest wall along a body-relative bearing."""
        a = self._theta + bearing
        dx, dy = math.cos(a), math.sin(a)
        hits = []
        if dx > 1e-6:
            hits.append((ROOM_W / 2 - self._x) / dx)
        elif dx < -1e-6:
            hits.append((-ROOM_W / 2 - self._x) / dx)
        if dy > 1e-6:
            hits.append((ROOM_H / 2 - self._y) / dy)
        elif dy < -1e-6:
            hits.append((-ROOM_H / 2 - self._y) / dy)
        return min(hits) - RADIUS

    def _values(self) -> dict:
        moving = self._wheels != (0, 0)
        return {
            "oi_mode": self._mode,
            "bumps_wheeldrops": self._bumps,
            "light_bumper": self._light,
            "wheel_overcurrents": 0,
            "stasis": 1 if moving else 0,
            "charging_state": 0,
            "charging_sources": 0,
            "voltage_mv": int(14100 + 900 * self._charge / CAPACITY
                              + self._rng.uniform(-15, 15)),
            "current_ma": self._current,
            "temperature_c": 27,
            "charge_mah": int(self._charge),
            "capacity_mah": CAPACITY,
            # Signed 16-bit on the wire and rolling over, like the real ones.
            "encoder_left": ((int(self._counts[0]) + 32768) % 65536) - 32768,
            "encoder_right": ((int(self._counts[1]) + 32768) % 65536) - 32768,
        }

    def _sim_loop(self) -> None:
        period = 1.0 / STREAM_HZ
        last = time.time()
        buf = bytearray()
        while self._running:
            time.sleep(period)
            now = time.time()
            dt, last = now - last, now
            with self._wlock:
                if not self._powered:
                    continue            # a powered-off robot streams nothing
                self._step(dt)
                frame = encode_frame(self._values())
            buf += frame
            self._consume(buf)

    def sensors(self) -> dict:
        s = super().sensors()
        s["sim"] = True
        return s
