#!/usr/bin/env python3
"""Regression tests for the three WebSocket trust-boundary gates in server.py.

These guard holes that were open until 2026-09-15, all of which let a caller
that should have no authority move the robot:

  1. no Origin check      -> any web page you visited could drive the robot
  2. e-stop did not gate the raw-opcode path
  3. arbitrary OI opcodes were forwarded to the serial link

Plus the link gate added when the server was made to boot without the robot:
opcodes must not be forwarded while nothing is answering on the serial line.

No hardware needed: the robot is a stub that records what it was told.
The last group checks the --sim robot, which must never open a serial port.
Run with `arch -arm64 .venv/bin/python tests/test_safety_gates.py` (the venv's
numpy is arm64-only and this Mac's shell defaults to Rosetta).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import websockets

import server
from roomba_oi import protocol as P

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{'  -- ' + detail if detail and not ok else ''}")
    if not ok:
        FAILURES.append(name)


# --------------------------------------------------------------- stub robot ---
class StubBot:
    """Records commands instead of writing to the serial port."""

    def __init__(self) -> None:
        self.commands: list[tuple[int, bytes]] = []
        self.last_drive = (0, 0)
        self.passive_only = False
        self.connected_at = 0.0
        self.frames_ok = 0

    def drive(self, left: int, right: int, force: bool = False) -> None:
        self.last_drive = (left, right)

    def command(self, opcode: int, data: bytes = b"") -> None:
        self.commands.append((opcode, data))

    def sensors(self) -> dict:
        return {}


class StubWS:
    """Async-iterable stand-in for a client connection."""

    def __init__(self, frames: list[dict]) -> None:
        self._frames = [json.dumps(f) for f in frames]
        self.sent: list[str] = []

    def __aiter__(self):
        async def gen():
            for f in self._frames:
                yield f
        return gen()

    async def send(self, payload: str) -> None:
        self.sent.append(payload)


async def run_frames(frames: list[dict], estop: bool = False,
                     linked: bool = True) -> StubBot:
    bot = StubBot()
    ctl = server.Controller(bot)
    ctl.estop = estop
    ctl.linked = linked          # the robot is present unless a test says not
    await server.ws_handler(StubWS(frames), ctl)
    return bot


# ------------------------------------------------------- 3. opcode allowlist ---
def test_allowlist() -> None:
    print("\nopcode allowlist (parse_op)")

    ok_cases = [
        ("dock 143",          {"opcode": P.SEEK_DOCK},            (P.SEEK_DOCK, b"")),
        ("clean 135",         {"opcode": P.CLEAN},                (P.CLEAN, b"")),
        ("brushes on 138[7]", {"opcode": P.MOTORS, "data": [7]},  (P.MOTORS, b"\x07")),
        ("brushes off 138[0]", {"opcode": P.MOTORS, "data": [0]}, (P.MOTORS, b"\x00")),
    ]
    for name, msg, want in ok_cases:
        try:
            got = server.parse_op(msg)
            check(f"accepts {name}", got == want, f"got {got!r}")
        except ValueError as e:
            check(f"accepts {name}", False, str(e))

    bad_cases = [
        ("BAUD 129 (would drop the link)",  {"opcode": P.BAUD, "data": [0]}),
        ("RESET 7 (factory reset)",         {"opcode": P.RESET}),
        ("DRIVE 137 (bypasses mixer limits)", {"opcode": P.DRIVE, "data": [1, 2, 3, 4]}),
        ("DRIVE_PWM 146",                   {"opcode": P.DRIVE_PWM, "data": [0, 0, 0, 0]}),
        ("SONG 140",                        {"opcode": P.SONG, "data": [0, 1, 60, 8]}),
        ("FULL 132 (drops cliff safety)",   {"opcode": P.FULL}),
        ("MOTORS with undefined bit 0x20",  {"opcode": P.MOTORS, "data": [0x20]}),
        ("MOTORS with wrong payload length", {"opcode": P.MOTORS, "data": [1, 2]}),
        ("dock with a stray payload",       {"opcode": P.SEEK_DOCK, "data": [1]}),
        ("missing opcode",                  {"data": [1]}),
        ("non-integer opcode",              {"opcode": "143"}),
        ("data not a list",                 {"opcode": P.MOTORS, "data": "7"}),
        ("boolean data byte",               {"opcode": P.MOTORS, "data": [True]}),
        ("out-of-range data byte",          {"opcode": P.MOTORS, "data": [300]}),
    ]
    for name, msg in bad_cases:
        try:
            got = server.parse_op(msg)
        except ValueError:
            check(f"refuses {name}", True)
        else:
            check(f"refuses {name}", False, f"accepted, returned {got!r}")


# ------------------------------------------------------------- 2. e-stop gate ---
def test_estop_gate() -> None:
    print("\ne-stop gates the raw-opcode path")

    bot = asyncio.run(run_frames([{"t": "op", "opcode": P.SEEK_DOCK}], estop=True))
    check("e-stopped: dock command is dropped", bot.commands == [],
          f"forwarded {bot.commands!r}")

    bot = asyncio.run(run_frames([{"t": "op", "opcode": P.MOTORS, "data": [7]}], estop=True))
    check("e-stopped: brush command is dropped", bot.commands == [],
          f"forwarded {bot.commands!r}")

    bot = asyncio.run(run_frames([{"t": "op", "opcode": P.SEEK_DOCK}], estop=False))
    check("not e-stopped: dock command goes through",
          bot.commands == [(P.SEEK_DOCK, b"")], f"forwarded {bot.commands!r}")

    # Releasing e-stop mid-stream must re-open the path, not latch it shut.
    bot = asyncio.run(run_frames([
        {"t": "op", "opcode": P.SEEK_DOCK},
        {"t": "estop", "on": False},
        {"t": "op", "opcode": P.SEEK_DOCK},
    ], estop=True))
    check("release re-opens the path", bot.commands == [(P.SEEK_DOCK, b"")],
          f"forwarded {bot.commands!r}")

    # And the drive path stays gated, which is what target() always did.
    bot = asyncio.run(run_frames([{"t": "keys", "down": ["w"]}], estop=True))
    check("e-stopped: keys do not move the wheels", bot.last_drive == (0, 0),
          f"last_drive {bot.last_drive!r}")


# -------------------------------------------------------------- 4. link gate ---
def test_link_gate() -> None:
    print("\nopcodes are not forwarded to a robot that is not there")

    bot = asyncio.run(run_frames([{"t": "op", "opcode": P.SEEK_DOCK}], linked=False))
    check("not connected: dock command is dropped", bot.commands == [],
          f"forwarded {bot.commands!r}")

    bot = asyncio.run(run_frames([{"t": "op", "opcode": P.MOTORS, "data": [7]}],
                                 linked=False))
    check("not connected: brush command is dropped", bot.commands == [],
          f"forwarded {bot.commands!r}")

    # poll_link is the only thing that may declare the robot present, and only
    # arriving frames may do it.
    stub = StubBot()
    ctl = server.Controller(stub)
    check("link starts down", ctl.linked is False)
    check("an idle serial line does not bring it up",
          ctl.poll_link(1000.0) is False)
    stub.frames_ok = 1
    check("an arriving frame brings it up", ctl.poll_link(1001.0) is True)
    check("it stays up while frames keep coming",
          ctl.poll_link(1001.2) is True)
    check("it drops after LINK_TIMEOUT_S of silence",
          ctl.poll_link(1001.2 + server.LINK_TIMEOUT_S + 0.1) is False)
    stub.frames_ok = 2
    check("and comes back when frames resume", ctl.poll_link(1010.0) is True)


# ------------------------------------------------------------ 1. origin gate ---
def _args(**kw) -> argparse.Namespace:
    base = dict(host="127.0.0.1", http_port=8666, allow_origin=[])
    base.update(kw)
    return argparse.Namespace(**base)


async def _handshake(port: int, origin: str | None) -> tuple[bool, str]:
    kw = {"additional_headers": {"Origin": origin}} if origin else {}
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}", **kw):
            return True, ""
    except Exception as e:
        return False, type(e).__name__


async def _origin_cases() -> None:
    args = _args(allow_origin=["http://roomba.local:8666"])
    origins = server.allowed_origins(args)

    async def handler(ws):
        async for _ in ws:
            pass

    async with websockets.serve(handler, "127.0.0.1", 0,
                                process_request=server.origin_gate(origins)) as srv:
        port = srv.sockets[0].getsockname()[1]

        ok, err = await _handshake(port, "http://127.0.0.1:8666")
        check("accepts the UI's own origin", ok, err)

        ok, err = await _handshake(port, "http://localhost:8666")
        check("accepts localhost alias", ok, err)

        ok, err = await _handshake(port, "http://roomba.local:8666")
        check("accepts an --allow-origin entry", ok, err)

        ok, err = await _handshake(port, None)
        check("accepts a non-browser client (no Origin)", ok, err)

        for bad in ["http://evil.example", "https://evil.example",
                    "http://127.0.0.1:1234", "null",
                    "http://127.0.0.1:8666.evil.example"]:
            ok, _ = await _handshake(port, bad)
            check(f"refuses Origin {bad}", not ok, "handshake succeeded")


def test_origin_gate() -> None:
    print("\norigin gate on the control socket")
    asyncio.run(_origin_cases())

    lan = server.allowed_origins(_args(host="0.0.0.0"))
    check("binding 0.0.0.0 adds the LAN address",
          any(o not in ("http://127.0.0.1:8666", "http://localhost:8666",
                        "http://[::1]:8666") for o in lan), f"got {lan!r}")

    named = server.allowed_origins(_args(host="192.168.1.50", http_port=9000))
    check("a named bind host is allowed on its own port",
          "http://192.168.1.50:9000" in named, f"got {named!r}")


# ------------------------------------------------------------- --sim robot ---
def test_sim() -> None:
    """The --sim robot streams frames the real parser accepts and never opens a port."""
    import time

    import serial
    from roomba_oi.sim import SimRoomba, encode_frame

    print("\n--sim robot")

    opened = []
    real_serial = serial.Serial

    def no_port(*a, **k):
        opened.append(a)
        raise AssertionError("sim opened a serial port")
    serial.Serial = no_port
    try:
        frame = encode_frame({"oi_mode": 2, "encoder_left": -5, "voltage_mv": 15000})
        values = P.parse_frame(frame)
        check("encoded frame passes the real checksum", P.checksum_ok(frame))
        check("and parses back", values["oi_mode"] == 2 and values["encoder_left"] == -5
              and values["voltage_mv"] == 15000, f"got {values!r}")

        bot = SimRoomba(seed=1)
        bot.connect()
        check("streams frames", bot.wait_awake(1.0))
        check("comes up in Safe", bot.sensors()["oi_mode_name"] == "safe")

        bot.drive(200, 200, force=True)
        time.sleep(0.5)
        s = bot.sensors()
        check("driving forward moves it forward", 50 < s["distance_mm"] < 150,
              f"distance {s['distance_mm']}")
        bot.drive(-100, 100, force=True)
        time.sleep(0.5)
        check("spinning left turns the heading", 5 < bot.sensors()["heading_ccw_deg"] < 40,
              f"heading {bot.sensors()['heading_ccw_deg']}")

        bot.set_mode("passive")
        d0 = bot.sensors()["distance_mm"]
        bot.drive(300, 300, force=True)
        time.sleep(0.3)
        check("Passive ignores drive commands", bot.sensors()["distance_mm"] == d0)

        bot.power_off()
        time.sleep(0.1)
        n = bot.frames_ok
        time.sleep(0.3)
        check("powered off, it stops streaming", bot.frames_ok == n)
        bot.close()
        check("no serial port was opened", not opened)
    finally:
        serial.Serial = real_serial


if __name__ == "__main__":
    test_allowlist()
    test_estop_gate()
    test_link_gate()
    test_origin_gate()
    test_sim()
    print()
    if FAILURES:
        print(f"FAILED: {len(FAILURES)} check(s): {', '.join(FAILURES)}")
        sys.exit(1)
    print("PASS: all safety gates hold")
