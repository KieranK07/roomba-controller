#!/usr/bin/env python3
"""Roomba control server: static UI over HTTP + control/telemetry over WebSocket.

Latency path for a keypress:
    keydown -> ws frame (localhost, ~0.2 ms)
            -> applied IMMEDIATELY on receipt (not deferred to a tick)
            -> DriveDirect serial write (~1.2 ms measured)

The 50 Hz loop below deliberately does NOT drive the robot on a schedule; it
only runs the watchdog, the keepalive refresh and telemetry push. Waiting for
a tick to apply input would add up to 20 ms of avoidable lag.
"""
from __future__ import annotations

import argparse
import asyncio
import functools
import http.server
import json
import os
import signal
import socketserver
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import websockets

from roomba_oi import RoombaOI, protocol as P
from roomba_oi.mixer import from_keys

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")

TICK_HZ        = 50
TELEMETRY_HZ   = 20
WATCHDOG_S     = 0.35     # no client traffic for this long -> halt wheels
KEEPALIVE_S    = 0.20     # re-assert current velocity at least this often

DEFAULT_SPEED  = 250      # mm/s
DEFAULT_TURN   = 180      # mm/s differential


class Controller:
    def __init__(self, bot: RoombaOI):
        self.bot = bot
        self.keys: set[str] = set()
        self.speed = DEFAULT_SPEED
        self.turn = DEFAULT_TURN
        self.last_input = time.time()
        self.estop = False
        self.halted_by_watchdog = False
        # Keep the robot in Safe mode so it never runs its own behaviours.
        # A paused clean cycle lives on in Passive and resumes when the robot
        # leaves the dock, which is exactly what we are preventing here.
        self.hold_safe = True
        self._last_rearm = 0.0
        self.clients: set = set()
        self.lock = asyncio.Lock()

    def target(self) -> tuple[int, int]:
        if self.estop:
            return (0, 0)
        return from_keys(self.keys, self.speed, self.turn)

    def apply(self, force: bool = False) -> None:
        left, right = self.target()
        try:
            self.bot.drive(left, right, force=force)
        except Exception as e:
            print(f"[drive] {e}", file=sys.stderr)


async def ws_handler(ws, ctl: Controller):
    ctl.clients.add(ws)
    peer = getattr(ws, "remote_address", None)
    print(f"[ws] client connected {peer}")
    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except Exception:
                continue

            ctl.last_input = time.time()
            ctl.halted_by_watchdog = False
            kind = msg.get("t")

            if kind == "keys":
                keys = {str(k).lower() for k in msg.get("down", [])}
                if keys:
                    ctl.hold_safe = True
                if keys != ctl.keys:
                    ctl.keys = keys
                    ctl.apply()               # immediate, not tick-deferred
            elif kind == "ping":
                # Echo for client-side RTT measurement.
                await ws.send(json.dumps({"t": "pong", "ts": msg.get("ts")}))
            elif kind == "estop":
                ctl.estop = bool(msg.get("on", True))
                ctl.keys.clear()
                ctl.apply(force=True)
            elif kind == "speed":
                ctl.speed = max(0, min(500, int(msg.get("speed", DEFAULT_SPEED))))
                ctl.turn = max(0, min(500, int(msg.get("turn", DEFAULT_TURN))))
                ctl.apply()
            elif kind == "mode":
                value = str(msg.get("value", "safe"))
                ctl.hold_safe = (value == "safe")
                try:
                    ctl.bot.set_mode(value)
                except Exception as e:
                    print(f"[mode] {e}", file=sys.stderr)
            elif kind == "power_off":
                try:
                    ctl.hold_safe = False
                    ctl.bot.power_off()
                    print("[oi] powered off by request")
                except Exception as e:
                    print(f"[power] {e}", file=sys.stderr)
            elif kind == "reset_heading":
                ctl.bot.reset_odometry()
            elif kind == "op":
                # Bare OI opcodes for the dock / brush buttons.
                try:
                    opcode = int(msg["opcode"])
                    # Docking and cleaning are autonomous by definition, so
                    # stop fighting them with the Safe-mode re-arm.
                    if opcode in (P.SEEK_DOCK, P.CLEAN, P.SPOT, P.MAX):
                        ctl.hold_safe = False
                    ctl.bot.command(opcode, bytes(msg.get("data", [])))
                except Exception as e:
                    print(f"[op] {e}", file=sys.stderr)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        ctl.clients.discard(ws)
        print(f"[ws] client gone {peer}")
        if not ctl.clients:
            ctl.keys.clear()
            ctl.apply(force=True)      # never leave the robot rolling


async def control_loop(ctl: Controller):
    period = 1.0 / TICK_HZ
    while True:
        await asyncio.sleep(period)
        now = time.time()
        if now - ctl.last_input > WATCHDOG_S:
            if ctl.keys or not ctl.halted_by_watchdog:
                ctl.keys.clear()
                ctl.apply(force=True)
                ctl.halted_by_watchdog = True
        elif ctl.bot.last_drive != (0, 0):
            ctl.apply()                # keepalive refresh (driver rate-limits)

        maybe_rearm(ctl, now)


def maybe_rearm(ctl: Controller, now: float) -> None:
    """Put the robot back into Safe if it fell out.

    Safe mode halts every motor and makes the robot ignore its own buttons and
    sensors, so holding it there is what guarantees it never starts cleaning.
    Skipped while charging: "charger plugged in and powered" is itself a
    Safe-mode trip, so re-arming on the dock would just loop forever."""
    if not ctl.hold_safe or ctl.bot.passive_only:
        return
    if now - ctl._last_rearm < 0.5:
        return
    s = ctl.bot.sensors()
    if s.get("oi_mode") != P.MODE_PASSIVE:
        return
    if s.get("charging_sources") or s.get("charging_state"):
        return                      # docked: Safe cannot stick, do not fight it
    ctl._last_rearm = now
    try:
        ctl.bot.set_mode("safe")
        ctl.bot.drive(0, 0, force=True)
        print("[oi] robot fell to Passive - re-armed Safe (no autonomous cleaning)")
    except Exception as e:
        print(f"[rearm] {e}", file=sys.stderr)


async def telemetry_loop(ctl: Controller):
    period = 1.0 / TELEMETRY_HZ
    while True:
        await asyncio.sleep(period)
        if not ctl.clients:
            continue
        s = ctl.bot.sensors()
        s.update({
            "t": "tel",
            "cmd_left": ctl.bot.last_drive[0],
            "cmd_right": ctl.bot.last_drive[1],
            "keys": sorted(ctl.keys),
            "speed": ctl.speed,
            "turn": ctl.turn,
            "estop": ctl.estop,
            "watchdog": ctl.halted_by_watchdog,
            "uptime": round(time.time() - ctl.bot.connected_at, 1),
        })
        payload = json.dumps(s)
        await asyncio.gather(*(c.send(payload) for c in list(ctl.clients)),
                             return_exceptions=True)


def _lan_ip() -> str:
    """Best-effort LAN address to print in the URL (no packets are sent)."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.0.2.1", 1))       # TEST-NET-1, never routed
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def serve_static(port: int, host: str = "127.0.0.1"):
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=WEB_DIR)

    class Quiet(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

    handler.log_message = lambda *a, **k: None
    httpd = Quiet((host, port), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


SLEEP_HELP = """
  The robot is ASLEEP (port is open, but it is not answering).

  This cable does not break out the BRC pin, so it cannot be woken in
  software -- press the CLEAN button on the Roomba once. It should light up.
  Waiting for it now; press Ctrl-C to give up.
"""


async def main_async(args, bot: RoombaOI):
    if not bot.wait_awake(2.0):
        print(SLEEP_HELP)
        while not bot.wait_awake(1.0):
            bot.rewake()                 # re-handshake until it responds
            await asyncio.sleep(0.2)
        print("[oi] robot is awake")

    await asyncio.sleep(0.4)
    s = bot.sensors()
    print(f"[oi] mode={s.get('oi_mode_name')} (raw {s.get('oi_mode_raw')}, "
          f"offset {bot._mode_offset:+d})  battery={s.get('battery_pct')}%  "
          f"frames={s.get('frames_ok')}")

    ctl = Controller(bot)
    serve_static(args.http_port, args.host)

    shown = args.host if args.host not in ("0.0.0.0", "::") else _lan_ip()
    print(f"\n  ->  open  http://{shown}:{args.http_port}/?ws={args.ws_port}\n")
    if args.host == "0.0.0.0":
        print("  NOTE: reachable by anyone on this network. There is no auth,")
        print("        so only do this on a network you trust.\n")

    async with websockets.serve(lambda w: ws_handler(w, ctl),
                                args.host, args.ws_port,
                                ping_interval=None, max_queue=8):
        await asyncio.gather(control_loop(ctl), telemetry_loop(ctl))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbserial-BG03LB59")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address; use 0.0.0.0 to reach it from other "
                         "machines on the LAN (no auth - trusted networks only)")
    ap.add_argument("--http-port", type=int, default=8666)
    ap.add_argument("--ws-port", type=int, default=8667)
    ap.add_argument("--park", choices=("off", "passive"), default="off",
                    help="on exit: 'off' powers the robot down so a paused "
                         "clean cycle cannot resume (default); 'passive' just "
                         "drops to Passive, which still charges but may resume "
                         "cleaning when undocked")
    ap.add_argument("--monitor", action="store_true",
                    help="read-only: stay in Passive so a docked robot keeps "
                         "charging; no actuator commands are sent")
    args = ap.parse_args()

    # SIGTERM must not bypass cleanup: a killed server would otherwise leave
    # the robot streaming and sitting in Safe mode, which drains the battery.
    def _term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    signal.signal(signal.SIGHUP, _term)

    bot = RoombaOI(args.port, args.baud, passive_only=args.monitor)
    if args.monitor:
        print("[oi] MONITOR MODE: staying in Passive, motors disabled")
    print(f"[oi] opening {args.port} @ {args.baud}")
    bot.connect()
    try:
        asyncio.run(main_async(args, bot))
    except KeyboardInterrupt:
        print("\n[oi] interrupted")
    finally:
        # Unconditional. Default is a full power-down: Passive alone would let
        # a paused cleaning cycle restart the next time the robot is undocked.
        if args.park == "off" and not args.monitor:
            print("[oi] stopping motors, powering robot off")
            try:
                bot.power_off()
                time.sleep(0.3)
            except Exception:
                pass
        else:
            print("[oi] stopping motors, parking in Passive")
        bot.close()


if __name__ == "__main__":
    main()
