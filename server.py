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
LINK_TIMEOUT_S = 1.5      # no sensor frames for this long -> robot is not there
KEEPALIVE_S    = 0.20     # re-assert current velocity at least this often

DEFAULT_SPEED  = 250      # mm/s
DEFAULT_TURN   = 180      # mm/s differential

# Opcodes the UI may send through the "op" escape hatch, each mapped to the
# per-byte masks its payload takes (so the tuple's length is also the payload
# length). Anything else is refused. The OI is not safe to expose verbatim:
# 129 changes the baud rate and drops the link, 7 factory-resets, 140/141
# rewrite and play songs, and 144/146 spin the wheels straight past every limit
# in mixer.py. The UI only ever sends 143 and 138.
ALLOWED_OPS = {
    P.SEEK_DOCK: (),         # 143 seek dock
    P.CLEAN:     (),         # 135 normal clean cycle
    P.SPOT:      (),         # 134 spot clean
    P.MAX:       (),         # 136 max clean
    P.MOTORS:    (0x1F,),    # 138 brush/vacuum bit field, bits 5-7 undefined
}

# Of those, the ones that hand the robot back to its own behaviours, so the
# Safe-mode re-arm has to stand down instead of fighting them.
AUTONOMOUS_OPS = frozenset((P.SEEK_DOCK, P.CLEAN, P.SPOT, P.MAX))


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

        # The robot is allowed to be absent: asleep at startup, or dropping off
        # later (it sleeps on its own after a spell in Passive). Sensor frames
        # actually arriving is the only proof it is answering -- an open port
        # proves nothing, so that is what "linked" tracks.
        self.linked = False
        self.link_announced = False
        self._frames_seen = 0
        self._frames_at = 0.0

    def poll_link(self, now: float) -> bool:
        """Refresh and return the link state: are frames still arriving?"""
        n = self.bot.frames_ok
        if n > self._frames_seen:
            self._frames_seen = n
            self._frames_at = now
            self.linked = True
        elif self.linked and now - self._frames_at > LINK_TIMEOUT_S:
            self.linked = False
        return self.linked

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


def parse_op(msg: dict) -> tuple[int, bytes]:
    """Validate an "op" message against ALLOWED_OPS.

    Raises ValueError with a printable reason; never returns a command the
    robot should not be given.
    """
    opcode = msg.get("opcode")
    # Strictly an int: int() would happily coerce "143" and 143.9 into a real
    # opcode, and a payload that sloppy has no business reaching the robot.
    if isinstance(opcode, bool) or not isinstance(opcode, int):
        raise ValueError(f"opcode must be an integer, got {type(opcode).__name__}")
    masks = ALLOWED_OPS.get(opcode)
    if masks is None:
        raise ValueError(f"opcode {opcode} is not on the allowlist")
    data = msg.get("data", [])
    if not isinstance(data, list):
        raise ValueError("data must be a list")
    if len(data) != len(masks):
        raise ValueError(f"opcode {opcode} takes {len(masks)} data byte(s), "
                         f"got {len(data)}")
    for b, mask in zip(data, masks):
        if isinstance(b, bool) or not isinstance(b, int):
            raise ValueError("data bytes must be integers")
        if not 0 <= b <= 255:
            raise ValueError(f"data byte {b} out of range 0..255")
        if b & ~mask:
            raise ValueError(f"data byte 0x{b:02x} sets bits outside "
                             f"0x{mask:02x} for opcode {opcode}")
    return opcode, bytes(data)


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
                # Bare OI opcodes for the dock / brush buttons. Both gates
                # below matter. Every allowed opcode starts a motor, and
                # target() only ever gated the DriveDirect path, so until now a
                # latched e-stop still let the UI send 143 and drive away.
                if ctl.estop:
                    print("[op] refused: e-stop engaged", file=sys.stderr)
                    continue
                if not ctl.linked:
                    print("[op] refused: robot not connected", file=sys.stderr)
                    continue
                try:
                    opcode, data = parse_op(msg)
                except ValueError as e:
                    print(f"[op] refused: {e}", file=sys.stderr)
                    continue
                # Docking and cleaning are autonomous by definition, so
                # stop fighting them with the Safe-mode re-arm.
                if opcode in AUTONOMOUS_OPS:
                    ctl.hold_safe = False
                try:
                    ctl.bot.command(opcode, data)
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


async def link_watch(ctl: Controller):
    """Track whether the robot is answering, and announce every change.

    Bookkeeping only -- this never touches the serial port, so nothing it does
    can stall. Keeping it out of the same task as rewake() is the whole point:
    when they shared one loop, a single slow handshake stopped poll_link being
    called, the link state froze at "down" while frames were still arriving,
    and the UI stayed wrong indefinitely with no way back.
    """
    started = time.time()
    helped = False
    while True:
        up = ctl.poll_link(time.time())
        if up and not ctl.link_announced:
            s = ctl.bot.sensors()
            print(f"[oi] link up: mode={s.get('oi_mode_name')} "
                  f"battery={s.get('battery_pct')}% frames={s.get('frames_ok')}")
            ctl.link_announced = True
            helped = False
        elif not up and ctl.link_announced:
            print("[oi] link lost: the robot stopped answering (it sleeps on "
                  "its own) - press CLEAN to bring it back")
            ctl.link_announced = False
        elif not up and not helped and time.time() - started > 2.0:
            print(SLEEP_HELP)
            helped = True
        await asyncio.sleep(0.1)


async def wake_loop(ctl: Controller):
    """Re-issue the OI handshake while, and only while, the robot is silent.

    rewake() flushes the input buffer and reads the port directly, which fights
    the reader thread draining the stream. Against a live robot that corrupts
    the very telemetry it is supposed to restore, so the `linked` check below is
    load-bearing, not an optimisation.
    """
    warned = False
    while True:
        await asyncio.sleep(0.5)
        if ctl.linked:
            warned = False
            continue
        try:
            # Blocks up to ~1.5 s on the serial line. Off-thread so it cannot
            # stall telemetry, the control socket, or link_watch.
            await asyncio.to_thread(ctl.bot.rewake)
        except Exception as e:
            if not warned:
                print(f"[oi] no answer yet ({e}); still waiting for CLEAN",
                      file=sys.stderr)
                warned = True


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
    if not ctl.linked or not ctl.hold_safe or ctl.bot.passive_only:
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
            "link": ctl.linked,
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


def allowed_origins(args) -> list[str]:
    """Origin values the control socket will accept.

    The UI is served by our own static server, so a legitimate page can only
    have come from this host on --http-port. Without this check ANY page you
    have open can connect to ws://127.0.0.1:<ws-port> and drive the robot: the
    same-origin policy does not apply to WebSockets, and binding to localhost
    is no defence, because the attacker's JavaScript is already running on this
    machine.
    """
    hosts = ["127.0.0.1", "localhost", "[::1]"]
    if args.host in ("0.0.0.0", "::"):
        hosts.append(_lan_ip())
    elif args.host not in hosts:
        hosts.append(args.host)
    origins = [f"http://{h}:{args.http_port}" for h in hosts]
    origins += [o for o in args.allow_origin if o not in origins]
    return origins


def origin_gate(origins: list[str]):
    """process_request hook that refuses handshakes from pages we did not serve.

    websockets' own `origins=` argument enforces the same rule, but it only
    logs at DEBUG, and from the browser side a silent refusal is
    indistinguishable from a dead server. This prints the reason.
    """
    def process_request(connection, request):
        try:
            origin = request.headers.get("Origin")
        except Exception:
            reason = "multiple Origin headers"
        else:
            # No Origin at all means a non-browser client (curl, a script on
            # the Jetson). Browsers always send one on a WebSocket handshake,
            # so allowing it does not reopen the hole this closes; it leaves
            # exactly the unauthenticated-LAN exposure --host 0.0.0.0 already
            # warns about.
            if origin is None or origin in origins:
                return None
            reason = f"Origin {origin!r}"
        print(f"[ws] REFUSED handshake: {reason} not allowed; expected one of "
              f"{', '.join(origins)}", file=sys.stderr)
        return connection.respond(
            http.HTTPStatus.FORBIDDEN,
            "This socket only accepts the control UI served by this server.\n"
            "If you reach the UI under another name, pass --allow-origin.\n",
        )
    return process_request


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
    ctl = Controller(bot)
    serve_static(args.http_port, args.host)
    origins = allowed_origins(args)

    shown = args.host if args.host not in ("0.0.0.0", "::") else _lan_ip()
    print(f"\n  ->  open  http://{shown}:{args.http_port}/?ws={args.ws_port}\n")
    if args.host == "0.0.0.0":
        print("  NOTE: reachable by anyone on this network. There is no auth,")
        print("        so only do this on a network you trust.\n")

    print(f"  control socket accepts pages from: {', '.join(origins)}")
    print("  reaching the UI under another name needs --allow-origin URL\n")

    async with websockets.serve(lambda w: ws_handler(w, ctl),
                                args.host, args.ws_port,
                                process_request=origin_gate(origins),
                                ping_interval=None, max_queue=8):
        await asyncio.gather(link_watch(ctl), wake_loop(ctl),
                             control_loop(ctl), telemetry_loop(ctl))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbserial-BG03LB59")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address; use 0.0.0.0 to reach it from other "
                         "machines on the LAN (no auth - trusted networks only)")
    ap.add_argument("--allow-origin", action="append", default=[], metavar="URL",
                    help="extra Origin allowed to open the control socket, e.g. "
                         "http://roomba.local:8666 -- repeatable. The UI's own "
                         "address is always allowed and every other page is "
                         "refused, so this is only needed when you reach the UI "
                         "under a name the server cannot guess")
    ap.add_argument("--http-port", type=int, default=8666)
    ap.add_argument("--ws-port", type=int, default=8667)
    ap.add_argument("--park", choices=("off", "passive"), default="off",
                    help="on exit: 'off' powers the robot down so a paused "
                         "clean cycle cannot resume (default); 'passive' just "
                         "drops to Passive, which still charges but may resume "
                         "cleaning when undocked")
    ap.add_argument("--sim", action="store_true",
                    help="no robot: drive a simulated Roomba in a 4 x 3 m room. "
                         "Never opens a serial port")
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

    if args.sim:
        from roomba_oi.sim import SimRoomba
        bot = SimRoomba(passive_only=args.monitor)
        print("[oi] SIMULATION: no serial port, the robot and its telemetry are fake")
    else:
        bot = RoombaOI(args.port, args.baud, passive_only=args.monitor)
        print(f"[oi] opening {args.port} @ {args.baud}")
    if args.monitor:
        print("[oi] MONITOR MODE: staying in Passive, motors disabled")
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
