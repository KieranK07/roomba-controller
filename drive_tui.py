#!/usr/bin/env python3
"""Terminal fallback controller.

Caveat worth understanding: a TTY delivers key PRESSES but never key RELEASES.
Holding W just produces an OS auto-repeat stream. So "held" is inferred -- a
key counts as held until HOLD_S passes with no repeat. That costs one
auto-repeat interval of stop latency (~30-60 ms) on release, which the browser
UI does not have. Use server.py for real driving; this is for quick checks
and for when you do not want a browser.
"""
from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from roomba_oi import RoombaOI
from roomba_oi.mixer import from_keys

HOLD_S = 0.14          # no repeat within this window => treat key as released
TICK   = 0.02

HELP = """
  W/S forward·back   A/D arc while moving, spin in place when stopped
  -/= speed trim     SPACE stop        0 zero heading      Q quit
"""


def read_keys(buf: str) -> tuple[list[str], str]:
    """Translate raw stdin into logical key names, handling ANSI arrows."""
    out, i = [], 0
    while i < len(buf):
        c = buf[i]
        if c == "\x1b":
            if buf.startswith("\x1b[", i) and len(buf) > i + 2:
                out.append({"A": "arrowup", "B": "arrowdown",
                            "C": "arrowright", "D": "arrowleft"}.get(buf[i + 2], ""))
                i += 3
                continue
            if len(buf) < i + 3:
                return out, buf[i:]        # incomplete escape; keep for next read
            i += 1
            continue
        out.append(c.lower())
        i += 1
    return [k for k in out if k], ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbserial-BG03LB59")
    ap.add_argument("--speed", type=int, default=250)
    ap.add_argument("--turn", type=int, default=180)
    args = ap.parse_args()

    bot = RoombaOI(args.port)
    print(f"connecting to {args.port} …")
    bot.connect()
    if not bot.wait_awake(2.0):
        print("Robot is asleep — press the CLEAN button, then re-run.")
        bot.close()
        sys.exit(1)

    speed, turn = args.speed, args.turn
    held: dict[str, float] = {}
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(HELP)
    try:
        tty.setcbreak(fd)
        pending = ""
        while True:
            if select.select([sys.stdin], [], [], TICK)[0]:
                chunk = os.read(fd, 256).decode("utf-8", "ignore")
                keys, pending = read_keys(pending + chunk)
                now = time.time()
                for k in keys:
                    if k == "q":
                        raise KeyboardInterrupt
                    if k == " ":
                        held.clear()
                        bot.stop()
                        continue
                    if k == "=":
                        speed = min(500, speed + 20); continue
                    if k == "-":
                        speed = max(30, speed - 20); continue
                    if k == "0":
                        bot.reset_odometry(); continue
                    if k not in ("w", "a", "s", "d"):
                        continue
                    held[k] = now

            now = time.time()
            for k in [k for k, t in held.items() if now - t > HOLD_S]:
                del held[k]

            left, right = from_keys(set(held), speed, turn)
            bot.drive(left, right)

            s = bot.sensors()
            bump = "".join(["L" if s.get("bump_left") else "·",
                            "R" if s.get("bump_right") else "·"])
            sys.stdout.write(
                f"\r  L{left:+5d} R{right:+5d} | spd {speed:3d} | "
                f"{s.get('oi_mode_name','?'):7} | batt {str(s.get('battery_pct')):>4}% | "
                f"hdg {s.get('heading_deg', 0):5.1f}° | "
                f"bump {bump} | keys {','.join(sorted(held)) or '-':<12}")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\nstopping, parking in Passive …")
        bot.close()


if __name__ == "__main__":
    main()
