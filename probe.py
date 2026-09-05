#!/usr/bin/env python3
"""Read-only-ish diagnostic: figure out if this Roomba speaks Open Interface.

Sends only Start (128) and sensor queries. Never sends a motion command.
"""
import sys, time
import serial
from serial.tools import list_ports

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-BG03LB59"

OI_MODES = {0: "off", 1: "passive", 2: "safe", 3: "full"}
CHARGING = {0: "not charging", 1: "reconditioning", 2: "full charging",
            3: "trickle", 4: "waiting", 5: "fault"}


def banner(s):
    print(f"\n=== {s} ===")


def try_baud(baud):
    banner(f"Trying {baud} baud on {PORT}")
    try:
        ser = serial.Serial(PORT, baud, timeout=0.4, write_timeout=1.0,
                            bytesize=8, parity='N', stopbits=1, rtscts=False,
                            dsrdtr=False, xonxoff=False)
    except Exception as e:
        print(f"  ! cannot open port: {e}")
        return None

    time.sleep(0.15)
    try:
        # A previous session may have left the sensor stream running; its
        # 64 Hz frames would be mistaken for query replies. Silence it first.
        ser.write(bytes([150, 0]))
        ser.flush()
        time.sleep(0.25)
        ser.reset_input_buffer()
        # Start -> Passive. Benign; robot beeps.
        ser.write(bytes([128]))
        ser.flush()
        time.sleep(0.3)
        unsolicited = ser.read(2048)
    except serial.SerialException as e:
        ser.close()
        print(f"  ! serial error: {e}")
        print("  ! A serial port can only be held by one process. Is server.py"
              " already running?")
        return None
    if unsolicited:
        print(f"  robot said (unsolicited, {len(unsolicited)}B): {unsolicited[:400]!r}")

    results = {}
    # (name, packet id, nbytes, signed)
    queries = [
        ("oi_mode",        35, 1, False),
        ("charging_state", 21, 1, False),
        ("voltage_mv",     22, 2, False),
        ("current_ma",     23, 2, True),
        ("temp_c",         24, 1, True),
        ("charge_mah",     25, 2, False),
        ("capacity_mah",   26, 2, False),
        ("bumps_drops",     7, 1, False),
        ("buttons",        18, 1, False),
    ]
    ok = 0
    for name, pid, nbytes, signed in queries:
        ser.reset_input_buffer()
        ser.write(bytes([142, pid]))
        ser.flush()
        data = ser.read(nbytes)
        if len(data) == nbytes:
            val = int.from_bytes(data, "big", signed=signed)
            results[name] = val
            ok += 1
        else:
            results[name] = None
    print(f"  sensor queries answered: {ok}/{len(queries)}")
    for k, v in results.items():
        print(f"    {k:16} = {v}")
    ser.close()
    return (ok, results)


def main():
    banner("USB serial adapters seen by the OS")
    for p in list_ports.comports():
        print(f"  {p.device}  |  {p.description}  |  {p.hwid}")

    best = None
    for baud in (115200, 19200):
        r = try_baud(baud)
        if r and r[0] > 0:
            best = (baud, r)
            break

    banner("VERDICT")
    if not best:
        print("  No sensor replies at either baud rate.")
        print("  Most likely: robot is ASLEEP. Press the CLEAN button once")
        print("  (it should light up), then re-run this probe immediately.")
        print("  Also verify TX/RX are not swapped on the cable.")
        sys.exit(1)

    baud, (ok, res) = best
    print(f"  OPEN INTERFACE IS ALIVE at {baud} baud ({ok} packets answered).")
    # We sent Start (128), so the robot must be in Passive. Any difference
    # between that and the reported byte is this firmware's mode offset.
    m = res.get("oi_mode")
    if m is not None:
        offset = m - 1
        note = "" if offset == 0 else f"  [firmware reports {offset:+d} vs spec]"
        print(f"  OI mode      : passive (raw {m}){note}")
    cs = res.get("charging_state")
    print(f"  charging     : {CHARGING.get(cs, cs)}")
    v, c, cap = res.get("voltage_mv"), res.get("charge_mah"), res.get("capacity_mah")
    if v:
        print(f"  battery      : {v/1000:.2f} V", end="")
        if c and cap:
            print(f", {c}/{cap} mAh ({100*c/cap:.0f}%)")
        else:
            print()
    print(f"  temperature  : {res.get('temp_c')} C")
    print(f"  current      : {res.get('current_ma')} mA")


if __name__ == "__main__":
    main()
