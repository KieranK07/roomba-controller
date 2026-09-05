# Roomba 690 · Open Interface research notes

Everything below was verified against **your** robot over `/dev/cu.usbserial-BG03LB59`
(FTDI FT232R, VID:PID `0403:6001`) on 2026-09-01, not just copied from the spec.
Primary source: *iRobot Roomba 600 Open Interface Specification*
(`docs/iRobot_Roomba_600_Open_Interface_Spec.pdf`, extracted to `oi_spec.txt`).

---

## 1. Headline result

**The 690 speaks the full Open Interface over serial.** This mattered because the
690 is a Wi-Fi model, and community reports are ambiguous about whether Wi-Fi
600-series units (690/675) retain OI. Yours does: 9/9 sensor queries answered on
the first attempt, and sensor streaming runs clean.

## 2. Physical layer

| | |
|---|---|
| Connector | 7-pin Mini-DIN, rear right, under a snap-away guard |
| Levels | TTL 0–5 V (**not** RS-232 — needs a level shifter; your FTDI cable has one) |
| Baud | **115200** confirmed working, 8N1, no flow control |
| Pinout | 1,2 = Vpwr · 3 = RXD · 4 = TXD · 5 = BRC · 6,7 = GND |

Pins 1–2 sit behind a 200 mA PTC fuse. Don't pull >200 mA continuous.

**Vpwr does not charge the robot.** Confirmed: `charging_sources = 0` and
current `−143 mA` while the serial cable was attached. The cable powers *your*
accessories from the battery; it does not feed the battery.

## 3. Mode state machine

```
  Off ──128 Start──> Passive ──131 Safe──> Safe ──132 Full──> Full
                        ^                    │                  │
                        └── safety trip ─────┘                  │
                        └────────── 128 Start ──────────────────┘
```

The safety trips that drop **Safe → Passive** are: cliff detected while moving
forward, any wheel drop, or charger plugged in and powered. Full mode disables
all three — in Full the robot *will* drive off a stair.

### Power behaviour — the part that actually bites

| Mode | Sleeps? | Charges? |
|---|---|---|
| Passive | after 5 min idle | yes |
| Safe / Full | **never** | **no — charging terminates** |

So leaving the robot in Safe mode slowly deep-discharges it *even on the dock*.
Every exit path in this project therefore parks it back in Passive
(`RoombaOI.close()`, called from a `finally`).

## 4. Measured latency — why the code is shaped the way it is

| Path | Median | Note |
|---|---|---|
| Sensor query round-trip (`142`) | **15.67 ms** | floored by the robot's 15 ms internal sensor refresh |
| DriveDirect write (`145`, 5 B) | **1.17 ms** | the actual control path |
| WebSocket localhost round-trip | **0.28 ms** | measured through the finished server |

The 15.7 ms figure is the whole reason for the architecture. Polling sensors on
the control thread would put a ~16 ms blocking read in front of every keypress.

**Sensor streaming (opcode 148) solves this**: measured **64.5 Hz, 129 frames in
2 s, 0 checksum failures**, ~2.3 kB/s — comfortably under the spec's ceiling of
172 bytes per 15 ms slot at 115200. Telemetry arrives pushed, on its own thread;
the control path only ever writes.

Estimated end-to-end keypress → wheels: **~1.5–2 ms**.

## 5. Firmware quirks found on this unit

### 5.1 OI-mode packet reads one too high

Packet 35 is documented as `0=off 1=passive 2=safe 3=full`, range 0–3. Yours
returns consistently one higher (3 reads each, stable):

| Commanded | Spec says | Yours returns |
|---|---|---|
| Start (128) → Passive | 1 | **2** |
| Safe (131) → Safe | 2 | **3** |
| Full (132) → Full | 3 | **4** |

Hardcoding the spec values would mislabel every mode and break the "dropped to
Passive" safety detection. `driver.py` instead **calibrates the offset at
connect time**: it commands Safe, reads whatever byte comes back, and derives
the offset. Correct on both compliant firmware and this unit.

### 5.2 No firmware banner on reset

The spec says opcode 7 prints a version string like
`r3_robot/tags/release-stm32-3.7.7`. Yours emits only idle-line noise
(`0xB1`/`0xFF` repeated, 6–12 bytes) at both 115200 and 19200 — no ASCII.

Consequence: the firmware version is unknowable this way, so the
version-dependent bugs the spec warns about must be assumed present. Packets 19
(`distance_mm`) and 20 (`angle_deg`) are unreliable before firmware 3.3.0/3.4.0.
**Use raw encoder counts (packets 43/44) instead** — which this project does.

### 5.3 Cannot be woken in software with this cable

Confirmed by test: pulsing both DTR and RTS three times (150 ms low, per the
spec's 50–500 ms BRC pulse spec) failed to wake a sleeping robot. Your cable
does not break out pin 5 (BRC).

**Press CLEAN to wake it.** The server detects this state and says so rather
than showing empty telemetry. Once awake and in Safe mode it never sleeps, so
this only bites at the start of a session.

### 5.4 Query/response can desync

During back-to-back `142` queries one reply came back as a stray `177`. Streaming
is immune to this — every frame is checksummed and the parser resynchronises on
the `19` header, which is why telemetry uses it exclusively.

## 5.5 Behaviour while docked (verified)

Three things are true on the dock and all are expected, not faults:

- **Safe mode will not hold.** "Charger plugged in and powered" is one of the
  documented Safe-mode safety trips, so commanding Safe on the dock bounces the
  robot straight back to Passive. You cannot drive it while docked.
- **Entering Safe/Full stops charging.** Verified: `charging_state` went 2 -> 0
  and current flipped from +1232 mA to negative. This is why `server.py
  --monitor` exists: it stays in Passive so telemetry can be watched while the
  battery actually charges.
- **All four cliff sensors read "cliff".** Verified: binary cliff = 1 and cliff
  *signal* = 0 on all four, because the dock lifts the sensors off the floor.
  Off the dock the same sensors read ~2500-2650. The UI will show four red cliff
  dots while docked; that is correct.

Charging on the dock was measured at **+1232 mA**, `charging_sources = 2`
(home base), taking the battery from 7 % to 12 % during this session.

## 5.6 Stopping it cleaning on its own

Observed in use: the robot begins a cleaning cycle as soon as it is taken off
the dock. The cause is a chain of documented behaviours:

1. Waking the robot means pressing **CLEAN**, which *starts* a cleaning cycle.
2. On the dock, "charger plugged in and powered" is a safety trip, so the cycle
   is held paused and the OI sits in Passive.
3. Passive is explicitly the mode where you "watch Roomba perform a cleaning
   cycle" — the robot keeps its own behaviours. So the moment the charger trip
   goes away (undocking), the paused cycle resumes.

The fix is to never leave it in Passive:

- **Hold Safe mode.** Per the spec, in Safe "Roomba waits with all motors and
  LEDs off and does not respond to button presses or other sensor input". The
  server re-asserts Safe whenever it sees the mode fall to Passive, except while
  charging (where Safe cannot stick and would loop).
- **Power off on exit** (opcode 133) rather than dropping to Passive. This is
  the only reliable way to guarantee a paused cycle cannot resume. The robot
  still charges while powered off.

## 5.7 Odometry

Heading is dead-reckoned from encoder packets 43/44 rather than the `angle`
packet, which the spec itself flags as wrong on firmware <= 3.4.0.

```
mm per count   = pi * 72.0 / 508.8 = 0.44456 mm
heading (rad) += (right_mm - left_mm) / 235.0     # CCW positive
one 360 deg in-place spin = 1661 counts per wheel
```

Encoders are signed 16-bit and roll over about every 14.5 m, so deltas are taken
mod 65536; verified that a 32700 -> -32700 wrap reads as +60 mm, not -29 m.
Unit-tested: a full spin integrates to 359.9 deg and a quarter turn to 90.0 deg.

The spec's own caveat matters here: these encoders are **square wave, not
quadrature**, and count using the *commanded* direction. They count up even when
commanded velocity is zero and the wheels are turned by hand. Dead reckoning is
therefore good for short manoeuvres and drifts with slip or handling.

## 6. Errata in the published spec

Page 22's stream example claims the decoded cliff signal is "549 (0x0225)". Its
own byte stream is `[19][5][29][2][25][13][0][163]`, where `2 25` are *decimal*
bytes = `0x02 0x19` = **537**. The checksum it publishes (sum ≡ 0 mod 256)
confirms 537 — the prose confuses decimal bytes for a hex literal. Our parser
returns 537 and is correct.

## 7. Command quick reference

| Op | Name | Bytes | Mode |
|---|---|---|---|
| 7 | Reset | 0 | any |
| 128 | Start | 0 | any → Passive |
| 131 | Safe | 0 | → Safe |
| 132 | Full | 0 | → Full |
| 135 / 134 / 143 | Clean / Spot / Seek Dock | 0 | → Passive |
| 137 | Drive | 4 (vel, radius) | Safe/Full |
| **145** | **Drive Direct** | 4 (right, **then** left) | Safe/Full |
| 146 | Drive PWM | 4 (±255) | Safe/Full |
| 138 | Motors (brushes/vac) | 1 bitmask | Safe/Full |
| 142 / 149 / 148 / 150 | Sensors / Query List / Stream / Pause | var | any |
| 173 | Stop OI | 0 | → Off |

Drive Direct takes the **right wheel first** — an easy bug; `protocol.drive_direct()`
takes `(right, left)` in wire order while `RoombaOI.drive()` exposes the more
natural `(left, right)`.

Velocity range ±500 mm/s, quantised by the robot to ~28.5 mm/s steps.

Opcode 137 special radii: `0x8000` straight, `0xFFFF` spin CW, `0x0001` spin CCW.

### Stream frame format
```
[19][n-bytes][id][data…][id][data…][checksum]
```
`n-bytes` counts everything between itself and the checksum. All bytes including
the checksum sum to 0 in the low byte.

## 8. Live sensor snapshot from your robot

```
oi_mode 2(=passive, offset-corrected)   charging_state 0 (not charging)
voltage 13.90 V    current −151 mA      temp 18 °C
charge 289/2068 mAh (14 %)              encoders L 113 / R 165
cliff signals L 2624 FL 2653 FR 2624 R 2510
light bumper L 20 FL 16 CL 32 CR 31 FR 21 R 3
```

**Battery was at 14 % and discharging** during this work. Dock it.

## 9. Sources

- iRobot Roomba 600 Open Interface Specification — bundled as `docs/*.pdf`
- [Create 2 OI Spec (iRobot Education)](https://edu.irobot.com/learning-library/create-2-oi-spec)
- [Create 2 OI Spec PDF (Adafruit mirror)](https://cdn-shop.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf)
- [Roomba 500 OI Specification](http://cfpm.org/~peter/bfz/iRobot_Roomba_500_Open_Interface_Spec.pdf)
- [Roomba Serial Command Interface (SCI), earlier models](https://cdn.hackaday.io/files/1747287475562752/Roomba_SCI_manual.pdf)
- [Mini-DIN pinout reference](https://pinoutguide.com/Electronics/irobot_roomba_serial_pinout.shtml)
