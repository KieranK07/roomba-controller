"""iRobot Roomba Open Interface (OI) protocol definitions.

Reference: iRobot Roomba 600 Open Interface Specification (docs/oi_spec.txt).
Verified against a physical Roomba 690 -- see docs/RESEARCH.md for the
firmware quirks this unit exhibits.
"""

# ---------------------------------------------------------------- opcodes ---
RESET        = 7
START        = 128
BAUD         = 129
CONTROL      = 130   # legacy alias for SAFE
SAFE         = 131
FULL         = 132
POWER        = 133
SPOT         = 134
CLEAN        = 135
MAX          = 136
DRIVE        = 137   # [vel_hi][vel_lo][radius_hi][radius_lo]
MOTORS       = 138
LEDS         = 139
SONG         = 140
PLAY         = 141
SENSORS      = 142   # [packet_id]
SEEK_DOCK    = 143
PWM_MOTORS   = 144
DRIVE_DIRECT = 145   # [right_hi][right_lo][left_hi][left_lo]
DRIVE_PWM    = 146
STREAM       = 148   # [n][id]...
QUERY_LIST   = 149
PAUSE_STREAM = 150   # [0|1]
STOP         = 173

STREAM_HEADER = 19

# Drive special radii (opcode 137)
RADIUS_STRAIGHT = 0x8000
RADIUS_CW       = 0xFFFF   # turn in place clockwise
RADIUS_CCW      = 0x0001   # turn in place counter-clockwise

MAX_VELOCITY_MMS = 500     # per-wheel hard limit from the spec
VELOCITY_STEP    = 28.5    # robot quantises wheel speed to ~28.5 mm/s steps

# Odometry geometry. mm-per-count is straight from the spec:
#   N counts * (pi * 72.0 / 508.8) = mm
# The 235 mm wheelbase is the standard Roomba 500/600/Create-2 figure; trim it
# if a measured 360 deg spin does not come back to 0.
MM_PER_COUNT = 3.141592653589793 * 72.0 / 508.8   # 0.44450 mm
WHEELBASE_MM = 235.0

# ------------------------------------------------------- sensor packet map ---
# packet id -> (name, n_bytes, signed)
PACKETS = {
     7: ("bumps_wheeldrops",   1, False),
     8: ("wall",               1, False),
     9: ("cliff_left",         1, False),
    10: ("cliff_front_left",   1, False),
    11: ("cliff_front_right",  1, False),
    12: ("cliff_right",        1, False),
    13: ("virtual_wall",       1, False),
    14: ("wheel_overcurrents", 1, False),
    15: ("dirt_detect",        1, False),
    17: ("ir_omni",            1, False),
    18: ("buttons",            1, False),
    19: ("distance_mm",        2, True),
    20: ("angle_deg",          2, True),
    21: ("charging_state",     1, False),
    22: ("voltage_mv",         2, False),
    23: ("current_ma",         2, True),
    24: ("temperature_c",      1, True),
    25: ("charge_mah",         2, False),
    26: ("capacity_mah",       2, False),
    27: ("wall_signal",        2, False),
    28: ("cliff_left_signal",       2, False),
    29: ("cliff_front_left_signal",  2, False),
    30: ("cliff_front_right_signal", 2, False),
    31: ("cliff_right_signal",       2, False),
    34: ("charging_sources",   1, False),
    35: ("oi_mode",            1, False),
    39: ("req_velocity",       2, True),
    40: ("req_radius",         2, True),
    41: ("req_right_velocity", 2, True),
    42: ("req_left_velocity",  2, True),
    43: ("encoder_left",       2, True),
    44: ("encoder_right",      2, True),
    45: ("light_bumper",       1, False),
    46: ("lb_left",            2, False),
    47: ("lb_front_left",      2, False),
    48: ("lb_center_left",     2, False),
    49: ("lb_center_right",    2, False),
    50: ("lb_front_right",     2, False),
    51: ("lb_right",           2, False),
    52: ("ir_left",            1, False),
    53: ("ir_right",           1, False),
    58: ("stasis",             1, False),
}

NAME_TO_ID = {v[0]: k for k, v in PACKETS.items()}

# Packets we stream for live telemetry. Total ~35 bytes/frame at 64 Hz,
# far below the 172 bytes-per-15ms ceiling the spec warns about.
STREAM_PACKETS = [35, 7, 45, 14, 58, 21, 34, 22, 23, 24, 25, 26, 43, 44,
                  9, 10, 11, 12]   # binary cliff sensors

# ------------------------------------------------------------- bit fields ---
BUMP_RIGHT       = 0x01
BUMP_LEFT        = 0x02
WHEELDROP_RIGHT  = 0x04
WHEELDROP_LEFT   = 0x08

LIGHT_BUMPER_BITS = [
    ("left",         0x01),
    ("front_left",   0x02),
    ("center_left",  0x04),
    ("center_right", 0x08),
    ("front_right",  0x10),
    ("right",        0x20),
]

CHARGING_STATES = {
    0: "not charging", 1: "reconditioning", 2: "full charging",
    3: "trickle", 4: "waiting", 5: "fault",
}

OI_MODES = {0: "off", 1: "passive", 2: "safe", 3: "full"}

MODE_OFF, MODE_PASSIVE, MODE_SAFE, MODE_FULL = 0, 1, 2, 3


# ------------------------------------------------------------- encoding -----
def i16(value: int) -> bytes:
    """Signed 16-bit, high byte first (two's complement)."""
    v = max(-32768, min(32767, int(value)))
    return v.to_bytes(2, "big", signed=True)


def drive_direct(right_mms: int, left_mms: int) -> bytes:
    """Opcode 145. Note the wire order is RIGHT wheel first, then LEFT."""
    r = max(-MAX_VELOCITY_MMS, min(MAX_VELOCITY_MMS, int(right_mms)))
    l = max(-MAX_VELOCITY_MMS, min(MAX_VELOCITY_MMS, int(left_mms)))
    return bytes([DRIVE_DIRECT]) + i16(r) + i16(l)


def parse_frame(frame: bytes) -> dict:
    """Decode one validated stream frame body into {name: value}."""
    out = {}
    body = frame[2:-1]          # strip [19][n] header and trailing checksum
    i = 0
    while i < len(body):
        pid = body[i]
        i += 1
        spec = PACKETS.get(pid)
        if spec is None:
            break               # unknown id -> can't know its width, bail
        name, n, signed = spec
        chunk = body[i:i + n]
        if len(chunk) < n:
            break
        out[name] = int.from_bytes(chunk, "big", signed=signed)
        i += n
    return out


def checksum_ok(frame: bytes) -> bool:
    """Spec: all bytes including the checksum sum to 0 in the low byte."""
    return (sum(frame) & 0xFF) == 0
