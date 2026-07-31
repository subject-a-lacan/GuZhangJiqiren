import struct, os, re

here = os.path.dirname(__file__)
inpath = os.path.join(here, "nihe.md")
outpath = os.path.join(here, "ball_id_all.csv")

HEADERS = [
    "run_id","phase_id","mcu_time_ms","cam_time_ms",
    "raw_x_mm","run_x0_mm","dx_mm","side","test_accel","test_ratio",
    "lut_rel_pulse","shaper_rel_pulse","uart_rel_pulse",
    "velocity_rpm","accel_param","publish_seq","last_started_seq",
    "stepper_reached","reverse_guard","vision_valid"
]

with open(inpath, "r", encoding="utf-8", errors="ignore") as f:
    text = f.read()

# Extract hex by splitting on whitespace and keeping 2-char tokens
tokens = text.split()
hex_tokens = [t for t in tokens if len(t) == 2 and all(c.upper() in '0123456789ABCDEF' for c in t)]
hex_str = ''.join(hex_tokens)
if len(hex_str) % 2: hex_str = hex_str[:-1]
data = bytes.fromhex(hex_str)

TAIL = bytes([0,0,0x80,0x7F])
N_FLT = 20
FRAME_SZ = N_FLT * 4

# Parse all frames
frames = []
pos = 0
while pos + FRAME_SZ + 4 <= len(data):
    tp = pos + FRAME_SZ
    if data[tp:tp+4] == TAIL:
        frames.append(list(struct.unpack("<" + "f" * N_FLT, data[pos:tp])))
        pos = tp + 4
    else:
        pos += 1
print(f"Frames: {len(frames)}")

# Detect run boundaries from data: run_id changes when the value in channel[0] changes
run_ids = []
current_run = 0
prev_rid = -1
for v in frames:
    rid = int(v[0])
    if rid != prev_rid and prev_rid >= 0:
        current_run += 1
    prev_rid = rid
    run_ids.append(current_run)

# phase_id already in channel[1]
# For entries where phase_id looks wrong, use heuristic
def fix_phase(v):
    if 0 <= v[1] <= 5: return int(v[1])
    s, a = v[7], v[8]
    if abs(s) > 0.1 and abs(a) > 1: return 1
    return 0

with open(outpath, "w") as out:
    out.write(",".join(HEADERS) + "\n")
    for i, v in enumerate(frames):
        rid = run_ids[i]
        ph = fix_phase(v)
        out.write(f"{rid},{ph}," + ",".join(f"{x:.6g}" for x in v) + "\n")

print(f"Rows: {len(frames)}")
if frames:
    rids = sorted(set(run_ids))
    data_rids = sorted(set(int(f[0]) for f in frames))
    print(f"Run IDs (from run_id column): {data_rids}")
    print(f"Run IDs (assigned): {rids}")
    f0 = frames[0]
    print(f"First: run={run_ids[0]} rid={f0[0]:.0f} ph={f0[1]:.0f} mcu={f0[2]:.0f} raw_x={f0[4]:.1f} side={f0[7]:.0f} ratio={f0[9]:.2f}")
    fn = frames[-1]
    print(f"Last:  run={run_ids[-1]} rid={fn[0]:.0f} ph={fn[1]:.0f} mcu={fn[2]:.0f} raw_x={fn[4]:.1f} side={fn[7]:.0f} ratio={fn[9]:.2f}")
print(f"Output: {outpath}")
