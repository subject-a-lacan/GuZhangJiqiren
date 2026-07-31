"""
V195 — Q4: bar-shape majority vote + NN cross-check, 3-frame lock.

Logic:
  1. Collect bar shapes over BUF_N frames → majority vote → confirmed bar shape.
  2. NN classifier runs in parallel → nn_shape (3-frame stability > CONF).
  3. When both confirmed AND bar_shape == nn_shape for 3 consecutive frames:
       print FINAL result, stop all output.

UART2: A28(TX) A29(RX), /dev/ttyS2, 115200 (for final output only)
"""

from maix import camera, display, image, nn, app, time, uart, pinmap, err

MODEL_PATH = "/root/models/model-269639.maixcam/model_269639.mud"

# ==================== UART2 ====================
err.check_raise(pinmap.set_pin_function("A28", "UART2_TX"), "set A28->UART2_TX failed")
err.check_raise(pinmap.set_pin_function("A29", "UART2_RX"), "set A29->UART2_RX failed")
uart2 = uart.UART("/dev/ttyS2", 115200)


def send_result(shape):
    """Send final result once."""
    mapping = {"Circle": 1, "Square": 2, "Triangle": 3, "Ellipse": 4}
    code = mapping.get(shape, 0)
    uart2.write(f"R{code}\r".encode())
# ================================================

# ==================== NN Classifier ====================
classifier = nn.Classifier(model=MODEL_PATH, dual_buff=True)
NN_W, NN_H = classifier.input_width(), classifier.input_height()
CONF_THRESHOLD = 0.98
print("NN Labels:", classifier.labels)
# ======================================================

DARK_THRESHOLD = [[0, 55, -20, 20, -20, 20]]
AREA_MIN       = 10
MIN_H          = 10
MAX_H          = 45
MIN_HALF_W     = 15
ASPECT_RATIO   = 1.0
MAX_ASPECT     = 10.0
BORDER_MARGIN  = 5
EDGE_MARGIN    = 40

VERT_MERGE     = 15
VERT_MIN_H     = 100
VERT_MAX_W     = 60
GAP_EXTRA      = 5

K_Y            = 0.0035
HALF_18CM      = 9.0
HALF_THRESH    = 4.8
TRUE_W_MIN     = 0.8
TRUE_W_MAX     = 14.0

SIDE_MIN_H     = 10
SIDE_MAX_H     = 55

TRUNK_SLICE_H  = 10
TRUNK_AREA_MIN = 20
MAX_TRUNK_W    = 80
CX_MIN_MARGIN  = 40

# Voting
BAR_BUF_N      = 15   # collect this many bar shapes for majority vote

# Lock
LOCK_FRAMES    = 3    # need this many consecutive bar==nn matches

cam = camera.Camera(640, 480)
disp = display.Display()

print("=== V195 Q4 Majority-Vote + Lock ===")
print(f"BAR_BUF_N={BAR_BUF_N}  LOCK_FRAMES={LOCK_FRAMES}  NN conf>{CONF_THRESHOLD}")
print()


def find_vertical_tape(img):
    w = img.width()
    cx0 = w // 3; cx1 = 2 * w // 3
    blobs = img.find_blobs(DARK_THRESHOLD, roi=[cx0, 0, cx1 - cx0, 480],
                           area_threshold=AREA_MIN, merge=True, margin=VERT_MERGE)
    candidates = []
    for b in blobs:
        _, _, bw, bh = b.rect()
        if bh >= VERT_MIN_H and bw <= VERT_MAX_W:
            candidates.append((b.x() + bw // 2, b.x(), b.x() + bw))
    if not candidates:
        return None
    candidates.sort(key=lambda c: abs(c[0] - w // 2))
    return candidates[0]


def find_trunk_x(img, ref_cx):
    w, h = img.width(), img.height()
    slices = [h // 2, h // 3, 2 * h // 3, h // 4, 3 * h // 4]
    best_cx = ref_cx
    best_dist = 9999
    for y in slices:
        roi = [0, max(0, y - TRUNK_SLICE_H // 2), w, TRUNK_SLICE_H]
        blobs = img.find_blobs(DARK_THRESHOLD, roi=roi,
                               area_threshold=TRUNK_AREA_MIN, merge=True)
        valid = []
        for b in blobs:
            bw = b.w()
            if 5 <= bw <= MAX_TRUNK_W:
                valid.append(b.x() + bw // 2)
        if valid:
            best_in_slice = min(valid, key=lambda c: abs(c - ref_cx))
            d = abs(best_in_slice - ref_cx)
            if d < best_dist:
                best_dist = d
                best_cx = best_in_slice
    return best_cx


def _collect_bars(img, roi, cx, side_label):
    h_img = img.height(); img_w = img.width()
    blobs = img.find_blobs(DARK_THRESHOLD, roi=roi,
                           area_threshold=AREA_MIN, merge=True, margin=15)
    bars = []
    for b in blobs:
        rx, ry, rw, rh = b.rect()
        if ry + rh >= h_img - BORDER_MARGIN and rh < 20: continue
        if ry > h_img // 2 and rh < 30: continue
        if not (SIDE_MIN_H <= rh <= SIDE_MAX_H): continue
        if rw < MIN_HALF_W: continue
        ar = rw / rh
        if ar < ASPECT_RATIO or ar > MAX_ASPECT: continue
        if ry < h_img * 2 // 3:
            if rx < EDGE_MARGIN or (rx + rw) > img_w - EDGE_MARGIN:
                continue
        if side_label == "L":
            true_x = rx; true_w = cx - rx
        else:
            true_x = cx; true_w = (rx + rw) - cx
        bars.append({"x": true_x, "y": ry, "w": true_w, "h": rh, "cy": ry + rh // 2})
    bars.sort(key=lambda b: b["cy"])
    return bars


def find_three_blobs(img, cx):
    w, h = img.width(), img.height()
    if cx < CX_MIN_MARGIN: cx = CX_MIN_MARGIN
    if cx > w - CX_MIN_MARGIN: cx = w - CX_MIN_MARGIN
    bars_L, bars_R = [], []
    left_w = cx - GAP_EXTRA
    if left_w >= 10:
        bars_L = _collect_bars(img, [0, 0, left_w, h], cx, "L")
    right_w = w - cx - GAP_EXTRA
    if right_w >= 10:
        bars_R = _collect_bars(img, [cx + GAP_EXTRA, 0, right_w, h], cx, "R")
    ok_L = len(bars_L) >= 3; ok_R = len(bars_R) >= 3
    if ok_L and ok_R:
        area_L = sum(b["w"] * b["h"] for b in bars_L[:3])
        area_R = sum(b["w"] * b["h"] for b in bars_R[:3])
        if area_L >= area_R:
            return bars_L[:3], "L"
        else:
            return bars_R[:3], "R"
    elif ok_L: return bars_L[:3], "L"
    elif ok_R: return bars_R[:3], "R"
    else:      return None, None


def classify_shape(bars):
    for b in bars: b["norm_w"] = b["w"] * (1.0 + K_Y * (480 - b["y"]))
    bar_18 = max(bars, key=lambda b: b["norm_w"]); idx_18 = bars.index(bar_18)
    if idx_18 == 1: return None
    norm_18 = bar_18["norm_w"]
    others = [b for b in bars if b is not bar_18]
    true_w0 = HALF_18CM * (others[0]["norm_w"] / norm_18)
    true_w1 = HALF_18CM * (others[1]["norm_w"] / norm_18)
    if not (TRUE_W_MIN < true_w0 < TRUE_W_MAX) or not (TRUE_W_MIN < true_w1 < TRUE_W_MAX):
        return None
    if true_w0 < HALF_THRESH and true_w1 > HALF_THRESH: shape = "Circle"
    elif true_w0 > HALF_THRESH and true_w1 < HALF_THRESH: shape = "Square"
    elif true_w0 > HALF_THRESH and true_w1 > HALF_THRESH: shape = "Triangle"
    else: shape = "Ellipse"
    return shape, true_w0, true_w1


def majority(seq):
    """Return most common element, breaking ties arbitrarily."""
    if not seq: return None
    counts = {}
    for x in seq:
        counts[x] = counts.get(x, 0) + 1
    return max(counts, key=counts.get)


def normalize_shape(name):
    n = name.strip().lower()
    for k in ["circle", "square", "triangle", "ellipse"]:
        if k in n: return k.capitalize()
    return name


def draw_results(img, bars, side_label, cx, fps, bar_buf, bar_confirmed, nn_name, nn_conf, nn_stable, match_ok, locked):
    w, h = img.width(), img.height()
    top_h = h * 2 // 3
    img.draw_line(EDGE_MARGIN, 0, EDGE_MARGIN, top_h, image.COLOR_RED, 1)
    img.draw_line(w - EDGE_MARGIN, 0, w - EDGE_MARGIN, top_h, image.COLOR_RED, 1)
    img.draw_line(EDGE_MARGIN, top_h, w - EDGE_MARGIN, top_h, image.COLOR_RED, 1)
    if side_label == "L":
        img.draw_line(cx - GAP_EXTRA, 0, cx - GAP_EXTRA, h, image.COLOR_GRAY, 1)
    else:
        img.draw_line(cx + GAP_EXTRA, 0, cx + GAP_EXTRA, h, image.COLOR_GRAY, 1)
    img.draw_line(cx, 0, cx, h, image.COLOR_WHITE, 1)

    colors = [image.COLOR_GREEN, image.COLOR_YELLOW, image.COLOR_RED]
    for i, b in enumerate(bars):
        img.draw_rect(b["x"], b["y"], b["w"], b["h"], colors[i], 2)
        img.draw_circle(b["x"] + b["w"] // 2, b["cy"], 3, colors[i], -1)

    img.draw_rect(0, 0, w, 56, image.COLOR_BLACK, -1)
    buf_str = "/".join(bar_buf[-8:]) if bar_buf else "?"

    if locked:
        img.draw_string(5, 2, f">>> LOCKED: {bar_confirmed}  MATCH={'OK' if match_ok else 'FAIL'} <<<",
                        image.COLOR_GREEN, 1.3)
    else:
        img.draw_string(5, 2, f"BAR buf=[{buf_str}]  -> {bar_confirmed or '...'}  |  "
                        f"NN: {nn_name} {nn_conf:.2f} [{nn_stable}/3]",
                        image.COLOR_WHITE, 1.2)
        img.draw_string(5, 22, f"cx={cx}  {side_label}  v195  {fps}FPS",
                        image.COLOR_GREEN, 0.9)


def draw_fallback(img, cx, fps):
    w, h = img.width(), img.height()
    if cx:
        img.draw_line(cx, 0, cx, h, image.COLOR_WHITE, 1)
        img.draw_line(cx - GAP_EXTRA, 0, cx - GAP_EXTRA, h, image.COLOR_GRAY, 1)
        img.draw_line(cx + GAP_EXTRA, 0, cx + GAP_EXTRA, h, image.COLOR_GRAY, 1)
    img.draw_string(5, 5, f"Searching... cx={cx} {fps}FPS [V195]", image.COLOR_RED, 1.3)


# ==================== Main ====================

last_bars = None; last_side = "?"; last_cx = 320
frame_times = [100] * 10
cx = None; smooth_cx = 320

# Bar shape buffer for majority voting
bar_buf = []
bar_confirmed = None  # once majority is reached, this is locked

# NN stability
nn_last = None; nn_stable = 0

# Match lock
match_count = 0
final_done = False

while not app.need_exit():
    if final_done:
        time.sleep_ms(100)
        continue

    t0 = time.ticks_ms()
    img = cam.read()

    # ---- Vertical tape ----
    vt = find_vertical_tape(img)
    cx = vt[0] if vt else (last_cx or 320)

    # ---- Trunk + EMA ----
    dynamic_cx = find_trunk_x(img, smooth_cx)
    smooth_cx = int(0.7 * smooth_cx + 0.3 * dynamic_cx)

    # ---- Bar detection ----
    bars, side_label = find_three_blobs(img, cx)
    bar_shape = None; true_w0 = 0; true_w1 = 0
    if bars is not None:
        result = classify_shape(bars)
        if result is not None:
            bar_shape, true_w0, true_w1 = result
            last_bars = bars; last_side = side_label; last_cx = cx

    # ---- Majority vote on bar shapes ----
    if bar_shape and bar_confirmed is None:
        bar_buf.append(bar_shape)
        if len(bar_buf) > BAR_BUF_N:
            bar_buf.pop(0)
        if len(bar_buf) >= BAR_BUF_N:
            bar_confirmed = majority(bar_buf)

    # ---- NN Classifier ----
    nn_img = img.resize(NN_W, NN_H, image.Fit.FIT_CONTAIN)
    out = classifier.classify(nn_img)
    idx, prob = out[0]
    nn_name = classifier.labels[idx]
    if prob > CONF_THRESHOLD and nn_name == nn_last:
        nn_stable += 1
    else:
        nn_last = nn_name
        nn_stable = 1 if prob > CONF_THRESHOLD else 0
    nn_confirmed = nn_name if nn_stable >= 3 else None

    # ---- Match check + lock ----
    match_ok = False
    if bar_confirmed and nn_confirmed:
        match_ok = (bar_confirmed == normalize_shape(nn_confirmed))
        if match_ok:
            match_count += 1
        else:
            match_count = 0
    else:
        match_count = 0

    locked = match_count >= LOCK_FRAMES

    # ---- FINAL ----
    if locked:
        final_done = True
        send_result(bar_confirmed)
        print(f"\n=== FINAL Q4 ===  bar={bar_confirmed}  nn={nn_confirmed}  MATCH=OK")
        print(f"  bar_buf={bar_buf}  match_count={match_count}")
        # Keep displaying last frame with locked message
        if last_bars:
            draw_results(img, last_bars, last_side, last_cx, 0,
                         bar_buf, bar_confirmed, nn_name, prob, nn_stable, True, True)
        disp.show(img)
        continue

    # ---- FPS ----
    t1 = time.ticks_ms(); dt = t1 - t0
    frame_times.pop(0); frame_times.append(dt)
    fps = int(1000.0 / max(sum(frame_times) / len(frame_times), 1))

    # ---- Draw ----
    if last_bars:
        draw_results(img, last_bars, last_side, last_cx, fps,
                     bar_buf, bar_confirmed, nn_name, prob, nn_stable, match_ok, False)
    else:
        draw_fallback(img, cx, fps)
    disp.show(img)

    # ---- Console ----
    bar_tag = f"BAR={'?' if bar_confirmed is None else bar_confirmed}"
    nn_tag = nn_confirmed or f"{nn_name}[{nn_stable}/3]"
    lock_tag = f" MATCH={match_count}/{LOCK_FRAMES}" if bar_confirmed and nn_confirmed else ""
    print(f"[{bar_tag}] buf={len(bar_buf)}/{BAR_BUF_N}  [NN] {nn_tag}{lock_tag}  "
          f"w0={true_w0:.1f} w1={true_w1:.1f}  FPS={fps}")

    time.sleep_ms(1)
