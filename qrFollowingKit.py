import cv2
import numpy as np
import serial
import time
import subprocess
from pyzbar.pyzbar import decode

print("🚀 QR Tracking Robot Started")

# ═══════════════════════════════════════════════════════════════
#  TUNING
# ═══════════════════════════════════════════════════════════════
SERVO_DIR   = 1

WIDTH, HEIGHT  = 640, 480
CENTER_X       = WIDTH // 2

Kp, Ki, Kd     = 0.06, 0.0, 0.018
DEAD_ZONE      = 15

MAX_STEP       = 3.5
MIN_STEP       = 0.3
SERVO_EMA      = 0.2
SERVO_HZ       = 30
SERVO_INTERVAL = 1.0 / SERVO_HZ

EMA_ALPHA      = 0.55
MAX_LOST       = 20
SEARCH_STEP    = 1.2
CAPTURE_FPS    = 15

# ═══════════════════════════════════════════════════════════════
#  SERIAL  (protocol: "pan,size\n")
#    pan  — servo angle 0-180
#    size — QR bounding box width in pixels (0 when not detected)
#           Arduino uses this to decide phase and forward speed
# ═══════════════════════════════════════════════════════════════
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(2)
    print("✅ Serial Connected")
except Exception as e:
    ser = None
    print(f"❌ Serial NOT connected: {e}")

_last_send  = 0.0
_last_pan   = -1
_last_size  = -1

def send_command(pan_angle: float, qr_size: int):
    """Send pan,size over serial, rate-limited to SERVO_HZ."""
    global _last_send, _last_pan, _last_size
    now = time.monotonic()
    p   = int(np.clip(pan_angle, 0, 180))
    s   = int(np.clip(qr_size,   0, 1000))
    if now - _last_send >= SERVO_INTERVAL and (p != _last_pan or s != _last_size):
        if ser:
            ser.write(f"{p},{s}\n".encode())
        _last_send  = now
        _last_pan   = p
        _last_size  = s

# ═══════════════════════════════════════════════════════════════
#  CAMERA
# ═══════════════════════════════════════════════════════════════
pipe = subprocess.Popen(
    ["rpicam-vid",
     "--width", str(WIDTH),
     "--height", str(HEIGHT),
     "--framerate", str(CAPTURE_FPS),
     "--codec", "yuv420",
     "--timeout", "0",
     "-o", "-"],
    stdout=subprocess.PIPE,
    stderr=subprocess.DEVNULL,
    bufsize=10**8
)

def get_frame():
    size = WIDTH * HEIGHT * 3 // 2
    raw  = pipe.stdout.read(size)
    if len(raw) != size:
        return None
    yuv = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT * 3 // 2, WIDTH))
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

def detect_qr(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return decode(gray)

# ═══════════════════════════════════════════════════════════════
#  KALMAN  (1-D, horizontal position only)
# ═══════════════════════════════════════════════════════════════
kf = cv2.KalmanFilter(1, 1)
kf.measurementMatrix   = np.array([[1.0]], np.float32)
kf.transitionMatrix    = np.array([[1.0]], np.float32)
kf.processNoiseCov     = np.array([[2.0]], np.float32)
kf.measurementNoiseCov = np.array([[15.0]], np.float32)
kf.errorCovPost        = np.array([[10.0]], np.float32)

def kalman_init(cx):
    kf.statePost = np.array([[cx]], np.float32)

def kalman_smooth(cx):
    kf.correct(np.array([[np.float32(cx)]]))
    return float(kf.predict()[0, 0])

# ═══════════════════════════════════════════════════════════════
#  STATE
# ═══════════════════════════════════════════════════════════════
S = dict(
    pan=90.0,
    smooth_pan=90.0,
    direction=1,
    state="search",
    consecutive_lost=0,
    prev_error=0.0,
    integral=0.0,
    ema_cx=float(CENTER_X),
    just_acquired=0
)

def reset_pid():
    S["prev_error"] = 0.0
    S["integral"]   = 0.0

def run_pid(cx):
    error = CENTER_X - cx
    if abs(error) >= DEAD_ZONE:
        S["integral"] += error
        derivative = error - S["prev_error"]
        pid  = Kp * error + Ki * S["integral"] + Kd * derivative
        step = SERVO_DIR * np.clip(pid, -MAX_STEP, MAX_STEP)
        if abs(step) >= MIN_STEP:
            S["pan"] = np.clip(S["pan"] + step, 0, 180)
    S["prev_error"] = error
    return error

# ═══════════════════════════════════════════════════════════════
#  MAIN LOOP
# ═══════════════════════════════════════════════════════════════
try:
    while True:
        frame = get_frame()
        if frame is None:
            continue

        qr_codes = detect_qr(frame)

        # ── DETECTED ─────────────────────────────────────────
        if qr_codes:
            S["consecutive_lost"] = 0

            x, y, w, h = qr_codes[0].rect
            cx      = float(x + w // 2)
            qr_size = int(w)           # bounding box width → proxy for distance

            if S["state"] == "search":
                S["pan"]       = S["smooth_pan"]
                S["direction"] = 0
                kalman_init(cx)
                reset_pid()
                S["just_acquired"] = 4
                S["state"] = "tracking"
                print("🎯 ACQUIRED")

            S["ema_cx"] = EMA_ALPHA * cx + (1 - EMA_ALPHA) * S["ema_cx"]
            est         = kalman_smooth(S["ema_cx"])
            error       = run_pid(est)

            # Smooth servo angle
            if S["just_acquired"] > 0:
                S["smooth_pan"] = S["pan"]
                S["just_acquired"] -= 1
            else:
                S["smooth_pan"] = SERVO_EMA * S["pan"] + (1 - SERVO_EMA) * S["smooth_pan"]

            send_command(S["smooth_pan"], qr_size)
            print(f"TRACK | Err:{error:.1f} | Pan:{int(S['smooth_pan'])} | QRsize:{qr_size}")

        # ── NOT DETECTED ─────────────────────────────────────
        else:
            if S["state"] == "search":
                S["pan"] += S["direction"] * SEARCH_STEP
                if S["pan"] >= 165 or S["pan"] <= 15:
                    S["direction"] *= -1
                S["pan"] = np.clip(S["pan"], 0, 180)
                S["smooth_pan"] = SERVO_EMA * S["pan"] + (1 - SERVO_EMA) * S["smooth_pan"]

                send_command(S["smooth_pan"], 0)   # size=0 → Arduino stops motors
                print(f"🔍 SEARCH | Pan:{int(S['smooth_pan'])}")
                continue

            S["consecutive_lost"] += 1

            if S["consecutive_lost"] > MAX_LOST:
                print("❌ LOST → SEARCH")
                S["state"]     = "search"
                S["direction"] = 1
                send_command(S["smooth_pan"], 0)   # stop motors
                reset_pid()
                continue

            # Hold pan, send size=0 to keep motors stopped while holding
            send_command(S["smooth_pan"], 0)
            print("🧠 HOLD")

        time.sleep(0.02)

except KeyboardInterrupt:
    print("⛔ Stopped by user")
    if ser:
        ser.write(b"90,0\n")
        time.sleep(0.1)
        ser.close()
    pipe.terminate()
