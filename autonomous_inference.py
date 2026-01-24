"""
Run:
1) Install deps: pip install opencv-python torch numpy pyserial
2) Put model at: models/steering_model.pth
3) Set SERIAL_PORT (e.g., COM7) and CAM_INDEX (0/1)
4) Run: python autonomous_inference.py
"""

import time
from pathlib import Path
import cv2
import numpy as np
import torch
import torch.nn as nn
import serial

# =========================
# CONFIG (edit these)
# =========================
SERIAL_PORT = "COM7"  # change to your Arduino port
BAUD = 115200

CAM_INDEX = 1          # try 0, 1, 2
SHOW_WINDOW = True

DRY_RUN = False      # if True, don't send to Arduino (for testing)

MODEL_PATH = Path(r"models/steering_model.pth")  # change to your saved .pth path

SEND_HZ = 20            # how often to send to Arduino (Hz)
SMOOTH_ALPHA = 0.35     # 0=no smoothing, 1=very heavy smoothing

# Your model outputs angle_norm in [-1, +1] (from your dataset)
MAX_ABS_DEG = 45.0      # -45..+45 (adjust to match your steering limits)

# Input preprocessing must match your training dataset:
NET_W, NET_H = 200, 66

# =========================
# MODEL (same as notebook)
# =========================
class SimpleCNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(   #convolution layers that extracts features from the image
            nn.Conv2d(3, 24, 5, stride=2), nn.ReLU(),
            nn.Conv2d(24, 36, 5, stride=2), nn.ReLU(),
            nn.Conv2d(36, 48, 5, stride=2), nn.ReLU(),
            nn.Conv2d(48, 64, 3), nn.ReLU(),
            nn.Conv2d(64, 64, 3), nn.ReLU(),
        )
        self.regressor = nn.Sequential(   #fully connected layers that maps the extracted features to a steering angle
            nn.Flatten(),
            nn.Linear(64 * 1 * 18, 100), nn.ReLU(),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 10), nn.ReLU(),
            nn.Linear(10, 1)
        )

    def forward(self, x):
        x = self.features(x)    #CNN extracts features from the input image
        return self.regressor(x)   #output the steering angle prediction


def preprocess_frame(frame_bgr: np.ndarray) -> torch.Tensor:
    """BGR frame -> model tensor (1,3,66,200), normalized to [-1,1]."""
    img = cv2.resize(frame_bgr, (NET_W, NET_H))  # Resize to model input size
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # BGR to RGB

    img = img.astype(np.float32) / 255.0    # to [0,1]
    img = (img - 0.5) / 0.5                 # to [-1,1]
    img = np.transpose(img, (2, 0, 1))      # CHW
    x = torch.from_numpy(img).unsqueeze(0)  # add batch dimension
    return x


def norm_to_deg(angle_norm: float) -> float:             # [-1,+1] -> degrees
    angle_norm = float(np.clip(angle_norm, -1.0, 1.0))  # safety clamp
    return angle_norm * MAX_ABS_DEG                     


def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Device:", device)

    # --- Load model ---
    # Verify model path
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"Model not found: {MODEL_PATH.resolve()}")

    model = SimpleCNN().to(device)
    model.load_state_dict(torch.load(MODEL_PATH, map_location=device))
    model.eval()
    print("✅ Loaded model:", MODEL_PATH.resolve())

    # --- Serial connect ---
    ser = None
    if not DRY_RUN:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.05)
        time.sleep(2.0)
        print("✅ Arduino connected:", SERIAL_PORT)
    else:
        print("Arduino not connected")

    # --- Camera connect ---
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"❌ Cannot open camera index {CAM_INDEX}")
    print("✅ Camera connected. Press Q to quit.")

    last_send = 0.0
    smoothed_norm = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("❌ Camera frame read failed.")
                break

            # Inference
            x = preprocess_frame(frame).to(device)
            with torch.no_grad():
                pred_norm = float(model(x).item())  # expected ~[-1,1]

            # Clamp + smooth to reduce jitter
            pred_norm = float(np.clip(pred_norm, -1.0, 1.0))
            smoothed_norm = (SMOOTH_ALPHA * smoothed_norm) + ((1.0 - SMOOTH_ALPHA) * pred_norm)

            # Send at fixed rate (normalized in [-1, +1])
            now = time.time()
            if (now - last_send) >= (1.0 / SEND_HZ):
                v = float(smoothed_norm)
                v = max(-1.0, min(1.0, v))
                msg = f"INF:{v:.3f}\n"

                if not DRY_RUN and ser is not None:
                    ser.write(msg.encode("utf-8"))
                else:
                    print("DRY:", msg.strip())
                    
                last_send = now

            # UI overlay
            if SHOW_WINDOW:
                overlay = frame.copy()
                cv2.putText(overlay, f"pred_norm={pred_norm:+.3f}  smooth={smoothed_norm:+.3f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(overlay, f"send_deg={norm_to_deg(smoothed_norm):+.2f}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(overlay, "Q=quit", (10, overlay.shape[0] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow("Autonomous Inference", overlay)

                if (cv2.waitKey(1) & 0xFF) in [ord('q'), ord('Q')]:
                    break

    finally:
        cap.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        if ser is not None:
            ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
