import cv2
import torch
import time
import numpy as np

# If you want to enable Arduino later, just uncomment these lines:
import serial
import serial.tools.list_ports


# ===== Try to open Arduino serial (optional) =====
def open_arduino():
    try:
        # Auto-detect a ttyACM* or ttyUSB* port
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if 'ACM' in p.device or 'USB' in p.device:
                print("[INFO] Found possible Arduino on", p.device)
                ser_ = serial.Serial(p.device, 9600, timeout=1)
                time.sleep(2)  # allow Arduino reset
                print("[INFO] Serial opened.")
                return ser_
        print("[WARN] No Arduino port found. Running WITHOUT serial.")
        return None
    except Exception as e:
        print("[WARN] Could not open serial port:", e)
        print("[WARN] Running WITHOUT serial.")
        return None


ser = open_arduino()   # will be None if Arduino not connected


# ===== Load trained model =====
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Running on:", device)

model = torch.load('model.pth', map_location=device)
model.eval()


# ===== Preprocessing (no torchvision) =====
def preprocess(frame):
    # Resize to (200x66) for NVIDIA-style model
    img = cv2.resize(frame, (200, 66), interpolation=cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = (img - 0.5) / 0.5        # [-1, 1]
    img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
    tensor = torch.from_numpy(img).unsqueeze(0)
    return tensor.to(device)


# ===== Camera Setup =====
cap = cv2.VideoCapture(0)   # change to CSI pipeline later if needed

if not cap.isOpened():
    print("Camera not detected!")
    exit()

print("Camera activated.")


# ===== Main Loop =====
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    img_tensor = preprocess(frame)

    with torch.no_grad():
        steering_angle = model(img_tensor).item()

    # ---- Send to Arduino ONLY if serial is available ----
    if ser is not None:
        msg = f"ANG:{steering_angle:.2f}\n"
        try:
            ser.write(msg.encode())
        except Exception as e:
            print("[WARN] Serial write failed:", e)
            ser = None  # stop using serial

    # ---- Debug Display ----
    cv2.putText(frame, f"Angle: {steering_angle:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow('Jetson Camera Feed', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# ===== Cleanup =====
cap.release()
cv2.destroyAllWindows()
if ser is not None:
    ser.close()