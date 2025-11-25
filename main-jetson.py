import cv2
import torch
import serial
import time
import numpy as np
from torchvision import transforms

# ---- Serial Communication ----
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # adjust if different port
time.sleep(2)  # wait for Arduino to initialize

# ---- Load trained CNN model ----
model = torch.load('model.pth', map_location='cuda')
model.eval()

# ---- Preprocessing ----
transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((66, 200)),
    transforms.CenterCrop((66, 200)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.5, 0.5, 0.5],
                         std=[0.5, 0.5, 0.5])
])

# ---- Camera Initialization ----
cap = cv2.VideoCapture(0)  # CSI: use GStreamer string if needed

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera frame missing!")
        break

    # ---- Image Preprocessing ----
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_tensor = transform(img).unsqueeze(0).to('cuda')

    # ---- Predict steering angle ----
    with torch.no_grad():
        steering_angle = model(img_tensor).item()

    # ---- Send angle to Arduino ----
    data = f"{steering_angle:.2f}\n"
    ser.write(data.encode())

    # ---- Display Debug ----
    cv2.putText(frame, f"Angle: {steering_angle:.2f}", (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    cv2.imshow('Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
