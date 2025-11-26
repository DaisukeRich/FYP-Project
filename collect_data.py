import cv2
import csv
import os
import time

# Output folders
DATASET_DIR = "dataset"
IMG_DIR = os.path.join(DATASET_DIR, "images")
CSV_PATH = os.path.join(DATASET_DIR, "labels.csv")

os.makedirs(IMG_DIR, exist_ok=True)

def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=480,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

# --- Camera (CSI with GStreamer) ---
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Camera not opened!")
    exit()

print("Controls:")
print("  A = steer left")
print("  D = steer right")
print("  S = center (straight)")
print("  Q = quit")
print("Use small smooth steering changes while moving the camera/car.")

# Current steering angle (degrees)
current_angle = 0.0
ANGLE_STEP = 2.0
ANGLE_MIN = -25.0
ANGLE_MAX = 25.0

# --- Main loop ---
csv_file = open(CSV_PATH, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["image_path", "steering_angle"])  # header

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("Frame grab failed.")
        break

    disp = frame.copy()
    cv2.putText(disp, f"Angle: {current_angle:.1f} deg", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("Data Collection", disp)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('a'):
        current_angle -= ANGLE_STEP
    elif key == ord('d'):
        current_angle += ANGLE_STEP
    elif key == ord('s'):
        current_angle = 0.0
    elif key == ord('q'):
        break

    current_angle = max(min(current_angle, ANGLE_MAX), ANGLE_MIN)

    ts = int(time.time() * 1000)
    img_name = f"{ts}.jpg"
    img_path = os.path.join(IMG_DIR, img_name)
    cv2.imwrite(img_path, frame)

    rel_path = os.path.join("images", img_name)
    csv_writer.writerow([rel_path, f"{current_angle:.3f}"])

cap.release()
cv2.destroyAllWindows()
csv_file.close()
print("Data collection finished.")
print(f"Saved CSV: {CSV_PATH}")