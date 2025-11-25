# FYP-Project
+-------------------------------------------------------------+
|                    NVIDIA Jetson Nano                       |
|-------------------------------------------------------------|
|  - Capture frame (CSI camera)                               |
|  - Preprocess image (crop, normalize, resize)               |
|  - CNN model predicts steering angle                        |
|  - Send angle to Arduino via serial (USB)                   |
|  - Optional: display debug info / frame visualization       |
+-------------------------------------------------------------+
                     ↓ Serial (UART/USB)
+-------------------------------------------------------------+
|                        Arduino Uno                          |
|-------------------------------------------------------------|
|  - Receive steering angle                                   |
|  - Compute PID correction                                   |
|  - Control steering servo (angle)                           |
|  - Control DC motor (PWM for forward motion)                |
+-------------------------------------------------------------+
