# CNN-Based Self-Driving RC Car

This project implements a vision-based steering control system for a small RC car using a Convolutional Neural Network (CNN) and a closed-loop PID controller.

## Project Overview
A front-facing camera captures images of the driving path. A CNN model running on a laptop predicts the steering angle in normalized form (-1 to 1). The predicted steering value is sent to an Arduino via serial communication, where a PID controller controls the steering motor using potentiometer feedback.

## Hardware Used
- Laptop (for CNN training and real-time inference)
- Arduino Uno
- USB Camera
- DC steering motor with potentiometer feedback
- Rear DC motor and motor driver

## System Pipeline
Camera → CNN Model (Laptop) → Serial Communication → Arduino PID Controller → Steering Motor

## CNN Model
- Input: RGB image
- Output: Normalized steering angle (-1 to 1)
- Model type: Regression-based CNN (PilotNet-style architecture)

## Control Method
- Closed-loop PID control is used for steering
- Three different PID parameter sets were tested
- PID Set 1 was selected based on better stability and lower overshoot

## Project Status
- CNN training completed on laptop
- Real-time steering angle inference implemented
- Arduino PID steering control integrated and tested

## Limitations
- Limited dataset size
- Single designated driving path
- Feedback noise affects steering accuracy

## Future Work
- Increase dataset size
- Improve steering feedback filtering
- Test system on more complex paths
