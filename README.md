# Home Position Tuning for Endoscopic Surgical Robot - NTU FYP

## Project Overview

This repository contains the source code and report for my Final Year Project (FYP), titled “Home Position Tuning for Endoscopic Surgical Robots.” The project presents an innovative control algorithm designed to calibrate the home position of robotic end-effectors using motor current feedback for enhanced accuracy in minimally invasive surgeries.

## Contents
- Final Code.py — Python implementation of the control algorithm.
- B217 FYP Report - Hakyung Yi.pdf — Detailed project report, including methodology, results, and discussion.
- README.md — Overview and description of the project.

## Project Motivation

Endoscopic surgical robots rely heavily on precise positioning for safe and effective procedures. However, due to the inherent friction and compliance in tendon-sheath mechanisms (TSM), traditional home positioning methods often lead to errors. This project addresses this issue by leveraging motor current feedback to automatically detect and set the home position, eliminating the need for manual calibration.

## Objectives
- Develop a control algorithm that uses motor current feedback to tune the home position of the robotic gripper.
- Validate the algorithm under varying tendon-sheath configurations and contact angles.
- Minimize manual adjustments and improve accuracy and efficiency.

## Key Features
- Sensorless Feedback: Home position is detected using motor current changes rather than external sensors.
- Adaptive Algorithm: Automatically adjusts to varying sheath configurations (straight, half-sine, and full-sine wave paths).
- Robust Experimental Setup: Simulated 1-meter tendon-sheath mechanism using an MX106 Dynamixel motor and custom-built prototypes.

## Experimental Setup
Hardware:
- MX106 Dynamixel motor for position and current control.
- 3D-printed pulley with ball bearings.
- Adjustable sheath holder to support different configurations.

Software:
- Python-based control algorithm with moving average filter and peak detection for current feedback smoothing.
- Real-time data acquisition and torque calculations.

## Methodology
1. System Initialization: The motor is calibrated using baseline current readings.
2. Peak Detection: Current feedback is monitored for spikes when the gripper touches the endoscope walls.
3. Angle Calculation: The angles at contact points are recorded and converted to degrees.
4. Home Position Tuning: The midpoint of the contact angles is computed as the home position.

## Configurations Tested:
- Straight Configuration: Baseline with minimal friction.
- Half-Sine Configuration (sin(0.5x)): Moderate bends.
- Full-Sine Configuration (sin(x)): Highly curved path with significant friction.

## Results
- Accuracy: The algorithm consistently detected the home position with minimal error.
- Performance: The moving average filter (window size of 8) and current thresholding improved detection stability.
- Limitations: High curvature configurations introduced more noise but were compensated by adaptive feedback.

## Future Work
- Extend the algorithm to support multi-degree-of-freedom (multi-DOF) robotic arms.
- Implement machine learning-based predictive models to improve accuracy further.
- Conduct in-vivo testing to validate performance in real surgical environments.


## Acknowledgments

Special thanks to Prof. Phee Soo Jay Louis, Dr. Wenjie, and Dr. Jiajun for guidance and the NTU Robotics Research Center for providing facilities and support throughout the project.


![image](https://github.com/user-attachments/assets/01ef5bbf-d353-4f26-9b21-fa8673cc03c8)
