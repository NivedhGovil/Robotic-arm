# Robotic Arm Project (Master-Follower Teleoperation)

This is a low-cost, 6-axis twin robotic arm system built with smart servos, controlled via teleoperation. 

---

## Project Goal & Overview
The goal of this project was to design and build a robotic arm using servo motors and a microcontroller to reproduce human-like movements. The setup features a **twin robot configuration**—consisting of a master arm (leader) and a follower arm—where movements performed on the master are mirrored precisely by the follower via teleoperation.

---

## Specifications & Architecture

### **Degrees of Freedom & Hardware**
* **Degrees of Freedom (DOF):** 6 axes
* **Actuators:** Waveshare ST3215-HS TTL serial-bus smart servos (12-bit magnetic encoder, 4096 counts/rev, providing real-time feedback on position, speed, load, voltage, and temperature).
* **Controller:** Bus Servo Driver HAT coded via devices. 
* **Power Supply:** Switch-Mode Power Supply (SMPS) providing steady 12V DC power.

### **Key Technical Concepts Explored**
* **Kinematics:** Forward and Inverse Kinematics, Joint angles ($\theta$), Link lengths ($a$), Joint offsets ($d$), and Link twists ($\alpha$).
* **Control & Communication:** Half-duplex TTL UART bus topology (Daisy chaining of a maximum limit of 256 motors with unique IDs).


---

## Bill of Materials (BOM)
* **Servos:** Waveshare ST3215-HS Smart Servos (High-speed variant for dynamic joints, higher torque options for base joints).
* **Driver:** Bus Servo Driver HAT (A)
* **Power:** 12V SMPS Power Adapter
* **Structure:** 3D-printed mechanical components featuring 45-degree self-supporting structural angles and custom linkages.
* **Electronics:** Multimeter, jumper wires, screwdrivers, connections with SMPS, light and camera. 

---

## Software & Programming Progression

The software development:
1. **Basic Port & Communication Tests:** Verifying serial port connections like COMs and pinging active servo IDs to establish connections with the motors. 
2. **ID Assignment & Response Checks:** Scanning for connected hardware and mapping physical joints to unique software IDs (1 through 6).
3. **Position Monitoring & Limit Calibration:** Reading encoder limits to establish safe working ranges for each joint.
4. **Teleoperation & Mimic Control:** Linking the master leader arm to the follower arm.
5. **Data Recording & Machine Learning Pipelines:** Integrating camera feeds and recording datasets for advanced policy training.
*The code was writtten using AI.*

---

## Advanced Robotics & AI Integration

The project incorporates machine learning policies using frameworks like [**LeRobot**](https://huggingface.co/lerobot):
* **ACT (Action Chunking with Transformers):** Teaches the robot to imitate human demonstrations by predicting chunks of future actions using Transformers and Conditional Variational Autoencoders (CVAEs).
* **Diffusion Policy:** Generates robust action trajectories by starting with noise and denoising paths conditioned on visual observations.
* **VLA (Vision-Language-Action):** Bridges visual scenes and language instructions to execute physical tasks.

---

## LICENSE

MIT License
