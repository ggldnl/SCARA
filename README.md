# SCARA

![Preview](media/preview.png)

The SCARA (Selective Compliance Assembly Robot Arm) is a type of robotic arm designed for tasks requiring high precision and speed, like assembly and pick-and-place operations. Its configuration allows for rigid vertical movement while maintaining flexibility in the horizontal plane. 

This repository contains the design files, source code, and documentation for my SCARA robotic arm (forgive the redundancy). The arm is mostly 3d printed in order to offer a balance between affordability and functionality.

## Project history and current state

I wanted to build a simple robotic arm and found out the [pyBot](https://jjrobots.com/) project that seemed to have a lot of potential and looked like a good starting point for further development. As I finished assembling the hardware according to the guide, the domain for the website hosting the project expired and was not renewed. At the time of writing this README, the site is still offline, meaning I no longer had access to the original code. The printable files are still available on Thingiverse [here](https://www.thingiverse.com/thing:4579405), and the Hackaday page is still accessible [here](https://hackaday.io/project/175419-pybot-scara-robotic-arm-3d-printed-python).

Considering the situation, I decided to build my own robotic arm taking inspiration from this project. I redesigned everything from scratch in Fusion 360 to have precise control over the robot arm's parameters and to have accurate measurements for the inverse kinematics. The improvements with respect to the previous version include:

- tensioners for joint 2 and 3.
- endstops for each motor, essential for calibration and to recover from missed steps.
- heat set inserts to avoid screwing directly into the plastic.
- simplified assembly for joints 2 and 3.
- improved support for joints 2 and 3 from the `z_base`, now consisting of two pieces.
- control box for better organization, with optional fan to keep the stepper driver cool.
- universal tool mounting point (not limited to the previous servo motor insert).
- cleaner code with no external dependency (I wrote all the libraries I used), modular and easy to read and understand .

Below are two renders of the arm in Fusion 360; the left image shows it in the standard material and the right one has colored plastic components that match the actual arm I built.

<p align="center">
  <img src="media/SCARA.png" alt="Standard Material" width="45%" />
  <img src="media/SCARA_colored.png" alt="Colored Plastic" width="45%" />
  <p align="center">* missing screws, nuts and heat set inserts</p>
</p>

The original project utilized a DEVIA board, which is an Arduino Zero with built-in sockets for three A4988 stepper motor drivers and onboard drivers for four servos. I opted for an Arduino Uno paired with a CNC shield since it is more accessible and I happened to have the two lying around. This setup provides four sockets for stepper drivers and three pins that can be used for the endstops or the end-effector. I also added a 12V mini fan (used for 3d printers) to cool the stepper drivers.

## Examples

TODO

## 3D printing

<p align="center">
  <a href="https://makerworld.com/en/models/572256#profileId-492414">
    <img src="https://gist.githubusercontent.com/ggldnl/0cb170e629d8188959e9c26c36bc32bd/raw/button_makerworld_fixed_w200_h50.png" alt="Open on Makerworld"/>
  </a>
</p>

<p align="center">
  <a href="https://www.thingiverse.com/thing:6743905">
    <img src="https://gist.githubusercontent.com/ggldnl/0cb170e629d8188959e9c26c36bc32bd/raw/button_thingiverse_fixed_w200_h50.png" alt="Open on Thingiverse"/>
  </a>
</p>

<p align="center">
  <a href="https://www.printables.com/en">
    <img src="https://gist.githubusercontent.com/ggldnl/0cb170e629d8188959e9c26c36bc32bd/raw/button_printables_fixed_w200_h50.png" alt="Open on Printables"/>
  </a>
</p>

## Assembly instructions

Refer [here](/hardware/BOM.md) for the Bill of Materials and some notes on what you will need.

<p align="center">
  <img src="media/cadasio.png"/>
</p>

<p align="center">
  <a href="https://instructions.online/?id=12092-scara">
    <img src="https://gist.githubusercontent.com/ggldnl/0cb170e629d8188959e9c26c36bc32bd/raw/button_assembly-instructions_fixed_w300_h50.png" alt="Assembly instructions"/>
  </a>
</p>

## TODO

- [ ] Design a universal mounting block to attach tools to the forearm
- [x] Forget AccelStepper/MobaTools and write your own library to simultaneously control multiple steppers
- [ ] Add forearm calibration endstop
- [x] Complete the BOM with the list of M3 screws needed
- [ ] Add schematics
