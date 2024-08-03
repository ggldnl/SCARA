# SCARA



## Project history and current state

I wanted to build a simple robotic arm and found out the [pyBot](https://jjrobots.com/) project that seemed to have a lot of potential and looked like a good starting point for further development. As I finished assembling the hardware according to the guide, the domain for the website hosting the project expired and was not renewed. At the time of writing this README, the site is still offline, meaning I no longer had access to the original code. The printable files are still available on Thingiverse [here](https://www.thingiverse.com/thing:4579405), and the Hackaday page is still accessible [here](https://hackaday.io/project/175419-pybot-scara-robotic-arm-3d-printed-python).

Considering the situation, I decided to build my own robotic arm taking inspiration from this project. I redesigned everything from scratch in Fusion 360 to have precise control over the robot arm's parameters and to have accurate measurements for the inverse kinematics. The improvements with respect to the previous version include:

- tensioners for joint 2 and 3.
- endstops for each motor, essential for calibration and to recover from missed steps.
- heat set inserts to avoid screwing directly into the plastic.
- simplified assembly for joints 2 and 3.
- improved support for joints 2 and 3 from the `z_base`, now composed of two pieces.
- control box for better organization.
- universal tool mounting point (not limited to the previous servo motor insert).

## Examples

## Tutorials

The assembly tutorial can be found [here]().\
The softare installation guide can be found [here]().

## TODO

- [ ] Design a universal mounting block to attach tools to the forearm
- [ ] Forget AccelStepper/MobaTools and write your own library to simultaneously control multiple steppers
