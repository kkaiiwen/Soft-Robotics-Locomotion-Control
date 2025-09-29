# Soft-Robotics-Locomotion-Control

<p align="center">
  <img width="500" src="https://github.com/kkaiiwen/Soft-Robotics-Locomotion-Control/blob/main/Graphics/Soft Locomotive Robot.GIF">
</p>
<p align="center">
    <text> Figure 1: Soft Locomotive Robot Prototype </text>
</p>

## Overview

This repository demonstrates a progression of control strategies for soft robotics actuation, moving from simple manual on-off control toward advanced methods that combine Proportional-Integral-Derivative (PID) and Linear Quadratic Regulator (LQR). It focuses on how a basic open-loop feedforward actuation can be extended into closed-loop feedback control frameworks that enable more precise and adaptive behavior.

## Phase 1: On-off Control

In the first stage, an Arduino Mega 2560 programmed in C++ is used to implement manual open-loop actuation of pneumatic valves. Each valve is driven by a Pulse Width Modulation (PWM) signal whose duty cycle is set manually through potentiometers and switches. This arrangement makes it easy to adjust valve outputs and run simple sequences, but it remains a purely feedforward system: the user defines the valve inputs directly, and no feedback from the sensors is used to modify the actuation. As a result, this stage is useful for basic experiments and for demonstrating locomotion patterns, but it is inherently limited in accuracy when the system dynamics change. It therefore represents the manual, open-loop baseline for soft robotic locomotion.

## Phase 2: PID and LQR Control

The second stage implements closed-loop feedback control in Python on a Raspberry Pi, extending the system beyond simple manual actuation. Here, each channel is regulated with a PID controller, which tracks target pressures, reduces steady-state error, and responds to disturbances more effectively than feedforward input alone. On top of this, an LQR framework is applied to the system model to compute optimal feedback gains. LQR balances tracking performance with actuation effort, yielding smoother and more coordinated regulation across all channels. Together, PID and LQR highlight how feedback mechanisms transform the system from a basic open-loop prototype into a precise and adaptive soft robotic platform.

## Prototyping

My soft robot prototype was fabricated using Ecoflex liquid silicone rubber, cast in custom 3D-printed molds designed in Autodesk Fusion. These molds define internal pneumatic channels, allowing the silicone body to bend and deform when pressurized.

<p align="center">
  <kbd>
    <img width="300" src="https://github.com/kkaiiwen/Soft-Robotics-Locomotion-Control/blob/main/Graphics/Actuator Molds.jpg">
  </kbd>
</p>
<p align="center">
    <text> Figure 2: Actuator Molds </text>
</p>

The actuators were connected to a fluidic control board fitted with a microcontroller, together with solenoid valves, a power system, pneumatic sensors, and manual switches. This setup was assembled on a compact platform to enable testing with different control approaches.

<p align="center">
  <kbd>
    <img width="300" src="https://github.com/kkaiiwen/Soft-Robotics-Locomotion-Control/blob/main/Graphics/Fluidic Control Board.jpg">
  </kbd>
</p>
<p align="center">
    <text> Figure 3: Fluidic Control Board </text>
</p>

For a detailed prototyping tutorial, see the Soft Robotics Toolkit: https://softroboticstoolkit.com/.


