# Quadrotor Control

This repository contains the code implementation for the paper titled [Trajectory Tracking Double Two-Loop Adaptive Neural Network Control for a Quadrotor](https://www.sciencedirect.com/science/article/abs/pii/S0016003223000492).

## Introduction

The purpose of this repository is to provide the code necessary to reproduce the results presented in the aforementioned paper. The code is implemented in MATLAB/Simulink and can be used to simulate and evaluate the performance of different controllers for trajectory tracking of a quadrotor.

## Usage

To run the simulation, follow these steps:

1. Open the "simulationFile.slx" file in MATLAB/Simulink.
2. Set the desired parameters for the controllers in the "globalParams.m" file.
3. Choose the controller mode by setting the "Mode" parameter in the "globalParams.m" file:
   - Mode = 1 for the paper proposed controller.
   - Mode = 2 for the TLMBC controller.
   - Mode = 3 for the NNSMC controller.
4. Run the simulation to observe the trajectory tracking performance of the selected controller.

## Requirements

- MATLAB/Simulink
