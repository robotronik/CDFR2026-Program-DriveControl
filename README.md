# CDFR 2026 Control Board

## Description

This program is designed for the CDFR control board. This board supports motor control, odometry for position sensing, closed-loop control, and I2C communication. The program is intended to run on an STM32F401RC.

## I2C (Slave mode only)

The I2C adress of this bord is 42 in decimal.

[find interface documentation ](/include/interface/drive_interface.h)

## Installation

To install this project, follow these steps:

1. Clone the repository to your local machine.
2. Open the project with Visual Studio Code and the PlatformIO extension.
3. Connect an STLink as follows:
4. Make sure the 2 STLink jumpers are disconnected.
5. Make sure the correct port is set up in the platform.ini file.

## Author

This program was created by Guillaume DALLE (2023-2025) (guillaume.dalle@grenoble-inp.org)
and Ludovic BOUCHARD (2025-) (ludovicb1239@gmail.com)