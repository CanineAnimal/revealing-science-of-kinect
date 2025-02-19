# The Revealing Science of Kinect

## Introduction

This C++ application saves data from the Azure Kinect to a CSV file in a custom location! Code is adapted from the simple_cpp_sample in [Azure Kinect Samples](<https://github.com/microsoft/Azure-Kinect-Samples/tree/master>). **Follow build instructions in that repository.**

Obviously, this script requires that the computer running it be connected to a Kinect.

## Usage

Run rsok.exe. In the console, enter when prompted:

1) The full path to the destination file when prompted (the program will create the file if it does not exist already);
2) The camera angle around each axis in degrees (0 if the camera is level or you don't know);
3) How long you want the recording to be in seconds; and
4) Any text to start the recording.

It will stop automatically after the specified period of time, after which the CSV data will be saved at the specified file.

## Technical

The script has a feature to automatically adjust the position of each joint based on the angle of the camera. You can enter the angle the camera is at in each axis when beginning the program.

The rotation is applied by the following code, wherein `position.v[0]`, `position.v[1]` and `position.v[2]` correspond to the raw X, Y and Z values respectively; `x_final`, `y_final` and `z_final` are the final processed values for X, Y and Z respectively.

```float y_rot = position.v[1] * cos(x_angle * M_PI/180) + position.v[2] * sin(x_angle * M_PI/180);
float z_rot = position.v[2] * cos(x_angle * M_PI/180) - position.v[1] * sin(x_angle * M_PI/180);
float x_rot = position.v[0] * cos(y_angle * M_PI/180) + z_rot * sin(y_angle * M_PI/180);
float z_final = z_rot * cos(y_angle * M_PI/180) - position.v[0] * sin(y_angle * M_PI/180);
float x_final = x_rot * cos(z_angle * M_PI/180) + y_rot * sin(z_angle * M_PI/180);
float y_final = y_rot * cos(z_angle * M_PI/180) - x_rot * sin(z_angle * M_PI/180);```

## Legal

As this is adapted from a project which uses the MIT license, the same license is used for all code in this repository. This code was originally written in 2025 for use by Massey University.
