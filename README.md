- Controlling Avatar Robot Fingertips
  - [introduction](#introduction)
  - [The objective of the project](#the-objective-of-the-project)
  - [Hardware](#hardware)
  - [Our Solution](#our-solution)
  - [Output Videos](#output-videos)
  - [Detailed report](#detailed-report)
  - [Files](#files)

## Introduction
An Avatar Robot is a robot, which is being controlled from a remote
location using a human operator, such that it can be used to reach dangerous
areas, that are unsafe for human beings.

## The objective of the project
Controlling the Avatar Robot’s hand fingertips, with a remote gloves-like operator, to make the Robot pick something with its fingertips.

![20201007_174736](https://user-images.githubusercontent.com/49837627/150657463-27f3529f-ed74-4da1-a0c5-a2d3c3a42ae5.jpg)

## Hardware
### Avatar Robot's Schunck Hand
The Schunk Hand has 9 DoF, so there are only 9 active joints that control the motion of the 5 fingertips of the hand, while all the other joints are just mimicking those 9 DoF joints, to achieve the goal pose of the fingertips.

![download](https://user-images.githubusercontent.com/49837627/150656795-4d56b38f-83b7-4c52-b12d-48e72901dee3.jpeg)

### Sense Glove (to control the Avatar remotely)
The Sense Glove has 20 DoF, such that there are 20 active joints that one can move freely, which means we need to map this flexible moving hand to the strict robotic hand (20 DoF to 9 DoF).

![20201007_182150](https://user-images.githubusercontent.com/49837627/150656834-5ad76ab4-6bc4-4283-a8f1-e44c247a7bb8.jpg)

## Our solution
We tried to address this problem using Inverse Kinematics and Gradient Descent. We implemented our solution using **ROS** library on simulated Avatar hand and sense glove, and later we deployed the solution on the hardware.

## Output videos
https://user-images.githubusercontent.com/49837627/150657486-72679f18-1dbe-4c5e-b36d-96211fcef5b7.mp4

https://user-images.githubusercontent.com/49837627/150657493-c01e57a9-e5de-4e8b-a8c3-04c2587976ea.mp4

https://user-images.githubusercontent.com/49837627/150657504-c3e8baa7-a4df-4076-8e96-bcc388475252.mp4

## Detailed report
Cognitive_Robotics_Lab_report.pdf is the report for the detailed explanation.

## Files
The hand_controller.cpp file contains the main class that controls the communication between the sense-glove and the avatar robotic hand (including the listner to the sense-glove and the publisher to the robotic hand).

The hand_ik.cpp file contains the class which its member functions compute the Forward Kinematics for the glove and the Inverse Kinematics for the robotic hand.
