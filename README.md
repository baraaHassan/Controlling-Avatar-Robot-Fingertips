# Inverse-Kinematics
Controlling the Schunk SVH robotic hand (9 DoF) fingertips using the sense-glove (20 DoF) fingertips to do precise manipulation (i.e picking something) using Inverse Kinematics and Gradient Descent

The hand_controller.cpp file contains the main class that controls the communication between the sense-glove and the avatar robotic hand (including the listner to the sense-glove and the publisher to the robotic hand).

The hand_ik.cpp file contains the class that contains the functions that compute the Forward Kinematics for the glove and the Inverse Kinematics for the robotic hand.
