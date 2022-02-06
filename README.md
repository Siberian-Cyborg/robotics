# robotics
Direct and inverse kinematics and dynamics of a serial 6DOF robot arm written in Matlab


Use the following browser based tool to animate the robot.
https://web.am.ed.tum.de/roboterdynamik_praktikum/

# Direct Kinematics
Direct Kinematics means you know the angles by which each of the joints/motors turns and you want to calculate the movement/position of the end effector/tool center point (meaning the end of the arm). In the following video the movement of the robot is simulated if each of the joints is rotated by 45 deg one after the other:

https://user-images.githubusercontent.com/96864967/152691136-05cdb22d-2ef9-404a-b397-7dfeab3a6933.mp4


# Inverse Kinematics
Inverse Kinematics means that you know the movement of the tool center point (TCP) and you want to calculate the movement of each of the motors/joints. This problem is way harder to solve than the direct kinematics. Usually there is no closed analytic solution (meaning there is no formula), instead the problem is solved numarically. In the code two different methods were used, namely Resolved Mption Rate Control (RMC) and Automatic Supervisory Control (ASC). Additionally a drift compensation for RMC was implemented. For ASC different weighting functions were proposed.

Here an Image of RMC withouth drift compensation. You can see the deviation of the real trjectory the robot executed and the desired TCP trajectory the robot should have executed:

<img src="https://user-images.githubusercontent.com/96864967/152691613-e69953cd-b348-4ca7-99d2-ab729ef8706b.png" width="500" height="500">

Here the same image but with drift compensation. You can see the real and desired trajectories match perfectly:

<img src="https://user-images.githubusercontent.com/96864967/152691625-f37757cb-e504-497a-8be3-0087272ca5ef.png" width="500" height="500">

Here a video of the robots motion for a pick and place task. In this case the trajectory of the TCP is given and RMC + Driftcompensation was used to calculate all joint trajectories.

https://user-images.githubusercontent.com/96864967/152691424-5a3f5edf-f25a-411d-b15b-5d8fed13aafe.mp4


# Dynamics and Control
The difference between dynamics and kinematics is that the former takes into account forces/torques whereas the latter only deals with velocities and accelerations and pretends the world is weightless. In the code a trjectory for the TCP was given. First the inverse dynamics for the trajectory were calculated, meaning we obtain the torques for every motor necessary to follow the fiven TCP trajectory. Afterwards the torques were used to simulate the trajectory using the direct dynamics of the system. A controller was added to compensate for drift due to numerical errors.


Here is a video showing the resulting motion of the robot arm:

https://user-images.githubusercontent.com/96864967/152691705-e65bdf13-ce32-4ed7-81c9-a93e528240bb.mp4


# Path Planning
Four different algorithms for planning a trjectory thru a set of waypoints were implemented.

## Point to Point methods: cubic and quintic polynomials

The image below shows a point to point method that uses quintic polynoms to connect the points. The robot makes a full stop at each waypoint.

<img src="https://user-images.githubusercontent.com/96864967/152689973-166e7668-1d7e-491f-b9a9-fa2f10d12d1b.png" width="500" height="500">

here the robots motion:

https://user-images.githubusercontent.com/96864967/152690627-c3e38723-28f4-441c-a012-d65541e7c0d9.mp4


## Cubic Splines
The image below shows a continous method using cubic splines to find a smooth trajectory thru each point. The robot will not stop at the waypoints.

<img src="https://user-images.githubusercontent.com/96864967/152690161-55ca5320-e841-46bd-84a5-63624dae5a84.png" width="500" height="500">


here the robots motion:

https://user-images.githubusercontent.com/96864967/152690650-dd3089a1-dae2-4e1a-9604-5895bed43b3c.mp4


## Parabolic Blends
The image below shows the method using parabolic blending. The robot will not reach the points but instead only approximate them. 

<img src="https://user-images.githubusercontent.com/96864967/152690219-51905f67-48e5-4085-b66b-f8c11a17c0f0.png" width="500" height="500">

here the robots motion:

https://user-images.githubusercontent.com/96864967/152690754-99215d63-f341-4b73-9ded-1333081f1998.mp4



