# robotics
Direct and inverse kinematics and dynamics of a serial 6DOF robot arm written in Matlab


Use the following browser based tool to animate the robot
https://web.am.ed.tum.de/roboterdynamik_praktikum/


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



