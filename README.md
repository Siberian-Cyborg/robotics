# robotics
Direct and inverse kinematics and dynamics of a serial 6DOF robot arm written in Matlab


Use the following browser based tool to animate the robot
https://web.am.ed.tum.de/roboterdynamik_praktikum/


# Path Planning
Four different algorithms for planning a trjectory thru a set of waypoints were implemented.
##Point to Point methods: cubic and quintic polynomials

The image below shows a point to point method that uses quintic polynoms to connect the points. The robot makes a full stop at each waypoint.
![Path](https://user-images.githubusercontent.com/96864967/152689973-166e7668-1d7e-491f-b9a9-fa2f10d12d1b.png)

##Cubic Splines
The image below shows a continous method using cubic splines to find a smooth trajectory thru each point. The robot will not stop at the waypoints. 
![PathCubicSplines](https://user-images.githubusercontent.com/96864967/152690161-55ca5320-e841-46bd-84a5-63624dae5a84.png)

here the robots motion:
https://user-images.githubusercontent.com/96864967/152690255-9e870a14-d251-4cae-81db-2d61efcb11c8.mp4

##Parabolic Blends
The image below shows the method using parabolic blending. The robot will not reach the points but instead only approximate them. 
![PathParablends](https://user-images.githubusercontent.com/96864967/152690219-51905f67-48e5-4085-b66b-f8c11a17c0f0.png)

here the robots motion:
https://user-images.githubusercontent.com/96864967/152690288-93c839e7-96bc-4e04-a398-b3c967cfb12e.mp4

