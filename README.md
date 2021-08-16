# DeliveryDrone
This drone was developed while participating in E-yantra 2020, IIT Bombay.

The package delivery drone has been simulated on Gazebo and scripted in ROS Melodic. Two PID controllers were used for orientation and position controlling. Path planning module consisting of a simple bug2 algorithm for manuevering was implemented. Pyzbar library was used for reading QR code on packages to take input as address in longitude, lattitude and altitude, while image processing was employed to identify and land on the landing marks using OpenCV.

Demo: https://youtu.be/uIXAZy_XAzM 

Note: It's only compatible with Melodic version of ROS. 
