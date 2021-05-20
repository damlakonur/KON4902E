# KON4902E
Control and Automation Engineering Design II

Vision Based Trajectory Control of The MIT Racecar

![car](https://user-images.githubusercontent.com/28154541/118905750-01334d80-b925-11eb-952d-5a9dbff1ad6c.jpeg)

In this project, MIT Racecar platform is used to real time lane detection and tracking. For image processing, OpenCV library of the Python is used. 
On the road, input image comes from the zed camera and it is processed, output of the image processing is lateral error of the vehicle with respect to body frame. 

For the lateral and longitudinal control of the vehicle different controller structures are implemented such as P, PD and Stanley Controllers.
