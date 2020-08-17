# Drone-Project
## Color Tracking Script, etc
This project includes scripts for flight and object detection. 
The drone takes off and assumes a pre-written gps flight plan, watching for objects within a specific range on the color spectrum. once an object is spotted, the drone stops, attempts to center the object in the frame, does a flip, and restarts its flight plan/


You need bebop_autonomy as well as OpenCV. Links for each and/or how to install each:
OpenCV: http://mohsaad.com/2017/07/17/Installing-ROS-With-OpenCV/
bebop_autonomy: https://bebop-autonomy.readthedocs.io/en/latest/installation.html
