# ROS2_and_Docker
The project demonstrating skills in using Docker and ROS2.

The project consists of two ROS2 nodes: robot_station and robot.  
Robot_station:  
    - generates a list of points to be gained by the robot,  
    - updates the list when any of the points are reached.  
Robot:  
    - starts and finishes moving at the robot station,  
    - selects the nearest point from the list of points as the target,  
    - communicates with the server to remove a point from the list.  
    - has a simulated way of moving.  

Robot is placed in one container and the robot_station in another.  

```bash
docker-compose up --build
```