# ROS2_and_Docker
The project demonstrating skills in using Docker and ROS2.

The project consists of thre ROS2 nodes: 
Robot_station:  
    - adds new points to the list to be gained by the robot,  
    - updates the list when any of the points are reached.  
Robot:  
    - starts and finishes moving at the robot station,  
    - selects the nearest point from the list of points as the target,  
    - communicates with the server to remove a point from the list,  
    - has a simulated way of moving.  
Points_spawner:  
    - generates new points every certain period of time,  
    - possibility to set the number of points at one generation.  
  
Each node runs in a different container.  
  
```bash
docker-compose up --build
```