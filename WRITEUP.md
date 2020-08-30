# SELF-DRIVING CAR NANODEGREE - Highway Driving Project 
In this project, the goal is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.  

## RUBRIC CRITERIA
For the project, each rubric criteria is satisfied.  

### Compilation  
The code is compiled without errors.  

![](img/compile.png)  

### Distance without an incident  
I run the simulator for 10 minutes and the car drive **6.97 miles** without an incident.  

![](img/distance.png)  

### Speed and Acceleration Limits  
The car did not exceed the speed limit (50 MPH), total acceleration (10 m/s^2) and jerk limit (10 m/s^3).  

![](img/speedlim.png)  

### No Collision  
The ego did not collide by check it's surrounding cars and predict their behavior.  
According to behavior prediction, the ego car changed it's lane safely.  

### Staying in the Lane  
Ego car stayed in its lane, except for the time between changing lanes.  

### Lane Change  
The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.  
(Please see the animation below)  

![](img/lc.png)

###  Reflection  
The coding details is provided in the README.md document.  

**Note:** A 10x faster video (simulation.mp4) for the simulation is provided.