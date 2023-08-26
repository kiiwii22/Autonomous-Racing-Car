# Safety Node

We've created a ROS2 package named 'safety_node' that implements the concept of Automatic Emergency Braking within the F1THENTH car simulator to prevent collisions and enhance the car's safety
We computed the Time to Collision (TTC) for every laser beam emitted by the LIDAR. The TTC represents the estimated time until a collision would occur between the vehicle and an object, assuming their current trajectories and velocities remain constant
We approximate the time to collision using Instantaneous Time to Collision (iTTC), which is the ratio of instantaneous range to range rate calculated from current range measurements and velocity measurements of the vehicle.
![Untitled](https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/f278de96-777e-4889-8b23-3d5e06bbcad8) 
Source: F1TENTH Course from University of Pennsylvania


