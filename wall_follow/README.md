# Wall Follow


The 'wall_follow' package is a ROS2 module designed to achieve autonomous driving while maintaining alignment with the centerline within the F1TENTH simulator. It implements a PD controller that takes the difference between the desired and measured distances from the wall as input and produces the appropriate steering angle as output.

https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/6be8bcd7-cfad-44a0-947c-261fc49a2e62

### Control objectives:

* Maintain the car's trajectory along the centerline (y = 0) of the map frame. The origin of the map frame is located at the starting point on the centerline.
* Keep the car  parallel to the walls => theta(the angle formed by the car  and x-axis) = 0

![Untitled (4)](https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/643c0589-1518-4aac-a02b-14275def5465)

source: Source: F1TENTH Course from University of Pennsylvania



##### What is the method for measuring the distance to the wall?

![image](https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/cfb801a5-0c9e-4afe-ae1f-e637edd0d6ae)

We take two laser scans (a and b) emitted by the LiDAR towards the wall. Beam b is positioned 90 degrees to the right of the car's x-axis, and beam a is at an angle theta relative to beam b. By utilizing the two distances a and b from the laser scans, along with the angle between the laser scans, we can calculate the angle alpha between the car's x-axis and the adjacent right wall:

![image](https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/faea7410-3c4b-4a15-b9cd-bf438ecbfad9)

Using the angle alpha, we can determine the current distance Dt to the car

![image](https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/b2d2bf4c-dfc6-4a66-af37-45e146d07c42)

Subsequently, we can calculate the error as follows:

Error = desired_distance - Dt"

And finally, apply the PD controller:

![image](https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/f9a478fd-ac75-44b5-ae5c-0e3884e38ef9)



Additionally, we will incorporate speed control for different driving scenarios(driving straight or turning):

* When the steering angle ranges from 0 degrees to 10 degrees, the car will maintain a speed of 1.5 meters per second.
* If the steering angle falls between 10 degrees and 20 degrees, the speed will be set to 1.0 meters per second.
* For all other cases, the speed will be adjusted to 0.5 meters per second
