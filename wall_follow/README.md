![kkkkk](https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/a8d7b0e8-d872-411e-835c-a0f05f7843fc)# Wall Follow

The 'wall_follow' package is a ROS2 module designed to achieve autonomous driving while maintaining alignment with the centerline within the F1TENTH simulator. It implements a PID controller that takes the difference between the intended and measured distances from the wall as input and produces the appropriate steering angle as output.

### Control objectives:
* Maintain the car's trajectory along the centerline (y = 0) of the map frame. The origin of the map frame is located at the starting point on the centerline.
* Keep the car  parallel to the walls => theta(the angle formed by the car  and x-axis) = 0
![kkkkk](https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/394053e0-05fc-415f-afa3-765722be3258)


how do we measure the distance to the wall?

![image](https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/cfb801a5-0c9e-4afe-ae1f-e637edd0d6ae)

we take two laser scans (a and b) emitted by the lidar to the wall. beam b: 90 degrees to the right of the car's x-axis and beam a: at angle theta to the beam b. 
Using the two distances a and b from the laser scan, the angle between the laser scans and we can calculate the angle alpha between the car's x-axis and the right wall :
   ![image](https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/faea7410-3c4b-4a15-b9cd-bf438ecbfad9)

Using angle alpha we can find the current distance Dt to the car
![image](https://github.com/kiiwii22/Autonomous-Racing-Car/assets/76494996/b2d2bf4c-dfc6-4a66-af37-45e146d07c42)
