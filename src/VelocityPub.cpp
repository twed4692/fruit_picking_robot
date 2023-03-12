/*--------------------------------------------------------------------------------------------------------
C++ implementation of VelocityPub class. Has a ROS publisher to the /cmd_vel topic, which controls the 
linear and angular velocity of the turtlebot:

- Velocity commands are published every time ros::spinOnce() is called.

Original revision: 510548722
--------------------------------------------------------------------------------------------------------*/

#include "VelocityPub.h"

/*--------------------------------------------------------------------------------------------------------
Constructor.
--------------------------------------------------------------------------------------------------------*/
CVelocityPub::CVelocityPub()
{
    // Initialize publisher
    _cmdVelPub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", _QUEUE_SIZE);
}


/*--------------------------------------------------------------------------------------------------------
Function to publish the new desired linear and angular velocity of the turtlebot.
--------------------------------------------------------------------------------------------------------*/
void CVelocityPub::UpdateCommandVelocity(double linear, double angular)
{
    geometry_msgs::Twist cmdVel;

    // Collect the current angular and linear velocities
    cmdVel.linear.x  = linear;
    cmdVel.angular.z = angular;
    
    // Publish the velocities
    _cmdVelPub.publish(cmdVel);
}