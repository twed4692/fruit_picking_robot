/*--------------------------------------------------------------------------------------------------------
Velocity pub has a ROS publisher for the linear and angular velocities of the turtlebot through the 
/cmd_vel topic. It is responsible for:

- Publishing the velocities

Original revision: 510548722
--------------------------------------------------------------------------------------------------------*/

#ifndef UPDATEVELOCITY_H
#define UPDATEVELOCITY_H

#include "ros/ros.h"                    // Used to create a ros node handle
#include <geometry_msgs/Twist.h>        // Needed to publish the velocity to elderberry

class CVelocityPub {

    public:
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        CVelocityPub();                                             // Constructor
        void UpdateCommandVelocity(double linear, double angular);  // Publishes the new desired velocity

    private:
        /*------------------------------------------------------------------------------------------------
        ROS node handle and publisher variables
        ------------------------------------------------------------------------------------------------*/
        ros::NodeHandle _nh;        // Node handle to communciate with ROS master
        ros::Publisher _cmdVelPub;  // ROS publisher to the /cmd_vel topic

        /*------------------------------------------------------------------------------------------------
        Constants
        ------------------------------------------------------------------------------------------------*/
        const int _QUEUE_SIZE = 10;
};

#endif