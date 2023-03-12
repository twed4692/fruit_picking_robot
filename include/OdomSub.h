/*--------------------------------------------------------------------------------------------------------
Odom Sub has a ROS subscriber to the odometry data (position and angle) published by turtlebot bringup. It 
is responsible for:

- Reading in the data using a subscriber callback.
- Updating previous and current state.
- Accounting for angle overflow.
- Passing data to the driver class, which has an odom sub.

Original revision: 510548722
--------------------------------------------------------------------------------------------------------*/

#ifndef ODOM_SUB_H
#define ODOM_SUB_H

#include "ros/ros.h"            // Used for creating a ros node handle
#include <utility>              // Needed for pairs
#include <cmath>                // PI, and in .cpp file: sin, cos, atan2
#include <nav_msgs/Odometry.h>  // Needed to get positions and angles

class COdomSub {

    public:
        /*------------------------------------------------------------------------------------------------
        Typedefines, enums and constants
        ------------------------------------------------------------------------------------------------*/
        typedef std::pair<double, double> coordinates;  // Typedefine a pair of doubles as coordinates
        enum ePose{PREV, CURRENT};                      // Define the pose (angle and position) required
        static constexpr double DEG2RAD = M_PI/180;     // Constant to convert degrees to radians
        
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        COdomSub();                                     // Constructor
        coordinates GetPos(ePose pos);                  // Get the position (pos for short) of the bot
        double GetAngle(ePose angle);                   // Get the angle of the bot
        void SetPrevPose();                             // Set the previous position as the current position
        void CheckAngleOverlap();                      // Check the angle overlap

    private:
        /*------------------------------------------------------------------------------------------------
        ROS node and subscriber variables/functions
        ------------------------------------------------------------------------------------------------*/
        ros::NodeHandle _nh;                            // Node handle to communicate with ROS master
        ros::Subscriber _odomSub;                       // ROS subscriber to /odom topic
        // Callback function to subscribe to odom
        void OdomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);

        /*------------------------------------------------------------------------------------------------
        Data storage variables
        ------------------------------------------------------------------------------------------------*/
        coordinates _elderberryPos;                     // Current position
        coordinates _elderberryPrevPos;                 // Previous position
        double _elderberryAngle;                        // Current angle (yaw)
        double _elderberryPrevAngle;                    // Previous angle
        
        /*------------------------------------------------------------------------------------------------
        Constant internal variables
        ------------------------------------------------------------------------------------------------*/
        const int _OVERLAP_ANGLE = 270;                 // Overlap angle checking value
        const int _QUEUE_SIZE = 10;                     // Queue size
        const double _ANG_ADJ = 2.0;                    // Angle adjustment factor for callback function
};

#endif