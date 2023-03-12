/*--------------------------------------------------------------------------------------------------------
A driver that provides all the functionality to update all the linear and angular velocities of the motors
to traverse the orchard.

All of the driver functions are called by the Movement Controller. During it's movement between nodes the 
Driver's function DriveForward is called continiously to control Elderberry to remain in the centre of the 
orchard, this is what allows Elderberry to go between nodes. 

Other functionalities use ros::Rate to do set turns and straights, comparing previous lidar and position data
with current values.

In order to access all the appropriate data the Driver has a Lidar Subscriber and an Odometer Subscriber.
To control the motors it also has a Velocity Publisher which allows the Driver to change the linear 
and angular velocities.

Original revision: 490401688
--------------------------------------------------------------------------------------------------------*/

#ifndef DRIVER_H
#define DRIVER_H

#include "ros/ros.h"            // Needed for driving functions to reset position while turning
#include "VelocityPub.h"
#include "OdomSub.h"
#include "LidarSub.h"

class CDriver {

    public:
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        CDriver();                  // Constructor
        void TurnLeft();            // Turn 90 degrees to the left
        void TurnRight();           // Turn 90 degrees to the right
        void DriveForward();        // Drive forward using PD controller
        void ExitNode();            // Drive forward to exit a node
        void EnterNode();           // Drive forward to enter a node
        void PickFruit();           // Do turn left to pick fruit and then turn back to straight
        void EmptyBasket();         // Stop to empty basket
        void TurnAround();          // Do a 180 degree turn
        bool AtNode();              // Check if we are in a node
        COdomSub* GetOdomPtr();     // Return a pointer to the odom sub

    private:
        /*------------------------------------------------------------------------------------------------
        Error variables
        ------------------------------------------------------------------------------------------------*/
        double _error;              // Current Error for the PD controller
        double _prevError;          // Previous Error for the PD controller
        
        /*------------------------------------------------------------------------------------------------
        Constant variables
        ------------------------------------------------------------------------------------------------*/
        const double _ORCHARD_WIDTH = 0.42;     // Orchard Width in metres
        const double _THETA = 20;               // Angle between the two lidar rays in controller
        const double _P = 0.7;                  // Proportional gain for the feedback controller
        const double _D = 0.7;                  // Differential gain for the feedback controller
        const double _LINEAR_SLOW = 0.05;       // Linear Velocity Slow
        const double _LINEAR_FAST = 0.07;       // Linear Velocity Fast
        const double _ANGULAR_SLOW = 0.1;      // Angular Velocity Slow
        const double _HALF = 0.5;               // Constant for half orchard width
        const double _PICK_FRUIT_STOP = 150;    // Constant to loop through for picking fruit
        const double _EMPTY_BASKET_STOP = 300;  // Constant to loop through for emptying basket
        const double _EMPTY_SPACE_CHECK = 1.3;  // Coefficient for orchard width empty space check
        
        // Length AC in PD controller (refer to references and diagram)
        const double _AC  = 0.2;              
        // Escape range for the angle the turtlebot is able to spin before stopping
        const double _ESCAPE_RANGE = 90.0 * COdomSub::DEG2RAD; 
        
        /*------------------------------------------------------------------------------------------------
        Owned objects
        ------------------------------------------------------------------------------------------------*/
        CLidarSub _lidarData;       // The Driver has Lidar Data subscriber
        COdomSub _positionData;     // The Driver has Position Data subscriber
        CVelocityPub _motorControl; // The Driver has Motor Control publisher

        /*------------------------------------------------------------------------------------------------
        Ros looprate
        ------------------------------------------------------------------------------------------------*/
        ros::Rate _loopRate;        // ROS looprate for driving functions that are set turns or straights

};

#endif