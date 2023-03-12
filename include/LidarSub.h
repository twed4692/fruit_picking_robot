/*--------------------------------------------------------------------------------------------------------
Lidar Sub has a ROS subscriber to the lidar data (distancec to walls) published on the /scan topic by 
turtlebot bringup. It is resposible for:

- Reading in the distances at every angle specified.
- Accounting for infinity readings.
- Passing data to the driver class (which has a lidar sub).

Original revision: 510548722
--------------------------------------------------------------------------------------------------------*/

#ifndef LIDAR_SUB_H
#define LIDAR_SUB_H

#include "ros/ros.h"                    // Used for creating a ros node handle
#include <sensor_msgs/LaserScan.h>      // Used for getting distances from walls

class CLidarSub {

    public:
        /*------------------------------------------------------------------------------------------------
        Enum of directions that the lidar can read from
        ------------------------------------------------------------------------------------------------*/
        enum eDirections{
            FORWARD,
            RIGHT,
            BEHIND,
            LEFT,
            LEFT_UPPER
        };
        
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        CLidarSub();                                // Constructor
        double GetDistance(eDirections direction);  // Getter function for a single lidar reading at the 
                                                    // specified direction

    private:
        /*------------------------------------------------------------------------------------------------
        Internal functions
        ------------------------------------------------------------------------------------------------*/
        // Subscriber callback function
        void LaserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);

        /*------------------------------------------------------------------------------------------------
        ROS node handle and subscriber
        ------------------------------------------------------------------------------------------------*/
        ros::NodeHandle _nh;                        // Node handle to communicate with ROS master
        ros::Subscriber _laserScanSub;              // ROS subscriber to /scan topic

        /*------------------------------------------------------------------------------------------------
        Data storage and angle specifier
        ------------------------------------------------------------------------------------------------*/
        const int _QUEUE_SIZE = 10;                 // Queue size                    
        static constexpr int _NUM_DIRECTIONS = 5;   // Specifies how many lidar readings to make
        const int _scanAngle[_NUM_DIRECTIONS] =     // Array of angles corresponding to the directions enum
                                            {       
                                                0,
                                                270,
                                                180,
                                                90,
                                                70
                                            };    
        double _scanData[_NUM_DIRECTIONS];          // Array of distance readings from the lidar      
};

#endif
