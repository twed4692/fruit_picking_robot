/*--------------------------------------------------------------------------------------------------------
C++ implementation of LidarSub class. A ROS subscriber to the turtlebot's scan data (from the lidar):

- Reads in the raw data, and sets infinity readings to the range max
- Passes data to the driver which has a lidar sub.

Original revision: 510548722
--------------------------------------------------------------------------------------------------------*/

#include "LidarSub.h"
#include <cmath> // isinf

/*--------------------------------------------------------------------------------------------------------
Constructor.
--------------------------------------------------------------------------------------------------------*/
CLidarSub::CLidarSub()
{
    // initialize subscriber
    _laserScanSub  = _nh.subscribe("scan", _QUEUE_SIZE, &CLidarSub::LaserScanMsgCallBack, this);
}


/*--------------------------------------------------------------------------------------------------------
Subscriber call back function to read the lidar distances into the scan data array.
--------------------------------------------------------------------------------------------------------*/
void CLidarSub::LaserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // Loop through the number of directions being checked
    for (int angleIdx = 0; angleIdx < _NUM_DIRECTIONS; angleIdx++)
    {
        // If data is infinity, store range max (3.5m)
        if (std::isinf(msg->ranges.at(_scanAngle[angleIdx])))
        {
            _scanData[angleIdx] = msg->range_max;
        }
        // Otherwise store the actual reading
        else
        {
            _scanData[angleIdx] = msg->ranges.at(_scanAngle[angleIdx]);
        }
    }
}


/*--------------------------------------------------------------------------------------------------------
Getter for the driver class to get a single lidar scan at the specified direction.
--------------------------------------------------------------------------------------------------------*/
double CLidarSub::GetDistance(eDirections direction) {

    return _scanData[direction];
}