/*--------------------------------------------------------------------------------------------------------
Main function called by rosrunning the node.

- Initialises the ros node.
- Creates an instance of the movement control class.
- "Warms up" the lidar.
- Runs a navigation loop to pick fruits until all rows have been picked.
--------------------------------------------------------------------------------------------------------*/

#include "ros/ros.h"
#include "MovementControl.h"

int main(int argc, char* argv[])
{    
    // Start the node
    ROS_INFO("Node started");
    ros::init(argc, argv, "orchard_mapping");

    // Bring Elderberry to life
    CMovementControl elderberry;

    ros::Rate loopRate(125);
    // warm up the lidar
    elderberry.WarmUp(loopRate);

    // Traverse the orchard and pick fruits
    while(ros::ok())
    {
        bool keepPicking = elderberry.StartPicking(); // control logic
        ros::spinOnce(); // update publishers and subscribers
        loopRate.sleep();
        if (!keepPicking) break;
    }

    return 0;
}