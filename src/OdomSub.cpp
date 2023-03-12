/*--------------------------------------------------------------------------------------------------------
C++ implementation of OdomSub class. A ROS subscriber to the turtlebot's odometry:

- Reads in the raw data, and converts the quaternion orientation to a yaw angle
- Adjusts previous angle state when current angle overflows

Original revision: 510548722
--------------------------------------------------------------------------------------------------------*/

#include "OdomSub.h"

/*--------------------------------------------------------------------------------------------------------
Constructor.
--------------------------------------------------------------------------------------------------------*/
COdomSub::COdomSub()
{
    // Initialise elderberry's initial position, previous position and angles
    _elderberryPos = std::make_pair(0.0, 0.0);
    _elderberryPrevPos = std::make_pair(0.0, 0.0);
    _elderberryAngle = 0.0;
    _elderberryPrevAngle = 0.0;

    // Initialize subscriber
    _odomSub = _nh.subscribe("odom", _QUEUE_SIZE, &COdomSub::OdomMsgCallBack, this);

}


/*--------------------------------------------------------------------------------------------------------
Callback function to update the current position and angle of elderberry
--------------------------------------------------------------------------------------------------------*/
void COdomSub::OdomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg) 
{
    // Converting the odometer data into an angle in which the turtlebot is facing relative to the initial 
    // angular position
    double siny = _ANG_ADJ * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - _ANG_ADJ * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  
	_elderberryAngle = atan2(siny, cosy);

    // Reading in the turtlebot's position
    _elderberryPos = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
}


/*--------------------------------------------------------------------------------------------------------
Set the previous pose (position and angle) to be the current pose
--------------------------------------------------------------------------------------------------------*/
void COdomSub::SetPrevPose() {

    _elderberryPrevPos = _elderberryPos;
    _elderberryPrevAngle = _elderberryAngle;
}


/*--------------------------------------------------------------------------------------------------------
Getter function for current or previous position, depending on the ePose passed as the
function parameter
--------------------------------------------------------------------------------------------------------*/
std::pair<double, double> COdomSub::GetPos(ePose pos) {

    if (pos == PREV) {
        return _elderberryPrevPos;
    }
    else {
        return _elderberryPos;
    }
}


/*--------------------------------------------------------------------------------------------------------
Getter function for current or previous angle, depending on the ePose passed as the 
function parameter
--------------------------------------------------------------------------------------------------------*/
double COdomSub::GetAngle(ePose angle) {

    if (angle == PREV) {
        return _elderberryPrevAngle;
    }
    else {
        return _elderberryAngle;
    }
}


/*--------------------------------------------------------------------------------------------------------
Corrects previous angle to account for angle overflow. When elderberry makes multiple turns the angle 
overflows from positive to negative (or vice versa). This adjusts the previous angle so that when making 
turns, the change in angle calculation is still valid.
--------------------------------------------------------------------------------------------------------*/
void COdomSub::CheckAngleOverlap() {

    /* Check if the angle has overlapped: if angle jumps by more than approximately 360 degrees it has 
    changed sign. We use 270 degrees to account for slight variation around 360 degrees without triggering
    on actual turns of 180 degrees */
    if (fabs(_elderberryAngle - _elderberryPrevAngle) > (_OVERLAP_ANGLE*DEG2RAD)) {
        _elderberryPrevAngle *= -1.0;
    }
}