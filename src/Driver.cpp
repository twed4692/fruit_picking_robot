/*--------------------------------------------------------------------------------------------------------
C++ implementation of Driver class. Has a Odometer and Lidar Subscriber to provide it with appropriate 
data, as well as a Velocity Publisher to update the linear and angular velocitiies of the motors.

Functionality:
- Enter Node - Enters a node to drive to the middle of that node
- Exit Node - Exit node untill orchard wall are on either side
- Driver Forward
- Turn Right 
- Turn Left
- Pick Fruit
- Empty Basket
- Turn Around
- Constantly checking if at a node (AtNode) returns bool

Original revision: 490410688
--------------------------------------------------------------------------------------------------------*/

#include "Driver.h"
#include <cmath>            // trig functions in PD controller

/*--------------------------------------------------------------------------------------------------------
Constructor, initialise contoller variables to 0 and update velocity to 0.
--------------------------------------------------------------------------------------------------------*/
CDriver::CDriver() : _loopRate(125)
{
    // Elderberry starts stationary update the velocity to 0
    _motorControl.UpdateCommandVelocity(0.0, 0.0);

    // Initialise errors for the PD controller
    _error = 0.0;
    _prevError = 0.0;
}


/*--------------------------------------------------------------------------------------------------------
Enter a node by driving to the center of it.
--------------------------------------------------------------------------------------------------------*/
void CDriver::EnterNode()
{
    // Set the previous position data to compare to current
    _positionData.SetPrevPose();
    COdomSub::coordinates prevPos = _positionData.GetPos(COdomSub::PREV);
    
    // Continuiosly loop until desired state is acheived
    while(true) 
    {    
        // Set the current Odometer data data
        COdomSub::coordinates currPos = _positionData.GetPos(COdomSub::CURRENT);

        // Calculate how far we have moved
        double deltaX = fabs(currPos.first - prevPos.first);
        double deltaY = fabs(currPos.second - prevPos.second);

        // Define conditions for hitting wall and going half the node distance
        bool hittingWall = _lidarData.GetDistance(CLidarSub::FORWARD) <= _HALF*_ORCHARD_WIDTH;
        bool travelledHalfNodeDist = deltaX + deltaY >= _HALF*_ORCHARD_WIDTH;

        // If Elderberry is about to drive into a wall or has driven the required distance finish entering
        // node
        if(hittingWall || travelledHalfNodeDist)
        {
           // Reset the previous position and angle
            _positionData.SetPrevPose();

            // Stop motors and break out of exit node
            _motorControl.UpdateCommandVelocity(0.0, 0.0);
            _loopRate.sleep();
            break; 
        }
        else
        {
            // keep driving forward slowly to improve accuracy
            _motorControl.UpdateCommandVelocity(_LINEAR_SLOW, 0.0);
            ros::spinOnce();
            _loopRate.sleep();
        }
    }
}


/*--------------------------------------------------------------------------------------------------------
Exit a node by driving out of it untill walls are detected on either side.
--------------------------------------------------------------------------------------------------------*/
void CDriver::ExitNode()
{
    // Continuiosly loop until desired state is acheived
    while(true) 
    {      
        // Define the lidar check values for exiting nodes
        bool LeftWallCheck = _lidarData.GetDistance(CLidarSub::LEFT) <= _ORCHARD_WIDTH;
        bool RightWallCheck = _lidarData.GetDistance(CLidarSub::RIGHT) <= _ORCHARD_WIDTH;
        bool RightLargerZero = _lidarData.GetDistance(CLidarSub::RIGHT) > 0;
        bool LeftLargerZero = _lidarData.GetDistance(CLidarSub::LEFT) > 0;

        // If wall on left or right is detected break stop exiting node
        if (LeftWallCheck && RightWallCheck && RightLargerZero && LeftLargerZero) 
        {
            // Stop motors and break out of exit node
            _motorControl.UpdateCommandVelocity(0.0, 0.0);
            ros::spinOnce();
            _loopRate.sleep();
            break;
        }
        // Continue to drive forward until a wall is detected on the left
        else {
            _motorControl.UpdateCommandVelocity(_LINEAR_SLOW, 0.0);
            ros::spinOnce();
            _loopRate.sleep();
        }
    }
}


/*--------------------------------------------------------------------------------------------------------
Drive Forward continuously using a PD controller to check for error and correct if we are in the center 
of the orchard.

This PD controller utilises the digram found in the report under Modular Design -> Driver Module. The 
diagram shows how the PD controller works and the different variable names and their meaning, 
corresponding to the code below.

The PD controller measures the lidar value on the left (b) and the lidar value 20 degrees north of left 
(a). These two values allow us to calculate the error in angle from the true right value (alpha). Then 
taking another state which is assuming the turtlebot drives AC distance forward at its current trajectory 
we can calculate the error which is CD - half the orchard width.
--------------------------------------------------------------------------------------------------------*/
void CDriver::DriveForward()
{
    // Set the previous error to the current error
    _prevError = _error;

    // Calculate all the values for the final PD controller error
    // Refer to diagram under Modular Design -> Driver Module for diagram showing variable names
    double a = _lidarData.GetDistance(CLidarSub::LEFT_UPPER);
    double b = _lidarData.GetDistance(CLidarSub::LEFT);
    double numerator = a * cos(_THETA*COdomSub::DEG2RAD) - b;
    double denominator = a * sin(_THETA*COdomSub::DEG2RAD);
    double alpha = atan(numerator / denominator);
    double AB = b * cos(alpha);
    double CD = AB + _AC * sin(alpha);

    // Update the error based on calcualted values
    _error = CD - _HALF*_ORCHARD_WIDTH;

    // Control input is based on PD (proportional and differential)
    double angularVel = _P*_error + _D*(_error - _prevError);

    // Drive forward with an angular velocity based on current error
    _motorControl.UpdateCommandVelocity(_LINEAR_FAST, angularVel);
    _loopRate.sleep();
}


/*--------------------------------------------------------------------------------------------------------
Turn right 90 degrees.
--------------------------------------------------------------------------------------------------------*/
void CDriver::TurnRight()
{
    // Set the previous position data to compare to current
    _positionData.SetPrevPose();
    double prevAngle = _positionData.GetAngle(COdomSub::PREV);

    // Continuiosly loop until desired state is acheived
    while(true) 
    {
        // Correct the angles if needed
        _positionData.CheckAngleOverlap();
        
        // Get the angles
        double currAngle = _positionData.GetAngle(COdomSub::CURRENT);

        // Calculate how far we have moved
        double deltaAngle = fabs(currAngle - prevAngle);

        if (deltaAngle >= _ESCAPE_RANGE)
        {
            // Reset previous position and angle
            _positionData.SetPrevPose();
            
            // Stop motors and break out of exit node
            _motorControl.UpdateCommandVelocity(0.0, 0.0);
            _loopRate.sleep();
            break;
        }
        else
        {
            // Turn right slowly to improve accuracy
            _motorControl.UpdateCommandVelocity(0.0, -_ANGULAR_SLOW);
            ros::spinOnce();
            _loopRate.sleep();
        }
    }
}


/*--------------------------------------------------------------------------------------------------------
Turn left 90 degrees.
--------------------------------------------------------------------------------------------------------*/
void CDriver::TurnLeft()
{
    // Set the previous position data to compare to current
    _positionData.SetPrevPose();
    double prevAngle = _positionData.GetAngle(COdomSub::PREV);

    // Continuiosly loop until desired state is acheived
    while(true)
    {
        // Correct the angles if needed
        _positionData.CheckAngleOverlap();
        
        // Get the angles
        double currAngle = _positionData.GetAngle(COdomSub::CURRENT);

        // Calculate how far we have moved
        double deltaAngle = fabs(currAngle - prevAngle);

        if (deltaAngle >= _ESCAPE_RANGE)
        {
            // Reset previous position and angle
            _positionData.SetPrevPose();
            // Stop motors and break out of exit node
            _motorControl.UpdateCommandVelocity(0.0, 0.0);
            _loopRate.sleep();
            break;
        }
        else
        {
            // Turn left slowly to improve accuracy
            _motorControl.UpdateCommandVelocity(0.0, _ANGULAR_SLOW);
            ros::spinOnce();
            _loopRate.sleep();
        }
    }
}


/*--------------------------------------------------------------------------------------------------------
Stop to pick fruit for a short duration.
--------------------------------------------------------------------------------------------------------*/
void CDriver::PickFruit() 
{
    // Loop though _PICK_FRUIT_STOP times and update velocities to zero
    for (int i = 0; i < _PICK_FRUIT_STOP; i++)
    {
        _motorControl.UpdateCommandVelocity(0.0, 0.0);
        ros::spinOnce();
        _loopRate.sleep();
    } 
}


/*--------------------------------------------------------------------------------------------------------
Stop to empty basket for a short duration.
--------------------------------------------------------------------------------------------------------*/
void CDriver::EmptyBasket() 
{
     // Loop though _EMPTY_BASKET_STOP times and update velocities to zero
    for (int i = 0; i < _EMPTY_BASKET_STOP; i++)
    {
        _motorControl.UpdateCommandVelocity(0.0, 0.0);
        ros::spinOnce();
        _loopRate.sleep();
    }
}


/*--------------------------------------------------------------------------------------------------------
Turn right twice to turn around.
--------------------------------------------------------------------------------------------------------*/
void CDriver::TurnAround() 
{
    // Turn right twice
    TurnRight();
    TurnRight();
}


/*--------------------------------------------------------------------------------------------------------
Function that returns true if at a node; robot is at a node if there is sufficient empty space on either
side.
--------------------------------------------------------------------------------------------------------*/
bool CDriver::AtNode()
{
    // Default state is not at node
    bool atNode = false;

    // Get values from lidar for empty space
    bool emptySpaceLeft = _lidarData.GetDistance(CLidarSub::LEFT) > _EMPTY_SPACE_CHECK*_ORCHARD_WIDTH;
    bool emptySpaceRight = _lidarData.GetDistance(CLidarSub::RIGHT) > _EMPTY_SPACE_CHECK*_ORCHARD_WIDTH;
    
    // We are at a node if there is empty space to the left or right
    if (emptySpaceLeft|| emptySpaceRight) 
    {
        // If there is empty space (no orchard wall) on the left or the right return true for at node
        atNode = true;
    }
    return atNode;
}

COdomSub* CDriver::GetOdomPtr()
{
    return &_positionData;
}