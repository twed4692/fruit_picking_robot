/*--------------------------------------------------------------------------------------------------------
C++ implementation of MovementControl class. This is responsible for determining future actions and
movements of elderberry based off of current state and exiting or entering node state.

Functions here will update the GUI and command both the drive and fruit picker classes to execute movement
logic and current state logic to pick fruit and drive. Including heading home and returning to picking
using Dijkstra's shortest route algorithm.

Initial revision: 500457324
--------------------------------------------------------------------------------------------------------*/

#include "MovementControl.h"

/*--------------------------------------------------------------------------------------------------------
Constructor to set initial values of elderberry's position and current state.
--------------------------------------------------------------------------------------------------------*/
CMovementControl::CMovementControl() : 
    // Initialise pointers for owned objects' known objects
    _dijkstra(&_adjacencyMatrix),
    _text(_driver.GetOdomPtr())
{
    // Initialise node variables, current position
    _node = _HOME;
    _prevNode = _node;
    _lastNodeReached = _node;
    _finalNode = _adjacencyMatrix.GetVertices()-1;   // Final node will be num nodes - 1: index start at 0
    _returnStep = 0;
    
    // Initialise all states to false
    _exitingNode    = false;
    _returning      = false;
    _returned       = false;
    _emptying       = false;
    _finished       = false;

    // Robot begins picking fruit as first state
    _movementState = PICKING;
    _movementDirection = 0;
}


/*--------------------------------------------------------------------------------------------------------
Destructor to shut down elderberry once returned and having traversed the whole orchard.
--------------------------------------------------------------------------------------------------------*/
CMovementControl::~CMovementControl()
{
    ros::shutdown();
}


/*--------------------------------------------------------------------------------------------------------
Warm up the lidar to avoid reading in all 0s at the start of program.
--------------------------------------------------------------------------------------------------------*/
void CMovementControl::WarmUp(ros::Rate loopRate)
{
    for (int warmUp = 0; warmUp < _WARM_UP_MAX; warmUp++) {
        ros::spinOnce();
        loopRate.sleep();
    }
}


/*--------------------------------------------------------------------------------------------------------
Main navigation function that determines the movement-based actions that the robot executes depending on 
a number of possible states.
--------------------------------------------------------------------------------------------------------*/
bool CMovementControl::StartPicking()
{
    // Case for the orchard having been traversed and elderberry returned home boolean false if finished
    bool keepPicking = true;
    if (_finished)
    {
        // GUI display to indicate finished
        _gui.Finished();
        
        // End the program and delete the movement control class
        keepPicking = false;
    }
    // Case for elderberry emptying the fruit basket
    else if (_emptying)
    {
        // Update the GUI to indicate that emptying is taking place
        _gui.Update(_fruitPicker.GetNumFruits(), _node, _node, UNLOADING);

        // Drive the empty basket pattern (stopping) then update the fruit basket as empty
        _driver.EmptyBasket();
        _fruitPicker.EmptyFruits();
        
        // If we were returning because we are finished, stop the program and set the finished state to true
        if (_movementState == FINISHED)
        {
            _finished = true;
        }
        // Otherwise, return to picking
        else
        {
            _movementState = RETURN_TO_PICKING;
            ContinuePicking();
        }
        
        _emptying = false;
    }
    // Case for exiting a node, do not pick while exiting a node or use the PD controller
    else if (_exitingNode)
    {
        /* Run function to allow robot to exit node uninterrupted, once the node is exited change the state
        to no longer exiting */
        _driver.ExitNode();
        _exitingNode = false;

        // Pick from a new row
        _fruitPicker.NewRow();
    }
    // Case for elderberry at a node, a decision on which node to head to next needs to be decided
    else if (_driver.AtNode())
    {
       // Enter the node undisturbed
        _driver.EnterNode();

        // Reset the available fruits to pick in the row (one of each colour)
        _fruitPicker.ResetFruitsToPick();

        // Determine the node exit based on Dijkstra's or the default traverse pattern (incrementing node)
        DetermineNodeExit();

        // Update the GUI
        _gui.Update(_fruitPicker.GetNumFruits(), _prevNode, _node, _movementState);

        // Update the text file to indicate fruits picked and nodes travelled to and from
        _text.PrintPath(_prevNode, _node);
    }
    // Default case, driving straight between nodes looking for fruits to pick
    else
    {
        // Continually scan for fruit while driving forward using the PD controller to drive straight
        _driver.DriveForward();
        
        // If elderberry is picking scan for fruits
        if (_movementState == PICKING)
        {
            // Update the GUI each iteration if a fruit is picked
            _gui.Update(_fruitPicker.GetNumFruits(), _prevNode, _node, PICKING);
            

            // If a fruit is found pick the fruit otherwise keep driving forward
            if(_fruitPicker.FindFruit())
            {
                // Pick the fruit and add this fruit to the fruit basket (indicated by stopping)
                _driver.PickFruit();

                // Update the text file to store a fruit picked and where it was picked
                _text.PrintPick();

                // Check if the fruit basket is full
                if (_fruitPicker.CheckFull())
                {
                    /* If the basket is full change the state to full, so that elderberry knows to return 
                    home */
                    _movementState = FULL;

                    // Update the GUI if the fruit basket is full
                    _gui.Update(_fruitPicker.GetNumFruits(), _prevNode, _node, FULL);
                } 
            }
        }
    }
    return keepPicking;
}

/*--------------------------------------------------------------------------------------------------------
Determines how the robot should exit a node depending on its current movement state and position.
--------------------------------------------------------------------------------------------------------*/
void CMovementControl::DetermineNodeExit()
{
    // Call node finding functions based on current movement state
    switch (_movementState)
    {  
        // If picking normally default case drive to the next node (increment the current node by one)
        case PICKING:
            NextNode();
            break;

        /* If full find the shortest route home using Dijkstra's algorithm setting the shortest route as 
        the next node */
        case FULL:
            ReturnHome();
            break;

        /* If returning to picking from home use Dijkstra's to find the shortest route to the node 
        elderberry left off from */
        case RETURN_TO_PICKING:
            ContinuePicking();
            break;

        // Once finished find the shortest route to return home
        case FINISHED:
            ReturnHome();
            break;
    }
}


/*--------------------------------------------------------------------------------------------------------
Used when the robot is in its default picking state; sends it to the next node in sequence.
--------------------------------------------------------------------------------------------------------*/
void CMovementControl::NextNode()
{
    // Determine the next node to go to (increment the current node by one)
    _prevNode = _node;
    if (_node < _finalNode)
    {
        _node = _node + 1;
    }
    // If at final node, return home and exit function
    else 
    {
        _movementState = FINISHED;
        return;
    }

    // Turn the required direction to reach the next node
    Turn();
}


/*--------------------------------------------------------------------------------------------------------
Used when the robot is full; records last node reached during picking, calls on Dijkstra to determine 
shortest path home and then traverses return path until home.
--------------------------------------------------------------------------------------------------------*/
void CMovementControl::ReturnHome()
{
    // If not returning yet decide the shortest route to return using Dijkstra's algorithm
    if (!_returning)
    {
        // Record last node reached
        _lastNodeReached = _prevNode;

        // Call Dijkstra's algorithm
        _returnPath = _dijkstra.GetPath(_node, _HOME);

        _returning = true;
        _returnStep = 0;
    }
    
    // Execute the driver commands to return home
    TraverseReturnPath(); 

    // Once returned empty the fruit basket, stop exiting the node and set _returned to false
    if (_returned)
    {
        _emptying = true;
        _returned = false;
        _exitingNode = false;
    }

}


/*--------------------------------------------------------------------------------------------------------
Used when robot has emptied its fruit; calls on Dijkstra to determine shortest path back to last node 
reached and then traverses return path until at node.
--------------------------------------------------------------------------------------------------------*/
void CMovementControl::ContinuePicking()
{
    // If not returning yet decide the shortest route to return using Dijkstra's algorithm
    if (!_returning)
    {
        // Call Dijkstra's algorithm
        _returnPath = _dijkstra.GetPath(_node, _lastNodeReached);

        _returning = true;
        _returnStep = 0;

        // Update the GUI each iteration to indicate which node travelling to
        _gui.Update(_fruitPicker.GetNumFruits(), _prevNode, _node, PICKING);
    }
    
    // Execute the commands to return to the spot elderberry left off picking
    TraverseReturnPath(); 

    // Once returned empty the fruit basket, stop exiting the node and set _returned to false
    if (_returned)
    {
        _movementState = PICKING;
        _returned = false;
        _exitingNode = false;
    }
}


/*--------------------------------------------------------------------------------------------------------
Traverses return path found by Dijkstra's algorithm until complete.
--------------------------------------------------------------------------------------------------------*/
void CMovementControl::TraverseReturnPath()
{
    // Find the final step
    int finalStep = _returnPath.size();
    
    // Go to next node on return path
    _returnStep = _returnStep + 1;

    // Cycle through nodes until we are at end of path
    _prevNode = _node;
    _node = _returnPath[_returnStep];

    // If elderberry has reached the final node set this as the current and previous node
    if (_returnStep == finalStep)
    {
    (_node = _prevNode);
    }

    // Turn the requisite direction
    Turn();

    // If we are at final node, indicated we have returned and exit function
    if (_returnStep == finalStep) 
    {
        _returning = false;
        _returned = true;
        _returnPath.clear();
    }
}


/*--------------------------------------------------------------------------------------------------------
Sends turning command to drive based node graph direction.
--------------------------------------------------------------------------------------------------------*/
void CMovementControl::Turn()
{
    // Find drive direction from node graph    
    int turnDirection = _adjacencyMatrix.GetDirection(_prevNode, _node);

    /* Determine relative direction to move based on current direction we are moving in absolute
    coordinate system (_movementDirection) and direction we want to go in absolute coordinate system
    (turnDirection). 
    
    Example: if we are going South (180 deg) and need to go West (270 deg), then the relative direction
    we need to turn is RIGHT (270 - 180 = 90 deg).
    Example 2: if we are going South (180 deg) and need to go East (90 deg), then the relative direction
    we need to turn is LEFT (360 + (90 - 180) = 270 deg). */
    int relativeDirection = _movementDirection > turnDirection ? _DIR_ADJ + (turnDirection - _movementDirection) : turnDirection - _movementDirection;
    
    // Update the movement direction to the new direction we are going in
    _movementDirection = turnDirection;
    
    // Send command to drive based on the requisite turn direction
    switch (relativeDirection)
    {
        case CAdjacencyMatrix::LEFT:
            _driver.TurnLeft();
            break;
        
        case CAdjacencyMatrix::RIGHT:
            _driver.TurnRight();
            break;

        case CAdjacencyMatrix::BACKWARDS:
            _driver.TurnAround();
            break;

        // No state for forwards since we don't need to turn, this is the default
        default:
            break;
    }

    _exitingNode = true;
}

/*--------------------------------------------------------------------------------------------------------
Sets the current movement direction, previous node and current node. Used for unit testing of Turn().
--------------------------------------------------------------------------------------------------------*/
void CMovementControl::TESTING_TurnSetUp(int movementDirection, int prevNode, int node)
{
    _movementDirection = movementDirection;
    _prevNode = prevNode;
    _node = node;
}

/*--------------------------------------------------------------------------------------------------------
Returns the current movement direction. Used for unit testing of Turn().
--------------------------------------------------------------------------------------------------------*/
int CMovementControl::TESTING_TurnCheck()
{
    return _movementDirection;
}