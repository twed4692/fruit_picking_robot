/*--------------------------------------------------------------------------------------------------------
A movement controller that determines the movement of a fruit picking robot based on its current state.

If the robot is in its default picking state, it will move sequentially through a series of known nodes.
which represent the edges of an orchard, continually scanning for fruit as it moves through the rows.
Once fruit is detected, it performs a sequence to represent that fruit it being picked. 

Once the robot is full, it will call on Dijkstra to find the shortest path home. After traversing this
path, the robot will perform a sequence to represent the fruit being emptied. It will then call on
Dijkstra again to find the shortest path to return to the last node it reached during picking and
continue picking again.

Once the robot reaches the final node, it will call on Dijkstra to find the shortest path home, empty 
the remaining fruit, and indicate to the user that the task is complete.

Initial revision: 500457324
--------------------------------------------------------------------------------------------------------*/

#ifndef MOVEMENT_CONTROL_H
#define MOVEMENT_CONTROL_H

#include "ros/ros.h"
#include "AdjacencyMatrix.h"
#include "FruitPicker.h"
#include "Driver.h"
#include "LidarSub.h"
#include "DijkstraAlgorithm.h"
#include "UserInterface.h"
#include "TextOutput.h"
#include <vector>               // Needed to store return path which is a vector of nodes (ints)

class CMovementControl
{
    public:
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        CMovementControl();             // Constructor
        ~CMovementControl();            // Destructor
        bool StartPicking();            // Overarching navigation logic
        void WarmUp(ros::Rate loopRate);// Run a few cycles to get better lidar values (first few spins 
                                        // return all 0)
        void Turn();                    // Turns as needed - made public for testing

        /*------------------------------------------------------------------------------------------------
        Movement states
        ------------------------------------------------------------------------------------------------*/
        enum _eMovementStates           // Different possible states that the robot may be in
        {
            PICKING,
            FULL,
            UNLOADING,
            RETURN_TO_PICKING,
            FINISHED
        };

        /*------------------------------------------------------------------------------------------------
        Testing functions
        ------------------------------------------------------------------------------------------------*/
        void TESTING_TurnSetUp(int movementDirection, int prevNode, int node);
        int TESTING_TurnCheck();
    
    private:
        /*------------------------------------------------------------------------------------------------
        Internal functions
        ------------------------------------------------------------------------------------------------*/
        void DetermineNodeExit();       // Determines the direction to move in once at a node
        void NextNode();                // When picking (default), directs robot to next node in graph
        void ReturnHome();              // Once cache is full, sends robot home to empty
        void ContinuePicking();         // Once cache is emptied, sends robot back to picking
        void TraverseReturnPath();      // Navigates return path either to home or last reached node
        
        /*------------------------------------------------------------------------------------------------
        Node related variables
        ------------------------------------------------------------------------------------------------*/
        int _node;                      // Node that robot is currently heading to
        int _prevNode;                  // Node that robot is moving from
        int _lastNodeReached;           // Node that robot last reached before
        int _finalNode;                 // Total number of nodes
        bool _exitingNode;              // Flag to indicate robot is exiting node

        /*------------------------------------------------------------------------------------------------
        Return and empty related variables
        ------------------------------------------------------------------------------------------------*/
        bool _returning;                // Flag to indicate that robot is returning home or to picking
        bool _returned;                 // Flag to indicate that robot has finished returning
        std::vector<int> _returnPath;   // Return path for robot as determine by Dijkstra's algorithm
        int _returnStep;                // Number of nodes passed whilst returning
        bool _emptying;                 // Flag to indicate that robot is emptying fruit
        bool _finished;                 // Flag to indicate that robot is finished picking

        /*------------------------------------------------------------------------------------------------
        State related variables
        ------------------------------------------------------------------------------------------------*/
        int _movementDirection;         // Current direction that robot is moving in
        int _movementState;             // Robot movement state; picking, full, emptied

        /*------------------------------------------------------------------------------------------------
        Owned objects
        ------------------------------------------------------------------------------------------------*/
        // Has a driver - used to send movement commands
        CDriver _driver;   
        // Has a node graph - used to find next node in sequence
        CAdjacencyMatrix _adjacencyMatrix;         
        // Has a fruit picker - used for picking, determining if full
        CFruitPicker _fruitPicker;     
        // Has a dijkstra - used to find shortest path between nodes
        CDijkstraAlgorithm _dijkstra;   
        // Has a GUI class
        CUserInterface _gui;      
        // Has a text output class
        CTextOutput _text;

        /*------------------------------------------------------------------------------------------------
        Static variables
        ------------------------------------------------------------------------------------------------*/
        static constexpr int _WARM_UP_MAX = 100;    // Warm up max
        static constexpr int _HOME = 0;             // Home node
        static constexpr int _DIR_ADJ = 360;        // Direction adjustment for Turn() function
};

#endif