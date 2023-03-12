/*--------------------------------------------------------------------------------------------------------
A text outputter that writes information about the path being travelled and picking fruit to a text file 
called fruitLocations.txt. Knows an odom subscriber and is owned by a movement controller.

Initial revision: 500456925
--------------------------------------------------------------------------------------------------------*/

#ifndef TEXTOUTPUT_H
#define TEXTOUTPUT_H

#include "OdomSub.h"        
#include "ros/package.h"    // Used to get a path to the text output file
#include <fstream>          // Printing to text
#include <ctime>            // Used for time_t datatype


class CTextOutput{

    public:
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        CTextOutput(COdomSub* odomSubPtr);          // Constructor, initialise with a known odom sub   
        ~CTextOutput();                             // Destructor   
        void PrintPath(int start, int end);         // Print the path that we are currently taking
        void PrintPick();                           // Print that we have picked fruit
        void UpdateOdom();                          // Update the current odom value

    private:
        /*------------------------------------------------------------------------------------------------
        Internal variables
        ------------------------------------------------------------------------------------------------*/
        std::ofstream locations;                    // Locations visited, text output
        std::time_t tt;                             // Times at which events occurred

        /*------------------------------------------------------------------------------------------------
        Known objects and related variables
        ------------------------------------------------------------------------------------------------*/
        COdomSub* _odomSubPtr;                      // Knows an odom subscriber
        COdomSub::coordinates _odometer;            // Odometry coords
};

#endif // TEXTOUTPUT_H
