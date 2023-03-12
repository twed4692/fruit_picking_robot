/*--------------------------------------------------------------------------------------------------------
C++ implementation of TextOutput class. Writes information about the path being travelled and picking 
fruit to a text file called fruitLocations.txt, from which the location of fruit within the orchard can be
discerned. Clears any contents in the text file prior to running.

Initial revision: 500456925
--------------------------------------------------------------------------------------------------------*/

#include "TextOutput.h"

/*--------------------------------------------------------------------------------------------------------
Constructor, sets a pointer to Odom Subscriber
--------------------------------------------------------------------------------------------------------*/
CTextOutput::CTextOutput(COdomSub* odomSubPtr)
{
    _odomSubPtr = odomSubPtr;
    std::string filePath = ros::package::getPath("orchard_mapping") + "/textOutput";
    locations.open(filePath + "/fruitLocations.txt");
    locations << "\nWriting fruit locations from test starting at " << std::asctime(std::localtime(&tt)) << "\n";
    _odometer = _odomSubPtr->GetPos(COdomSub::CURRENT);

}


/*--------------------------------------------------------------------------------------------------------
Destructor.
--------------------------------------------------------------------------------------------------------*/
CTextOutput::~CTextOutput()
{
    locations.close();
}


/*--------------------------------------------------------------------------------------------------------
Print the current path we are travelling to the text file.
--------------------------------------------------------------------------------------------------------*/
void CTextOutput::PrintPath(int start, int end)
{
    UpdateOdom();
    if (start != end)
    {
        locations << "Travelling from node " << start << " to node " << end << "[" << _odometer.first << ", " << _odometer.second << "]\n";
    }
    
}


/*--------------------------------------------------------------------------------------------------------
Print that we are picking a fruit to the text file.
--------------------------------------------------------------------------------------------------------*/
void CTextOutput::PrintPick()
{
    UpdateOdom();
    locations << "Fruit was picked!" << "[" << _odometer.first << ", " << _odometer.second << "]\n";
}


/*--------------------------------------------------------------------------------------------------------
Get our current odom position.
--------------------------------------------------------------------------------------------------------*/
void CTextOutput::UpdateOdom()
{
    _odometer = _odomSubPtr->GetPos(COdomSub::CURRENT);
}