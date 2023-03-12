/*--------------------------------------------------------------------------------------------------------
Class to display GUI. Add all the images in the constructor required to display to the GUI.
This class contains all of the functions to update the GUI.

Initial revision: 500456925
--------------------------------------------------------------------------------------------------------*/

#include "UserInterface.h"
#include "MovementControl.h"                // For movement state enums

/*--------------------------------------------------------------------------------------------------------
Constructor to set initialise all of the required images and store them in vectors.
--------------------------------------------------------------------------------------------------------*/
CUserInterface::CUserInterface()
{
    // Get the absolute path to the images
    std::string imgFolderPath = ros::package::getPath("orchard_mapping") + "/img";

    // Initialise all the image matrices
    // Initialise the base image which other images are laid on top of
    _baseDefault=cv::imread(imgFolderPath + "/base.jpg",1);

    // Initialise all of the apple images (displaying the number of apples)
    _apples[0]=cv::imread(imgFolderPath + "/1_apple.jpg",1);
    _apples[1]=cv::imread(imgFolderPath + "/2_apple.jpg",1);
    _apples[2]=cv::imread(imgFolderPath + "/3_apple.jpg",1);
    _apples[3]=cv::imread(imgFolderPath + "/4_apple.jpg",1);
    _apples[4]=cv::imread(imgFolderPath + "/5_apple.jpg",1);
    _apples[5]=cv::imread(imgFolderPath + "/6_apple.jpg",1);

    // Store the number of apples images (x/6)
    _fullness[0]=cv::imread(imgFolderPath + "/0_6.jpg",1);
    _fullness[1]=cv::imread(imgFolderPath + "/1_6.jpg",1);
    _fullness[2]=cv::imread(imgFolderPath + "/2_6.jpg",1);
    _fullness[3]=cv::imread(imgFolderPath + "/3_6.jpg",1);
    _fullness[4]=cv::imread(imgFolderPath + "/4_6.jpg",1);
    _fullness[5]=cv::imread(imgFolderPath + "/5_6.jpg",1);
    _fullness[6]=cv::imread(imgFolderPath + "/6_6.jpg",1);

    // Images to display the current and next node
    _numbers[0]=cv::imread(imgFolderPath + "/0 (Home).jpg",1);
    _numbers[1]=cv::imread(imgFolderPath + "/1.jpg",1);
    _numbers[2]=cv::imread(imgFolderPath + "/2.jpg",1);
    _numbers[3]=cv::imread(imgFolderPath + "/3.jpg",1);
    _numbers[4]=cv::imread(imgFolderPath + "/4.jpg",1);
    _numbers[5]=cv::imread(imgFolderPath + "/5.jpg",1);
    _numbers[6]=cv::imread(imgFolderPath + "/6.jpg",1);
    _numbers[7]=cv::imread(imgFolderPath + "/7.jpg",1);
    _numbers[8]=cv::imread(imgFolderPath + "/8.jpg",1);
    _numbers[9]=cv::imread(imgFolderPath + "/9.jpg",1);

    // Images to display the current state of elderberry
    _states[0]=cv::imread(imgFolderPath + "/Picking Fruit.jpg",1);
    _states[1]=cv::imread(imgFolderPath + "/Full.jpg",1);
    _states[2]=cv::imread(imgFolderPath + "/Unloading.jpg",1);
    _states[3]=cv::imread(imgFolderPath + "/Returning to last point.jpg",1);
    _states[4]=cv::imread(imgFolderPath + "/Returning to last point.jpg",1);

    // Image to display to user when the whole orchard is traversed
    _finishedBase = cv::imread(imgFolderPath + "/finished.jpg",1);

}


/*--------------------------------------------------------------------------------------------------------
Function to close the openCV window once the orchard is traversed.
--------------------------------------------------------------------------------------------------------*/
CUserInterface::~CUserInterface()
{
    cv::destroyWindow(_GUI);
}


/*--------------------------------------------------------------------------------------------------------
Public function to be called by movement control which will update all aspects of the GUI
--------------------------------------------------------------------------------------------------------*/
void CUserInterface::Update(int numFruit, int currentNode, int nextNode, int state)
{
    ResetIm();                          // Set the base back to default
    AddApples(numFruit);                // Add the images of apples
    ChangeCapacity(numFruit);           // Update the capacity text
    UpdateNodes(currentNode,nextNode);  // Update the current and previous node text
    UpdateState(state);                 // Update the current state (picking, full, returning, etc.)
    ShowGUI();                          // Display the openCV window
}


/*--------------------------------------------------------------------------------------------------------
Function to add the apples to be displayed to the user.
--------------------------------------------------------------------------------------------------------*/
void CUserInterface::AddApples(int numFruit)
{
    /* Determine the position to display the apples based on the number of apples. This is to 
    ensure that they are displayed in line and do not overlap with other images. */
    int xCoord, yCoord;
    if (numFruit <= _MAX_FRUIT_ONE_LINE)
    {
        xCoord = _X_FRUIT;
        yCoord = _Y_FRUIT_ONE_LINE;
    }
    else
    {
        xCoord = _X_FRUIT;        
        yCoord = _Y_FRUIT_OTHER;
    }

    if (numFruit != 0)
    {
        // Copy the appropriate apples image to the base image using the rectangle of interest defined by the coordinates
        _apples[numFruit-1].copyTo(_base(cv::Rect(xCoord,yCoord,_apples[numFruit-1].cols, _apples[numFruit-1].rows)));
    }

}


/*--------------------------------------------------------------------------------------------------------
Function to display the number of apples in the basket (x/6)
--------------------------------------------------------------------------------------------------------*/
void CUserInterface::ChangeCapacity(int numFruit)
{
    // Copy the appropriate capacity image to the base image using the rectangle of interest defined by the coordinates
    _fullness[numFruit].copyTo(_base(cv::Rect(_X_CAP,_Y_CAP,_fullness[numFruit].cols, _fullness[numFruit].rows)));

}


/*--------------------------------------------------------------------------------------------------------
Function to reset the base image.
--------------------------------------------------------------------------------------------------------*/
void CUserInterface::ResetIm()
{
    // Set base back to default i.e. with no apples or capacity text etc.
    _base = _baseDefault.clone();
}


/*--------------------------------------------------------------------------------------------------------
Update the current node elderberry is at and the next node that will be travelled to.
--------------------------------------------------------------------------------------------------------*/
void CUserInterface::UpdateNodes(int currentNode, int nextNode)
{
    // Copy the appropriate node images to the base image using the rectangles of interest defined by the coordinates
    _numbers[currentNode].copyTo(_base(cv::Rect(_X_CUR,_Y_CUR,_numbers[currentNode].cols, _numbers[currentNode].rows)));
    _numbers[nextNode].copyTo(_base(cv::Rect(_X_NEXT,_Y_NEXT,_numbers[nextNode].cols, _numbers[nextNode].rows)));
}


/*--------------------------------------------------------------------------------------------------------
Function to update the state of Elderberry (picking, finished, next node etc).
--------------------------------------------------------------------------------------------------------*/
void CUserInterface::UpdateState(int state)
{
    // Determine the position to place the text on the base image based on the state (text size)
    int xCoord, yCoord;

    if (state == CMovementControl::RETURN_TO_PICKING || state == CMovementControl::FINISHED)
    {
        xCoord = _X_STATE_RETURN;
        yCoord = _Y_STATE_RETURN;
    }
    else
    {
        xCoord = _X_STATE_OTHER;
        yCoord = _Y_STATE_OTHER;
    }

    // Copy the appropriate state image to the base image using the rectangle of interest defined by the coordinates
    _states[state].copyTo(_base(cv::Rect(xCoord,yCoord,_states[state].cols, _states[state].rows)));
}


/*--------------------------------------------------------------------------------------------------------
Display the GUI to the user.
--------------------------------------------------------------------------------------------------------*/
void CUserInterface::ShowGUI()
{
    cv::imshow(_GUI, _base);
    cv::waitKey(_FRAME_SHOW_TIME);
}


/*--------------------------------------------------------------------------------------------------------
Image to display when the whole orchard is traversed.
--------------------------------------------------------------------------------------------------------*/
void CUserInterface::Finished()
{
    ResetIm();
    cv::imshow(_GUI, _finishedBase);
    cv::waitKey(_WAIT_TIME);
}