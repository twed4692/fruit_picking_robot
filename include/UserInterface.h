/*--------------------------------------------------------------------------------------------------------
Class to display GUI. This will contain all of the functions to update the GUI based on 
the number of nodes, apples in the basket and state of Elderberry. This will also contain all
of the base images required to display the GUI.

Initial revision: 500456925
--------------------------------------------------------------------------------------------------------*/

#ifndef USERINTEFACE_H
#define USERINTEFACE_H

#include "ros/package.h"                    // Used to determine path to images
#include <opencv2/core/core.hpp>            // Needed to display images using openCV
#include <opencv2/highgui/highgui.hpp>      // Needed to display images using openCV

class CUserInterface{

    public:
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        CUserInterface();                                           // Constructor
        ~CUserInterface();                                          // Destructor
        // Single externally callable function to update all aspects of the GUI
        void Update(int numFruit, int currentNode, int nextNode, int state);
        // Turtlebot has finished picking, display the finished emoji image
        void Finished();

    private:
        /*------------------------------------------------------------------------------------------------
        Internal functions shown on the GUI
        ------------------------------------------------------------------------------------------------*/
        void AddApples(int numFruit);                               // Show pictures of number of apples
        void ChangeCapacity(int numFruit);                          // Display fullness of the basket x/6
        void UpdateNodes(int currentNode, int nextNode);            // Update the current and next node
        void UpdateState(int state);                                // Update the state of Elderberry
        void ShowGUI();                                             // Show the GUI to the user
        void ResetIm();                                             // Reset the image when state changes
                                                                    // or apples are picked

        /*------------------------------------------------------------------------------------------------
        BGR matrices of image data, all of the images required to display all states of the GUI
        ------------------------------------------------------------------------------------------------*/
        cv::Mat3b _base;                                            // Base image
        cv::Mat3b _baseDefault;                                     // Default base 
        cv::Mat3b _apples[6];                                       // Apple image
        cv::Mat3b _fullness[7];                                     // How much apple basket is full
        cv::Mat3b _numbers[10];                                     // Numbers 0-9 to print
        cv::Mat3b _states[5];                                       // Different movement states
        cv::Mat3b _finishedBase;                                    // Image to show when finished

        /*------------------------------------------------------------------------------------------------
        Constants
        ------------------------------------------------------------------------------------------------*/
        inline static const std::string _GUI = "User Interface";    // GUI name
        const int _FRAME_SHOW_TIME = 5;                             // Time in milliseconds to show a frame
        const int _WAIT_TIME = 5000;                                // Time to show image for
        const int _MAX_FRUIT_ONE_LINE = 3;                          // Max fruit displayed on a line
        const int _X_FRUIT = 750;                                   // X coord for fruit disp
        const int _Y_FRUIT_ONE_LINE = 630;                          // Y coord for fruit on one line
        const int _Y_FRUIT_OTHER = 590;                             // Y coord for fruit otherwise
        const int _X_CAP = 610;                                     // X coord for capacity disp
        const int _Y_CAP = 660;                                     // Y coord for capacity disp
        const int _X_CUR = 610;                                     // X coord for current node disp
        const int _Y_CUR = 360;                                     // Y coord for current node disp
        const int _X_NEXT = 610;                                    // X coord for next node disp
        const int _Y_NEXT = 460;                                    // Y coord for next node disp
        const int _X_STATE_RETURN = 500;                            // X coord for return state disp
        const int _Y_STATE_RETURN = 150;                            // Y coord for return state disp
        const int _X_STATE_OTHER = 450;                             // X coord for other state disp
        const int _Y_STATE_OTHER = 200;                             // Y coord for other state disp
};

#endif