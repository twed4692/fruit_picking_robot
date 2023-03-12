/*--------------------------------------------------------------------------------------------------------
CameraSub has a ROS subscriber to the camera data published by raspicam_node.

It is responsible for:
    - reading in the image using a callback.
    - converting it to openCV format using cv_bridge.
    - using openCV algorithms to convert the image to HSV and detect the required colour.

A single instance of CameraSub exists as a member variable of the FruitPicker class.

Original revision: 510548722
--------------------------------------------------------------------------------------------------------*/

#ifndef CAMERA_SUB_H
#define CAMERA_SUB_H

#include <ros/ros.h>                        // Initialises a ros node handle
#include <cv_bridge/cv_bridge.h>            // Gets the ros image into an openCV format
#include <opencv2/imgproc/imgproc.hpp>      // Needed to display images using openCV
#include <opencv2/highgui/highgui.hpp>      // Needed to display images using openCV

class CCameraSub
{
    public:
        /*------------------------------------------------------------------------------------------------
        Public enum
        ------------------------------------------------------------------------------------------------*/
        enum eColour                            // Different colours (or hues) that can be detected
        {
            YELLOW = 30,
            GREEN = 50,
            RED = 90                            // Hue value of red is actually 0, but an inversion trick 
                                                // is used to simplify detection
        };
        
        /*------------------------------------------------------------------------------------------------
        Public functions
        ------------------------------------------------------------------------------------------------*/
        CCameraSub();                           // Constructor
        ~CCameraSub();                          // Destructor
        bool CheckForColour(eColour hueVal);    // Function to detect a particular colour based on the hue

    private:
        /*------------------------------------------------------------------------------------------------
        Node related variables
        ------------------------------------------------------------------------------------------------*/
        ros::NodeHandle _nh;                    // Ros node handle to allow connection to ROS master
        ros::Subscriber _compressedImageSub;    // Ros subscriber for the compressed image topic

        /*------------------------------------------------------------------------------------------------
        openCV related variables
        ------------------------------------------------------------------------------------------------*/
        cv_bridge::CvImagePtr _bgrImgPointer;   // Pointer to the image data in openCV bgr8 format
        // Constant strings to name openCV windows
        inline static const std::string _ORIGINAL_IMG = "Turtlebot Camera Feed";
        inline static const std::string _THRESH_IMG = "Colour Detection Feed";

        /*------------------------------------------------------------------------------------------------
        Private functions
        ------------------------------------------------------------------------------------------------*/
        // Subscriber call back function
        void CameraMsgCallBack(const sensor_msgs::CompressedImage::ConstPtr &msg);

        /*------------------------------------------------------------------------------------------------
        Constant thresholding parameters
        ------------------------------------------------------------------------------------------------*/
        const int _FRAME_SHOW_TIME = 5;         // Time to show a frame for
        const int _HUE_TOLERANCE = 5;           // Check for a hue value plus or minus the tolerance
        const int _SATURATION_MIN = 70;         // Minimum required saturation of colour
        const int _VALUE_MIN = 60;              // Minimum value (brightness)
        const int _SAT_VAL_MAX = 255;           // Maximum for both saturation and value
        const double _AREA_DETECTION = 0.13;    // Percentage area of image required
};

#endif