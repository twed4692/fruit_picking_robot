/*--------------------------------------------------------------------------------------------------------
C++ implementation of CameraSub class. A subscriber to the compressed image topic published by the 
raspicam_node.

- Converts the image to openCV bgr8 format and stores it (accessed by a pointer).
- Uses openCV functions to look for one of the colours in the enum, passed as a parameter.

Original revision: 510548722
--------------------------------------------------------------------------------------------------------*/

#include "CameraSub.h"

/*--------------------------------------------------------------------------------------------------------
Constructor
--------------------------------------------------------------------------------------------------------*/
CCameraSub::CCameraSub()
{
    // Initialise the subscriber to the compressed image
    // Queue length of 1 as each message is quite large
    _compressedImageSub = _nh.subscribe("raspicam_node/image/compressed", 1, &CCameraSub::CameraMsgCallBack, this);

    // Name the openCV windows using the static constant strings
    cv::namedWindow(_ORIGINAL_IMG);
    cv::namedWindow(_THRESH_IMG);
}


/*--------------------------------------------------------------------------------------------------------
Destructor to close the openCV windows
--------------------------------------------------------------------------------------------------------*/
CCameraSub::~CCameraSub()
{
    cv::destroyWindow(_ORIGINAL_IMG);
    cv::destroyWindow(_THRESH_IMG);
}


/*--------------------------------------------------------------------------------------------------------
Camera call back to subscribe to the image and convert to openCV format (bgr8). Image is copied so it can 
be freely edited without affecting the publishing node.
--------------------------------------------------------------------------------------------------------*/
void CCameraSub::CameraMsgCallBack(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
    try
    {
        // Copy the image to the member variable pointer
        _bgrImgPointer = cv_bridge::toCvCopy(msg, "bgr8");

        // Show the original camera feed in a window
        cv::imshow(_ORIGINAL_IMG, _bgrImgPointer->image);
        cv::waitKey(_FRAME_SHOW_TIME);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


/*--------------------------------------------------------------------------------------------------------
Checks the colour passed into the function. Returns true if enough pixels in the image are that colour.
--------------------------------------------------------------------------------------------------------*/
bool CCameraSub::CheckForColour(eColour hueVal)
{
    bool thereIsColour = false;

    cv::Mat3b bgrImg = _bgrImgPointer->image;
    if (hueVal == RED)
    {
        // Invert colours of original image
        // Since red wraps around 0/360 hue on hsv this is simpler
        // than two inRange statements
        bgrImg = ~bgrImg;
    }
    
    // Convert bgr to hsv
    cv::Mat3b hsvImg; // 3b for hsv (colour)
    cv::cvtColor(bgrImg, hsvImg, cv::COLOR_BGR2HSV);

    // Threshold the image for the hue
    cv::Mat1b thresholdImg; // 1b for threshold (black and white)
    cv::inRange(hsvImg, cv::Scalar(hueVal - _HUE_TOLERANCE, _SATURATION_MIN, _VALUE_MIN), 
        cv::Scalar(hueVal + _HUE_TOLERANCE, _SAT_VAL_MAX, _SAT_VAL_MAX), thresholdImg);

    // Calculate percentage of the image that is coloured
    int numColoured = cv::countNonZero(thresholdImg);
    double totalPixels = thresholdImg.rows * thresholdImg.cols;
    double percentColoured = (double) numColoured / totalPixels;

    if (percentColoured > _AREA_DETECTION)
    {
        thereIsColour = true;
    }

    // Show the thresholded image during
    cv::imshow(_THRESH_IMG, thresholdImg);
    cv::waitKey(_FRAME_SHOW_TIME);

    return thereIsColour;
}