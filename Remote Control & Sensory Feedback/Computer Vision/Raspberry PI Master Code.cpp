// Include files for required libraries
#include <stdio.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv_aee.hpp"
//#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

// Configure the I2C interface to the Car as a global variable
//Pi2c car(0x22);

void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV
}


#include <opencv2/opencv.hpp>

void determineContours(cv::Mat originalImage, std::vector<std::vector<cv::Point>  >* approxedContours) {
    cv::Mat contourImage = originalImage.clone(); // Clone to avoid modifying the original
    cv::cvtColor(contourImage, contourImage, cv::COLOR_GRAY2BGR); // Convert to BGR for color drawing

    std::vector<std::vector<cv::Point>  > contours;
    std::vector<cv::Vec4i> hierarchy;

    // Find contours
    cv::findContours(originalImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Resize approxedContours to match the number of contours
    approxedContours->resize(contours.size());

    for (size_t i = 0; i < contours.size(); i++) {
        // Approximate contours
        cv::approxPolyDP(contours[i], (*approxedContours)[i], 10, true);

        // Draw contours
        cv::drawContours(contourImage, *approxedContours, i, cv::Scalar(0, 255, 0), 2); // Green contours
    }

    // Show the contours in a window
     //cv::imshow("Contours", contourImage);
    //cv::waitKey(0); // Wait for a key press
}







    // Display contour information
    //transformPerspective(approxedContours,originalImage, 320, 240 );





int main(int argc, char** argv)
{
   float percentageF = 0 , percentageR = 0, percentageL = 0;
    std::vector<std::vector<cv::Point>  > boundary;
    setup();    // Call a setup function to prepare IO and devices
    int lowH = 0, highH = 179, lowS = 0, highS = 255, lowV = 0, highV = 255;
    int16_t forwardSignal = 0, leftSigal = 0, rightSign = 0; //Initialise the vehicle action signals to be sent to the ESP32 MAinboard
    //(0 correspond to a non-execution of actions and 1 to their execution
    cv::namedWindow("Photo");   // Create a GUI window called "Photo"
    // Create matrices for the frame and HSV image

    while (1) // Main loop to perform image processing
    {
        cv::Mat frame, HSVImage, imageGREY, transformedFrame;
        cv::Mat convertedForward, convertedRight, convertedLeft;
        cv::Mat hsvForward, hsvLeft, hsvRight;
        cv::Mat forwardSign = cv::imread("forwards.png");
        cv::Mat rightSign = cv::imread("right.png");
        cv::Mat leftSign = cv::imread("left.png");


        while (frame.empty()) {
            frame = captureFrame();


        }
               imshow("Photo", frame);

        // Convert frame from BGR to HSV
        cv::cvtColor(frame, HSVImage ,COLOR_BGR2HSV);
        inRange(HSVImage, Scalar(0, 0,0), Scalar(95,255, 255), imageGREY);

        cv::cvtColor(forwardSign, hsvForward, cv::COLOR_BGR2HSV);

        cv::cvtColor(leftSign, hsvLeft, cv::COLOR_BGR2HSV);
       // imshow("Photo", hsvLeft);
        cv::cvtColor(rightSign, hsvRight, cv::COLOR_BGR2HSV);
        //inRange(frame, Scalar(95, 0, 0), Scalar(179,255, 255), imageGREY);

        inRange(hsvForward, Scalar(0, 0, 0), Scalar(95,255, 255), convertedForward);
        //cv::resize(convertedForward,convertedForward, cv::Size(320, 240));
      //  imshow("Photo", convertedForward);

        inRange(hsvLeft, Scalar(0, 0, 0), Scalar(95,255, 255), convertedLeft);
        //cv::resize(convertedLeft,convertedLeft, cv::Size(320, 240));

        inRange(hsvRight, Scalar(0, 0, 0), Scalar(95,255, 255), convertedRight);
          //cv::resize(convertedRight, convertedRight, cv::Size(320, 240));

        //cv::cvtColor(frame, imageGREY, cv::COLOR_BGR2GRAY);
        int lowH = 0, highH = 179, lowS = 0, highS = 255, lowV = 0, highV = 255;

        // Display the images in the window
         //cv::imshow("Photo", frame);

        //inRange(HSVImage, Scalar(0, 179, 0), Scalar(239, 51, 255), imageGREY);
        //`cv::imshow("HSV Image", HSVImage);
        //cv:imwrite("instantFrame.bmp", HSVImage);

         determineContours(imageGREY, &boundary);

         //find largest area contour array number

    double area = 0;
    double maxArea = 0.0;
    int maxAreaCountour = 0;

    for (int i = 0; i < boundary.size(); i++) {
        area = cv::contourArea(boundary[i]); // Get area of current contour
        if (area > maxArea) {
            maxArea = area; // Update maxArea if the current one is larger
            maxAreaCountour = i;
         if(boundary[i].size() == 4 ){
         transformedFrame = transformPerspective(boundary[maxAreaCountour], imageGREY, 350, 350);

          //resize(transformedFrame, transformedFrame, cv::Size(320,240));
//          imshow("")
           //printf("Biggest Index: %d\n", maxAreaCountour);
         }


//         printf("TF %d\n CF %d\n TF %d\n CF %d\n", transformedFrame.size().width, convertedForward.size().width, transformedFrame.size().height, convertedForward.size().height);
         //printf("Columns Values:%d\n%d\n", transformedFrame.cols, convertedRight.cols);
         //printf("Columns Row:%d\n%d\n", transformedFrame.rows, convertedRight.rows);
         //cvtColor(forwardSign,convertedForward, COLOR_BGR2GRAY);
         //cvtColor(rightSign,convertedLeft, COLOR_BGR2GRAY);
         //cvtColor(leftSign, convertedRight, COLOR_BGR2GRAY);

         percentageF = compareImages(transformedFrame, convertedForward);
         percentageL = compareImages(transformedFrame, convertedLeft);
         percentageR = compareImages(transformedFrame, convertedRight);
         //printf("F: %f, L: %f, R: %f\n", percentageF, percentageL, percentageR);

             
             

`if(percentageF>=10 && percentageL<=10 && percentageR<=10){
//printf("\No Sign Detected\nCommand Data:%d");
cmdData = 0;
}`

`if(percentageF>=35&& (percentageF>(percentageR&&percentageL))){
printf("\nForward Sign Detected\nCommandData:%d");
cmdData = 1;
}
if(percentageR>=35  && (percentageR>(percentageL&&percentageF))){
//printf("\Right Sign Detected\nCommandData:%d");
cmdData = 2;
}
if(percentageL>=32 && (percentageL>(percentageR&&percentageF))){
//printf("\nLeft Sign DetectednCommandData:%d");
cmdData = 3;
}
//printf("\nCommand Data:%d", cmdData);
car.i2cWriteArduinoInt(cmdData);`
        /** \brief
         *
         * \param %f "F:
         * \param %f L:
         * \param %f\n" R:
         * \param percentageF
         * \param percentageL
         * \param percentageR
         *
         */




        }

    }





         //}
         //simplify contours approxpolydb
         //if contour[maxcontourindex].size() == 4 then transform perspective
         //imwrite("NewFrame.png", transformedFrame);
         //compareImages(imageGREY,convertedForward);
         //compareImages(imageGREY,convertedLeft);
         //compareImages(imageGREY,convertedRight)
        //printf("Hello");


        // Wait 1ms for a key press (required to update windows)
        int key = cv::waitKey(1);
        key = (key == 255) ? -1 : key;

        // Break the loop if the ESC key (27) is pressed
        if (key == 27) {
            break;
        }
    }

//    closeCV(); // Disable the camera and close any windows

    return 0;
}
