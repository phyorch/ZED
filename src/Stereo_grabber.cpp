//
// Created by phyorch on 25/07/18.
//
//Grab image from zed stereo camera
#include <iostream>
#include <string>
#include <sl/Camera.hpp>

#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
//using namespace cv;
using namespace sl;
int main(int argc, char **argv) {

    InitParameters init_params;
    init_params.camera_fps = 30;

    // Create a ZED Camera object
    Camera zed;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        cout << to_string(err) << endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Print help in console
    //printHelp();

    // Print camera information
    printf("ZED Model                 : %s\n", to_string(zed.getCameraInformation().camera_model).c_str());
    printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
    printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
    printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
    printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());

    // Create a Mat to store images
    Mat zed_image;

    // Capture new images until 'q' is pressed
    char key = ' ';
    while (key != 'q') {

        // Check that grab() is successful
        if (zed.grab() == SUCCESS) {
            // Retrieve left image
            zed.retrieveImage(zed_image, VIEW_LEFT);

            // Display image with OpenCV
            cv::imshow("VIEW", cv::Mat((int) zed_image.getHeight(), (int) zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM_CPU)));
            key = cv::waitKey(5);

            // Change camera settings with keyboard
            //updateCameraSettings(key, zed);
        } else
            key = cv::waitKey(5);
    }

    // Exit
    zed.close();
    return EXIT_SUCCESS;
}
