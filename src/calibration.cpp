//
// Created by phyorch on 31/07/18.
//

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include </home/phyorch/OPENCV_PROJECT/Stereo_test/src/StereoMatch.cpp>
#include </home/phyorch/OPENCV_PROJECT/Stereo_test/src/StereoCalib.cpp>

#include <sl/Camera.hpp>

using namespace std;

bool image_capture = false;
bool calibration = false;
bool disparity_capture = true;
int num_frames = 1;

sl::CAMERA_SETTINGS camera_settings_ = sl::CAMERA_SETTINGS_BRIGHTNESS;
string str_camera_settings = "BRIGHTNESS";
int step_camera_setting = 1;

int main(){

    clock_t start = clock();
    double time_start = time(0);
    cv::Size image_size = cv::Size(1280, 720);
    cv::Size board_size = cv::Size(7, 5);
    float square_width = 30;
    string image_path = "/home/phyorch/PROJECT/zed_grabber/data2/";
    string para_path = "/home/phyorch/PROJECT/zed_grabber/output/calibration_parameters.xml";

    if(image_capture){

/*        Method for using zed API    */
/*------------------------------------*/
        // Create a ZED Camera object
        sl::Camera zed;

        // Open the camera
        sl::ERROR_CODE err = zed.open();
        if (err != sl::SUCCESS) {
            cout << toString(err) << endl;
            zed.close();
            return 1; // Quit if an error occurred
        }

        sl::Mat zed_image_left, zed_image_right;
        char key = ' ';
        int count = 1;
        while (key != 'q' && count <= num_frames) {

            // Check that grab() is successful
            if (zed.grab() == sl::SUCCESS) {
                // Retrieve left image
                zed.retrieveImage(zed_image_left, sl::VIEW_LEFT);
                zed.retrieveImage(zed_image_right, sl::VIEW_RIGHT);
                cv::Mat left = cv::Mat((int) zed_image_left.getHeight(), (int) zed_image_left.getWidth(), CV_8UC4, zed_image_left.getPtr<sl::uchar1>(sl::MEM_CPU));
                cv::Mat right = cv::Mat((int) zed_image_right.getHeight(), (int) zed_image_right.getWidth(), CV_8UC4, zed_image_right.getPtr<sl::uchar1>(sl::MEM_CPU));
                string image_msg = "Please capture "" image" + to_string(count);
                int baseLine = 0;
                cv::Size textSize = cv::getTextSize(image_msg, cv::FONT_HERSHEY_COMPLEX, 1, 1, &baseLine);
                cv::Point textOrigin(10, 10);
                cv::putText(left, image_msg, textOrigin, 1, 1, (0, 255, 0));
                cv::putText(right, image_msg, textOrigin, 1, 1, (0, 255, 0));

                // Display image with OpenCV
                cv::imshow("VIEW1", left);
                cv::imshow("VIEW2", right);
                key = cv::waitKey(30);
                if(key==' '){
                    cv::imwrite(image_path+"testl.jpg", left);
                    cv::imwrite(image_path+"testr.jpg", right);
//                  cv::imwrite(image_path+"imagel"+to_string(count+18)+".jpg", left);
//                  cv::imwrite(image_path+"imager"+to_string(count+18)+".jpg", right);
                    count++;
                }

                // Change camera settings with keyboard
                //updateCameraSettings(key, zed);
            } else
                key = cv::waitKey(30);
        }

/*        Method for directly read image from cv::VideoCapture      */
/*-----------------------------------------------------------------*/
//        int count = 1;
//        cv::Mat frame;
//        cv::VideoCapture cap;
//        cap.open(0);
//        while (cap.read(frame) && count <= num_frames){
//            if (frame.empty())
//            {
//                cerr << "can not read frame";
//                break;
//            }
//            cv::Mat left, right;
//            frame(cv::Rect(0, 0, 672, 376)).copyTo(left);
//            frame(cv::Rect(672, 0, 672, 376)).copyTo(right);
//
//            string image_msg = "Please capture "" image" + to_string(count);
//            int baseLine = 0;
//            cv::Size textSize = cv::getTextSize(image_msg, cv::FONT_HERSHEY_COMPLEX, 1, 1, &baseLine);
//            cv::Point textOrigin(10, 10);
//            cv::putText(left, image_msg, textOrigin, 1, 1, (0, 255, 0));
//            cv::putText(right, image_msg, textOrigin, 1, 1, (0, 255, 0));
//
//            cv::imshow("camera1", left);
//            cv::imshow("camera2", right);
//            char c = cv::waitKey(30);
//            if(c==' '){
//                cv::imwrite(image_path+"imagel"+to_string(count)+".jpg", left);
//                cv::imwrite(image_path+"imager"+to_string(count)+".jpg", right);
//                count++;
//            }
//        }
    }

    if(calibration){

        StereoCalib caliber = StereoCalib();
        caliber.initCornerData(num_frames, image_size, board_size, square_width);
        cv::Mat view1, view2;
        int count = 1;


        while(count<=num_frames) {
            view1 = cv::imread(image_path+"imagel"+to_string(count)+".jpg");
            view2 = cv::imread(image_path+"imager"+to_string(count)+".jpg");
            int find = caliber.detectCorners(view1, view2, caliber.corner_datas, count-1);
            if(find==0){
                cout << "image" << count <<" is wrong input" << endl;
                return 1;
            }

            count++;
        }
        caliber.calibrateStereoCamera(caliber.corner_datas, caliber.stereo_params, true);
        double error;
        caliber.getStereoCalibrateError(caliber.corner_datas, caliber.stereo_params, error);
        cout << "The stereo calibration error is " << error << endl;
        RECTIFYMETHOD method = RECTIFY_BOUGUET;
        caliber.rectifyStereoCamera(caliber.corner_datas, caliber.stereo_params, caliber.remap_matrixs, method);
        caliber.saveCalibrationDatas(para_path, method, caliber.corner_datas, caliber.stereo_params,
                                     caliber.remap_matrixs);
        double time_end = time(0);
        //double runtime = time_end - time_start;
        clock_t end = clock();
        double runtime = (double) (end - start) / CLOCKS_PER_SEC;

        cout << "runtime is " << runtime << endl;
    }

    if(disparity_capture){
        sl::Camera zed;
        // Open the camera
        sl::ERROR_CODE err = zed.open();
        if (err != sl::SUCCESS) {
            cout << toString(err) << endl;
            zed.close();
            return 1; // Quit if an error occurred
        }
        sl::Mat zed_image_left, zed_image_right;
        char key = ' ';
        StereoMatch matcher = StereoMatch();
        cv::Mat left, right, disparity, left_output, right_output, color;
        while(key != 'q'){
            if (zed.grab() == sl::SUCCESS) {
                // Retrieve left image
                zed.retrieveImage(zed_image_left, sl::VIEW_LEFT);
                zed.retrieveImage(zed_image_right, sl::VIEW_RIGHT);
                left = cv::Mat((int) zed_image_left.getHeight(), (int) zed_image_left.getWidth(), CV_8UC4,
                                       zed_image_left.getPtr<sl::uchar1>(sl::MEM_CPU));
                right = cv::Mat((int) zed_image_right.getHeight(), (int) zed_image_right.getWidth(), CV_8UC4,
                                        zed_image_right.getPtr<sl::uchar1>(sl::MEM_CPU));
                matcher.bmMatch(left, right, disparity, left_output, right_output, para_path);
                matcher.getDisparityImage(disparity, color, true);
                cv::imshow("real time disparty map", color);
                //cv::imshow("real time disparity", disparity);
                //cv::imshow("Reference map", left);
                char c = cv::waitKey(30);
            }
        }

    }

    return 0;
}