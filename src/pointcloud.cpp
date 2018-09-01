//
// Created by phyorch on 01/08/18.
//

#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pangolin/pangolin.h>
#include <unistd.h>
using namespace std;


int num_frames = 9;
string image_path = "/home/phyorch/PROJECT/zed_grabber/data/";
string para_path = "/home/phyorch/PROJECT/zed_grabber/output/calibration_parameters.xml";

void showPointCloud(const vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud);
int main(){
    // 内参
//    double fxl = 3.3724713570363969e+02, fyl = 3.3588963152636410e+02, cxl = 3.3908881508879523e+02, cyl = 1.9714293462243322e+02;
//    double fxr = 3.4451512947289274e+02, fyr = 3.4468839154267368e+02, cxr = 3.4782442856188936e+02, cyr = 2.0375539191900506e+02;
    double fx = 3.3724713570363969e+02, fy = 3.3588963152636410e+02, cx = 3.3908881508879523e+02, cy = 1.9714293462243322e+02;
    // 间距
    double d = 0.12;

    //vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>* Pointclouds;
    vector<cv::Point3f>* Pointclouds;
    vector<cv::Point3f>* ptr = Pointclouds;
    int count = 1;
    while(count<=num_frames){
        cv::Mat left = cv::imread(image_path+"imagel"+to_string(count)+".jpg", 0);
        cv::Mat right = cv::imread(image_path+"imager"+to_string(count)+".jpg", 0);
        cv::Mat disparity = cv::imread(image_path+"disparity"+to_string(count)+".jpg", 0);
        // 生成点云
        //vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud;
        vector<cv::Point3f> pointcloud;

        // TODO 根据双目模型计算点云
        // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2
        for (int v = 0; v < left.rows; v++)
            for (int u = 0; u < left.cols; u++) {
                cv::Point3f point;
                double depth = d * fx / disparity.at<uchar>(v, u);
                point.x = depth * (u - cx)/fx;
                point.y = depth * (v - cy)/fy;
                point.z = depth;

//                Eigen::Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色
//                double depth = d * fx / disparity.at<uchar>(v, u);
//                point(0) = depth * (u - cx)/fx;
//                point(1) = depth * (v - cy)/fy;
//                point(2) = depth;
                pointcloud.push_back(point);
                // 根据双目模型计算 point 的位置
                // end your code here
            }
        ptr = &pointcloud;
        ptr++;
        count++;

    }

    vector<cv::Point3f> test = *Pointclouds;
    return 0;
}


//void showPointCloud(const vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointcloud) {
//
//    if (pointcloud.empty()) {
//        cerr << "Point cloud is empty!" << endl;
//        return;
//    }
//
//    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
//    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//    pangolin::OpenGlRenderState s_cam(
//            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
//            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
//    );
//
//    pangolin::View &d_cam = pangolin::CreateDisplay()
//            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
//            .SetHandler(new pangolin::Handler3D(s_cam));
//
//    while (pangolin::ShouldQuit() == false) {
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//        d_cam.Activate(s_cam);
//        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//
//        glPointSize(2);
//        glBegin(GL_POINTS);
//        for (auto &p: pointcloud) {
//            glColor3f(p[3], p[3], p[3]);
//            glVertex3d(p[0], p[1], p[2]);
//        }
//        glEnd();
//        pangolin::FinishFrame();
//        usleep(5000);   // sleep 5 ms
//    }
//    return;
//}