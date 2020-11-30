
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "boost/thread.hpp"
#include "sensor_msgs/CameraInfo.h"
#include <ros/package.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;


class pcl2point {

  public:

    pcl2point();
    void run();
    void depth_cb( sensor_msgs::ImageConstPtr depth );
    void cam_parameters( sensor_msgs::CameraInfo camera_info);

  private:

    ros::NodeHandle _nh;
    ros::Subscriber _depth_img_sub;
    ros::Subscriber _cam_info_sub;
    Mat _depth_src;
    bool _depth_ready;


    double cx, cy, fx_inv, fy_inv;
    double cx_c1, cy_c1, cz_c1;
    float zd_c1;
    float zd_c2;
    bool _cam_info_first;
    cv::Mat *_cam_cameraMatrix, *_cam_distCo, *_cam_R, *_cam_P;
};



pcl2point::pcl2point() {
    _depth_img_sub = _nh.subscribe( "/depth_camera/depth/disparity", 1,&pcl2point::depth_cb, this );
    _cam_info_sub = _nh.subscribe("/depth_camera/depth/camera_info", 0, &pcl2point::cam_parameters, this);
    _depth_ready = false;
    _cam_info_first = false;
}

void pcl2point::cam_parameters( sensor_msgs::CameraInfo camera_info) {

    /*
     *  ROS topic data
     *  K = cameraMatrix
     *  D = distCoeffs
     *  R = Rettification
     *  P = Projection
     */

    if( _cam_info_first == false ) {

        ROS_INFO("Start camera parameters initialization...");
        //---resize calibration matrix
        _cam_cameraMatrix = new cv::Mat(3, 3, CV_64FC1);
        _cam_distCo = new cv::Mat(1, 5, CV_64FC1);
        _cam_R = new cv::Mat(3, 3, CV_64FC1);
        _cam_P = new cv::Mat(3, 4, CV_64FC1);
        //---

        //---K
        for(int i=0; i<3;i++) {
            for(int j=0; j<3; j++) {
                _cam_cameraMatrix->at<double>(i,j) = camera_info.K[3*i+j];

                cout << "[" << i << ", " << j << "]: " << _cam_cameraMatrix->at<double>(i,j) << endl;
            }
        }
        //---D
				if( camera_info.D.size() >= 5 ) {
	        for(int i=0; i<5;i++) {
            _cam_distCo->at<double>(0,i) = camera_info.D[i];
  	      }
				}
        //---R
        for(int i=0; i<3;i++) {
            for(int j=0; j<3; j++) {
                _cam_R->at<double>(i,j) = camera_info.R[3*i+j];
            }
        }
        //---P
        for(int i=0; i<3;i++) {
            for(int j=0; j<4; j++) {
                _cam_P->at<double>(i,j) = camera_info.P[4*i+j];
            }
        }
        _cam_info_first = true;

        ROS_INFO("...camera parameters initialization complete!");
    }

}

void pcl2point::depth_cb( sensor_msgs::ImageConstPtr depth ) {
    
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
        _depth_src = cv_ptr->image;
        _depth_ready = true;


        //Get 3d Point
        cx = _cam_cameraMatrix->at<double>(0,2);
        cy = _cam_cameraMatrix->at<double>(1,2);
        fx_inv = 1.0 / _cam_cameraMatrix->at<double>(0,0);
        fy_inv = 1.0 / _cam_cameraMatrix->at<double>(1,1);


        cout << cx << ", " << cy << " " << fx_inv << " " << fy_inv << endl;
        zd_c1 = _depth_src.at<float>(0, 0);
        cout << "zd_c1: " << zd_c1 << endl;
        cx_c1 = (zd_c1) * ( (0 - cx) * fx_inv );
        cy_c1 = (zd_c1) * ( (0 - cy) * fy_inv );
        cz_c1 = zd_c1;
    
        cout << "3d Point: (" << cx_c1 << ", " << cy_c1 << ", " << cz_c1 << ")" << endl; 
    
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
}

void pcl2point::run() {
    ros::spin();
}


int main(int argc, char** argv) {
    ros::init( argc, argv, "shape_tracking");
    pcl2point p2p;
    p2p.run();

    return 0;

}