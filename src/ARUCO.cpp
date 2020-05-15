#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>

#include <stdlib.h>
#include <iostream>

#include <aruco_detection.h>
#include "aruco_detection/Aruco.h"
#include "aruco_detection/ArucoArray.h"
#include "rtabmap_ros/Point2f.h"
#include <opencv2/aruco.hpp>

using namespace std;
using namespace Eigen;


ArucoDetection *Ar_Detection;
cv::Mat image;
cv::Mat printed_image;

ros::Publisher aruco_pub;

// Camera infor parameters
int height, width;
cv::Mat K_mat = cv::Mat_<float>(3,3);
cv::Mat R_mat = cv::Mat_<float>(3,3);
cv::Mat P_mat = cv::Mat_<float>(3,4);
cv::Mat D_mat = cv::Mat_<float>(1,3);


void proc_aruco(cv::Mat &frame, cv::Mat &Mt, cv::Mat &Ds){
	ArucoDetection::Aruco_pose aruco_data;
	aruco_data = Ar_Detection->Pose_Estimation(frame, Mt, Ds);
	if (aruco_data.isPose){
		aruco_detection::ArucoArray aruco_msg;
		aruco_detection::Aruco data;
		rtabmap_ros::Point2f corners_aux;
		for(int i=0; i < aruco_data.id.size(); i++){
			data.id = aruco_data.id[i];
			for (int k=0; k<4; k++){
				corners_aux.x = aruco_data.corners[i][k].x;
				corners_aux.y = aruco_data.corners[i][k].y;
				data.corners.push_back(corners_aux);
			}
			aruco_msg.aruco.push_back(data);
			data.corners.clear();
		}
		cout << aruco_msg << endl;
		aruco_pub.publish(aruco_msg);
	}
}

cv::Ptr<cv::aruco::Dictionary>  dictionary;
cv::Ptr<cv::aruco::DetectorParameters> parameters;
std::vector<std::vector<cv::Point2f>> corners, rejected;
std::vector<int> ids;

//-----------------Callback Image Topic------------------
//-------------------------------------------------------
void callback_img(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    image = cv_ptr->image;
    cv::flip(image, image, +1);

    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    parameters = cv::aruco::DetectorParameters::create();

    if (!image.empty()){

    	
    	cv::Mat gray;
  		cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY); /// NOT WORKING
  		cv::aruco::detectMarkers(gray, dictionary, corners, ids, parameters, rejected);
  		//cv::aruco::drawDetectedMarkers(printed_image, corners, ids);

  		cout<<corners.size()<<endl;

  		printed_image = gray.clone();
        // proc_aruco(image, K_mat, D_mat);
    }
}
//-------------------------------------------------------
//-------------------------------------------------------


void callback_cameraInfo(const sensor_msgs::CameraInfoConstPtr& msg){
	height = msg->height;
	width = msg->width;
	for(int i=0; i<4; i++){
		D_mat.at<float>(i) = msg->D[i];
	}
	// K = msg->K;
	int cont = 0;
	int cont2 = 0;
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			K_mat.at<float>(0,0) = msg->K[cont];
			R_mat.at<float>(0,0) = msg->R[cont];
			cont++;
		}
		for(int j=0; j<4; j++){
			P_mat.at<float>(0,0) = msg->P[cont2];
			cont2++;
		}
		
	}

}

int main(int argc, char **argv){
    Ar_Detection = new ArucoDetection();
    ros::init(argc, argv, "aruco");
    ros::NodeHandle AR;
    ros::Rate loop_rate(10);
    aruco_pub = AR.advertise<aruco_detection::ArucoArray>("aruco", 1);
    ros::Publisher img_pub = AR.advertise<sensor_msgs::Image>("/aruco_image", 1);

    //ros::Subscriber img_subscriber = AR.subscribe("/usb_cam/image_raw", 1, callback_img);
    ros::Subscriber img_subscriber = AR.subscribe("/camera/floor/camera_info", 1, callback_cameraInfo);
    ros::Subscriber info_subscriber = AR.subscribe("/camera/floor/image_raw", 1, callback_img);
    ros::spinOnce();
    // ros::spin();

    while (ros::ok()){
    	// convert opencv to sensor msgs
		if(!printed_image.empty()){
			sensor_msgs::Image img_msg;
			cv_bridge::CvImage img_bridge;
			std_msgs::Header header;
			header.stamp = ros::Time::now();
			img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, printed_image);
			img_bridge.toImageMsg(img_msg);
			//publish identified gate image
			img_pub.publish(img_msg);
		}
	

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
