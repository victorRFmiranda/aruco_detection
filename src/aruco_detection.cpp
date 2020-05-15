#include "aruco_detection.h"

using namespace std;
using namespace Eigen;

ArucoDetection::ArucoDetection(){
  W = 1.0/2.0;
  H = 1.0/2.0;
  //dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
  parameters = cv::aruco::DetectorParameters::create();

  aruco_3d_points.push_back(cv::Point3f(-W,-H,0.0));
  aruco_3d_points.push_back(cv::Point3f(W, -H, 0.0));
  aruco_3d_points.push_back(cv::Point3f(W,H,0.0));
  aruco_3d_points.push_back(cv::Point3f(-W,H,0.0));

//  rvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
//  tvec = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
};

ArucoDetection::~ArucoDetection(){};


//void ArucoDetection::setVectors(cv::Mat rot_vec, cv::Mat transl_vec){
//	rvec = rot_vec;
//	tvec = transl_vec;
//}



ArucoDetection::Aruco_pose ArucoDetection::Pose_Estimation(cv::Mat cam_image, cv::Mat c_matrix, cv::Mat dist_vec){
  intrinsic_matrix = c_matrix;
  distortion_coeff = dist_vec;
  std::vector<VectorXd> pose_vec;
  std::vector<int> init_id;
  VectorXd pose(7);
  pose << 0.0,0.0,0.0,  1.0,0.0,0.0,0.0;
  // pose.push_back(init)
  ArucoDetection::Aruco_pose position_data = {false, pose_vec, ids};

  image = cam_image;
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
  cv::aruco::detectMarkers(gray, dictionary, corners, ids, parameters, rejected);

  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);

  cv::waitKey(0.5);


  if(ids.size() > 0){
    MatrixXd H_a2_w(4,4), H_a_a2(4,4), H_a_c(4,4), H_c_d(4,4), H_d_w(4,4);
    VectorXd aruco_pos_w(3),aruco_angle_w(4);
    for(unsigned int k=0; k<ids.size(); k++){
      // solve PNP

	  cv::Mat distortion_null = (cv::Mat_<float>(1,4) <<  0.0, 0.0, 0.0, 0.0, 0.0);

//	  cv::solvePnP(aruco_3d_points, corners[k], intrinsic_matrix, distortion_null, rvec, tvec, true, 1); // 10 ms


      cv::solvePnP(aruco_3d_points, corners[k], intrinsic_matrix, distortion_coeff, rvec, tvec, cv::SOLVEPNP_ITERATIVE);

      // Create Opencv Rotation Matrix
      cv::Mat rotation_mat(3, 3, cv::DataType<float>::type);
      cv::Rodrigues(rvec, rotation_mat);

      // Convert Rotation Matrix to Eigen
      Matrix3d mat2;
      cv::cv2eigen(rotation_mat, mat2);

//      if(ids[k] == 1){
//    	  H_a2_w << -0.000000, 1.000000, -0.000000, 0.000000, -0.987688, -0.000000, -0.156434, 0.000000, -0.156434, -0.000000, 0.987688, 0.230000, 0.000000, 0.000000, 0.000000, 1.000000;
//      }
//      else if(ids[k] == 2){
//    	  H_a2_w << 0.188460, 0.981627, 0.029849, 0.920000, -0.969542, 0.190809, -0.153560, -1.470000, -0.156434, 0.000000, 0.987688, 0.230000, 0.000000, 0.000000, 0.000000, 1.000000;
//      }
//      else if(ids[k] == 3){
//    	  H_a2_w << -0.698401, 0.707107, -0.110616, -1.410000, -0.698401, -0.707107, -0.110616, -1.180000, -0.156434, 0.000000, 0.987688, 0.245000, 0.000000, 0.000000, 0.000000, 1.000000;
//      }


      if(ids[k] == 0){
		  H_a2_w << 0.0, 1.0, 0.0, -0.5, -1.0, 0.0, 0.0, -1.99, 0.0, 0.0, 1.0, 1.63, 0.0, 0.0, 0.0, 1.0;
      }
      else if(ids[k] == 1){
		  H_a2_w << 0.0, 1.0, 0.0, 1.22, -1.0, 0.0, 0.0, -1.99, 0.0, 0.0, 1.0, 1.63, 0.0, 0.0, 0.0, 1.0;
      }
      else if(ids[k] == 30){
		  H_a2_w << 0.0, -1.0, 0.0, -0.32, 1.0, 0.0, 0.0, 1.91, 0.0, 0.0, 1.0, 1.63, 0.0, 0.0, 0.0, 1.0;
      }
      else if(ids[k] == 55){
      	  H_a2_w << 0.0, -1.0, 0.0, 1.69, 1.0, 0.0, 0.0, 1.91, 0.0, 0.0, 1.0, 1.63, 0.0, 0.0, 0.0, 1.0;
      }

      // FRAME ROTATION
      // H_a2_w << 1.0, 0.0, 0.0, 0.0,   0.0, 1.0, 0.0, 0.0,    0.0, 0.0, 1.0, 0.0,    0.0, 0.0, 0.0, 1.0;
      //H_a2_w << cos(), -sin(), 0.0, aruco_pos_w(0),   sin(), cos(), 0.0, aruco_pos_w(1),    0.0, 0.0, 1.0, aruco_pos_w(2),    0.0, 0.0, 0.0, 1.0;
      H_a_a2 << 0, 0, 1, 0,   -1, 0, 0, 0,   0, -1, 0, 0,   0, 0, 0, 1;
      H_a_c << mat2(0,0), mat2(0,1), mat2(0,2), tvec.at<double>(0),   mat2(1,0), mat2(1,1), mat2(1,2), tvec.at<double>(1),   mat2(2,0), mat2(2,1), mat2(2,2), tvec.at<double>(2),   0, 0, 0, 1;
      H_c_d << -0.5, 0.2241, 0.8365, 0.17,   -0.8660, -0.1294, -0.4830, -0.05,   0, -0.9659, 0.2588, -0.02,   0, 0, 0, 1; // par direito, camera esquerda
      //H_c_d << 0.5, 0.2241, 0.8365, 0.17,   -0.8660, 0.1294, 0.4830, -0.05,   0, -0.9659, 0.2588, 0.02,   0, 0, 0, 1; // par esquerdo camera direita

      MatrixXd aux(4,4),aux2(4,4);
      aux = H_a2_w*H_a_a2;
      aux2 = aux*(H_a_c).inverse();
      H_d_w = aux2*(H_c_d).inverse();

      // tranlation points
      pose.block(0,0,3,1) = H_d_w.block(0,3,3,1);


      // rotation quaternion
      Matrix3d mat;
      mat = H_d_w.block(0,0,3,3);
      Quaterniond quat(mat);
      pose(3) = quat.w();
      pose(4) = quat.x();
      pose(5) = quat.y();
      pose(6) = quat.z();

      pose_vec.push_back(pose);
    }
    position_data.isPose = true;
    position_data.id = ids;
    position_data.pose = pose_vec;
    position_data.corners = corners;
  }
  else{
    position_data.isPose = false;
  }

  return position_data;
}


// VectorXd ArucoDetection::Pose_Estimation(cv::Mat cam_image, cv::Mat c_matrix, cv::Mat dist_vec){
//   intrinsic_matrix = c_matrix;
//   distortion_coeff = dist_vec;
//   VectorXd pose(7);
//   pose << 0.0,0.0,0.0,  1.0,0.0,0.0,0.0;
//   image = cam_image;
//   cv::Mat gray;
//   cv::cvtColor(image, gray, CV_BGR2GRAY);
//   cv::aruco::detectMarkers(gray, dictionary, corners, ids, parameters, rejected);
//
//   cv::Mat rvec(3, 1, cv::DataType<double>::type);
//   cv::Mat tvec(3, 1, cv::DataType<double>::type);
//
//   cv::waitKey(0.5);
//
//   if(ids.size() > 0){
//     MatrixXd H_a2_w(4,4), H_a_a2(4,4), H_a_c(4,4), H_c_d(4,4), H_d_w(4,4);
//     for(int k=0; k<ids.size(); k++){
//
//         cv::solvePnP(aruco_3d_points, corners[k], intrinsic_matrix, distortion_coeff, rvec, tvec, cv::SOLVEPNP_ITERATIVE);
//
//         cv::Mat rotation_mat(3, 3, cv::DataType<float>::type);
//         cv::Rodrigues(rvec, rotation_mat);
//
//         Matrix3d mat2;
//         cv::cv2eigen(rotation_mat, mat2);
//
//
//         // FRAME ROTATION
//         H_a2_w << 1.0, 0.0, 0.0, 0.0,   0.0, 1.0, 0.0, 0.0,    0.0, 0.0, 1.0, 0.0,    0.0, 0.0, 0.0, 1.0;
//   		  H_a_a2 << 0, 0, 1, 0,   -1, 0, 0, 0,   0, -1, 0, 0,   0, 0, 0, 1;
//   		  H_a_c << mat2(0,0), mat2(0,1), mat2(0,2), tvec.at<double>(0),   mat2(1,0), mat2(1,1), mat2(1,2), tvec.at<double>(1),   mat2(2,0), mat2(2,1), mat2(2,2), tvec.at<double>(2),   0, 0, 0, 1;
//   		  H_c_d << -0.5, 0.2241, 0.8365, 0.17,   -0.8660, -0.1294, -0.4830, -0.05,   0, -0.9659, 0.2588, -0.02,   0, 0, 0, 1;
//
//         MatrixXd aux(4,4),aux2(4,4);
//         aux = H_a2_w*H_a_a2;
//         aux2 = aux*(H_a_c).inverse();
//         H_d_w = aux2*(H_c_d).inverse();
//
//         // tranlation points
//   		  pose.block(0,0,3,1) = H_d_w.block(0,3,3,1);
//
//   		  cout << "Pose aruco: " << pose.block(0,0,3,1).transpose() << endl;
//
//         // rotation quaternion
//         Matrix3d mat;
//         mat = H_d_w.block(0,0,3,3);
//         Quaterniond quat(mat);
//         pose(3) = quat.w();
// 	      pose(4) = quat.x();
// 	      pose(5) = quat.y();
// 	      pose(6) = quat.z();
//
// 	      cout << "Quat aruco: " << pose.block(3,0,4,1).transpose() << endl  << endl;
//     }
//   }
//     return pose;
// }
