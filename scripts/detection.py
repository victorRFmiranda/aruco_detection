#!/usr/bin/env python
import rospy
import cv2
from cv2 import aruco
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose

from tf.transformations import rotation_matrix, quaternion_from_matrix




class aruco_detection:
	def __init__(self):
		self.image = 0
		self.image_marker = 0
		self.gray_image = 0
		self.dictionary = aruco.Dictionary_get(aruco.DICT_6X6_100)
		self.parameters =  aruco.DetectorParameters_create()
		self.intrinsic_matrix = 0
		self.distortion_matrix = 0
		self.corners = 0
		self.rot_mat = 0
		W = 1.0/2.0;
  		H = 1.0/2.0;
  		self.objp = np.array([[-W,-H,0.0], [W,-H,0.0], [W,H,0.0], [-W,H,0.0]])

		rospy.init_node('aruco_detection', anonymous=True)



		#Load parameters
		try:
			aruco_size = float(rospy.get_param("/aruco_detection/aruco_size"))
			self.aruco_pose = eval(rospy.get_param("/aruco_detection/aruco_pose"))
			image_topic = rospy.get_param("/aruco_detection/image_topic")
			pose_topic = rospy.get_param("/aruco_detection/pose_topic")
			print "\n\33[92mParameters loaded\33[0m"
			print "\33[94maruco_size: ", aruco_size,"\33[0m"
			print "\33[94maruco_pose: ", self.aruco_pose,"\33[0m"
			print "\33[94mimage_topic: ", image_topic,"\33[0m"
			print "\33[94mpose_topic: ", pose_topic,"\33[0m"
		except:
			print "\33[41mProblem occurred when trying to read the parameters!\33[0m"
			print "\33[41mNode detection.py\33[0m"

		W = aruco_size/2.0
		H = aruco_size/2.0


		rospy.Subscriber(image_topic+"image_raw", Image, self.callback_image)
		rospy.Subscriber(image_topic+"camera_info", CameraInfo, self.callback_camera_info)
		self.pub_draw = rospy.Publisher("/aruco_image", Image, queue_size=1)
		self.pub_pose = rospy.Publisher(pose_topic, Pose, queue_size=1)



		rospy.spin()




	def callback_image(self, data):
		bridge = CvBridge()
		self.image = bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
		#self.image = cv2.flip(self.image, 0) # vertical flip
		self.gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
		self.corners, ids, rejectedImgPoints = aruco.detectMarkers(self.gray_image, self.dictionary, parameters=self.parameters)
		frame_markers = aruco.drawDetectedMarkers(self.image.copy(), self.corners, ids)
		pub_image = bridge.cv2_to_imgmsg(frame_markers, encoding="rgb8")
		if ids is not None:
			self.pose_from_markers(ids)
		
		self.pub_draw.publish(pub_image)



	def callback_camera_info(self, data):
		self.intrinsic_matrix = np.array([[data.K[0],data.K[1],data.K[2]],[data.K[3],data.K[4],data.K[5]],[data.K[6],data.K[7],data.K[8]]], dtype = "double")
		self.distortion_matrix = np.array([[data.D[0]],[data.D[1]],[data.D[2]],[data.D[3]],[data.D[4]]], dtype = "double")




	def pose_from_markers(self, ids):
		for i in range(0,ids.size):
			succes, rvecs, tvecs = cv2.solvePnP(self.objp, self.corners[i][0], self.intrinsic_matrix, self.distortion_matrix, flags=cv2.SOLVEPNP_ITERATIVE)
			self.rot_mat, _ = cv2.Rodrigues(rvecs) 
			R_ = np.concatenate((self.rot_mat,tvecs), axis=1 )
			R = np.concatenate((R_,np.array([[0,0,0,1]])), axis = 0)

			H_a_c = R; #H_a^c
			H_c_d = np.array([[0,-1,0,0],[-1,0,0,0],[0,0,-1,-0.1],[0,0,0,1]]) #H_c^d


			# print "R:\n", R, "\n\n"
			# print "R:\n", R[0][3], " ", R[1][3], " ", R[2][3], "\n\n"

			pose = Pose()
			if (ids[i] == 0):
				H_a2_w = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
				H_c_d = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
				H_a_a2 = np.array([[0, 0, 1, 0],   [1, 0, 0, 0],   [0, 1, 0, 0],   [0, 0, 0, 1]])
				# R = rotation_matrix(0.123, (1, 2, 3))

				H_a_w = H_a2_w.dot(H_a_a2)
				H_c_w = H_a_w.dot(np.linalg.inv(R))
				H_d_w = H_c_w.dot(np.linalg.inv(H_c_d))

				quat = quaternion_from_matrix(H_d_w)


				pose.position.x = H_d_w[0][3]
				pose.position.y = H_d_w[1][3]
				pose.position.z = H_d_w[2][3]
				pose.orientation.x = quat[0]
				pose.orientation.y = quat[1]
				pose.orientation.z = quat[2]
				pose.orientation.w = quat[3]
				self.pub_pose.publish(pose)

			elif (ids[i] == 2):

				# H_a_w = np.array([[0,-1,0,18],[-1,0,0,0],[0,0,-1,0],[0,0,0,1]])
				H_a_w = np.array(self.aruco_pose)

				print "aruco_pose:\n", self.aruco_pose, "\n\n"

				H_c_w = H_a_w.dot(np.linalg.inv(H_a_c))
				H_d_w = H_c_w.dot(np.linalg.inv(H_c_d))

				# print "H_d_w:\n", H_d_w, "\n\n"

				quat = quaternion_from_matrix(H_d_w)

				pose.position.x = H_d_w[0][3]
				pose.position.y = H_d_w[1][3]
				pose.position.z = H_d_w[2][3]
				pose.orientation.x = quat[0]
				pose.orientation.y = quat[1]
				pose.orientation.z = quat[2]
				pose.orientation.w = quat[3]

			self.pub_pose.publish(pose)




########### MAIN #####################
if __name__ == '__main__':

	try:
		Detect_markers = aruco_detection()		
	except rospy.ROSInterruptException:
		pass



