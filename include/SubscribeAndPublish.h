/**
* This header file is a part of vid_orbslam3 package.
* \brief SubscribeUndPublish will access the Camera topic and do some pre-processing such as reducing the resolution, encoding etc for better usage of the camera feed.
* \author: Manjunath, Shreyas
* \version: 1.0.0

**/

// Necessary header files

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <string>
#include "System.h"
#include "Converter.h"
#include <sensor_msgs/NavSatFix.h>

class SubscribeAndPublish{

private:
	ros::NodeHandle nh;
	std::unique_ptr<ORB_SLAM3::System> objSLAM;
	std::string vocabularyFilePath;
	std::string settingsFilePath;
	cv::Mat cameraMtx;
	cv::Mat distMtx;
	std::vector<tf::Transform> previousCameraPoses;
	ros::Publisher posePublisher;
	ros::Time currentFrameTime;
	ros::Publisher KeyframeGraphPublisher;
	ros::Subscriber gpsSubcriber;
	sensor_msgs::NavSatFix currentGpsCoordinate; 
public:
	/**
	* \brief Default constructor, deleted. Always SubscribeAndPublish Object has to be instanciated 
	* with paratemers. Use Paramerterised constructor
	*/
	SubscribeAndPublish() =  delete;
	/** 
	* \brief Parameterised constructor used to instanciate the SubcribeAndPublish Object.
	* \param nh node handle argument, node object from the main flow is passed as parameter.
	* \param vocabFile ORB vocabulary file path including filename with extension.
	* \param settingsFile ORB settings file path including filename with extension.
	*/ 
	SubscribeAndPublish(ros::NodeHandle& nh, std::string vocabFile, std::string settingsFile);
	/**
	* \brief ConvertROSImageToCV method is a helper method used for Image conversion from ros message to opencv image.
	* It is an additional method for future usage incase of any need.
	* \param img An image pointer of the type sensor_msgs::ImageConstPtr
	* \return Returns a CvImagePtr after conversion of rosmessage to cv.
	*/
	cv_bridge::CvImagePtr ConvertROSImageToCV(const sensor_msgs::ImageConstPtr& img);
	/**
	* \brief callBackSubAndPub is a method callback to a image topic subscriber. 
	* All the necessary processing and publishing takes place inside this callback method.
	* \param img is a constant image pointer of the type sensor_msgs::ImageConstPtr
	* \return void
	*/
	void callbackSubAndPub(const sensor_msgs::ImageConstPtr& img);
	/**
	* \brief Saves Key Frames onto a text or CSV file.
	* \param fileName should contain path/filename.<extension>, type std::string
	* \return void
	*/
	void saveKeyFramesTrajectory(std::string fileName);
	/**
	* \brief Set Camera parameters from settings file.
	* \param settingsFile should contain path/settingsFileName.<extension>, type std::string
	* \return  void
	*/
	void setCameraParams(std::string& settingsFile);
	/**
	* \brief processNextImage calls the TrackMonocular method of SLAM object to obtain the current camera pose Tcw.
	* \param imgPtr a image pointer of the type cv_bridge::CvImageConstPtr
	* \return void
	*/
	void processNextImage(cv_bridge::CvImageConstPtr imgPtr);
	/**
	* \brief publishCameraPose publishes the camera pose as TF to a Tf boardcaster of the node nh.
	* \param Tcw camera Pose obtained from TrackMonocular method of the SLAM object, type cv::Mat
	* \return void
	*/ 
	void publishCameraPose(cv::Mat& Tcw);
	/**
	* \brief Method to compute hamilton product between 2 quaternions.
	* \param a Quaternion 1, type tf::Quaternion
	* \param b Quaternion 2, type tf::Quaternion
	* \return tf::Quaternion object
	*/
	tf::Quaternion hamiltonProduct(tf::Quaternion a, tf::Quaternion b);
	/**
	* \brief Destructor of the class
	*/
	void publishCameraPoseAsPoseStamped (cv::Mat position);
	
	tf::Transform TransformFromMat (cv::Mat position_mat);

	void UpdateKeyframesTopic();
	void UpdateGPSCoordinates(const sensor_msgs::NavSatFixConstPtr& gps);
	void test(cv::Mat& Tcw);
	~SubscribeAndPublish();
//	friend class ORB_SLAM3::System;	

};
