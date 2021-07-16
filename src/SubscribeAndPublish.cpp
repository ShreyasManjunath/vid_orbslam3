#include "SubscribeAndPublish.h"
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include "vid_orbslam3/KeyframeGraph.h"
#include "vid_orbslam3/KeyframeStamped.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

#define WIDTH 640    // Width of the Image Frame
#define HEIGHT 360   // Height of the Image Frame

#define DISPLAY_FLAG false  // Display the ORB SLAM3 Keypoints and Camera Pose on GUI.

SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle& nh, std::string vocabFile, std::string settingsFile):vocabularyFilePath{vocabFile},
settingsFilePath{settingsFile},nh{nh} 
{
	
	objSLAM = std::unique_ptr<ORB_SLAM3::System>(new ORB_SLAM3::System{vocabularyFilePath, settingsFilePath, ORB_SLAM3::System::MONOCULAR, true});
	KeyframeGraphPublisher =  nh.advertise<vid_orbslam3::KeyframeGraph> ("/KeyframeGraph", 1);
	posePublisher = nh.advertise<geometry_msgs::PoseStamped> ("/pose/MAIN_CAMERA", 2);
	setCameraParams(settingsFilePath);
}

cv_bridge::CvImagePtr SubscribeAndPublish::ConvertROSImageToCV(const sensor_msgs::ImageConstPtr& img){

        return cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

}

void SubscribeAndPublish::callbackSubAndPub(const sensor_msgs::ImageConstPtr& img){
	cv_bridge::CvImageConstPtr cv_image_ptr;
	currentFrameTime =  img->header.stamp;
	try{
	cv_image_ptr = cv_bridge::toCvShare(img);
	}

	catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }

	processNextImage(cv_image_ptr);
}


void SubscribeAndPublish::saveKeyFramesTrajectory(std::string fileName){

	objSLAM->Shutdown();
	objSLAM->SaveKeyFrameTrajectoryTUM(fileName);

}

void SubscribeAndPublish::setCameraParams(std::string& settingsFile){

	cv::FileStorage file(settingsFile, cv::FileStorage::READ);
	cameraMtx = cv::Mat::eye(3, 3, CV_32F);

/**
*	| fx 0 cx |
*	| 0 fy cy |
*	| 0 0  1  |
*
*/
	cameraMtx.at<float>(0,0) = file["Camera.fx"];
	cameraMtx.at<float>(1,1) = file["Camera.fy"];
	cameraMtx.at<float>(0,2) = file["Camera.cx"];
	cameraMtx.at<float>(1,2) = file["Camera.cy"];
	
	distMtx = cv::Mat(4, 1, CV_32F);
	distMtx.at<float>(0) = file["Camera.k1"];
	distMtx.at<float>(1) = file["Camera.k2"];
	distMtx.at<float>(2) = file["Camera.p1"];
	distMtx.at<float>(3) = file["Camera.p2"];

	float k3 = file["Camera.k3"];	
	if(k3 != 0){
		distMtx.resize(5);
		distMtx.at<float>(4) =  file["Camera.k3"];
	}	

} 


void SubscribeAndPublish::processNextImage(cv_bridge::CvImageConstPtr imgPtr){

	cv::Mat Tcw = objSLAM->TrackMonocular(imgPtr->image, imgPtr->header.stamp.toSec());
	std::cout << Tcw << std::endl;
	if(!Tcw.empty())
		publishCameraPose(Tcw);
	
	UpdateKeyframesTopic();

}

SubscribeAndPublish::~SubscribeAndPublish(){

	cv::destroyAllWindows();
}

void SubscribeAndPublish::publishCameraPose(cv::Mat& Tcw){	

	static tf::TransformBroadcaster br;	
	
	tf::Matrix3x3 rotMtx;
	rotMtx.setValue(Tcw.at<float>(0,0), Tcw.at<float>(0,1), Tcw.at<float>(0,2),
			Tcw.at<float>(1,0), Tcw.at<float>(1,1), Tcw.at<float>(1,2),
			Tcw.at<float>(2,0), Tcw.at<float>(2,1), Tcw.at<float>(2,2));

	tf::Quaternion tfQt;
	rotMtx.getRotation(tfQt);
		
	double temp = tfQt[0];
	tfQt[0] = -tfQt[2];
	tfQt[2] = tfQt[1];
	tfQt[1] = temp;

	tf::Vector3 origin;
	origin.setValue(Tcw.at<float>(0,3),
			Tcw.at<float>(1,3),
			Tcw.at<float>(2,3));
	tf::Matrix3x3 rot270degXZ(-1, 0, 0, 
				  0, 0, -1,
				  0,1, 0);

	tf::Vector3 translationForCamera = origin * rot270degXZ;
	tf::Quaternion qtForHamilton1(tfQt[3], tfQt[0], tfQt[1], tfQt[2]);
	tf::Quaternion qtForHamilton2(tfQt[3], -tfQt[0], -tfQt[1], -tfQt[2]);
	tf::Quaternion translationHamilton(0, translationForCamera[0], translationForCamera[1], translationForCamera[2]);
		
	tf::Quaternion translationStepQuat;
	translationStepQuat = hamiltonProduct(hamiltonProduct(qtForHamilton1, translationHamilton), qtForHamilton2);
	tf::Vector3 translation(translationStepQuat[1], translationStepQuat[2], translationStepQuat[3]);

	tf::Transform transformCurrent;
	transformCurrent.setOrigin(translation);
	transformCurrent.setRotation(tfQt);
	br.sendTransform(tf::StampedTransform(transformCurrent, ros::Time::now(), "map", "MAIN_CAMERA"));
		
	// Saving the current transform 
	previousCameraPoses.push_back(transformCurrent); 	


}


tf::Quaternion SubscribeAndPublish::hamiltonProduct(tf::Quaternion a, tf::Quaternion b) {

	tf::Quaternion c;

		c[0] = (a[0]*b[0]) - (a[1]*b[1]) - (a[2]*b[2]) - (a[3]*b[3]);
		c[1] = (a[0]*b[1]) + (a[1]*b[0]) + (a[2]*b[3]) - (a[3]*b[2]);
		c[2] = (a[0]*b[2]) - (a[1]*b[3]) + (a[2]*b[0]) + (a[3]*b[1]);
		c[3] = (a[0]*b[3]) + (a[1]*b[2]) - (a[2]*b[1]) + (a[3]*b[0]);

	return c;
}

tf::Transform SubscribeAndPublish::TransformFromMat (cv::Mat position_mat) {

  //std::cout << "Inside Transform from Mat, before tf_camera_rotation....!"<<std::endl;
  tf::Matrix3x3 tf_camera_rotation; 
	tf_camera_rotation.setValue(position_mat.at<float> (0,0), position_mat.at<float> (0,1), position_mat.at<float> (0,2),
                                    position_mat.at<float> (1,0), position_mat.at<float> (1,1), position_mat.at<float> (1,2),
                                    position_mat.at<float> (2,0), position_mat.at<float> (2,1), position_mat.at<float> (2,2)
                                   );
	//std::cout << "Inside Transform from Mat....!"<<std::endl;

  tf::Vector3 tf_camera_translation;
  tf_camera_translation.setValue(position_mat.at<float> (0,3), position_mat.at<float> (1,3), position_mat.at<float> (2,3));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf::Matrix3x3 tf_orb_to_ros(0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);
//std::cout << "After tf_orb_to_ros....!"<<std::endl;

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf::Transform (tf_camera_rotation, tf_camera_translation);
}


void SubscribeAndPublish::publishCameraPoseAsPoseStamped (cv::Mat position) {
  std::cout << "Inside publishCameraPoseAsPoseStamped....!"<<std::endl;  
  tf::Transform grasp_tf = TransformFromMat(position);
  tf::Stamped<tf::Pose> grasp_tf_pose(grasp_tf, currentFrameTime, "map");
  geometry_msgs::PoseStamped pose_msg;
  tf::poseStampedTFToMsg (grasp_tf_pose, pose_msg);
  // current_frame_time
  pose_msg.header.stamp = ros::Time::now();
  posePublisher.publish(pose_msg);
}

void SubscribeAndPublish::UpdateKeyframesTopic(){

	std::vector<ORB_SLAM3::KeyFrame*> vpKFs = objSLAM->getAtlas()->GetAllKeyFrames();
	std::sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM3::KeyFrame::lId);
	vid_orbslam3::KeyframeGraph kfGraph;

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM3::KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        std::vector<float> q = ORB_SLAM3::Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
		double timestamp = pKF->mTimeStamp;
		vid_orbslam3::KeyframeStamped kfPose;
		geometry_msgs::Point point;
		point.x = t.at<float>(0);
		point.y = t.at<float>(1);
		point.z = t.at<float>(2);
		geometry_msgs::Quaternion rot;
		rot.x = q[0];
		rot.y = q[1];
		rot.z = q[2];
		rot.w = q[3];
		geometry_msgs::Pose pose;
		pose.position = point;
		pose.orientation = rot;
		kfPose.timestamp_slam = timestamp;
		kfPose.Keyframe.pose = pose;
		kfPose.Keyframe.header.stamp = ros::Time(timestamp);
		kfPose.Keyframe.header.seq = i;
		kfGraph.KeyframeList.push_back(kfPose);
	
    }
    //std::cout << "KFs: " << vpKFs.size() << std::endl;
    KeyframeGraphPublisher.publish(kfGraph);


}

void SubscribeAndPublish::test(cv::Mat& Tcw){	

	static tf::TransformBroadcaster br;	
	
	tf::Matrix3x3 rotMtx;
	rotMtx.setValue(Tcw.at<float>(0,0), Tcw.at<float>(0,1), Tcw.at<float>(0,2),
			Tcw.at<float>(1,0), Tcw.at<float>(1,1), Tcw.at<float>(1,2),
			Tcw.at<float>(2,0), Tcw.at<float>(2,1), Tcw.at<float>(2,2));

	tf::Quaternion tfQt;
	rotMtx.getRotation(tfQt);
		
	tf::Vector3 origin;
	origin.setValue(Tcw.at<float>(0,3),
			Tcw.at<float>(1,3),
			Tcw.at<float>(2,3));
	tf::Matrix3x3 rot270degXZ(0, 1, 0, 
				  0, 0, 1,
				  -1,0, 0);

	tf::Transform transformCurrent;
	transformCurrent.setOrigin(origin);
	transformCurrent.setRotation(tfQt);
	br.sendTransform(tf::StampedTransform(transformCurrent, ros::Time::now(), "map", "MAIN_CAMERA"));	


}



int main(int argc, char** argv){

	ros::init(argc, argv, "vid_orbslam3_node");
	ros::NodeHandle nh;
	std::string vocabFile;
	std::string settingsFile;
	nh.getParam("/vid_orbslam3_node/vocabulary_path", vocabFile);
	nh.getParam("/vid_orbslam3_node/settings_path", settingsFile); 
	SubscribeAndPublish sap(nh,vocabFile, settingsFile);
	ros::Subscriber subcriber = nh.subscribe("/dji_osdk_ros/main_camera_images", 1, &SubscribeAndPublish::callbackSubAndPub, &sap);
	ros::spin();
	sap.saveKeyFramesTrajectory("/home/manshr/Keyframe.txt");
	ros::shutdown();
	return 0;
}







