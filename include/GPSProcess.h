/**
* This header file is a part of vid_orbslam3 package.
* \brief GPS_node will access the GPS topic and do similarity transform with the camera keyframes.
* \author: Manjunath, Shreyas
* \version: 1.0.0

**/

// Necessary header files

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/NavSatFix.h>
#include <colmap/base/similarity_transform.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <vector>
#include <utility>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// Message format headers of vid_orbslam3 package
#include "vid_orbslam3/KeyframeGraph.h"
#include "vid_orbslam3/KeyframeStamped.h"
#include "vid_orbslam3/TrajectoryPoints.h"


/**
 * \brief GPSKeyframe is a data structure used to hold relevant data of a Keyframe. 
 * */
struct GPSKeyframe{
	/*@{*/
	tf::Vector3 ENU; /** ENU position of the keyframe **/ 
	sensor_msgs::NavSatFix gpsCoord; /** GPS coordinates of the keyframe in WGS84 format**/ 
	vid_orbslam3::KeyframeStamped KF; /** Actual keyframe message published by vid_orbslam node **/ 
	/*@{*/
};

/**
 * \brief GPSProcess is a class which is used to instantiate geo-referencing node
 * */
class GPSProcess{

private:
	/**
	 * Data member to for accessing handling the external node
	 * */
	ros::NodeHandle nh;
	/**
	 * Earth object instantiated with WGS84 earth format
	 * */
	const GeographicLib::Geocentric& earth = GeographicLib::Geocentric::WGS84();
	/**
	 * Vector conatiner which stores all the positions of the drone as gps coordinates.
	 * */
	std::vector<sensor_msgs::NavSatFix> AllPositionsOfDroneInWGS84;
	/**
	 * Vector container used to store current GPS stamped Keyframes. Uses the 'GPSKeyframe' structure format
	 * */
	std::vector<GPSKeyframe> gpsStampedKeyframeList;
	/**
	 * Starting position of the drone in gps coordinates (WGS84) 
	 * */
	sensor_msgs::NavSatFix initCoord;
	/**
	 * Current T_SLAM_2_ENU matrix calculated from 3-KFs windowed approach 
	 * */
	Eigen::Matrix4d T_SLAM_2_ENU;
	/**
	 * T_ENU_2_ECEF matrix used for publishing static TF between ENU and ECEF frames
	 * */
	Eigen::Matrix4d T_ENU_2_ECEF;
	/**
	 * ROS publisher for KeyframegraphInENU (PoseArray) 
	 * */
    ros::Publisher KeyframegraphInENUPublisher;
	/**
	 * Previous KF list count
	 * */
	int previousSizeOfKFList;
	/**
	 * Vector container to store all the slam2enu converted trajectory points. Format: 'Eigen::Vector3d'
	 * */
	std::vector<Eigen::Vector3d> convertedTrajectoryList;
	/**
	 * A Pose array which accumulates all the converted slam2enu KFs (Orientation and Translation), ready for publishing. 
	 * */
	geometry_msgs::PoseArray posearray;
		
	
	//flags
	/**
	 * A flag to check whether the initial ground coordinate is initialized or not. 
	 * */
	bool IsGroundCoordinateInitialized;

public:
	/**
	 * \brief Default constructor, deleted. Always GPSProcess Object has to be instantiated 
	 * with paratemers. Use Paramerterized constructor
	 * */
	GPSProcess() = delete;
	/**
	 * \brief Parameterised constructor used to instanciate the GPSProcess Object.
	 * \param nh node handle argument, node object from the main flow is passed as parameter.
	 * */
	GPSProcess(ros::NodeHandle& nh);
	
	//void ConvertGPSDataToECEFandENU(sensor_msgs::NavSatFix& gps_msg);

	/**
	 * \brief gpsCallback is a method callback to a gps topic subscriber. 
	 * Stores every position of the drone as gps coordinates (WGS84) into AllPositionsOfDroneInWGS84
	 * \param gps_msg is a constant gps pointer of the type sensor_msgs::NavSatFixConstPtr
	 * \return void
	 * */
	void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg);
	/**
	 * \brief kfGraphCallback is a method callback to a keyframe graph topic subscriber. 
	 * \param kfg is a constant Keyframe graph pointer of the message type vid_orbslam3::KeyframeGraphConstPtr
	 * \return void
	 * */
	void kfGraphCallback(const vid_orbslam3::KeyframeGraphConstPtr& kfg);
	/**
	 * \brief setENUSystemCoordinates method is used to fix the ENU coordinate system at the beginning of the run.
	 * \return bool
	 * */
	bool setENUSystemCoordinates();
	/**
	 * \brief getSimilarityTransform method is used to calculate similarity transform between 2 different coordinate systems.
	 * Uses robust alignment to calculate the similarity. Only works if the input vectors has 2 or more elements.
	 * \param t1 Vector of points in source coordinate system, holds elements of the type Eigen::Vector3d
	 * \param t2 Vector of points in destination coordinate system, holds elements of the type Eigen::Vector3d
	 * \return A 4x4 Tranformation matrix T_src2dst of the type Eigen::Matrix4d 
	 * */
	Eigen::Matrix4d getSimilarityTransform(std::vector<Eigen::Vector3d>& t1, std::vector<Eigen::Vector3d>& t2);
	
  	//void setupPoseArrayENU(GPSKeyframe& ele);

	/**
	 * \brief publish_T_ENU2ECEF method calculates and publishes a static TF T_ENU_2_ECEF
	 * \return void
	 * */  
  	void publish_T_ENU2ECEF();
	/**
	 * \brief savePositionsOfKFInENU method writes out a text file containing all the positions of KF / Drone in ENU system.
	 * \param filename Absolute file path to write the values. Ex: /<-path->/<-filename->.txt 
	 * */
 	void savePositionsOfKFInENU(std::string filename);
	 /**
	 * \brief saveTrajectoryInENU method writes out a text file containing all the trajectory points of KF in ENU system.
	 * \param filename Absolute file path to write the values. Ex: /<-path->/<-filename->.txt 
	 * */
	void saveTrajectoryInENU(std::string filename);
	/**
	 * \brief insertIntoPoseArray method stores the poses into PoseArray.
	 * \param q orientation of the Keyframe in ENU, bearing the type Eigen::Quaterniond
	 * \param t translation of the Keyframe in ENU, bearing the type Eigen::Vector4d i.e (x, y, z, 1)
	 * \return void
	 * */
	void insertIntoPoseArray(Eigen::Quaterniond& q, Eigen::Vector4d& t); 
	

};
