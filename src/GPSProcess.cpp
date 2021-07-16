#include "GPSProcess.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <colmap/optim/loransac.h>
#include <colmap/estimators/similarity_transform.h>
#include <colmap/util/math.h>
#include <Eigen/Geometry>

using namespace colmap;

GPSProcess::GPSProcess(ros::NodeHandle& nh):nh{nh}
{
	IsGroundCoordinateInitialized = false;
  	KeyframegraphInENUPublisher = nh.advertise<geometry_msgs::PoseArray> ("/GPSProcess/KeyframegraphInENU", 1);
	previousSizeOfKFList = 0;

}

void GPSProcess::gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg){
	
	sensor_msgs::NavSatFix gps;
	gps.header = gps_msg->header;
	gps.status = gps_msg->status;
	gps.latitude = gps_msg->latitude;
	gps.longitude = gps_msg->longitude;
	gps.altitude = gps_msg->altitude;
	gps.position_covariance = gps_msg->position_covariance;
	
	this->AllPositionsOfDroneInWGS84.push_back(gps);
	if(!IsGroundCoordinateInitialized){
		setENUSystemCoordinates();
		IsGroundCoordinateInitialized = true;
	}
	
}

/*
void GPSProcess::ConvertGPSDataToECEFandENU(sensor_msgs::NavSatFix& gps_msg){
	
	double X,Y,Z;
	this->earth.Forward((double)gps_msg.latitude, (double)gps_msg.longitude, (double)gps_msg.altitude, X, Y, Z);
	//GeographicLib::LocalCartesian LC_ENU(49.015522, 8.425932, 68.2009203, this->earth);
	GeographicLib::LocalCartesian LC_ENU((double)initCoord.latitude, (double)initCoord.longitude, (double)initCoord.altitude, this->earth);
	double X_ENU, Y_ENU, Z_ENU;
	LC_ENU.Forward((double)gps_msg.latitude, (double)gps_msg.longitude, (double)gps_msg.altitude, X_ENU, Y_ENU, Z_ENU);

	
	std::cout << "X: "<< std::setprecision(9) << X_ENU << "  Y: " 
	<< std::setprecision(9) << Y_ENU << "    Z: "<< std::setprecision(9) << Z_ENU << std::endl;

} 
*/

void GPSProcess::kfGraphCallback(const vid_orbslam3::KeyframeGraphConstPtr& kfg){

	vid_orbslam3::KeyframeGraph kfGraph = *kfg;
	int i = 0;
	std::vector<GPSKeyframe> CurrentGpsKFlist;
	std::vector<Eigen::Vector3d> PositionsOfDroneInENU;
	std::vector<Eigen::Vector3d> PositionsOfKFInSLAM;
	std::vector<Eigen::Vector3d> PositionsOfDroneInGC; // GC -> Geo-Centric
	GeographicLib::LocalCartesian LC_ENU((double)initCoord.latitude, (double)initCoord.longitude, 68.2009203, this->earth);

	for(vid_orbslam3::KeyframeStamped& kf : kfGraph.KeyframeList){
		for(sensor_msgs::NavSatFix& gps : AllPositionsOfDroneInWGS84){
			
			// Check if the timestamp duration is less than 2 ms.
			if(kf.Keyframe.header.stamp - gps.header.stamp < ros::Duration(0.2)){
				//std::cout << "Yes!" << std::endl;
				double X_ENU, Y_ENU, Z_ENU;
				// Convert GPS coord to ENU
				LC_ENU.Forward((double)gps.latitude, (double)gps.longitude, (double)gps.altitude, X_ENU, Y_ENU, Z_ENU);

				GPSKeyframe thisFrame;
				// Vector of translation vectors of drone position in ENU frame (for Similarity Transform using colmap)
				PositionsOfDroneInENU.push_back(Eigen::Vector3d(X_ENU, Y_ENU, Z_ENU));
				// Creating a GPSKeyframe which has ENU, WGS84 and Current KF
				thisFrame.ENU = tf::Vector3(X_ENU, Y_ENU, Z_ENU);
				thisFrame.gpsCoord = gps;
				thisFrame.KF = kf;
				
				// Vector of translation vectors of drone position in Geo-Centric frame (for Similarity Transform using colmap)
				double X_gc,Y_gc,Z_gc;
				this->earth.Forward((double)gps.latitude, (double)gps.longitude, (double)gps.altitude, X_gc, Y_gc, Z_gc);
				PositionsOfDroneInGC.push_back(Eigen::Vector3d(X_gc, Y_gc, Z_gc));

				// Vector of translation vectors of all current keyframes required (for Similarity Transform using colmap)
				geometry_msgs::Point t_slam = kf.Keyframe.pose.position;
				PositionsOfKFInSLAM.push_back(Eigen::Vector3d(t_slam.x, t_slam.y, t_slam.z));


				CurrentGpsKFlist.push_back(thisFrame);
				i++;
				//std::cout << i << std::endl;
				break; // Break out of the inner loop once the KF with it's corresponding gps coordinates are found and inserted to a vector
			}
		}
	}


	gpsStampedKeyframeList = CurrentGpsKFlist;

	int currentCount = gpsStampedKeyframeList.size();
	int n = 5; // window size
	int mid = n/2;
	
	
	if(PositionsOfKFInSLAM.size() >= n && currentCount > previousSizeOfKFList){
		

		std::vector<Eigen::Vector3d> windowVectorSLAM = std::vector<Eigen::Vector3d>(PositionsOfKFInSLAM.end() - n , PositionsOfKFInSLAM.end());
		std::vector<Eigen::Vector3d> windowVectorENU = std::vector<Eigen::Vector3d>(PositionsOfDroneInENU.end() - n , PositionsOfDroneInENU.end());
		std::vector<GPSKeyframe> windowVectorKeyframeList = std::vector<GPSKeyframe>(gpsStampedKeyframeList.end() - n , gpsStampedKeyframeList.end());
		this->T_SLAM_2_ENU = getSimilarityTransform(windowVectorSLAM, windowVectorENU);
		Eigen::Vector4d point(windowVectorSLAM.at(mid)[0],windowVectorSLAM.at(mid)[1],windowVectorSLAM.at(mid)[2],1);
		Eigen::Vector4d previousPoint(windowVectorSLAM.at(mid-1)[0],windowVectorSLAM.at(mid-1)[1],windowVectorSLAM.at(mid-1)[2],1);
		Eigen::Vector4d pointInENU = this->T_SLAM_2_ENU * point;
		Eigen::Vector4d previousPointInENU = this->T_SLAM_2_ENU * previousPoint;
		this->convertedTrajectoryList.push_back(Eigen::Vector3d(pointInENU[0],pointInENU[1],pointInENU[2]));

		Eigen::Vector4d diffVector =  previousPointInENU - pointInENU;
		diffVector = diffVector.normalized();
		Eigen::Vector3d diffVector3d(diffVector[0],diffVector[1],diffVector[2]);
		Eigen::Quaterniond q;
		Eigen::Vector3d t(windowVectorSLAM.at(mid)[0],windowVectorSLAM.at(mid)[1],windowVectorSLAM.at(mid)[2]);
		q.x() = windowVectorKeyframeList.at(mid).KF.Keyframe.pose.orientation.x;
		q.y() = windowVectorKeyframeList.at(mid).KF.Keyframe.pose.orientation.y;
		q.z() = windowVectorKeyframeList.at(mid).KF.Keyframe.pose.orientation.z;
		q.w() = windowVectorKeyframeList.at(mid).KF.Keyframe.pose.orientation.w;

		// Previous KF pose
		Eigen::Quaterniond q1;
		Eigen::Vector3d t1(windowVectorSLAM.at(mid-1)[0],windowVectorSLAM.at(mid-1)[1],windowVectorSLAM.at(mid-1)[2]);
		q1.x() = windowVectorKeyframeList.at(mid-1).KF.Keyframe.pose.orientation.x;
		q1.y() = windowVectorKeyframeList.at(mid-1).KF.Keyframe.pose.orientation.y;
		q1.z() = windowVectorKeyframeList.at(mid-1).KF.Keyframe.pose.orientation.z;
		q1.w() = windowVectorKeyframeList.at(mid-1).KF.Keyframe.pose.orientation.w;

		Eigen::Quaterniond direction = q.normalized() * Eigen::Quaterniond(0, 0, 1, 0);

		Eigen::Vector3d q_in_vector(direction.x(), direction.y(), direction.z()); 
		Eigen::Vector3d diff = t1 - t;
		diff = diff.normalized();
		Eigen::Vector3d v = diff.cross(q_in_vector.normalized());
		double sin_th = v.norm();
		double cos_th = diff.dot(q_in_vector.normalized());

		Eigen::Matrix3d skew_symm_v;
		skew_symm_v << 0, -v[2], v[1],
					   v[2], 0, -v[0],
					   -v[1], v[0], 0;

		Eigen::Matrix3d R_tmp = Eigen::Matrix3d::Identity() + skew_symm_v + (skew_symm_v * skew_symm_v) * (1.0/(1.0+cos_th));

				
		// ----------------------------------------------------------//
		Eigen::Matrix3d R_slam2enu;
		double scale = T_SLAM_2_ENU.block<1, 3>(0, 0).norm();
		R_slam2enu << this->T_SLAM_2_ENU(0,0), this->T_SLAM_2_ENU(0,1), this->T_SLAM_2_ENU(0,2),
						this->T_SLAM_2_ENU(1,0), this->T_SLAM_2_ENU(1,1), this->T_SLAM_2_ENU(1,2),
						this->T_SLAM_2_ENU(2,0), this->T_SLAM_2_ENU(2,1), this->T_SLAM_2_ENU(2,2); 
		R_slam2enu = R_slam2enu / scale;
		auto vec_in_ENU = R_tmp * diffVector3d;
		Eigen::Quaterniond orientation_in_ENU;
		orientation_in_ENU = Eigen::AngleAxisd(vec_in_ENU[0], Eigen::Vector3d::UnitX())
							* Eigen::AngleAxisd(vec_in_ENU[1], Eigen::Vector3d::UnitY())
							* Eigen::AngleAxisd(vec_in_ENU[2], Eigen::Vector3d::UnitZ());
		insertIntoPoseArray(orientation_in_ENU, pointInENU);
		//setupPoseArrayENU(windowVectorKeyframeList.at(mid));

		std::cout << vec_in_ENU << std::endl;
		std::cout << "--------------------------------" << std::endl;
	
		posearray.header.stamp = ros::Time::now();
		posearray.header.frame_id = "map";
		KeyframegraphInENUPublisher.publish(posearray);
	}
	previousSizeOfKFList = currentCount;
	publish_T_ENU2ECEF();
	
  

}

bool GPSProcess::setENUSystemCoordinates(){

	initCoord = AllPositionsOfDroneInWGS84.front();
	return true;

}

Eigen::Matrix4d GPSProcess::getSimilarityTransform(std::vector<Eigen::Vector3d>& t1, std::vector<Eigen::Vector3d>& t2){
	// Windowed Robust alignment
	colmap::RANSACOptions ransac_options;
    ransac_options.max_error = 0.1;
    ransac_options.min_inlier_ratio = 0.2;
    ransac_options.confidence = 0.99;
    ransac_options.dyn_num_trials_multiplier = 3.0;

	LORANSAC<SimilarityTransformEstimator<3>, SimilarityTransformEstimator<3>>
      		ransac(ransac_options);
    const auto report = ransac.Estimate(t1, t2);
    SimilarityTransform3 model = SimilarityTransform3(report.model);

    return model.Matrix();

}

void GPSProcess::insertIntoPoseArray(Eigen::Quaterniond& q, Eigen::Vector4d& t){
	geometry_msgs::Point kf_tran_msg;
    geometry_msgs::Quaternion kf_rot_msg;
      
    kf_tran_msg.x = t[0]; 
    kf_tran_msg.y = t[1];
    kf_tran_msg.z = t[2];
      
    kf_rot_msg.x = q.x();
    kf_rot_msg.y = q.y();
    kf_rot_msg.z = q.z();
    kf_rot_msg.w = q.w();
      
    geometry_msgs::Pose kf_pose_enu;
    kf_pose_enu.orientation = kf_rot_msg;
    kf_pose_enu.position = kf_tran_msg;
	posearray.poses.push_back(kf_pose_enu);
}

/*
void GPSProcess::setupPoseArrayENU(GPSKeyframe& ele){
	
   tf::Matrix3x3 tf3d;
   tf3d.setValue(static_cast<double>(this->T_SLAM_2_ENU(0,0)), static_cast<double>(this->T_SLAM_2_ENU(0,1)), static_cast<double>(this->T_SLAM_2_ENU(0,2)), 
        static_cast<double>(this->T_SLAM_2_ENU(1,0)), static_cast<double>(this->T_SLAM_2_ENU(1,1)), static_cast<double>(this->T_SLAM_2_ENU(1,2)), 
        static_cast<double>(this->T_SLAM_2_ENU(2,0)), static_cast<double>(this->T_SLAM_2_ENU(2,1)), static_cast<double>(this->T_SLAM_2_ENU(2,2)));
   
   tf::Quaternion tfqt;
   tf3d.getRotation(tfqt);
       
   tf::Vector3 tran;
   tran.setValue(static_cast<double>(this->T_SLAM_2_ENU(0,3)), static_cast<double>(this->T_SLAM_2_ENU(1,3)), static_cast<double>(this->T_SLAM_2_ENU(2,3)));
   tf::Transform tr_slam2enu;
   tr_slam2enu.setOrigin(tran); 
   tr_slam2enu.setRotation(tfqt);

  
    tf::Quaternion q;
    q.setW(ele.KF.Keyframe.pose.orientation.w);
    q.setX(ele.KF.Keyframe.pose.orientation.x);
    q.setY(ele.KF.Keyframe.pose.orientation.y);
    q.setZ(ele.KF.Keyframe.pose.orientation.z);
      
    tf::Vector3 origin;
    origin.setValue(ele.KF.Keyframe.pose.position.x, ele.KF.Keyframe.pose.position.y, ele.KF.Keyframe.pose.position.z);
      
    tf::Transform transform;
    transform.setOrigin(origin); 
    transform.setRotation(q);
      
    tf::Transform kf_tf_ENU = tr_slam2enu * transform;
    tf::Quaternion kf_rot = kf_tf_ENU.getRotation();
    tf::Vector3 kf_tra =  kf_tf_ENU.getOrigin();
    geometry_msgs::Point kf_tran_msg;
    geometry_msgs::Quaternion kf_rot_msg;
      
    kf_tran_msg.x = kf_tra[0]; 
    kf_tran_msg.y = kf_tra[1];
    kf_tran_msg.z = kf_tra[2];
      
    kf_rot_msg.x = kf_rot.x();
    kf_rot_msg.y = kf_rot.y();
    kf_rot_msg.z = kf_rot.z();
    kf_rot_msg.w = kf_rot.w();
      
    geometry_msgs::Pose kf_pose_enu;
    kf_pose_enu.orientation = kf_rot_msg;
    kf_pose_enu.position = kf_tran_msg;
	posearray.poses.push_back(kf_pose_enu);
}
*/

void GPSProcess::publish_T_ENU2ECEF(){
	
	double X_ENU, Y_ENU, Z_ENU;
	double X_gc,Y_gc,Z_gc;
	GeographicLib::LocalCartesian ENU_Frame((double)initCoord.latitude, (double)initCoord.longitude, 68.2009203, this->earth);
	ENU_Frame.Forward((double)initCoord.latitude, (double)initCoord.longitude, 68.2009203, X_ENU, Y_ENU, Z_ENU);
	this->earth.Forward((double)initCoord.latitude, (double)initCoord.longitude, 68.2009203, X_gc, Y_gc, Z_gc);


	Eigen::Vector3d C_enu(X_gc, Y_gc, Z_gc);
	Eigen::Vector3d r_z =  C_enu.normalized();
	r_z = r_z.normalized();
	Eigen::Vector3d r_x = Eigen::Vector3d(0,0,1).cross(r_z);
	r_x = r_x.normalized();
	Eigen::Vector3d r_y = r_z.cross(r_x);
	r_y = r_y.normalized();

	Eigen::Matrix4d transformation;
	transformation << r_x[0], r_y[0], r_z[0],C_enu[0],
					  r_x[1], r_y[1], r_z[1],C_enu[1],
					  r_x[2], r_y[2], r_z[2],C_enu[2],
					  0.0,	  0.0,    0.0,   1; 

	//std::cout << transformation << std::endl;


	static tf::TransformBroadcaster br;
	
	tf::Matrix3x3 rotMtx;
	rotMtx.setValue(transformation(0,0), transformation(0,1), transformation(0,2),
			transformation(1,0), transformation(1,1), transformation(1,2),
			transformation(2,0), transformation(2,1), transformation(2,2));
	tf::Quaternion rot;
	rotMtx.getRotation(rot);
	tf::Vector3 translation;
	translation.setValue(transformation(0,3),
			transformation(1,3),
			transformation(2,3));

	tf::Transform transform;
	transform.setOrigin(translation);
	transform.setRotation(rot);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "ecef_frame"));
	
}



void GPSProcess::savePositionsOfKFInENU(std::string filename){

	std::ofstream myfile;
	myfile.open(filename);

	for(auto& ele : gpsStampedKeyframeList){
		
		tf::Vector3 vec = ele.ENU;
		myfile << vec[0] << "," << vec[1] << "," << vec[2] << std::endl; 
	}
	myfile.close();
	std::cout << "File was written at " << filename << std::endl;

}

void GPSProcess::saveTrajectoryInENU(std::string filename){
	std::ofstream myfile;
	myfile.open(filename);
	for(auto& vec : convertedTrajectoryList){
		
		myfile << vec[0] << "," << vec[1] << "," << vec[2] << std::endl; 
	}
	myfile.close();
	std::cout << "File was written at " << filename << std::endl;
}




int main(int argc, char** argv){
	
	ros::init(argc, argv, "GPS_node");
	ros::NodeHandle nh;
	std::string GPSTopic;
	nh.getParam("/GPS_node/gps_topic", GPSTopic);
	GPSProcess GPS(nh);
	
	ros::Subscriber gpsSub = nh.subscribe(GPSTopic, 1, &GPSProcess::gpsCallback, &GPS);
	ros::Subscriber kfGraph = nh.subscribe("/KeyframeGraph", 1 ,&GPSProcess::kfGraphCallback, &GPS);
	ros::spin();
	GPS.savePositionsOfKFInENU("/home/manshr/KFPostionsInENU.txt");
	GPS.saveTrajectoryInENU("/home/manshr/converted_SLAM2ENU_gpsnode.txt");
	ros::shutdown(); 

	return 0;
}
