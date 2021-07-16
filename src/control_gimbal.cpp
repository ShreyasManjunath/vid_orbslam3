#include <ros/ros.h>
#include <dji_osdk_ros/GimbalAction.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <dji_osdk_ros/common_type.h>
#include <iostream>
#include <cmath>


using namespace dji_osdk_ros;

class GimbalService{
	
	private:
		ros::NodeHandle nh;
		ros::ServiceClient gimbal_control_client;
	public:
		GimbalService() = delete;
		GimbalService(std::string gimbal_task_control, ros::NodeHandle& nh);
		void controlGimbal(const sensor_msgs::Imu::ConstPtr& msg);
		void sendCommandToGimbal(double pitch, double roll, double yaw);
		double convertRadToDeg(double angle_in_rad);
	
	
	
};



int main(int argc, char** args){

	ros::init(argc, args, "control_gimbal_node");
	ros::NodeHandle nh;
	GimbalService gimbalService("gimbal_task_control", nh);
	ros::Subscriber subImu = nh.subscribe("/dji_osdk_ros/imu", 400, &GimbalService::controlGimbal, &gimbalService);
	
	ros::spin();
	return 0; 

}

/**
* Constructor for initializing the gimbal control client.
*/

GimbalService::GimbalService(std::string gimbal_task_control, ros::NodeHandle& nh):nh(nh){
	this->gimbal_control_client = nh.serviceClient<GimbalAction>(gimbal_task_control);
	
}

/**
* \brief controlGimbal - Callback method for IMU subscriber
* \param msg - ConstPtr to sensor_msgs::Imu messages
* \return void
*/

void GimbalService::controlGimbal(const sensor_msgs::Imu::ConstPtr& msg){

	double pitch, pitch_deg;
	double roll, roll_deg;
	double yaw, yaw_deg;
	
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	tf::Matrix3x3 m(q);
	
	m.getRPY(roll, pitch, yaw); // Roll - X  | Pitch - Y | Yaw - Z (Standard format)

	pitch_deg = convertRadToDeg(pitch);
	roll_deg = convertRadToDeg(roll);
	yaw_deg = convertRadToDeg(yaw);

	std::cout << "roll: " << roll_deg << "  pitch: "<<pitch_deg << "  yaw: " << yaw_deg << std::endl;
	sendCommandToGimbal(pitch_deg, roll_deg, yaw_deg);


}

/**
* \brief sendCommandToGimbal - publishes commands to gimbal to fix the gimbal in one position.
* \param pitch - rotation along X axis 
* \param roll - rotation along Y axis
* \param yaw - rotation along Z axis 
* \return void 
*/

void GimbalService::sendCommandToGimbal(double pitch, double roll, double yaw){
	
	// DJI gimbal has different different rotation format in terms of ROLL, PITCH and YAW.
	// X -> Pitch , Y -> Roll, Z -> Yaw

		GimbalAction gimbalAction;
        gimbalAction.request.is_reset = false;
        gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        gimbalAction.request.rotationMode = 1; // free mode = 1
        gimbalAction.request.pitch = pitch;
        gimbalAction.request.roll = roll;
        gimbalAction.request.yaw = yaw;
        gimbalAction.request.time = 0.5;
        this->gimbal_control_client.call(gimbalAction);


}

double GimbalService::convertRadToDeg(double angle_in_rad){

	double angle_in_deg;
	angle_in_deg = angle_in_rad * 180.0/M_PI;
	return angle_in_deg;

}