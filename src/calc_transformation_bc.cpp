/**
*\author Shreyas Manjunath
*\brief Calculates Transoformation from body frame to camera frame
*
*/

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <ImuTypes.h>
#include "ImuGrabber.h"
#include <queue>
#include <mutex>
#include <thread>
#include <iostream>
#include <geometry_msgs/Quaternion.h>
#include <chrono>


class GimbalGrabber
{
public:
	GimbalGrabber(ImuGrabber* pImuGb):mpImuGb(pImuGb){};
	void GrabGimbalAngle(const geometry_msgs::Vector3Stamped& msg);
	void SyncImuAndGimbal();	


	std::queue<geometry_msgs::Vector3Stamped> gimbalBuf;
	std::mutex mBufMutex;
	ImuGrabber* mpImuGb;

	~GimbalGrabber(); 
	
};


int main(int argc, char** args){
	ros::init(argc, args, "calc_transformation_bc_node");
	ros::NodeHandle nh;
	ImuGrabber imugb;
	GimbalGrabber gimbalgb(&imugb);	
	ros::Subscriber subImu = nh.subscribe("/dji_osdk_ros/imu", 4000, &ImuGrabber::GrabImu, &imugb); // 400 Hz 
	ros::Subscriber subGimbalAngle = nh.subscribe("/dji_osdk_ros/gimbal_angle", 500,&GimbalGrabber::GrabGimbalAngle, &gimbalgb); // 50 Hz

	std::thread sync_threads(&GimbalGrabber::SyncImuAndGimbal, &gimbalgb);
	sync_threads.detach();	 
	ros::spin();
	return 0;
}

void GimbalGrabber::GrabGimbalAngle(const geometry_msgs::Vector3Stamped& msg)
{
  mBufMutex.lock();
  gimbalBuf.push(msg);
  mBufMutex.unlock();
  return;
}

void GimbalGrabber::SyncImuAndGimbal(){

	while(ros::ok())
	{

		double time = 0;
		if(!gimbalBuf.empty() and !mpImuGb->imuBuf.empty())
		{
			time = gimbalBuf.back().header.stamp.toSec();
			if(time > mpImuGb->imuBuf.back().header.stamp.toSec())
				continue;
			
			{
                                geometry_msgs::Vector3 gimbal_current(gimbalBuf.back().vector);
				geometry_msgs::Quaternion imuOrientation_current(mpImuGb->imuBuf.back().orientation);
				
				std::cout << "[this_node] IMU : " <<  imuOrientation_current 
				<< "GimbalAngle : " << gimbal_current << std::endl;
				 
				mpImuGb->imuBuf.pop();
				gimbalBuf.pop();

			}
				
		}
	  std::chrono::milliseconds tSleep(1);
  	  std::this_thread::sleep_for(tSleep);	
	  if(!ros::ok())
		return;
	}

}

GimbalGrabber::~GimbalGrabber()
{

	delete mpImuGb;

}

