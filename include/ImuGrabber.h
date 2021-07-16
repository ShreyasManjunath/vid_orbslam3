/**
* IMU data grabber class
*/

#include <sensor_msgs/Imu.h>
#include <queue>
#include <mutex>

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::Imu& imu_msg);

    std::queue<sensor_msgs::Imu> imuBuf;
    std::mutex mBufMutex;
};

