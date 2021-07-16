#include "ImuGrabber.h"

void ImuGrabber::GrabImu(const sensor_msgs::Imu& imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}

