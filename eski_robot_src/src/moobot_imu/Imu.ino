///////////////////////////////////////////////////////////////////////////////
//sensor_msgs/Imu.h
///////////////////////////////////////////////////////////////////////////////
//Header header
//
//geometry_msgs/Quaternion orientation
//float64[9] orientation_covariance # Row major about x, y, z axes
//
//geometry_msgs/Vector3 angular_velocity
//float64[9] angular_velocity_covariance # Row major about x, y, z axes
//
//geometry_msgs/Vector3 linear_acceleration
//float64[9] linear_acceleration_covariance # Row major x, y z



sensors_event_t  angVelocityData , linearAccelData;
imu::Quaternion orientationData;
void updateIMU() {
  
  orientationData = bno.getQuat();
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  publishHeader();
  publishOrientation();
  publishAngVel();
  publishLinearAccel();
  imu_pub.publish(&imu_msg);
 
}
void publishHeader(){
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "/imu_link";
}
void publishOrientation(){

  imu_msg.orientation.w = orientationData.w();
  imu_msg.orientation.x = imu_msg.orientation_covariance[0] = orientationData.x();
  imu_msg.orientation.y = imu_msg.orientation_covariance[4] = orientationData.y();
  imu_msg.orientation.z = imu_msg.orientation_covariance[8] = orientationData.z();

  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;
  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[5] = 0;
  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
}

void publishAngVel(){
  imu_msg.angular_velocity.x = imu_msg.angular_velocity_covariance[0] = angVelocityData.gyro.x;
  imu_msg.angular_velocity.y = imu_msg.angular_velocity_covariance[4] = angVelocityData.gyro.y;
  imu_msg.angular_velocity.z = imu_msg.angular_velocity_covariance[8] = angVelocityData.gyro.z;

  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;
  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[5] = 0;
  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
}

void publishLinearAccel(){

  imu_msg.linear_acceleration.x = imu_msg.linear_acceleration_covariance[0] = linearAccelData.acceleration.x;
  imu_msg.linear_acceleration.y = imu_msg.linear_acceleration_covariance[4] = linearAccelData.acceleration.y;
  imu_msg.linear_acceleration.z = imu_msg.linear_acceleration_covariance[8] = linearAccelData.acceleration.z;

  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;
  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[5] = 0;
  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
}
