# altinay

17 - 21 Temmuz 2023 
--------------------
Aceinna OpenIMU 300ZI 
Data Sheet: https://navview.blob.core.windows.net/web-resources/6020-3885-01_G%20OpenIMU300ZI%20Datasheet.pdf?_t=1606313626280
1. ROS OpenIMU package
ros_openimu package publishs "/imu_acc_ar" and "/imu_mag" topics
`/imu_acc_ar` topic -> geometry_msgs/Vector3 angular_velocity (rad/s) and geometry_msgs/Vector3 linear_acceleration (m/s^2?)
`/imu_mag` topic -> geometry_msgs/Vector3 magnetic_field

2. ROS imu-tools package
IMU-related filters and visualizers. 
`imu_filter_madgwick`: a filter that fuses angular velocities, accelerations, and (optionally) magnetic readings from a generic IMU device into an `orientation`.
`imu_complementary_filter`: a filter that fuses angular velocities, accelerations, and (optionally) magnetic readings from a generic IMU device into an orientation quaternion using a novel approach based on a complementary fusion.
`rviz_imu_plugin` a plugin for rviz which displays `sensor_msgs::Imu` messages

3. Position and Orientation from IMU Accelerometer and Gyroscope 
We can calculate the linear position of the IMU in 3D from the linear acceleration raw data instantly published by the IMU.

Numerical integration is defined as the approximation of the area under the curve which can be found by dividing the area up into rectangles and then summing the contribution from all the rectangles. --> trapezoidal rule.

In python, there is ‘scipy’ package which has an implementation of the trapezoidal numerical integration to get approximate position and rotation of the IMU by integration of data.
scipy.integrate.cumtrapz(y[, x, dx, axis, initial]) --> Cumulatively integrate y(x) using the composite trapezoidal rule.

*The problem when using this methos (trapezoidal rule)

position in x,y,z is cumulatively calculated. these values are constantly increasing even if the imu is not moving  

4. Open source similar applicaiton (IMU calibration)
The same result was obtained in another open source code trial using the same method.

https://makersportal.com/blog/calibration-of-an-inertial-measurement-unit-imu-with-raspberry-pi-part-ii#gyro

5. Kalman Filter
Info: The aim of a KF is to estimate a state (a vector of time varying quantities) given the data from one or more sensors and the knowledge of a process model/system dynamics ("how" the system is moving). 

https://github.com/xaedes/ROS-Kalman-Filter-for-IMU
