/*void publishSensorStateMsg(void)  //frame_id yazılacak
{
  bool dxl_comm_result = false;

  sensor_state_msg.header.stamp = nh.now();
  //sensor_state_msg.battery = sensors.checkVoltage();

  
  sensor_state_msg.left_encoder = Tire_l.read();
  sensor_state_msg.right_encoder = Tire_r.read();

  
  sensor_state_pub.publish(&sensor_state_msg);
}*/
