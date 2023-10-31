float convert_ms_to_RPM(float ms) {
  float inRPM = (60 * ms) / wheel_perimeter;
  return inRPM;
}

float convert_RPM_to_ms(float rpm) {
  float inms = (wheel_perimeter * rpm) / 60;
  return inms;
}



void giveSpeedtoAGV() {
  digitalWrite(m1_nSleep, HIGH);
  digitalWrite(m2_nSleep, HIGH);
  getSpeed();
  calSpeedOfTwoWheel();
  PIDWheel();
  calActualSpeeds();

}

void calSpeedOfTwoWheel() {
  float leftSpeed_ms, rightSpeed_ms;

  leftSpeed_ms = linearSpeed - (angularSpeed * left_wheel_distance);
  rightSpeed_ms = linearSpeed + (angularSpeed * right_wheel_distance);

  w_speed_des[0] = convert_ms_to_RPM(leftSpeed_ms);
  w_speed_des[1] = convert_ms_to_RPM(rightSpeed_ms);

}

void calActualSpeeds() {
  linearSpeed_act = (convert_RPM_to_ms(w_speed_act[1]) + convert_RPM_to_ms(w_speed_act[0])) / 2 ;
  angularSpeed_act = (convert_RPM_to_ms(w_speed_act[1]) - convert_RPM_to_ms(w_speed_act[0])) / wheel_length;
}
void update_cmd_vel_act() {
  cmd_vel_act_msg.linear.x = linearSpeed_act;
  //cmd_vel_act_msg.linear.y = w_speed_act[0];
  //cmd_vel_act_msg.linear.z = w_speed_act[1];
  cmd_vel_act_msg.angular.z = angularSpeed_act;
  cmd_vel_act_pub.publish(&cmd_vel_act_msg);
}



double errorl = 0.0, errorr = 0.0, cumerrorl = 0.0, cumerrorr = 0.0, elapsedTime = 0.0;
//burada input w_speed_act, setpoint w_speed_des, output ise ades oluyor
unsigned long currentTime_pid = 0, previousTime_pid = 0;

// Check windup state
bool windup_state_l,
     windup_state_r;

void PIDWheel() {

  errorl = w_speed_des[0] - w_speed_act[0]; //in RPM
  errorr = w_speed_des[1] - w_speed_act[1];

  //////////////////////////////////////////////////
  //  cumerrorl += errorl;
  //  cumerrorr += errorr;
  //////////////////////////////////////////////////
  // Anti-Windup
  windup_state_l = (ceil(ades_l) != ceil(saturated_ades_l)) && (sameSign(errorl, ades_l));
  windup_state_r = (ceil(ades_r) != ceil(saturated_ades_r)) && (sameSign(errorr, ades_r));

  if (!windup_state_l) {
    cumerrorl += errorl;
  }
  if (!windup_state_r) {
    cumerrorr += errorr;
  }
  //////////////////////////////////////////////////
  //////////////////////////////////////////////////




  // Calculate proper PWM duty cycle
  ades_l = (speedl_kp * errorl + speedl_ki * cumerrorl);  // in PWM 
  ades_r = (speedr_kp * errorr + speedr_ki * cumerrorr);


  /////////////////////////////////////////////////
  //  Burada hesaplanan ades değerlerine göre
  //  motorların dönüş yönü setMotorDir()
  //  fonksiyonu ile ayarlanıyor.
  /////////////////////////////////////////////////
  if (ades_l >= 0) {
    setMotorDir(0, false);
  }
  else
  {
    setMotorDir(0, true);
  }
  if (ades_r >= 0) {
    setMotorDir(1, true);
  }
  else
  {
    setMotorDir(1, false);
  }

  saturated_ades_l = saturationCheck(ades_l);
  saturated_ades_r = saturationCheck(ades_r);

  if ( (w_speed_des[0] == 0 && w_speed_des[1] == 0) || agv_stopped) {
    saturated_ades_r = 0.0;
    saturated_ades_l = 0.0;
    cumerrorl = 0.0;
    cumerrorr = 0.0;
    w_speed_des[0] = 0.0;
    w_speed_des[1] = 0.0;
  }

  analogWrite(m1_in2, abs(saturated_ades_l));
  analogWrite(m2_in2, abs(saturated_ades_r));

}

float saturationCheck(float ctrl_pwm) {
  float saturated_ctrl_pwm = 0.0;
  if (abs(ctrl_pwm) >= 255.0) {
    if (ctrl_pwm > 0) saturated_ctrl_pwm = 255.0;
    if (ctrl_pwm < 0) saturated_ctrl_pwm = -255.0;
  }
  else {
    saturated_ctrl_pwm = (float)ctrl_pwm;
  }

  return saturated_ctrl_pwm;
}

boolean sameSign(float a, float b) {
  bool x = ((a > 0) - (a < 0)) == ((b > 0) - (b < 0));
  return x;
}
