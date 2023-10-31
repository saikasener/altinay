void resetEncoders() {
  Tire_l.write(0);
  Tire_r.write(0);
}




unsigned long vel_elapsed_time;
long     tire_new_l,
         tire_new_r;
double w_speed_act_new[2];

////////////////////////////////////////////////////////////////////////////
/// This functions calculates actual wheel speed for left and right wheels.
///////////////////////////////////////////////////////////////////////

void getSpeed() {
  velTimer_new = millis();
  vel_elapsed_time = velTimer_new - velTimer_old;


  tire_new_l = Tire_l.read();
  tire_new_r = Tire_r.read();

  long new_timer_motor_speed = millis();

  // Burada ms 'yi 1000 ile çarpıp s cinsine çevirdim
  w_speed_act[0] = (tire_new_l - tire_old_l) * 60000.0 / (new_timer_motor_speed - timer_motor_speed)  / encoder_ppr ;
  w_speed_act[1] = (tire_new_r - tire_old_r) * 60000.0 / (new_timer_motor_speed - timer_motor_speed)  / encoder_ppr ;

  // Keep the old values
  tire_old_l = tire_new_l;
  tire_old_r = tire_new_r;
  timer_motor_speed = new_timer_motor_speed;
  velTimer_old = velTimer_new;

}
//
//unsigned long acc_elapsed_time;
//float new_linearSpeed_act,
//      new_angularSpeed_act,
//      linearAcc_act,
//      angularAcc_act;
//
//void getAcceleration() {
//  accTimer_new = millis();
//  acc_elapsed_time = accTimer_new - accTimer_old;
//  if (acc_elapsed_time > 40) {
//    linearAcc_act = (linearSpeed_act - new_linearSpeed_act) * 1000.0 / acc_elapsed_time;
//    angularAcc_act = (angularSpeed_act - new_angularSpeed_act) * 1000.0 / acc_elapsed_time;
//
//    new_angularSpeed_act = angularSpeed_act;
//    new_linearSpeed_act = linearSpeed_act;
//    accTimer_old = accTimer_new;
//  }
//}
