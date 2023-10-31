void reverseMotors()
{
  //      analogWrite(m1_in2, 0);
  delay(200);
  digitalWrite(m1_in1, !digitalRead(m1_in1));
  digitalWrite(m2_in1, !digitalRead(m2_in1));
}

void setMotorPWM(unsigned int m, unsigned int p) //hız methodu m motor d yön yönsüz
{
  switch (m) {
    case 0:
      analogWrite(m1_in2, p);
      break;
    case 1:
      analogWrite(m2_in2, p);
      break;
  }
}

void setMotorDir(unsigned int m, bool d) //m motor d yön sadece yön için
{
  switch (m) {
    case 0: // 1. motor
      digitalWrite(m1_in1, d);
      break;
    case 1: // 2. motor
      digitalWrite(m2_in1, d);
      break;
  }
}
