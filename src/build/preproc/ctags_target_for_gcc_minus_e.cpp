# 1 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\MECH3907BallShooter_Code\\sketch.ino"





int lastValue = 0;
void setup() {
  Serial.begin(9600);
  pinMode(9, 0x1);

}

void loop() {
  int rawxValue = analogRead(A0);

  int SpeedValue = rawxValue-572;
  if (SpeedValue >= -10 && SpeedValue <= 10){
    SpeedValue = 0;
      }
  else if (SpeedValue > 0){
    SpeedValue = SpeedValue * 255 / 452;
    }
  else if (SpeedValue < 0){
    SpeedValue = SpeedValue * 255 / 564;
    }

  Serial.print("Speed Value: ");
  Serial.print(SpeedValue);
  Serial.print("\n");

  if (SpeedValue == 0){
    digitalWrite(7, 0);
    digitalWrite(8, 0);
    digitalWrite(9, 0);
  }
  else {
    digitalWrite(7, (SpeedValue > 0));
    digitalWrite(8, !(SpeedValue > 0));
    analogWrite(9, SpeedValue);
  }



}
