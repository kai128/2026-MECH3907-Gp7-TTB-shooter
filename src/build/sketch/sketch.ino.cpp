#include <Arduino.h>
#line 1 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\MECH3907BallShooter_Code\\sketch.ino"
#define VoltageDevider A0
#define MotorPin 9
#define ForwardPin 7
#define BackwardPin 8

int lastValue = 0;
#line 7 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\MECH3907BallShooter_Code\\sketch.ino"
void setup();
#line 13 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\MECH3907BallShooter_Code\\sketch.ino"
void loop();
#line 7 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\MECH3907BallShooter_Code\\sketch.ino"
void setup() {
  Serial.begin(9600);
  pinMode(MotorPin, OUTPUT);
    
}
 
void loop() {
  int rawxValue = analogRead(VoltageDevider);
  
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
    digitalWrite(ForwardPin, 0);
    digitalWrite(BackwardPin, 0);
    digitalWrite(MotorPin, 0);
  }
  else {
    digitalWrite(ForwardPin, (SpeedValue > 0));
    digitalWrite(BackwardPin, !(SpeedValue > 0));
    analogWrite(MotorPin, SpeedValue);
  }
  
  
  
}
