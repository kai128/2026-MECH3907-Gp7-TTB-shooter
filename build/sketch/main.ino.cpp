#include <Arduino.h>
#line 1 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
#define DebugMode
#define IRSensorDistance 200 // Distance between two IR in mm
#define LoaderMovingDistance 73 // Distance for the loader to move when loading in mm
#define NumberOfMotor 3
#include <stdlib.h>
#include <Wire.h>

// Define the usage for Pins
#define TriggerPin 3
#define PitchDirPin 4
#define PitchStpPin 5
#define LoadDirPin 6
#define LoadStpPin 7
#define GreenLEDPin 8
#define TopMotorPin 9
#define BLMotorPin 10
#define BRMotorPin 11
#define IR1Pin 12
#define IR2Pin 13


#define TCA9548A_ADDR  0x70   // Default I2C address of TCA9548A
#define AS5600_ADDR     0x36  // Fixed I2C address of AS5600

#define ANGLE_HIGH_REG  0x0C  // AS5600 angle register (high byte 0x0C, low byte 0x0D)

#define SAMPLE_INTERVAL 10 // Sampling interval in milliseconds

// ======================== PID Parameters ========================
float Kp = 2.8;
float Ki = 0.17;
float Kd = -0.05;
// ================================================================

float targetRPS[3] = {0, 0, 0};
float dt[3];

struct Encoder {
  float lastAngle;       // Previous raw angle (0..4095)
  unsigned long lastTime;       // Previous timestamp (ms)
  float rps;                    // Last computed RPS
  float avg={0};
};
Encoder encoderData[3];



struct PID {
  float integral;
  float prevError;
};
PID pidData[3];

float getEncoderData();
void selectTCAChannel(uint8_t channel);
uint16_t readAS5600Angle();

float computePID(uint8_t motorIndex, float setpoint, float measurement, float dt);
void setMotorSpeed(uint8_t motorIndex, float pwmValue);
void configPins(float dt);





#line 66 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void setup();
#line 99 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void loop();
#line 277 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void selectTCAChannel(int channel);
#line 344 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void setMotorSpeed();
#line 355 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void configPins();
#line 66 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void setup() {
  Serial.begin(115200);
  while (!Serial);
  configPins();
  Wire.begin();

  for (int ch = 0; ch < NumberOfMotor; ch++) {
    selectTCAChannel(ch);
    delay(10);
    encoderData[ch].lastAngle = readAS5600Angle();
    encoderData[ch].lastTime = millis();
    encoderData[ch].rps = 0.0;

    pidData[ch].integral = 0.0;
    pidData[ch].prevError = 0.0;
  }

  // Setup motor rps using Target ball speed and rps;
  float TargetBallSpeed = 100;
  float TargetBallRPS = 50;
  float TopRPS = (TargetBallSpeed * 2 / 0.25 + TargetBallRPS * (0.04 * 3.14) / 0.25) / 2;
  float BottomRPS = (TargetBallSpeed * 2 / 0.25 - TargetBallRPS * (0.04 * 3.14) / 0.25) / 2;
  targetRPS[0] = TopRPS;
  targetRPS[1] = BottomRPS * 0.8;
  targetRPS[2] = BottomRPS * 1.2;
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print("targetRPS");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(targetRPS[i]);
  }
}

void loop() {
  uint8_t datainput = 0;
  static bool loaderEnable = false;
  static bool loaderUp = false;
  static uint16_t loaderPos = 0;
  
  // digitalRead(TriggerPin);
  if (Serial.available() > 0) {
    datainput = Serial.read();
    if (datainput == 0xFF) {
      loaderEnable = true;
    }
    datainput = 0;
  }

  static unsigned long lastTimeChange = 0;
  unsigned long now = millis();
  if (now - lastTimeChange >= 10 * 1000) {
    lastTimeChange = now;
    loaderEnable = true;
  }

  if (loaderEnable) {
    if (!loaderUp) {
      digitalWrite(LoadDirPin, 1);
      digitalWrite(LoadStpPin, HIGH);
      delay(1);
      digitalWrite(LoadStpPin, LOW);
      loaderPos++;
      //Serial.println(loaderPos);
    }
    else {
      digitalWrite(LoadDirPin, 0);
      digitalWrite(LoadStpPin, HIGH);
      delay(1);          
      digitalWrite(LoadStpPin, LOW);
      loaderPos--;
    }
    
    if (loaderPos >= 50 * LoaderMovingDistance || loaderPos <= 0) {
      loaderUp = !loaderUp;
      loaderEnable = false;
    }
  }
/*
  if (loaderEnable) {
    if (!loaderUp) {
      digitalWrite(PitchDirPin, 1);
      digitalWrite(PitchStpPin, HIGH);
      delay(1);
      digitalWrite(PitchStpPin, LOW);
      loaderPos++;
      //Serial.println(loaderPos);
    }
    else {
      digitalWrite(PitchDirPin, 0);
      digitalWrite(PitchStpPin, HIGH);
      delay(1);          
      digitalWrite(PitchStpPin, LOW);
      loaderPos--;
    }
    
    if (loaderPos >= 50 * LoaderMovingDistance || loaderPos <= 0) {
      loaderUp = !loaderUp;
      loaderEnable = false;
    }
  }*/
    

  getEncoderData();
  setMotorSpeed();
  /*
  if (digitalRead(TriggerPin)) {
    targetRPS[0] = 80;
  }else {
    targetRPS[0] = 0;
  }*/
  /*
  static unsigned long IR1Time, IR2Time;

  // Loader - load TTB when trigger was pressed then return
  static bool loaderUp = false;
  if (digitalRead(TriggerPin)) {
    static uint16_t i;
    if (!loaderUp) {
      digitalWrite(LoadDirPin, 0);
      
      for (i = 0; i < LoaderMovingDistance * 50; i++) {
        if (!digitalRead(IR1Pin)) {
          IR1Time = millis();
          break;
        }
        digitalWrite(LoadStpPin, 0);
        delay(10);
      }
    }
    else {
      digitalWrite(LoadDirPin, 1);
      for (; i >= 0; i--) {
        digitalWrite(LoadStpPin, 0);
        delay(10);
      }
    }
    loaderUp = !loaderUp;
  }

  // IR sensor for meassuring relation between init V. and RPS
  if (!digitalRead(IR1Pin)) IR1Time = millis();
  if (!digitalRead(IR2Pin)) IR2Time = millis();
  Serial.print("Ball_init_velocity:");
  if (IR2Time > IR1Time)  Serial.println(IRSensorDistance / (IR2Time - IR1Time));
  */
}

float getEncoderData() {
  static unsigned long lastPrint = 0;
  static unsigned long lastPrint2 = 0;
  unsigned long now = millis();
  if (now - lastPrint2 >= 90) {

    lastPrint2 = now;
    char RPSString[] = "> avg1:";
    for (int i = 0; i < NumberOfMotor; i++) {
      RPSString[5] = i + '1';
      Serial.print(RPSString);
      Serial.print(encoderData[i].avg, 2);
      RPSString[0] = ',';
    }
    Serial.println();
    
  }
  if (now - lastPrint >= SAMPLE_INTERVAL) {
    lastPrint = now;
    for (int ch = 0; ch < NumberOfMotor; ch++) {
      // Select the desired channel on the TCA9548A
      selectTCAChannel(ch);
      //delay(2);                  // Short delay to let the mux settle

      // Read the angle from the AS5600 on the currently active channel
      uint16_t rawAngle = readAS5600Angle();


      // Calculate time difference (seconds)
      unsigned long dt_ms = now - encoderData[ch].lastTime;
      float dt_s = dt_ms / 1000.0;

      // Convert raw 12‑bit value to degrees (0.0 – 360.0)
      float rev = rawAngle / 4096.0;
      
      // Handle wrap-around
      if (encoderData[ch].lastAngle - rev > 0.001) {
        encoderData[ch].lastAngle -= 1;
      }
      float delta_rev = rev - encoderData[ch].lastAngle;

      // Compute RPS
      // Tons of magic numbers of signal filtering 
      float accel = (delta_rev / dt_s - encoderData[ch].rps) / dt_s;
      if (dt_s > (SAMPLE_INTERVAL + 0.1) / 2000.0 && delta_rev / dt_s < 110 && delta_rev > 0.07 && abs(accel) < 8000)  {
        encoderData[ch].rps = delta_rev / dt_s;
      }else {encoderData[ch].rps -= (encoderData[ch].rps > 0)? 1 : 0;}

      // Low-pass filter
      encoderData[ch].avg = (encoderData[ch].rps > 0)? (1-0.1) * encoderData[ch].avg + (0.1) * encoderData[ch].rps : 0;

      // Store current values for next iteration
      encoderData[ch].lastAngle = rev;
      encoderData[ch].lastTime = now;
      dt[ch] = dt_s;
    }
  }
}


/*
 * Select a channel on the TCA9548A.
 * channel: 0..7 for the eight possible channels.
 */
void selectTCAChannel(int channel) {
  if (channel > 7) return;                     // Safety check
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0x01 << channel);  // Send channel select byte
  //Serial.println(0x01 << channel);                   
  Wire.endTransmission();
}

/*
 * Read the 12‑bit angle from the AS5600.
 * Returns a value between 0 and 4095.
 */
uint16_t readAS5600Angle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(ANGLE_HIGH_REG);                   // Point to the high byte of the angle
  if (Wire.endTransmission(false) != 0) {       // Send repeated start
    Serial.println("Error communicating with AS5600");
    return 0;
  }

  // Request 2 bytes from the AS5600
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() >= 2) {
    uint8_t highByte = Wire.read();              // High byte (bits 11..8)
    uint8_t lowByte  = Wire.read();              // Low byte  (bits 7..0)
    return ( (highByte << 8) | lowByte ) & 0x0FFF; // Mask to 12 bits
  } else {
    Serial.println("Failed to read enough bytes from AS5600");
    return 0;
  }
}

float computePID(uint8_t motorIndex, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;

  // Proportional term
  float P = Kp * error;

  // Integral term (with anti-windup: clamp integral)
  pidData[motorIndex].integral += error * dt;
  // Simple integral clamping – adjust limits as needed
  float maxIntegral = 255.0 / (Ki + 0.001);   // Rough limit based on max PWM
  if (pidData[motorIndex].integral > maxIntegral) pidData[motorIndex].integral = maxIntegral;
  if (pidData[motorIndex].integral < -maxIntegral) pidData[motorIndex].integral = -maxIntegral;
  float I = Ki * pidData[motorIndex].integral;

  // Derivative term (on measurement to avoid derivative kick)
  float D = Kd * ( (measurement - (setpoint - error)) / dt );  // Actually we need previous measurement

  D = Kd * (error - pidData[motorIndex].prevError) / dt;
  pidData[motorIndex].prevError = error;

  // Total output
  float output = P + I + D;

  // Constrain output to PWM range (0-255)
  // output < 15 for dead zone
  if (output < 15) output = 0;
  if (output > 255) output = 255;

  return output;
}





void setMotorSpeed() {
  for (uint8_t ch = 0; ch < NumberOfMotor; ch++) {
    float pwmValue = computePID(ch, targetRPS[ch], encoderData[ch].avg, dt[ch]);
    //Serial.println(pwmValue);
    analogWrite(ch + TopMotorPin, (int) pwmValue);
    //analogWrite(ch + TopMotorPin, (int) 255);
  }
}



void configPins() {
  pinMode(TriggerPin, INPUT);

  pinMode(PitchDirPin, OUTPUT);
  pinMode(PitchStpPin, OUTPUT);

  pinMode(LoadDirPin, OUTPUT);
  pinMode(LoadStpPin, OUTPUT);

  pinMode(GreenLEDPin, OUTPUT);

  pinMode(TopMotorPin, OUTPUT);
  pinMode(BLMotorPin, OUTPUT);
  pinMode(BRMotorPin, OUTPUT);

  pinMode(IR1Pin, INPUT);
  pinMode(IR2Pin, INPUT);
}
