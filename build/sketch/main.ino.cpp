#include <Arduino.h>
#line 1 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
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

struct Target {
  float speed;
  int angle;
};
Target shootingTarget[2];


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
void setPitchAngle(uint16_t targetAngle);
void calculateSpeed(float TargetBallSpeed);

int pitchAngle = 488; // The angle times 10



#line 74 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void setup();
#line 107 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void loop();
#line 232 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void selectTCAChannel(int channel);
#line 313 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void setMotorSpeed();
#line 322 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void setPitchAngle(int targetAngle);
#line 338 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void configPins();
#line 74 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void setup() {
  delay(2000);
  Serial.begin(115200);
  while (!Serial);
  configPins();
  Wire.begin();
  digitalWrite(GreenLEDPin, 0);
  for (int ch = 0; ch < NumberOfMotor; ch++) {
    selectTCAChannel(ch);
    delay(10);
    Serial.println(ch);
    encoderData[ch].lastAngle = readAS5600Angle();
    
    encoderData[ch].lastTime = millis();
    encoderData[ch].rps = 0.0;

    pidData[ch].integral = 0.0;
    pidData[ch].prevError = 0.0;
  }
  // 2 meter: Top: 450, 10.55; Middle: 190, 10; Bottom: 100, 6.35
  // 4 meter: Top: ; Middle: 200, 14.3; Bottom: 100, 12.83;
  // 6 meter: Top: 300, 18; Middle: 230/225, 17.5; Bottom: 150, 17.5
  // Setup first target para
  shootingTarget[0].angle = 190;
  shootingTarget[0].speed = 10;
  // Setup second target para
  shootingTarget[1].angle = 100;
  shootingTarget[1].speed = 6.35;
  // Setup motor rps using Target ball speed and rps;
  calculateSpeed(shootingTarget[0].speed);
  setPitchAngle(shootingTarget[0].angle);
}

void loop() {
  uint8_t datainput = 0;
  static bool loaderEnable = false;
  static bool loaderUp = false;
  static uint16_t loaderPos = 0;
  static bool targetNum = 0;

  // Check if shooter is ready
  bool shooterReady = true;
  for (uint8_t i = 0; i < NumberOfMotor; i++) {
    if (encoderData[i].rps < targetRPS[i] * 0.90 || encoderData[i].rps > targetRPS[i] * 1.1 ) {
      shooterReady = false;
    }
    
  }
  shooterReady &= !loaderUp;
  shooterReady &= !loaderEnable;
  digitalWrite(GreenLEDPin, shooterReady);
  // Loader - load TTB when trigger was pressed then return
  if (digitalRead(TriggerPin) && shooterReady) {
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
      if (!loaderUp) {
        loaderEnable = false;
        targetNum = !targetNum;
        calculateSpeed(shootingTarget[targetNum].speed);
        setPitchAngle(shootingTarget[targetNum].angle);
        
      }
    }
  }

    
  getEncoderData();
  setMotorSpeed();
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
      
      
      //Serial.println(ch);
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
  int channelIndex[] = {0, 4, 2};
  if (channel > 7) return;                     // Safety check
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(0x01 << channelIndex[channel]);  // Send channel select byte
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


void calculateSpeed(float TargetBallSpeed) {
  float TargetBallRPS = 40;
  float TopRPS = (TargetBallSpeed * 2 / 0.25 + TargetBallRPS * (0.04 * 3.14) / 0.25) / 2;
  float BottomRPS = (TargetBallSpeed * 2 / 0.25 - TargetBallRPS * (0.04 * 3.14) / 0.25) / 2;
  targetRPS[0] = TopRPS;
  targetRPS[1] = BottomRPS;
  targetRPS[2] = BottomRPS;
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print("targetRPS");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(targetRPS[i]);
  }
}


void setMotorSpeed() {
  for (uint8_t ch = 0; ch < NumberOfMotor; ch++) {
    float pwmValue = computePID(ch, targetRPS[ch], encoderData[ch].avg, dt[ch]);
    //Serial.println(pwmValue);
    analogWrite(ch + TopMotorPin, (int) pwmValue);
    //analogWrite(ch + TopMotorPin, (int) 255);
  }
}

void setPitchAngle(int targetAngle) {
  bool pitchDir = (targetAngle > pitchAngle)? 0 : 1;
  Serial.print("Delta angle: ");
  Serial.println((targetAngle-pitchAngle));
  digitalWrite(PitchDirPin, pitchDir);
  for (int i = 0; i < (int) abs((targetAngle-pitchAngle)) * 120 / 20 / 18; i++) {
    digitalWrite(PitchStpPin, HIGH);
    delay(8);
    digitalWrite(PitchStpPin, LOW);
    delay(2);
  }
  pitchAngle = targetAngle;
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
