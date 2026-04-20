# 1 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"



# 5 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino" 2
# 6 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino" 2

// Define the usage for Pins
# 28 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
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
  float lastAngle; // Previous raw angle (0..4095)
  unsigned long lastTime; // Previous timestamp (ms)
  float rps; // Last computed RPS
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



void setup() {
  Serial.begin(115200);
  while (!Serial);
  configPins();
  Wire.begin();
  digitalWrite(8, 0);
  for (int ch = 0; ch < 3; ch++) {
    selectTCAChannel(ch);
    delay(10);
    encoderData[ch].lastAngle = readAS5600Angle();
    encoderData[ch].lastTime = millis();
    encoderData[ch].rps = 0.0;

    pidData[ch].integral = 0.0;
    pidData[ch].prevError = 0.0;
  }
  // Setup first target para
  shootingTarget[0].speed = 10;
  shootingTarget[0].angle = 300;
  // Setup second target para
  shootingTarget[1].speed = 20;
  shootingTarget[1].angle = 0;
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
  for (uint8_t i = 0; i < 3; i++) {
    if (encoderData[i].rps < targetRPS[i] * 0.95 || encoderData[i].rps > targetRPS[i] * 1.05 ) {
      shooterReady = false;
    }
    shooterReady &= !loaderUp;
  }
  digitalWrite(8, shooterReady);
  // Loader - load TTB when trigger was pressed then return
  if (digitalRead(3) && shooterReady) {
    loaderEnable = true;
  }
  if (loaderEnable) {
    if (!loaderUp) {
      digitalWrite(6, 1);
      digitalWrite(7, 0x1);
      delay(1);
      digitalWrite(7, 0x0);

      loaderPos++;
      //Serial.println(loaderPos);
    }
    else {
      digitalWrite(6, 0);
      digitalWrite(7, 0x1);
      delay(1);
      digitalWrite(7, 0x0);

      loaderPos--;
    }

    if (loaderPos >= 50 * 73 /* Distance for the loader to move when loading in mm*/ || loaderPos <= 0) {
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
    for (int i = 0; i < 3; i++) {
      RPSString[5] = i + '1';
      Serial.print(RPSString);
      Serial.print(encoderData[i].avg, 2);
      RPSString[0] = ',';
    }
    Serial.println();

  }
  if (now - lastPrint >= 10 /* Sampling interval in milliseconds*/) {
    lastPrint = now;
    for (int ch = 0; ch < 3; ch++) {
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
      if (dt_s > (10 /* Sampling interval in milliseconds*/ + 0.1) / 2000.0 && delta_rev / dt_s < 110 && delta_rev > 0.07 && ((accel)>0?(accel):-(accel)) < 8000) {
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
# 221 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void selectTCAChannel(int channel) {
  if (channel > 7) return; // Safety check
  Wire.beginTransmission(0x70 /* Default I2C address of TCA9548A*/);
  Wire.write(0x01 << channel); // Send channel select byte
  //Serial.println(0x01 << channel);                   
  Wire.endTransmission();
}

/*

 * Read the 12‑bit angle from the AS5600.

 * Returns a value between 0 and 4095.

 */
# 233 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
uint16_t readAS5600Angle() {
  Wire.beginTransmission(0x36 /* Fixed I2C address of AS5600*/);
  Wire.write(0x0C /* AS5600 angle register (high byte 0x0C, low byte 0x0D)*/); // Point to the high byte of the angle
  if (Wire.endTransmission(false) != 0) { // Send repeated start
    Serial.println("Error communicating with AS5600");
    return 0;
  }

  // Request 2 bytes from the AS5600
  Wire.requestFrom(0x36 /* Fixed I2C address of AS5600*/, 2);
  if (Wire.available() >= 2) {
    uint8_t highByte = Wire.read(); // High byte (bits 11..8)
    uint8_t lowByte = Wire.read(); // Low byte  (bits 7..0)
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
  float maxIntegral = 255.0 / (Ki + 0.001); // Rough limit based on max PWM
  if (pidData[motorIndex].integral > maxIntegral) pidData[motorIndex].integral = maxIntegral;
  if (pidData[motorIndex].integral < -maxIntegral) pidData[motorIndex].integral = -maxIntegral;
  float I = Ki * pidData[motorIndex].integral;

  // Derivative term (on measurement to avoid derivative kick)
  float D = Kd * ( (measurement - (setpoint - error)) / dt ); // Actually we need previous measurement

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
  for (uint8_t ch = 0; ch < 3; ch++) {
    float pwmValue = computePID(ch, targetRPS[ch], encoderData[ch].avg, dt[ch]);
    //Serial.println(pwmValue);
    analogWrite(ch + 9, (int) pwmValue);
    //analogWrite(ch + TopMotorPin, (int) 255);
  }
}

void setPitchAngle(int targetAngle) {
  bool pitchDir = (targetAngle > pitchAngle)? 0 : 1;
  Serial.print("Delta angle: ");
  Serial.println((targetAngle-pitchAngle));
  digitalWrite(4, pitchDir);
  for (int i = 0; i < (int) (((targetAngle-pitchAngle))>0?((targetAngle-pitchAngle)):-((targetAngle-pitchAngle))) * 120 / 20 / 18; i++) {
    digitalWrite(5, 0x1);
    delay(8);
    digitalWrite(5, 0x0);
    delay(2);
  }
  pitchAngle = targetAngle;
}



void configPins() {
  pinMode(3, 0x0);

  pinMode(4, 0x1);
  pinMode(5, 0x1);

  pinMode(6, 0x1);
  pinMode(7, 0x1);

  pinMode(8, 0x1);

  pinMode(9, 0x1);
  pinMode(10, 0x1);
  pinMode(11, 0x1);

  pinMode(12, 0x0);
  pinMode(13, 0x0);
}
