# 1 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"




# 6 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino" 2

// Define the usage for Pins
# 28 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
// ======================== PID Parameters ========================
float Kp = 20;
float Ki = 50;
float Kd = 20;

float targetRPS[3] = {80, 80, 80};
float dt[3];

struct Encoder {
  float lastAngle; // Previous raw angle (0..4095)
  unsigned long lastTime; // Previous timestamp (ms)
  float rps; // Last computed RPS
  float avg={0};
  float avg2={0};
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





void setup() {
  Serial.begin(115200);
  while (!Serial);
  configPins();
  Wire.begin();

  for (uint8_t ch = 0; ch < 3; ch++) {
    selectTCAChannel(ch);
    delay(10);
    encoderData[ch].lastAngle = readAS5600Angle();
    encoderData[ch].lastTime = millis();
    encoderData[ch].rps = 0.0;

    pidData[ch].integral = 0.0;
    pidData[ch].prevError = 0.0;
  }
}

void loop() {
  getEncoderData();
  setMotorSpeed();

  if (digitalRead(3)) {
    targetRPS[0] = 80;
  }else {
    targetRPS[0] = 0;
  }
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
# 127 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
}

float getEncoderData() {
  static unsigned long lastPrint = 0;
  static unsigned long lastPrint2 = 0;
  unsigned long now = millis();
  if (now - lastPrint2 >= 30) {

    lastPrint2 = now;
    Serial.print(">Motor_1:");
    Serial.print(encoderData[0].rps, 2);
    Serial.print(",avg:");
    Serial.print(encoderData[0].avg, 2);
    Serial.print(",avg2:");
    Serial.println(encoderData[0].avg2, 2);
  }
  if (now - lastPrint >= 0 /* Sampling interval in milliseconds*/) {
    lastPrint = now;
    for (uint8_t ch = 0; ch < 1; ch++) {
      // Select the desired channel on the TCA9548A
      selectTCAChannel(ch);
      delay(2); // Short delay to let the mux settle

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
      if (dt_s > (0 /* Sampling interval in milliseconds*/ + 0.1) / 2000.0 && delta_rev / dt_s < 110 && delta_rev > 0.07 && ((accel)>0?(accel):-(accel)) < 8000) {
        encoderData[ch].rps = delta_rev / dt_s;
      }else {encoderData[ch].rps -= (encoderData[ch].rps > 0)? 1 : 0;}

      // 2nd order low-pass filter
      encoderData[ch].avg = (encoderData[ch].rps > 0)? (1-0.01) * encoderData[ch].avg + (0.01) * encoderData[ch].rps : 0;
      encoderData[ch].avg2 = (1-0.01) * encoderData[ch].avg2 + (0.01) * encoderData[ch].avg;

      // Store current values for next iteration
      encoderData[ch].lastAngle = rev;
      encoderData[ch].lastTime = now;
      dt[ch] = dt_s;
    }
  // Plot the results
  //Serial.print(">Motor_1:");
  //Serial.print(encoderData[0].rps, 2);
  //Serial.print(",angle:");
  //Serial.println(encoderData[0].lastAngle, 2);
  /*Serial.print(",");

  Serial.print("Motor_2:");

  Serial.print(encoderData[1].rps, 2);

  Serial.print(",");

  Serial.print("Motor_3:");

  Serial.println(encoderData[2].rps, 2);*/
# 194 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
  }


}


/*

 * Select a channel on the TCA9548A.

 * channel: 0..7 for the eight possible channels.

 */
# 204 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
void selectTCAChannel(uint8_t channel) {
  if (channel > 7) return; // Safety check
  Wire.beginTransmission(0x70 /* Default I2C address of TCA9548A*/);
  Wire.write(1 << channel); // Send channel select byte
  Wire.endTransmission();
}

/*

 * Read the 12‑bit angle from the AS5600.

 * Returns a value between 0 and 4095.

 */
# 215 "c:\\Users\\hksdg\\OneDrive - HKUST Connect\\26 Spring\\MECH3907\\2026-MECH3907-Gp7-TTB-shooter\\src\\main.ino"
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
  if (output < 15) output = 0;
  if (output > 255) output = 255;

  return output;
}





void setMotorSpeed() {
  for (uint8_t ch = 0; ch < 1; ch++) {
    float pwmValue = computePID(ch, targetRPS[ch], encoderData[ch].avg2, dt[ch]);
    analogWrite(ch + 9, (int) pwmValue);
    //analogWrite(ch + TopMotorPin, (int) 255);
  }
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
