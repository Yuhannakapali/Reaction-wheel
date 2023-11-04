#include <Wire.h>

#define MPU6050       0x68         // Device address
#define ACCEL_CONFIG  0x1C         // Accelerometer configuration address
#define GYRO_CONFIG   0x1B         // Gyro configuration address
#define PWR_MGMT_1    0x6B
#define PWR_MGMT_2    0x6C

#define BRAKE         8 
#define PWM           9
#define DIRECTION     4

#define BUZZER        12
#define VBAT          A7

const uint16_t PWM_FREQUENCY = 20000;                 // The motor driver can handle a PWM frequency up to 20kHz
const uint16_t PWMVALUE = F_CPU / PWM_FREQUENCY / 2;  // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

float X1 = 75.0; 
float X2 = 5.25;   
float X3 = 0.04;  
float loop_time = 10;  

int pwm_s = 0;
byte dir;
int32_t motor_speed; 
uint32_t timer;
long currentT, previousT_1, previousT_2 = 0; 
int16_t AcX, AcY, AcZ, GyZ, gyroZ;

//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
#define Gyro_amount 0.996     // percent of gyro in complementary filter

//IMU offset values
int16_t  AcX_offset = -750;
int16_t  AcY_offset = 360;
int16_t  AcZ_offset = 0;
int16_t  GyZ_offset = 0;
int32_t  GyZ_offset_sum = 0;

float alpha = 0.40; 
float gyroZfilt;

float robot_angle;
float Acc_angle;

bool vertical = false;  

uint8_t i2cData[14]; // Buffer for I2C data

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyZ_offset_sum += GyZ;
    delay (5);
  }
  GyZ_offset = GyZ_offset_sum >> 10;
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  delay(80);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
}

void angle_calc() {
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  
  GyZ = Wire.read() << 8 | Wire.read(); 

  AcX += AcX_offset;
  AcY += AcY_offset;  
  AcZ += AcZ_offset;
  GyZ -= GyZ_offset;

  robot_angle += GyZ * loop_time / 1000 / 65.536; 
  Acc_angle = atan2(AcY, -AcX) * 57.2958;               // angle from acc. values       * 57.2958 (deg/rad)
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);
  
  if (abs(robot_angle) > 9) vertical = false;
  if (abs(robot_angle) < 0.3) vertical = true;
  
  //Serial.print("Angle: "); Serial.println(robot_angle);
}

void battVoltage(double voltage) {
  //Serial.print("batt: "); Serial.println(voltage); //debug
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void setPWM(uint16_t dutyCycle) { // dutyCycle is a value between 0-ICR1
    OCR1A = dutyCycle;
}

void Motor_control(int pwm) {
  if (pwm <= 0) {
    digitalWrite(DIRECTION, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(DIRECTION, HIGH);
  }
  setPWM(map(pwm, 0, 255, PWMVALUE, 0));
}

int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  char param = Serial.read();               // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                 // get command byte
  Serial.flush();
  switch (param) {
    case 'p':
      if (cmd == '+')    X1 += 1;
      if (cmd == '-')    X1 -= 1;
      printValues();
      break;
    case 'i':
      if (cmd == '+')    X2 += 0.01;
      if (cmd == '-')    X2 -= 0.01;
      printValues();
      break;
     case 's':
      if (cmd == '+')    X3 += 0.005;
      if (cmd == '-')    X3 -= 0.005;
      printValues();
      break;  
  }
}

void printValues() {
  Serial.print("X1: "); Serial.print(X1);
  Serial.print(" X2: "); Serial.print(X2);
  Serial.print(" X3: "); Serial.println(X3, 3);
}

void setup() {
  Serial.begin(115200);

  // Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/doc8272.pdf page 128-135 
  // Set up PWM, Phase and Frequency Correct on pin 9 (OC1A) & pin 10 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = (1 << WGM13) | (1 << CS10);  // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1 = PWMVALUE;                      // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz

  // Clear OC1A/OC1B on compare match when up-counting - Set OC1A/OC1B on compare match when downcounting
  TCCR1A = (1 << COM1A1) | (1 << COM1B1);
  setPWM(400); 
  
  Serial.print("PWM: "); Serial.println(PWMVALUE); Serial.print("CPU_FREQ: "); Serial.println(F_CPU); 
  
  pinMode(PWM, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(BRAKE, OUTPUT);
  pinMode(DIRECTION, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  delay(1000);
  angle_setup();
}

void loop() {
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    Tuning(); 
    angle_calc();
    if (vertical) {
      digitalWrite(BRAKE, HIGH);
      gyroZ = GyZ / 131.0; // Convert to deg/s
      
      gyroZfilt = alpha * gyroZ + (1 - alpha) * gyroZfilt;
      pwm_s = -constrain(X1 * robot_angle + X2 * gyroZfilt + X3 * -motor_speed, -255, 255); 

      Motor_control(pwm_s);
      motor_speed += pwm_s;
    } else {
      Motor_control(0);
      digitalWrite(BRAKE, LOW);
      motor_speed = 0;
    }
    previousT_1 = currentT;
  }
  if (currentT - previousT_2 >= 500) {
    battVoltage((double)analogRead(VBAT) / 53.7);  // This is then connected to a 47k-12k voltage divider
    previousT_2 = currentT;
  }
}