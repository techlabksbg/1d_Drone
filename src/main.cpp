#include <Arduino.h>
#include <Wire.h>
// #include <Servo.h>

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
const int avg_number = 220; //100; //oben = 0; unten ca. 300
int16_t average[avg_number];
int16_t index = 0, setPoint=230, setPointBegin = 0;
float sum = 0;
float AcX, error;
float turn, Kp = 0.007, Ki = 0.00001, Kd = 0.0001, derivative = 0, lastError = 0, integral;
Arduino.Servo drone;
void setup()
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    Serial.begin(9600);
    drone.attach(10);
    drone.write(20);
    delay(2000);
    drone.write(60);

//    Wire.beginTransmission(MPU_addr);
//    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
//    Wire.endTransmission(false);
//    Wire.requestFrom(MPU_addr, 2, true);             // request a total of 2 registers
//    setPoint = (Wire.read() << 8 | Wire.read())-setPointBegin; // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)

//    Wire.beginTransmission(MPU_addr);
//    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
//    Wire.endTransmission(false);
//    Wire.requestFrom(MPU_addr, 2, true);             // request a total of 2 registers
//    setPoint = setPointBegin+(Wire.read() << 8 | Wire.read()); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
//    Serial.print(setPoint);
    //  drone.write(25);
    //  delay(2000);
    //  drone.write();
}
void loop()
{
    sum = 0;
    if (index < avg_number - 1)
    {
        index += 1;
    }
    else
    {
        index = 0;
    }
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    // Wire.write(0x43);  // starting with register 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 2, true);             // request a total of 2 registers
    average[index] = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    for (int n = 0; n < avg_number; n++)
    {
        sum += average[n];
    }
    AcX = sum / avg_number;
    //drone.write()// 50 = minimum, 65 = maximum
    error = abs(AcX) - setPoint;
    Serial.print(error); // 0 = minimum, 320 = maximum
    Serial.print("\n");
    integral += error;
    derivative = error - lastError;
    turn = error * Kp + integral * Ki + derivative * Kd;
    drone.write(turn + 60);
    lastError = error;
    delay(1);

    //  drone.write(50);
    //  delay(1000);
    //  drone.write(65);
    //  delay(1000);
}
