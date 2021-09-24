#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

const int MPU_addr = 0x68; // I2C address of the MPU-6050
const int avg_number = 10;
int16_t average[avg_number];
int16_t index = 0, setPoint = 0, setPointBegin = 0;
float sum = 0;
float AcY, error; // AcY maximum (top) = 2500, minimum (bottom) = -3300, 0 = best
float turn, Kp = 0.0008, Ki = 0.0, Kd = 0.001, derivative = 0, lastError = 0, integral;
Servo drone; // drone.write() //50 = minimum, 65 = maximum
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
    Wire.write(0x3D); // starting with register 0x3D (ACCEL_YOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 2, true);             // request a total of 2 registers
    average[index] = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    for (int n = 0; n < avg_number; n++)
    {
        sum += average[n];
    }
    AcY = sum / avg_number;
    error = setPoint - AcY;
    Serial.print(error);
    // Serial.print(average[index]);
    Serial.print("\n");
    integral += error;
    derivative = error - lastError;
    turn = error * Kp + integral * Ki + derivative * Kd;
    drone.write(turn + 60);
    lastError = error;
    delay(1);
}
