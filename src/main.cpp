// CORE
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>

// DEPENDS
#include <RF24.h>
#include <nRF24L01.h>
#include <MPU9250.h>

/**
 * =============================================================================================================================
 * =========================================================== NOTES ===========================================================
 * N/A
 * =============================================================================================================================
 **/

// ============================================================================================================================= 
// =============================================================================================================================
// =============================================================================================================================

// SPI
const uint8_t CE = 7;
const uint8_t CSN = 8;

// SERVOS
const uint8_t FL = 3;
const uint8_t FR = 5;
const uint8_t BL = 9;
const uint8_t BR = 6;

// ADDRESSES
const uint8_t IMU_ADDRESS = 0x68;
const uint8_t RF24_ADDRESS[6] = "00001";

// PID
const uint8_t CALIBRATION_ITERATIONS = UINT8_MAX;

float rawAccelerationX = 0;
float rawAccelerationY = 0;
float rawAccelerationZ = 0;

float rawGyroX = 0;
float rawGyroY = 0;
float rawGyroZ = 0;

float rawAccelerationErrorX = 0;
float rawAccelerationErrorY = 0;
float rawAccelerationErrorZ = 0;

float rawGyroErrorX = 0;
float rawGyroErrorY = 0;
float rawGyroErrorZ = 0;

float netAccelerationX = 0;
float netAccelerationY = 0;
float netAccelerationZ = 0;

float netGyroX = 0;
float netGyroY = 0;
float netGyroZ = 0;

float angleX = 0;
float angleY = 0;

// ============================================================================================================================= 
// =============================================================================================================================
// =============================================================================================================================

struct Transmission
{
    uint8_t up; // up button
    uint8_t dn; // down button

    uint8_t lx; // left analog x offsest
    uint8_t ly; // left analog y offsest
    uint8_t rx; // right analog x offsest
    uint8_t ry; // right analog y offsest

    uint8_t lb; // left analog button
    uint8_t rb; // right analog button

    uint8_t sp; // potentiometer
} transmission;

RF24 radio(CE, CSN);
MPU9250 IMU(Wire, IMU_ADDRESS);

Servo FL_MOTOR;
Servo FR_MOTOR;
Servo BL_MOTOR;
Servo BR_MOTOR;

uint32_t timeCurrent = 0;
uint32_t timePrevious = 0;
uint32_t timeElapsed = 0;

void init_RF24();
void init_IMU();
void init_MOTORS();
void init_CALIBRATIONS();

// ============================================================================================================================= 
// =============================================================================================================================
// =============================================================================================================================

void setup()
{
    init_RF24();
    init_IMU();
    init_MOTORS();
    init_CALIBRATIONS();

    timeCurrent = millis();
}

void loop()
{
    if (radio.available())
    {
        radio.read(&transmission, sizeof(Transmission));
    }

    // TODO change
    FL_MOTOR.writeMicroseconds(map(transmission.sp, 0, 255, 1000, 2000));
    FR_MOTOR.writeMicroseconds(map(transmission.sp, 0, 255, 1000, 2000));
    BL_MOTOR.writeMicroseconds(map(transmission.sp, 0, 255, 1000, 2000));
    BR_MOTOR.writeMicroseconds(map(transmission.sp, 0, 255, 1000, 2000));

    IMU.readSensor();


}

// ============================================================================================================================= 
// =============================================================================================================================
// =============================================================================================================================

void init_RF24()
{
    radio.begin();
    radio.setAutoAck(false);
    radio.openReadingPipe(1, RF24_ADDRESS);
    radio.setPALevel(RF24_PA_MIN);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(125);
    radio.startListening();
}

void init_IMU()
{
    IMU.begin();
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
    IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
}

void init_MOTORS()
{
    FL_MOTOR.attach(FL, 1000, 2000);
    FR_MOTOR.attach(FR, 1000, 2000);
    BL_MOTOR.attach(BL, 1000, 2000);
    BR_MOTOR.attach(BR, 1000, 2000);

    FL_MOTOR.writeMicroseconds(1000);
    FR_MOTOR.writeMicroseconds(1000);
    BL_MOTOR.writeMicroseconds(1000);
    BR_MOTOR.writeMicroseconds(1000);
}

void init_CALIBRATIONS()
{
    for (int i = 0; i < CALIBRATION_ITERATIONS; ++i)
    {
        IMU.readSensor();

        rawGyroX = IMU.getGyroX_rads();
        rawGyroY = IMU.getGyroY_rads();
        rawGyroZ = IMU.getGyroZ_rads();

        rawAccelerationX = IMU.getAccelX_mss();
        rawAccelerationY = IMU.getAccelY_mss();
        rawAccelerationZ = IMU.getAccelZ_mss();

        rawGyroErrorX += rawGyroX / 32.8;
        rawGyroErrorY += rawGyroY / 32.8;
        rawGyroErrorZ += rawGyroZ / 32.8;

        rawAccelerationErrorX += atan(     rawAccelerationY / sqrt(pow(rawAccelerationX, 2) + pow(rawAccelerationZ, 2))) * RAD_TO_DEG;
        rawAccelerationErrorY += atan(-1 * rawAccelerationX / sqrt(pow(rawAccelerationY, 2) + pow(rawAccelerationZ, 2))) * RAD_TO_DEG; 
    }

    rawGyroErrorX /= CALIBRATION_ITERATIONS;
    rawGyroErrorY /= CALIBRATION_ITERATIONS;
    
    rawAccelerationErrorX /= CALIBRATION_ITERATIONS;
    rawAccelerationErrorY /= CALIBRATION_ITERATIONS;
}

// ============================================================================================================================= 
// =============================================================================================================================
// =============================================================================================================================
