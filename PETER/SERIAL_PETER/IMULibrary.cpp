// IMULibrary.cpp

#include "IMULibrary.h"

IMULibrary::IMULibrary()
    : RateCalibrationRoll(0), RateCalibrationPitch(0), RateCalibrationYaw(0),
      bufferIndex(0), KalmanAngleRoll(0), KalmanUncertaintyAngleRoll(4),
      KalmanAnglePitch(0), KalmanUncertaintyAnglePitch(4), AngleYaw(0) {}

void IMULibrary::begin(int sda, int scl) {
    Wire.begin(sda, scl);
    delay(250);
    calibrateIMU();
}

void IMULibrary::calibrateIMU() {
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    for (int i = 0; i < 2000; i++) {
        gyro_signals();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
}

void IMULibrary::gyro_signals() {
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(); 
    Wire.requestFrom(0x68,6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();
    Wire.beginTransmission(0x68);
    Wire.write(0x1B); 
    Wire.write(0x8);
    Wire.endTransmission();     
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);
    int16_t GyroX=Wire.read()<<8 | Wire.read();
    int16_t GyroY=Wire.read()<<8 | Wire.read();
    int16_t GyroZ=Wire.read()<<8 | Wire.read();
    RateRoll=(float)GyroX/65.5;
    RatePitch=(float)GyroY/65.5;
    RateYaw=(float)GyroZ/65.5;
    AccX=(float)AccXLSB/4096;
    AccY=(float)AccYLSB/4096;
    AccZ=(float)AccZLSB/4096;
    AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
    AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void IMULibrary::kalman_1d(float& KalmanState, float& KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState=KalmanState+0.004*KalmanInput;
    KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
    KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
    KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0]=KalmanState; 
    Kalman1DOutput[1]=KalmanUncertainty;
}

void IMULibrary::updateIMU() {
    gyro_signals();
    RateRoll-=RateCalibrationRoll;
    RatePitch-=RateCalibrationPitch;

    // Filtrado y cálculo de ángulos
    GyyawBuffer[bufferIndex] = RateYaw;
    GyfilteredYaw = meanFilter(GyyawBuffer, BUFFERSIZE);
    GyfilteredYaw = (abs(GyfilteredYaw - 2.94) < 0.09) ? 0 : GyfilteredYaw - 2.85;
    AngleYaw += GyfilteredYaw / FREQ;

    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll=Kalman1DOutput[0]; 
    KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch=Kalman1DOutput[0]; 
    KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

    bufferIndex = (bufferIndex + 1) % BUFFERSIZE;
}

float IMULibrary::meanFilter(float *buffer, int size) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    float mean = sum / size;
    mean = (fabs(mean) < 0.03) ? 0 : mean;
    return mean;
}

float IMULibrary::getRoll() {
    return KalmanAngleRoll;
}

float IMULibrary::getPitch() {
    return KalmanAnglePitch;
}

float IMULibrary::getYaw() {
    return AngleYaw;
}

float IMULibrary::getAccX() {
    return AccX;
}

float IMULibrary::getGyYaw() {
    return GyfilteredYaw;
}