// IMULibrary.h

#ifndef IMULIBRARY_H
#define IMULIBRARY_H

#include <Wire.h>
#include <Arduino.h>

class IMULibrary {
public:
    IMULibrary();  // Constructor
    void begin(int sda = 11, int scl = 12);
    void calibrateIMU();
    void updateIMU();
    float getRoll();
    float getPitch();
    float getYaw();
    float getAccX();
    float getGyYaw();

private:
    void gyro_signals();
    float meanFilter(float *buffer, int size);
    void kalman_1d(float& KalmanState, float& KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
    
    // Variables del giroscopio
    float RateRoll, RatePitch, RateYaw;
    float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
    float AccX, AccY, AccZ;
    float AngleRoll, AnglePitch, AngleYaw;
    float GyfilteredYaw;

    // Filtros y buffers
    static const int BUFFERSIZE = 5;
    int bufferIndex;
    float GyyawBuffer[BUFFERSIZE];
    float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
    float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
    float Kalman1DOutput[2];

    // Constantes de frecuencia
    const int FREQ = 142;
};

#endif