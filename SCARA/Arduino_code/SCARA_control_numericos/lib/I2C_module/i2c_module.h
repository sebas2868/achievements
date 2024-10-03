#ifndef i2c_module_h
#define i2c_module_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

class i2c_module {
  public:
    // Constructor
    i2c_module(uint8_t tcaAddress);

    // Métodos para iniciar el sensor y leer valores
    void begin();
    int readAS5600Angle(uint8_t bus);
    float readVL53L0XDistance(uint8_t bus);

  private:
    uint8_t _tcaAddress;
    Adafruit_VL53L0X _lox;

    void TCA9548A(uint8_t bus);
};

#endif
