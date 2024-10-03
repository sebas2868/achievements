#include "i2c_module.h"

// Constructor que inicializa la dirección del TCA9548A
i2c_module::i2c_module(uint8_t tcaAddress) {
  _tcaAddress = tcaAddress;
}

// Inicializa los sensores
void i2c_module::begin() {
  Wire.begin();  // Iniciar la comunicación I2C

  TCA9548A(1);
  // Iniciar el sensor VL53L0X
  if (!_lox.begin()) {
    Serial.println(F("Error al iniciar el sensor VL53L0X!"));
    while (1);
  }
  Serial.println(F("Sensor VL53L0X iniciado correctamente"));
}

// Selecciona el bus del multiplexor I2C TCA9548A
void i2c_module::TCA9548A(uint8_t bus) {
  Wire.beginTransmission(_tcaAddress);  // Dirección del TCA9548A
  Wire.write(1 << bus);                 // Seleccionar el bus
  Wire.endTransmission();
}

// Leer el ángulo del sensor AS5600 en un bus específico
int i2c_module::readAS5600Angle(uint8_t bus) {
  TCA9548A(bus);  // Selecciona el bus correspondiente
  Wire.beginTransmission(0x36);  // Dirección del AS5600
  Wire.write(0x0C);              // Registro de ángulo
  Wire.endTransmission();
  Wire.requestFrom(0x36, 2);     // Solicitar 2 bytes

  while (Wire.available() < 2);  // Esperar a que lleguen los datos
  int highByte = Wire.read();
  int lowByte = Wire.read();

  int angle = ((highByte << 8) | lowByte) & 0x0FFF;  // 12 bits
  return angle;
}

// Leer la distancia del sensor VL53L0X en un bus específico
float i2c_module::readVL53L0XDistance(uint8_t bus) {
  TCA9548A(bus);  // Seleccionar el bus correspondiente

  VL53L0X_RangingMeasurementData_t measure;
  _lox.rangingTest(&measure, false);  // Realizar la medición

  if (measure.RangeStatus != 4) {  // Si no hay error
    return measure.RangeMilliMeter;
  } else {
    return -1;  // Error en la medición
  }
}
