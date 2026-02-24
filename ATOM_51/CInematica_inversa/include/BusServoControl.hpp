#pragma once
#include <iostream>
#include <vector>
#include <cstdint>
#include <string>

// ====================================================
// 🔧 Declaraciones públicas para usar la librería
// ====================================================

extern const uint8_t LOBOT_SERVO_FRAME_HEADER;
extern const uint8_t LOBOT_SERVO_MOVE_TIME_WRITE;
extern const uint8_t LOBOT_SERVO_POS_READ;
extern const uint8_t LOBOT_SERVO_VIN_READ;
extern const uint8_t LOBOT_SERVO_TEMP_READ;
extern const uint8_t LOBOT_SERVO_ID_READ;

bool serial_open(const std::string &port = "/dev/ttyUSB0", int baudrate = 115200);
void serial_close();

void serial_serro_wirte_cmd(uint8_t id, uint8_t w_cmd, int16_t dat1 = -9999, int16_t dat2 = -9999);
void serial_servo_read_cmd(uint8_t id, uint8_t r_cmd);
int  serial_servo_get_rmsg(uint8_t cmd);

// Funciones principales (alto nivel)
int move_servo(uint8_t id, int pulse, int duration = 500);
int get_servo_position(uint8_t id);
int get_servo_voltage(uint8_t id);
int get_servo_temperature(uint8_t id);
int get_servo_id();
