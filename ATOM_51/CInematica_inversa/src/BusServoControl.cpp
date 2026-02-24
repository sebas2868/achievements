#include "BusServoControl.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <thread>
#include <chrono>
// ====================================================
// ⚙️ CONFIGURACIÓN GENERAL
// ====================================================
#define FAST_MODE 1   // ⚡ 1 = rápido (robot), 0 = modo seguro (depuración)

using namespace std;

// ====================================================
// --- Constantes del protocolo LOBOT ---
// ====================================================
const uint8_t LOBOT_SERVO_FRAME_HEADER         = 0x55;
const uint8_t LOBOT_SERVO_MOVE_TIME_WRITE      = 1;
const uint8_t LOBOT_SERVO_POS_READ             = 28;
const uint8_t LOBOT_SERVO_VIN_READ             = 27;
const uint8_t LOBOT_SERVO_TEMP_READ            = 26;
const uint8_t LOBOT_SERVO_ID_READ              = 14;

// ====================================================
// --- Puerto serial global ---
// ====================================================
int serialHandle = -1;

bool serial_open(const string &port, int baudrate) {
    // 🔧 Abrir puerto con escritura sincronizada
    serialHandle = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serialHandle == -1) {
        cerr << "❌ No se pudo abrir el puerto serial: " << port << endl;
        perror("Error");
        return false;
    }

    struct termios options{};
    if (tcgetattr(serialHandle, &options) != 0) {
        cerr << "❌ Error al obtener configuración del puerto" << endl;
        close(serialHandle);
        return false;
    }

    // ================================================
    // ⚙️ Configuración de velocidad
    // ================================================
    speed_t speed;
    switch (baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default:
            cerr << "⚠️ Baudrate no estándar, usando 115200" << endl;
            speed = B115200;
            break;
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // ================================================
    // ⚙️ Configuración de formato: 8N1
    // ================================================
    options.c_cflag |= (CLOCAL | CREAD);  // habilita recepción y evita control de modem
    options.c_cflag &= ~PARENB;           // sin paridad
    options.c_cflag &= ~CSTOPB;           // 1 bit de stop
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;               // 8 bits de datos
    options.c_cflag &= ~CRTSCTS;          // sin control de flujo por hardware

    options.c_iflag &= ~(IXON | IXOFF | IXANY); // sin control de flujo por software
    options.c_lflag = 0;                        // modo no canónico
    options.c_oflag = 0;                        // sin procesamiento de salida

#if FAST_MODE
    options.c_cc[VMIN]  = 0;   // lectura no bloqueante
    options.c_cc[VTIME] = 1;   // timeout corto (100 ms)
#else
    options.c_cc[VMIN]  = 1;   // lectura bloqueante mínima de 1 byte
    options.c_cc[VTIME] = 2;   // timeout 200 ms
#endif

    // Aplica configuración inmediatamente
    if (tcsetattr(serialHandle, TCSANOW, &options) != 0) {
        cerr << "❌ Error al configurar atributos del puerto" << endl;
        close(serialHandle);
        return false;
    }

    // ================================================
    // 🚫 Desactiva señales DTR/RTS (evita reset del adaptador)
    // ================================================
    int flags;
    if (ioctl(serialHandle, TIOCMGET, &flags) == 0) {
        flags &= ~(TIOCM_DTR | TIOCM_RTS);
        ioctl(serialHandle, TIOCMSET, &flags);
    }

    // ================================================
    // 🧹 Limpieza de buffers y estabilización del bus
    // ================================================
    tcflush(serialHandle, TCIOFLUSH);
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); // espera a que el bus se estabilice

#if !FAST_MODE
    cout << "✅ Puerto abierto correctamente: " << port
         << " @ " << baudrate << " bps" << endl;
#endif

    return true;
}




// =======================================================
void serial_close() {
    if (serialHandle >= 0) {
        tcflush(serialHandle, TCIOFLUSH);
        close(serialHandle);
        serialHandle = -1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        cout << "🔌 Puerto serial cerrado correctamente." << endl;
    }
}

// ====================================================
// --- Envío de comandos ---
// ====================================================
void serial_serro_wirte_cmd(uint8_t id, uint8_t w_cmd, int16_t dat1, int16_t dat2) {
    uint8_t buf[16];
    int len = 0;

    buf[len++] = 0x55;
    buf[len++] = 0x55;
    buf[len++] = id;

    // Longitud del paquete
    if (dat1 == -9999 && dat2 == -9999)
        buf[len++] = 3;
    else if (dat1 != -9999 && dat2 == -9999)
        buf[len++] = 4;
    else
        buf[len++] = 7;

    buf[len++] = w_cmd;

    // Datos opcionales
    if (dat1 != -9999 && dat2 == -9999) {
        buf[len++] = dat1 & 0xFF;
    } else if (dat1 != -9999 && dat2 != -9999) {
        buf[len++] = dat1 & 0xFF;
        buf[len++] = (dat1 >> 8) & 0xFF;
        buf[len++] = dat2 & 0xFF;
        buf[len++] = (dat2 >> 8) & 0xFF;
    }

    // Checksum rápido inline
    int sum = 0;
    for (int i = 0; i < len; i++) sum += buf[i];
    buf[len++] = (~(sum - 0xAA)) & 0xFF;

    // Escribe y fuerza envío inmediato
    write(serialHandle, buf, len);
    tcdrain(serialHandle);
}

// ====================================================
// --- Lectura de comandos ---
// ====================================================
void serial_servo_read_cmd(uint8_t id, uint8_t r_cmd) {
    uint8_t buf[6] = {0x55, 0x55, id, 3, r_cmd, 0};
    int sum = 0;
    for (int i = 0; i < 5; i++) sum += buf[i];
    buf[5] = (~(sum - 0xAA)) & 0xFF;

    write(serialHandle, buf, sizeof(buf));
    tcdrain(serialHandle);

#if FAST_MODE
    this_thread::sleep_for(chrono::milliseconds(2));
#else
    this_thread::sleep_for(chrono::milliseconds(5));
#endif
}

// ====================================================
// --- Recepción de mensajes ---
// ====================================================
int serial_servo_get_rmsg(uint8_t cmd) {
#if FAST_MODE
    this_thread::sleep_for(chrono::milliseconds(5));
#else
    this_thread::sleep_for(chrono::milliseconds(10));
#endif

    static uint8_t recv_data[64];
    int count = read(serialHandle, recv_data, sizeof(recv_data));

    if (count > 5 && recv_data[0] == 0x55 && recv_data[1] == 0x55 && recv_data[4] == cmd) {
        uint8_t dat_len = recv_data[3];
        if (dat_len == 4)
            return recv_data[5];
        else if (dat_len == 5)
            return static_cast<int16_t>(recv_data[5] | (recv_data[6] << 8));
        else if (dat_len == 7)
            return static_cast<int16_t>(recv_data[5] | (recv_data[6] << 8));
    }

    return -1;
}

// ====================================================
// --- Funciones principales ---
// ====================================================
const int TIMEOUT = 15;  // 🔹 reducido (más rápido)

int move_servo(uint8_t id, int pulse, int duration) {
    pulse = max(0, min(1000, pulse));
    duration = max(0, min(30000, duration));
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, duration);
    return pulse;
}

int get_servo_position(uint8_t id) {
    for (int i = 0; i < TIMEOUT; i++) {
        serial_servo_read_cmd(id, LOBOT_SERVO_POS_READ);
        int msg = serial_servo_get_rmsg(LOBOT_SERVO_POS_READ);
        if (msg != -1) return msg;
    }
    return -1;
}

int get_servo_voltage(uint8_t id) {
    for (int i = 0; i < TIMEOUT; i++) {
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_READ);
        int msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_READ);
        if (msg != -1) return msg;
    }
    return -1;
}

int get_servo_temperature(uint8_t id) {
    for (int i = 0; i < TIMEOUT; i++) {
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_READ);
        int msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_READ);
        if (msg != -1) return msg;
    }
    return -1;
}

int get_servo_id() {
    for (int i = 0; i < TIMEOUT; i++) {
        serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ);
        int msg = serial_servo_get_rmsg(LOBOT_SERVO_ID_READ);
        if (msg != -1) return msg;
    }
    return -1;
}
