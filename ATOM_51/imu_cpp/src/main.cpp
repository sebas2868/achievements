#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <thread>
#include <cstring>
#include <sys/mman.h>   // mmap
#include <sys/stat.h>   // fstat
#include <sys/types.h>
#include <cerrno>

#define BMI160_I2C_ADDR 0x69
#define I2C_BUS "/dev/i2c-1"

#define BMI160_CMD_REG          0x7E
#define BMI160_ACC_DATA_ADDR    0x12
#define BMI160_GYR_DATA_ADDR    0x0C
#define BMI160_ACC_MODE_NORMAL  0x11
#define BMI160_GYR_MODE_NORMAL  0x15

// ======================================================
// 🧱 Estructura de datos compartidos
// ======================================================
struct OrientationData {
    double roll;
    double pitch;
    double yaw;
};

// ======================================================
// ⚙️ Funciones auxiliares
// ======================================================
inline bool writeRegister(int fd, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return (write(fd, buf, 2) == 2);
}

inline bool readBlock(int fd, uint8_t reg, uint8_t* buf, size_t len) {
    if (write(fd, &reg, 1) != 1) return false;
    return (read(fd, buf, len) == static_cast<ssize_t>(len));
}

// ======================================================
// 🧠 Programa principal
// ======================================================
// ... [includes y defines iguales]

int main() {
    // =====================================
    // 🔐 Configurar memoria compartida
    // =====================================
    const char* SHM_NAME = "/orientation";
    const size_t SHM_SIZE = sizeof(OrientationData);

    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) {
        std::cerr << "❌ Error creando memoria compartida: " << strerror(errno) << std::endl;
        return 1;
    }
    ftruncate(shm_fd, SHM_SIZE);

    OrientationData* shared_data = static_cast<OrientationData*>(
        mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0)
    );
    if (shared_data == MAP_FAILED) {
        std::cerr << "❌ Error en mmap: " << strerror(errno) << std::endl;
        close(shm_fd);
        return 1;
    }

    // Inicializar valores
    shared_data->roll = shared_data->pitch = shared_data->yaw = 0.0;

    // =====================================
    // ⚙️ Inicializar IMU
    // =====================================
    int fd = open(I2C_BUS, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "❌ Error abriendo I2C: " << strerror(errno) << std::endl;
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, BMI160_I2C_ADDR) < 0) {
        std::cerr << "❌ Error configurando dirección I2C (0x"
                  << std::hex << BMI160_I2C_ADDR << "): " << strerror(errno) << std::endl;
        close(fd);
        return 1;
    }

    // Activar sensores
    if (!writeRegister(fd, BMI160_CMD_REG, BMI160_ACC_MODE_NORMAL) ||
        !writeRegister(fd, BMI160_CMD_REG, BMI160_GYR_MODE_NORMAL)) {
        std::cerr << "❌ Error inicializando BMI160" << std::endl;
        close(fd);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // ✅ Mensaje de inicio correcto
    std::cout << "🚀 BMI160 inicializado y memoria compartida lista. Sistema iniciado correctamente." << std::endl;

    // =====================================
    // 📏 Constantes y variables
    // =====================================
    constexpr double ACC_SCALE = 0.0005986; // m/s² por LSB
    constexpr double GYR_SCALE = 0.00763;   // °/s por LSB
    constexpr double ALPHA = 0.95;
    constexpr double LOOP_HZ = 100.0;
    constexpr auto LOOP_DT = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / LOOP_HZ)
    );

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    uint8_t data[12];
    auto next_time = std::chrono::steady_clock::now();

    // =====================================
    // 🔁 Bucle principal
    // =====================================
    while (true) {
        next_time += LOOP_DT;

        if (!readBlock(fd, BMI160_GYR_DATA_ADDR, data, sizeof(data))) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // Desempaquetar datos
        int16_t gx = (data[1] << 8) | data[0];
        int16_t gy = (data[3] << 8) | data[2];
        int16_t gz = (data[5] << 8) | data[4];
        int16_t ax = (data[7] << 8) | data[6];
        int16_t ay = (data[9] << 8) | data[8];
        int16_t az = (data[11] << 8) | data[10];

        // Escalar a unidades físicas
        double ax_m = ax * ACC_SCALE;
        double ay_m = ay * ACC_SCALE;
        double az_m = az * ACC_SCALE;
        double gx_d = gx * GYR_SCALE;
        double gy_d = gy * GYR_SCALE;
        double gz_d = gz * GYR_SCALE;

        // Cálculo de ángulos
        double roll_acc  = std::atan2(ay_m, az_m) * 57.2958;
        double pitch_acc = std::atan2(-ax_m, std::sqrt(ay_m * ay_m + az_m * az_m)) * 57.2958;

        // Filtro complementario
        roll  = ALPHA * (roll + gx_d / LOOP_HZ) + (1.0 - ALPHA) * roll_acc;
        pitch = ALPHA * (pitch + gy_d / LOOP_HZ) + (1.0 - ALPHA) * pitch_acc;
        yaw  += gz_d / LOOP_HZ;

        // ✅ Publicar en memoria compartida
        shared_data->roll  = roll;
        shared_data->pitch = pitch;
        shared_data->yaw   = yaw;

        std::this_thread::sleep_until(next_time);
    }

    // =====================================
    // 🔒 Cierre limpio
    // =====================================
    munmap(shared_data, SHM_SIZE);
    close(shm_fd);
    close(fd);
    shm_unlink(SHM_NAME);

    return 0;
}
