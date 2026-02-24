#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <cmath>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <sys/mman.h> // Para memoria compartida
#include <sys/stat.h>
#include <cstdint>
// =============================================================================
// ESTRUCTURA COMPARTIDA
// =============================================================================
struct SharedData {
    double cmd_x;
    double cmd_y;
    double cmd_giro;
    bool active;
};

const char* SHM_NAME = "/robot_memory_link"; // Nombre del archivo en RAM

// =============================================================================
// CONFIGURACIÓN
// =============================================================================
const double MAX_VEL_X_MM   = 30.0;
const double MAX_VEL_Y_MM   = 25.0;
const double MAX_VEL_GIRO   = 20.0;

const int JOY_MIN        = -32767;
const int JOY_MAX        =  32767;
const int DEADZONE_XY    = 4000;
const int DEADZONE_GIRO  = 8000;

double map_joystick(int raw_input, double output_max_limit, int deadzone, bool invert = false) {
    if (std::abs(raw_input) < deadzone) return 0.0;
    double input_clamped = std::clamp(raw_input, JOY_MIN, JOY_MAX);
    double normalized = 0.0;
    if (input_clamped > 0) normalized = (input_clamped - deadzone) / (double)(JOY_MAX - deadzone);
    else normalized        = (input_clamped + deadzone) / (double)(std::abs(JOY_MIN) - deadzone);
    double output = normalized * output_max_limit;
    if (invert) output = -output;
    return output;
}

int main() {
    // 1. CONFIGURAR MEMORIA COMPARTIDA
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) { std::cerr << "❌ Error creando memoria compartida.\n"; return 1; }
    ftruncate(shm_fd, sizeof(SharedData));
    SharedData* memory = (SharedData*)mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    memory->cmd_x = 0; memory->cmd_y = 0; memory->cmd_giro = 0; memory->active = false;

    // 2. CONFIGURAR JOYSTICK
    const char* device_path = "/dev/input/js0";
    int js_fd = open(device_path, O_RDONLY | O_NONBLOCK);

    bool connected = false;
    if (js_fd != -1) {
        std::cout << "🎮 Control Conectado. Publicando en memoria: " << SHM_NAME << "\n";
        connected = true;
    } else {
        std::cerr << "⚠️ Esperando control...\n";
    }

    struct js_event e;

    // Variables locales
    double current_x = 0, current_y = 0, current_giro = 0;
    bool button_active = false; // NUEVA variable para el botón

    // 3. BUCLE PRINCIPAL
    while (true) {
        if (connected) {
            while (read(js_fd, &e, sizeof(e)) > 0) {
                uint8_t etype = e.type & ~JS_EVENT_INIT;

                if (etype == JS_EVENT_AXIS) {
                    switch (e.number) {
                        case 0: current_y   = map_joystick(e.value, MAX_VEL_Y_MM, DEADZONE_XY, false); break;
                        case 1: current_x   = map_joystick(e.value, MAX_VEL_X_MM, DEADZONE_XY, true);  break;
                        case 2:
                        case 3: current_giro= map_joystick(e.value, MAX_VEL_GIRO, DEADZONE_GIRO, false); break;
                    }
                } else if (etype == JS_EVENT_BUTTON) {
                    // Detectar botón específico (ejemplo: botón 0)
                     if (e.number == 0 && e.value == 1) { 
                // Toggle solo al pulsar (no al soltar)
                	button_active = !button_active;
            	}

                }
            }
        }

        // --- Publicar en memoria compartida ---
        memory->cmd_x   = -1 * current_x;
        memory->cmd_y   = current_y;
        memory->cmd_giro= current_giro;
        memory->active  = button_active; // AHORA depende del botón

        // Feedback visual
        printf("MEMORIA => X:%6.2f Y:%6.2f G:%6.2f | Active:%d \r",
               current_x, current_y, current_giro, memory->active);
        fflush(stdout);

        usleep(20000); // 20ms (50Hz)
    }

    return 0;
}