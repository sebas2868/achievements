#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <array>
#include <algorithm>
#include <cstring>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iomanip>
#include <uv.h> // Librería libuv
#include <wiringPi.h>
// Headers simulados (Asegúrate de tenerlos en tu carpeta)
#include "kinematics.hpp"
#include "BusServoControl.hpp"

// Definir los pines BCM que queremos usar
#define PIN_LECTURA_1 24 // BCM 24
#define PIN_LECTURA_2 25 // BCM 25
enum Estado
{
    STOP,
    MARCHA,
    SENTADO,
    INICIO,
    MUERTO,
    BAILAR,
    CONTACTO,
    DETECTAR
};
// =============================================================================
// CONFIGURACIÓN
// =============================================================================
const char *SHM_INPUT_NAME = "/robot_memory_link";
const char *SHM_VOLT_NAME = "/robot_voltage";
const char *SHM_NAME = "/orientation";

#define SHM_AUDIO_NAME "/audio_cmd" // Debe coincidir con el servidor de audio
#define SHM_AUDIO_SIZE sizeof(int)  // Debe coincidir con el servidor de audio

char command_char = ' ';

struct SharedCommandData
{
    char command = ' ';
    // ⚠️ NUEVO: Indicador de que el comando es nuevo (lo pone el Joystick, lo limpia el Robot)
    bool updated = false;
};
const double Z_SUELO = 135.0;
const double Z_AIRE = 100.0;
const double SCALE = 1000.0 / 240.0;
const int PULSE_MIN = 0;
const int PULSE_MAX = 1000;

// Ganancias Estabilización
const double GAIN_TRANS_X = -0.0;
const double GAIN_TRANS_Y = -0.0;
const double GAIN_ROT_ROLL = 0.8;
const double GAIN_ROT_PITCH = 0.5;

int *shm_audio_ptr = nullptr;

SharedCommandData *shm_cmd_ptr = nullptr; // <--- CORREGIDO (Debe apuntar al struct)

static enum Estado estado_actual = STOP;
static enum Estado estado_anterior = STOP;

double velocidad_stop = 400; // ms
bool once = true;
// =============================================================================
// ESTRUCTURAS DE DATOS
// =============================================================================
struct OrientationData
{
    double roll, pitch, yaw;
};
struct SharedInputData
{
    double cmd_x, cmd_y, cmd_giro;
    bool active;
};
struct Pata
{
    double x, y, z;
};
struct Compensacion
{
    double x = 0, y = 0, roll = 0, pitch = 0;
};

using AngulosRobot = std::array<std::array<double, 3>, 4>;

#define SHM_CMD_NAME "/robot_cmd"
#define SHM_CMD_SIZE sizeof(SharedCommandData) // <- Usa el tamaño del

Pata patas_pos[4] = {};
double orientacion[3] = {0.0, 0.0, 0.0}; // roll, pitch, yaw
bool move_motores = true;

struct MotorLeg
{
    std::string leg;
    std::array<int, 3> ids;
    std::array<int, 3> offsets;
};

std::array<MotorLeg, 4> motores = {{{"FL", {7, 8, 9}, {-10, 7, -5}},
                                    {"FR", {4, 5, 6}, {30, -25, -45}},
                                    {"BL", {1, 2, 3}, {-25, 10, 0}},
                                    {"BR", {10, 11, 12}, {0, 10, -20}}}};

// Utilidad Filtro a
double filter(double input, double previous_output, double alpha_percent)
{
    double alpha = alpha_percent / 100.0;
    return (input * alpha) + (previous_output * (1.0 - alpha));
}

// =============================================================================
// GENERADOR DE MARCHA
// =============================================================================
struct GeneradorMarcha
{
    int paso_actual = 0;
    double duracion_paso = 200.0;
    double x_fil = 0, y_fil = 0, giro_fil = 0;
    Pata patas[4];
    bool en_suelo[4] = {true, true, true, true}; // Estado de tracción
    bool update_needed = false;

    void setPata(int id, double x, double y, double giro, double z)
    {
        double lateral = y + ((id == 0 || id == 3) ? -giro : giro);
        patas[id] = {x, lateral, z};
        // Actualizamos estado de suelo
        en_suelo[id] = (z > (Z_AIRE + 10.0));
    }

    double update(double cmd_x, double cmd_y, double cmd_giro)
    {
        // 1. Filtrado
        x_fil = x_fil * 0.8 + cmd_x * 0.2;
        y_fil = y_fil * 0.8 + cmd_y * 0.2;
        giro_fil = giro_fil * 0.8 + cmd_giro * 0.2;

        // 2. Calcular duración
        double hipotenusa = std::hypot(x_fil, y_fil);
        duracion_paso = (110.0 + (hipotenusa / 3.5));

        // 3. Máquina de estados
        paso_actual = (paso_actual + 1) % 4;

        if (paso_actual == 0 || paso_actual == 2)
        {
            for (int i = 0; i < 4; i++)
            {
                patas[i].z = Z_SUELO;
                en_suelo[i] = true;
            }
        }
        else if (paso_actual == 1)
        {
            setPata(2, x_fil, y_fil, giro_fil, Z_AIRE);
            setPata(1, x_fil, y_fil, giro_fil, Z_AIRE);
            setPata(3, -x_fil, -y_fil, -giro_fil, Z_SUELO);
            setPata(0, -x_fil, -y_fil, -giro_fil, Z_SUELO);
        }
        else if (paso_actual == 3)
        {
            setPata(3, x_fil, y_fil, giro_fil, Z_AIRE);
            setPata(0, x_fil, y_fil, giro_fil, Z_AIRE);
            setPata(2, -x_fil, -y_fil, -giro_fil, Z_SUELO);
            setPata(1, -x_fil, -y_fil, -giro_fil, Z_SUELO);
        }

        return duracion_paso;
    }
};

void send_audio_command(int audio_id)
{
    if (shm_audio_ptr != nullptr)
    {
        // El servidor de audio limpiará la memoria a 0 después de leerlo.
        *shm_audio_ptr = audio_id;
        std::cout << "📢 Audio ID publicado: " << audio_id << std::endl;
    }
    else
    {
        std::cerr << "❌ Error: La memoria SHM de audio no está inicializada." << std::endl;
    }
}

SharedCommandData *open_robot_command_shm(int &fd_shm)
{
    // 1. ABRIR / CREAR LA MEMORIA COMPARTIDA
    // O_CREAT: Si no existe, créala.
    // O_RDWR: Permite lectura y escritura.
    // 0666: Permisos para que cualquiera pueda leer y escribir.
    fd_shm = shm_open(SHM_CMD_NAME, O_CREAT | O_RDWR, 0666);

    if (fd_shm == -1)
    {
        std::cerr << "❌ Error: Falló shm_open para SHM de comandos.\n";
        return nullptr;
    }

    // 2. DIMENSIONAR LA MEMORIA
    // Siempre dimensionamos la memoria para asegurar que tiene el tamaño correcto.
    if (ftruncate(fd_shm, SHM_CMD_SIZE) == -1)
    {
        std::cerr << "❌ Error: Falló ftruncate para SHM de comandos.\n";
        // Cierra el descriptor si falla el dimensionamiento
        close(fd_shm);
        return nullptr;
    }

    // 3. MAPEAR MEMORIA (Tu código original, ahora completo)
    SharedCommandData *cmd_ptr = (SharedCommandData *)mmap(
        NULL,
        SHM_CMD_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd_shm,
        0);

    if (cmd_ptr == MAP_FAILED)
    {
        std::cerr << "❌ Error: Falló mmap para SHM de comandos.\n";
        // Cierra el descriptor si falla el mapeo
        close(fd_shm);
        return nullptr;
    }

    // 4. INICIALIZACIÓN
    // Inicializar el estado y la bandera (Solo si sabes que lo estás creando)
    // Para ser robustos, inicializamos siempre a un estado conocido.
    cmd_ptr->command = 'k';
    cmd_ptr->updated = false;

    std::cout << "📡 Memoria compartida de comandos lista. Tamaño: " << SHM_CMD_SIZE << " bytes.\n";
    return cmd_ptr;
}
// ==========
int *open_robot_audio_shm(int &fd_shm)
{
    // Abrir o crear la memoria compartida de AUDIO
    fd_shm = shm_open(SHM_AUDIO_NAME, O_CREAT | O_RDWR, 0666);
    if (fd_shm == -1)
    {
        std::cerr << "❌ Error creando/abriendo SHM de audio\n";
        return nullptr;
    }

    // Ajustar tamaño al tamaño de un entero (int)
    if (ftruncate(fd_shm, SHM_AUDIO_SIZE) == -1)
    {
        std::cerr << "❌ Error con ftruncate en SHM de audio\n";
        return nullptr;
    }

    // Mapear memoria
    int *audio_ptr = (int *)mmap(NULL, SHM_AUDIO_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_shm, 0);
    if (audio_ptr == MAP_FAILED)
    {
        std::cerr << "❌ Error mapeando memoria compartida de audio\n";
        return nullptr;
    }

    *audio_ptr = 0; // Inicializar a 0 (limpio)
    std::cout << "📡 Memoria compartida de audio lista. ID inicial: 0\n";
    return audio_ptr;
}

// CINEMÁTICA CON CORRECCIÓN CONDICIONAL
// =============================================================================
AngulosRobot calcular_angulos(GeneradorMarcha &m, Compensacion comp)
{
    AngulosRobot angulos;
    double body_yaw = 0.0;

    for (int leg = 0; leg < 4; leg++)
    {
        bool pisando = m.en_suelo[leg];

        // Solo aplicamos compensación si la pata está en el suelo
        double final_x = m.patas[leg].x - (pisando ? comp.x : 0.0);
        double final_y = m.patas[leg].y - (pisando ? comp.y : 0.0);
        double final_z = m.patas[leg].z;

        double eff_roll = pisando ? comp.roll : 0.0;
        double eff_pitch = pisando ? comp.pitch : 0.0;

        angulos[leg] = IK::kinematics_array(leg + 1, final_x, final_y, final_z,
                                            eff_roll, eff_pitch, body_yaw);
    }
    return angulos;
}

// CÓMO DEBERÍA SER LA FIRMA (NO recibe nada, usa las globales)
// Además, la función ya NO NECESITA LOS ARGUMENTOS OBSOLETOS.

AngulosRobot calcular_angulos_pos()
{
    AngulosRobot angulos;
    for (int leg = 0; leg < 4; leg++)

    {
        // Usa las variables globales 'patas_pos' y 'orientacion'
        angulos[leg] = IK::kinematics_array(leg + 1, patas_pos[leg].x, patas_pos[leg].y, patas_pos[leg].z,
                                            orientacion[0], orientacion[1], orientacion[2]); // Ahora orientacion[0] es válido
    }
    return angulos;
}

// =============================================================================
// CONTEXTO Y TAREA
// =============================================================================
struct RobotContext
{
    SharedInputData *shared_input = nullptr;
    OrientationData *shared_data = nullptr;

    GeneradorMarcha marcha; // Instancia única aquí

    double ini_roll = 0.0;
    double ini_pitch = 0.0;

    double comp_trans_x_fil = 0.0;
    double comp_trans_y_fil = 0.0;
    double comp_roll_fil = 0.0;
    double comp_pitch_fil = 0.0;
};

void tarea_control_robot(uv_timer_t *handle)
{
    RobotContext *ctx = static_cast<RobotContext *>(handle->data);

    // Inicialización fuera del IF (Solución al Problema #1)
    uint64_t next_tick = 50;

    if (estado_actual == MARCHA)
    {
        // 1. Inputs (Ya sabemos que están activos por gestor_estados)
        double input_x = ctx->shared_input->cmd_x;
        double input_y = ctx->shared_input->cmd_y;
        double input_giro = ctx->shared_input->cmd_giro;

        // 2. Lógica IMU y Filtros
        double r_roll = ctx->shared_data->roll - ctx->ini_roll;
        double r_pitch = ctx->shared_data->pitch - ctx->ini_pitch;

        // ... (resto de filtros) ...

        Compensacion correccion = {ctx->comp_trans_x_fil, ctx->comp_trans_y_fil, ctx->comp_roll_fil, ctx->comp_pitch_fil};

        // 3. Marcha y Actualización
        double duracion_paso = ctx->marcha.update(input_x, input_y, input_giro);

        AngulosRobot angulos = calcular_angulos(ctx->marcha, correccion);
        for (int leg = 0; leg < 4; leg++)
        {
            for (int j = 0; j < 3; j++)
            {
                int p = static_cast<int>(angulos[leg][j] * SCALE) + motores[leg].offsets[j];
                p = std::clamp(p, PULSE_MIN, PULSE_MAX);
                move_servo(motores[leg].ids[j], p, duracion_paso);
            }
        }

        // 4. Re-programar Timer
        next_tick = (uint64_t)duracion_paso;
    } // <--- LLAVE DE CIERRE CLARA para el bloque MARCHA

    // 5. Siempre se ejecuta para mantener el loop activo
    if (next_tick < 20)
        next_tick = 20;
    uv_timer_set_repeat(handle, next_tick);
}

// Helper SHM
template <typename T>
T *open_shm(const char *name, bool create_mode)
{
    int flags = create_mode ? (O_CREAT | O_RDWR) : (O_RDWR);
    int fd = shm_open(name, flags, 0666);
    if (fd == -1)
        return nullptr;
    if (create_mode)
        ftruncate(fd, sizeof(T));
    void *ptr = mmap(0, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    return (ptr == MAP_FAILED) ? nullptr : static_cast<T *>(ptr);
}

char ant_cmd = 'k';
void selec_modo(bool condi)
{

    // 1. LECTURA Y PROCESAMIENTO DE COMANDO SHM
    if (shm_cmd_ptr != nullptr)
    {
        if (shm_cmd_ptr->updated) // ⬅️ ¡NUEVA LÓGICA DE DETECCIÓN!
        {
            char nuevo_cmd = shm_cmd_ptr->command;

            // Procesar el comando solo si está actualizado
            command_char = (char)tolower((unsigned char)nuevo_cmd);

            // LIMPIAR LA BANDERA: Indicamos que el robot ya leyó el comando
            shm_cmd_ptr->updated = false;

            std::cout << "📥 Comando SHM detectado y procesado: '" << command_char << "'" << std::endl;
        }
    }

    if (condi && estado_anterior == STOP)
    {
        estado_actual = MARCHA;
        move_motores = true;
    }
    else
    {
        if (estado_actual == MARCHA)
        {
            estado_actual = STOP;
        }
        if (ant_cmd != command_char)
        {
            switch (command_char) // <--- Corregido: 'command_char' como argumento del switch
            {
            case 'k':
                estado_actual = STOP;
                printf("\n[COMANDO] 🟢 Estado cambiado a: STOP/Reposo\n");
                break;
            case 'w':
                estado_actual = SENTADO;
                printf("\n[COMANDO] 🟠 Estado cambiado a: SENTADO\n");
                break;
            case 'q':
                send_audio_command(1);
                estado_actual = INICIO;
                printf("\n[COMANDO] 🟠 Estado cambiado a: SEC_inico\n");
                break;
            case 'v':
                send_audio_command(4);
                estado_actual = MUERTO;
                printf("\n[COMANDO] 🟠 Estado cambiado a: MUERTO\n");
                break;
            case 'd':
                estado_actual = BAILAR;
                printf("\n[COMANDO] 🟠 Estado cambiado a: BAILAR\n");
                break;
            case 'p':
                estado_actual = CONTACTO;
                printf("\n[COMANDO] 🟠 Estado cambiado a: CONTACTO\n");
                break;
            case 'x':
                once = true;
                estado_actual = DETECTAR;
                printf("\n[COMANDO] 🟠 Estado cambiado a: Detectar\n");
                break;
            default:
                // No hacer nada si el comando no es reconocido
                break;
            }
            ant_cmd = command_char;
        }

        if (estado_actual != estado_anterior)
        {
            estado_anterior = estado_actual;
            move_motores = true;
        }
    }
}

void parado()
{
    patas_pos[0] = {-30.0, 0.0, 105.0};
    patas_pos[1] = {-30.0, 0.0, 105.0};
    patas_pos[2] = {30.0, 0.0, 105.0};
    patas_pos[3] = {30.0, 0.0, 105.0};
    orientacion[0] = 0.0;
    orientacion[1] = 0.0;
    orientacion[2] = 0.0;
}

int paso = -1;
std::chrono::steady_clock::time_point tiempo_inicio_espera; // Cuándo empezó la espera
int duracion_secuencia_ms = 0;
bool espera = 0;

bool espera_sec(std::chrono::steady_clock::time_point tiempo_inicio, int duracion_ms)
{
    auto ahora = std::chrono::steady_clock::now();

    auto transcurrido = std::chrono::duration_cast<std::chrono::milliseconds>(
                            ahora - tiempo_inicio)
                            .count();

    // Devuelve true si aún estamos esperando
    return transcurrido < duracion_ms;
}

int vel_inicio = 1000;
int repeticion = 0;
int rep_global = 0;
void sec_inicio()
{
    // Si estamos esperando que termine un paso anterior, no hacemos nada más que comprobar.
    if (espera)
    {
        espera = espera_sec(tiempo_inicio_espera, vel_inicio);
        return; // Detenemos la ejecución de la función hasta que el paso termine.
    }

    tiempo_inicio_espera = std::chrono::steady_clock::now();

    paso++;
    move_motores = true;
    if (paso == 0)
    {
        printf("[SECUENCIA] Paso Cero: Posición Inicial Estática. Duración: %dms.\n", 1500);
        patas_pos[0] = {0.0, 0.0, 80.0};
        patas_pos[1] = {0.0, 0.0, 80.0};
        patas_pos[2] = {0.0, 0.0, 80.0};
        patas_pos[3] = {0.0, 0.0, 80.0};
        orientacion[0] = 0.0;
        orientacion[1] = 0.0;
        orientacion[2] = 0.0;
        vel_inicio = 1500;
        espera = true;
    }
    else if (paso == 1)
    {
        patas_pos[0] = {0.0, 0.0, 120.0};
        patas_pos[1] = {0.0, 0.0, 120.0};
        patas_pos[2] = {0.0, 0.0, 120.0};
        patas_pos[3] = {0.0, 0.0, 120.0};
        orientacion[0] = 0.0;
        orientacion[1] = 0.0;
        orientacion[2] = 0.0;
        vel_inicio = 1500;
        espera = true;
    }
    else if (paso == 2)
    {
        patas_pos[0] = {0.0, 0.0, 130.0};
        patas_pos[1] = {0.0, 0.0, 130.0};
        patas_pos[2] = {0.0, 0.0, 60.0};
        patas_pos[3] = {0.0, 0.0, 60.0};
        orientacion[0] = 0.0;
        orientacion[1] = 0.0;
        orientacion[2] = 0.0;
        vel_inicio = 1500;
        espera = true;
    }
    else if (paso == 3)
    {
        patas_pos[0] = {0.0, 0.0, 60.0};
        patas_pos[1] = {0.0, 0.0, 60.0};
        patas_pos[2] = {0.0, 0.0, 130.0};
        patas_pos[3] = {0.0, 0.0, 130.0};
        vel_inicio = 1500;
        espera = true;
    }
    else if (paso == 4)
    {
        patas_pos[0] = {0.0, 0.0, 120.0};
        patas_pos[1] = {0.0, 0.0, 120.0};
        patas_pos[2] = {0.0, 0.0, 120.0};
        patas_pos[3] = {0.0, 0.0, 120.0};
        vel_inicio = 1500;
        espera = true;
    }
    else if (paso == 5)
    {
        orientacion[0] = 15.0;
        patas_pos[0] = {0.0, 0.0, 130.0};
        patas_pos[1] = {0.0, 0.0, 130.0};
        patas_pos[2] = {0.0, 0.0, 130.0};
        patas_pos[3] = {0.0, 0.0, 130.0};
        vel_inicio = 1200;
        espera = true;
    }
    else if (paso == 6)
    {
        orientacion[0] = -15.0;
        patas_pos[0] = {0.0, 0.0, 130.0};
        patas_pos[1] = {0.0, 0.0, 130.0};
        patas_pos[2] = {0.0, 0.0, 130.0};
        patas_pos[3] = {0.0, 0.0, 130.0};
        vel_inicio = 1200;
        espera = true;
    }
    else if (paso == 7)
    {
        repeticion++;
        if (repeticion < 2)
        {
            paso = 4;
        }
    }
    else if (paso == 8)
    {
        repeticion = 0;
        orientacion[0] = 0.0;
        espera = true;
    }
    else if (paso == 9)
    {
        patas_pos[0] = {40.0, 0.0, 130.0};
        patas_pos[1] = {40.0, 0.0, 130.0};
        patas_pos[2] = {40.0, 0.0, 130.0};
        patas_pos[3] = {40.0, 0.0, 130.0};
        espera = true;
        vel_inicio = 1600;
    }
    else if (paso == 10)
    {
        patas_pos[0] = {-40.0, 0.0, 130.0};
        patas_pos[1] = {-40.0, 0.0, 130.0};
        patas_pos[2] = {-40.0, 0.0, 130.0};
        patas_pos[3] = {-40.0, 0.0, 130.0};
        espera = true;
        vel_inicio = 1600;
    }
    else if (paso == 11)
    {
        patas_pos[0] = {0.0, 30.0, 130.0};
        patas_pos[1] = {0.0, 30.0, 130.0};
        patas_pos[2] = {0.0, -30.0, 130.0};
        patas_pos[3] = {0.0, -30.0, 130.0};
        espera = true;
    }
    else if (paso == 12)
    {
        patas_pos[0] = {0.0, -30.0, 130.0};
        patas_pos[1] = {0.0, -30.0, 130.0};
        patas_pos[2] = {0.0, 30.0, 130.0};
        patas_pos[3] = {0.0, 30.0, 130.0};
        espera = true;
    }
    else if (paso == 13)
    {
        patas_pos[0] = {0.0, 0.0, 130.0};
        patas_pos[1] = {0.0, 0.0, 130.0};
        patas_pos[2] = {0.0, 0.0, 130.0};
        patas_pos[3] = {0.0, 0.0, 130.0};
        espera = true;
    }
    else if (paso == 14)
    {
        patas_pos[0] = {0.0, 0.0, 140.0};
        patas_pos[1] = {0.0, 0.0, 140.0};
        patas_pos[2] = {50.0, 0.0, 40.0};
        patas_pos[3] = {50.0, 0.0, 40.0};
        espera = true;
    }
    else if (paso == 15)
    {
        patas_pos[0] = {-70.0, 0.0, 80.0};
        patas_pos[1] = {0.0, 0.0, 130.0};
        patas_pos[2] = {50.0, 0.0, 40.0};
        patas_pos[3] = {50.0, 0.0, 40.0};
        espera = true;
        vel_inicio = 1500;
    }
    else if (paso == 16)
    {
        patas_pos[0] = {-70.0, 0.0, 25.0};
        patas_pos[1] = {0.0, 0.0, 130.0};
        patas_pos[2] = {50.0, 0.0, 40.0};
        patas_pos[3] = {50.0, 0.0, 40.0};
        espera = true;
        vel_inicio = 1500;
    }
    else if (paso == 17)
    {
        patas_pos[0] = {-70.0, 0.0, 60.0};
        patas_pos[1] = {0.0, 0.0, 130.0};
        patas_pos[2] = {50.0, 0.0, 40.0};
        patas_pos[3] = {50.0, 0.0, 40.0};
        espera = true;
        repeticion++;
        vel_inicio = 1500;
    }
    else if (paso == 18)
    {
        if (repeticion < 3)
        {
            paso = 14;
        }
    }
    else if (paso == 19)
    {
        patas_pos[0] = {0.0, 0.0, 130.0};
        patas_pos[1] = {0.0, 0.0, 130.0};
        patas_pos[2] = {0.0, 0.0, 130.0};
        patas_pos[3] = {0.0, 0.0, 130.0};
    }

    else if (paso == 20)
    {
        patas_pos[0] = {0.0, 0.0, 90.0};
        patas_pos[1] = {10.0, 0.0, 145.0};
        patas_pos[2] = {-10.0, 0.0, 135.0};
        patas_pos[3] = {-10.0, 0.0, 145.0};
    }

    else if (paso == 21)
    {
        if (rep_global == 1)
        {
            command_char = 'k';
            rep_global = 0;
        }
        paso = -1;
        rep_global++;
    }
}

void sec_bailar()
{
    // Si estamos esperando que termine un paso anterior, no hacemos nada más que comprobar.
    if (espera)
    {
        espera = espera_sec(tiempo_inicio_espera, vel_inicio);
        return; // Detenemos la ejecución de la función hasta que el paso termine.
    }

    tiempo_inicio_espera = std::chrono::steady_clock::now();
    paso++;
    move_motores = true;
    if (paso == 0)
    {
        orientacion[0] = -10.0;
        orientacion[1] = 5.0;
        patas_pos[0] = {0.0, 0.0, 130.0};
        patas_pos[1] = {0.0, 0.0, 130.0};
        patas_pos[2] = {0.0, 0.0, 130.0};
        patas_pos[3] = {0.0, 0.0, 130.0};
        vel_inicio = 700;
        espera = true;
    }
    else if (paso == 1)
    {
        orientacion[0] = 10.0;
        orientacion[1] = -5.0;
        patas_pos[0] = {0.0, 0.0, 130.0};
        patas_pos[1] = {0.0, 0.0, 130.0};
        patas_pos[2] = {0.0, 0.0, 130.0};
        patas_pos[3] = {0.0, 0.0, 130.0};
        vel_inicio = 700;
        espera = true;
    }
    else if (paso == 2)
    {
        repeticion++;
        if (repeticion < 4)
        {
            paso = -1;
        }
    }
    else if (paso == 3)
    {
        command_char = 'k';
        velocidad_stop = 400;
    }
}

void sec_muerto()
{
    patas_pos[0] = {-150.0, 0.0, 10.0};
    patas_pos[1] = {-150.0, 0.0, 10.0};
    patas_pos[2] = {150.0, 0.0, 10.0};
    patas_pos[3] = {150.0, 0.0, 10.0};
    velocidad_stop = 3000;
}
void sec_sentado()
{
    patas_pos[0] = {-10.0, 0.0, 120.0};
    patas_pos[1] = {-10.0, 0.0, 120.0};
    patas_pos[2] = {-10.0, 0.0, 10.0};
    patas_pos[3] = {-10.0, 0.0, 10.0};
    orientacion[0] = 0.0;
    orientacion[1] = 0.0;
    orientacion[2] = 0.0;
    velocidad_stop = 2500;
}

void move_pos_motores(double vel_move)
{
    if (move_motores)
    {
        AngulosRobot angulos = calcular_angulos_pos();
        for (int leg = 0; leg < 4; leg++)
        {
            for (int j = 0; j < 3; j++)
            {
                int p = static_cast<int>(angulos[leg][j] * SCALE) + motores[leg].offsets[j];
                p = std::clamp(p, PULSE_MIN, PULSE_MAX);
                move_servo(motores[leg].ids[j], p, vel_move);
            }
        }
    }
}


int sockfd;
void sec_detectar()
{

    int puerto = 5010;
    int estado = 1; // 0 = No escuchar, 1 = Escuchar
    char buffer[1024];

    if (once)
    {
        // 1. Crear Socket UDP
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);

        // 2. Configurar el socket como NO BLOQUEANTE
        fcntl(sockfd, F_SETFL, O_NONBLOCK);

        struct sockaddr_in servaddr;
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(puerto);

        // Vincular puerto
        if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        {
            std::cerr << "Error en bind" << std::endl;
        }
        once = false;
        std::cout << "Iniciado. Esperando comandos en estado 1..." << std::endl;
    }

    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);

    // Intenta recibir datos
    int n = recvfrom(sockfd, (char *)buffer, 1024, 0, (struct sockaddr *)&cliaddr, &len);

    if (n > 0)
    {
        buffer[n] = '\0';
        std::string mensaje(buffer);

        int mensaje_int = atoi(buffer);
        send_audio_command(mensaje_int);
        std::cout << "Dato recibido en Estado 1: " << mensaje << std::endl;
    }
}

// Variables para el conteo de toques
int contador_toques = 0;
bool detectando_toques = false;
std::chrono::steady_clock::time_point tiempo_inicio_toques;
int toques_finales = 0; // Aquí se guardará el resultado final
int ultimo_estado_contacto = LOW;
bool sentado = false;
void sec_contacto()
{
    move_motores = true;
    // 1. Iniciar el cronómetro si es el primer toque o si reseteamos la lógica
    if (!detectando_toques)
    {
        tiempo_inicio_toques = std::chrono::steady_clock::now();
        contador_toques = 0;
        detectando_toques = true;
    }

    // 2. Leer pines
    int estado_1 = digitalRead(PIN_LECTURA_1);
    int estado_2 = digitalRead(PIN_LECTURA_2);
    int estado_actual = (estado_1 == HIGH || estado_2 == HIGH) ? HIGH : LOW;

    // 3. Lógica de conteo (solo cuenta cuando pasa de LOW a HIGH)
    if (estado_actual == HIGH && ultimo_estado_contacto == LOW)
    {
        contador_toques++;
        std::cout << "¡Toque detectado! Conteo: " << contador_toques << std::endl;
    }
    ultimo_estado_contacto = estado_actual;

    // 4. Comprobar si ya pasó 1 segundo
    auto ahora = std::chrono::steady_clock::now();
    auto transcurrido = std::chrono::duration_cast<std::chrono::milliseconds>(ahora - tiempo_inicio_toques).count();

    if (transcurrido >= 1500)
    {
        // --- RESULTADO FINAL ---
        toques_finales = contador_toques;
        std::cout << "✅ Intervalo terminado. Toques totales en 1s: " << toques_finales << std::endl;
        // Reiniciar para el siguiente intervalo
        detectando_toques = false;

        if (contador_toques >= 2 && contador_toques <= 3 && !sentado)
        {
            std::cout << "Acción: Sentarse" << std::endl;
            sec_sentado();
            move_pos_motores(3500);
            move_motores = false;
            sentado = true;
        }
        else if (contador_toques > 3 && !sentado)
        {
            command_char = 'd';
        }
        else if (contador_toques >= 1 && sentado) // Solo se evalúa si el de arriba fue falso
        {
            std::cout << "Acción: Pararse" << std::endl;
            parado();
            move_pos_motores(2000);
            move_motores = false;
            sentado = false;
        }
    }
}

void gestor_estados(uv_timer_t *handle)
{
    RobotContext *ctx = static_cast<RobotContext *>(handle->data); // NECESARIO
    bool input_activo = (ctx->shared_input->active);
    selec_modo(input_activo);

    if (estado_actual == STOP)
    {
        parado();
        move_pos_motores(velocidad_stop);
        move_motores = false;
        velocidad_stop = 400;
    }
    else if (estado_actual == SENTADO)
    {
        sec_sentado();
        move_pos_motores(3500);
        move_motores = false;
    }
    else if (estado_actual == INICIO)
    {
        sec_inicio();
        move_pos_motores(vel_inicio);
        move_motores = false;
    }
    else if (estado_actual == MUERTO)
    {
        sec_muerto();
        move_pos_motores(velocidad_stop);
        move_motores = false;
    }
    else if (estado_actual == BAILAR)
    {
        sec_bailar();
        move_pos_motores(vel_inicio);
        move_motores = false;
    }
    else if (estado_actual == CONTACTO)
    {
        sec_contacto();
    }
    else if (estado_actual == DETECTAR)
    {

        sec_detectar();
    }
}

void pines_setup()
{
    // 1. Inicializar wiringPi usando la numeración BCM
    if (wiringPiSetupGpio() == -1)
    {
        std::cerr << "Error al inicializar wiringPi." << std::endl;
    }

    // --- Configuración de Pines ---

    // 2. Configurar el GPIO 24 como entrada y Pull-Down
    pinMode(PIN_LECTURA_1, INPUT);
    pullUpDnControl(PIN_LECTURA_1, PUD_DOWN);

    // 3. Configurar el GPIO 25 como entrada y Pull-Down
    pinMode(PIN_LECTURA_2, INPUT);
    pullUpDnControl(PIN_LECTURA_2, PUD_DOWN);
}
// =============================================================================
// MAIN
// =============================================================================
int main()
{
    if (!serial_open("/dev/servos", 115200))
        return 1;
    pines_setup();
    // 1. Memorias Compartidas
    SharedInputData *shared_input = open_shm<SharedInputData>(SHM_INPUT_NAME, false);

    int shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
    int fd_cmd; // Descriptor para la memoria compartida
    if (shm_fd < 0)
    {
        std::cerr << "Error IMU SHM\n";
        return 1;
    }
    OrientationData *shared_data = static_cast<OrientationData *>(
        mmap(nullptr, sizeof(OrientationData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));

    while (!shared_input)
    {
        std::cerr << "⏳ Esperando control...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
        shared_input = open_shm<SharedInputData>(SHM_INPUT_NAME, false);
    }

    double *shared_voltage = open_shm<double>(SHM_VOLT_NAME, true);
    if (shared_voltage)
        *shared_voltage = 0.0;

    shm_cmd_ptr = open_robot_command_shm(fd_cmd);
    if (!shm_cmd_ptr)
        return -1;

    int fd_audio; // 🟢 Descriptor para la SHM de audio
    shm_audio_ptr = open_robot_audio_shm(fd_audio);
    if (!shm_audio_ptr)
        std::cerr << "⚠️ Advertencia: El servidor de audio no está activo o la SHM falló.\n";
    std::cout << "🚀 ATOM-51 INICIADO (Modo libuv).\n";

    // 2. Inicializar Loop
    uv_loop_t *loop = (uv_loop_t *)malloc(sizeof(uv_loop_t));
    uv_loop_init(loop);

    // 3. Crear Contexto
    RobotContext robot_ctx;
    robot_ctx.shared_input = shared_input;
    robot_ctx.shared_data = shared_data;
    robot_ctx.ini_roll = shared_data->roll;
    robot_ctx.ini_pitch = shared_data->pitch;

    // -------------------------------------------------------------
    // TIMER 1: CONTROL DE MARCHA (Velocidad Dinámica)
    // -------------------------------------------------------------
    uv_timer_t timer_control;
    uv_timer_init(loop, &timer_control);
    timer_control.data = &robot_ctx;
    // Arranca en 0, repetición inicial 100 (se sobrescribe dinámicamente)
    uv_timer_start(&timer_control, tarea_control_robot, 0, 100);

    // -------------------------------------------------------------
    // TIMER 2: MÁQUINA DE ESTADOS (Fijo 50ms) <--- NUEVO BLOQUE
    // -------------------------------------------------------------
    uv_timer_t timer_estados;            // 1. Declaramos nueva variable de timer
    uv_timer_init(loop, &timer_estados); // 2. Inicializamos
    timer_estados.data = &robot_ctx;     // 3. Compartimos el MISMO contexto (memoria)

    uv_timer_start(&timer_estados, gestor_estados, 0, 50);

    // 6. Loop infinito (Maneja ambos timers en paralelo)
    uv_run(loop, UV_RUN_DEFAULT);

    // Limpieza
    free(loop);
    return 0;
}
