
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <uv.h> // Librería libuv
#include <iostream>
#include <string>
#include <array>
#include <vector>
#include <cmath>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include <numeric>
#include <algorithm>
#include <map>
#include <functional>
#include <limits>
#include <cstdio> // Para fgetc

#include "kinematics.hpp"
#include "BusServoControl.hpp" // Librería de comunicación con servos

// ======================================================
// 1. GESTOR DE ESTADOS (La "Tarea" Controlada)
// ======================================================

bool DEBUG = false;

// ======================================================
// 📌 GESTIÓN DE COMANDOS POR MEMORIA COMPARTIDA
// ======================================================
#define SHM_CMD_NAME "/robot_cmd"
#define SHM_CMD_SIZE sizeof(char)

char *shm_cmd_ptr = nullptr; // Puntero al comando en memoria

// 🟢 Handler Global para el Timer de Movimiento
uv_timer_t movement_timer;
uv_loop_t *loop_global = uv_default_loop();
// Declaración de la nueva función de continuación (el "callback" del timer)
void on_movement_complete(uv_timer_t *handle);
// Definición de estados posibles
enum Estado
{
    STOP,
    ADELANTE,
    IZQUIERDA,
    DERECHA,
    ATRAS,
    GIRO_DERECHA,
    GIRO_IZQUIERDA
};

using Posicion3D = std::array<double, 3>;
// ======================================================
// Estructura y Vector de motores
// ======================================================
struct MotorLeg
{
    std::string leg;
    std::array<int, 3> ids;
    std::array<int, 3> offsets;
    std::array<int, 3> llego;
};
// 🔴 ANTES ERA: std::vector<MotorLeg> motores = ...

// 🟢 CAMBIAR A ESTO:
std::array<MotorLeg, 4> motores = {{{"FL", {7, 8, 9}, {-10, 7, -5}, {0, 0, 0}},
                                    {"FR", {4, 5, 6}, {30, -25, -45}, {0, 0, 0}},
                                    {"BL", {1, 2, 3}, {-25, 10, 0}, {0, 0, 0}},
                                    {"BR", {10, 11, 12}, {0, 10, -20}, {0, 0, 0}}}};
// ======================================================
static std::array<std::array<double, 3>, 4> current_feet_positions = {{{0, 0, 130}, {0, 0, 130}, {0, 0, 130}, {0, 0, 130}}};
static std::array<std::array<double, 3>, 4> last_sent_feet_positions = {{{0, 0, 130}, {0, 0, 130}, {0, 0, 130}, {0, 0, 130}}};
static std::array<double, 3> current_body_orientation = {0, 0, 0};
static std::array<int, 4> current_move_time_ms_per_leg = {500, 500, 500, 500};

int TIEMPO_BASE_MARCHA_MS = 130;
int tiempo_leg_move = 40;

// --- Alturas (Eje Z) ---
const double Z_SUELO = 130.0;  // Altura cuando la pata toca el suelo
const double Z_AIRE = 100.0;   // Altura cuando la pata se levanta
const double Z_INICIO = 105.0; // Altura de la pose inicial (agachado)

// --- Centros de las Patas (Eje X - Longitudinal) ---
const double X_FRONTAL = -30.0; // Posición base pata delantera
const double X_TRASERA = 30.0;  // Posición base pata trasera

const double X_FRONT_LADO = -15.0; // Posición base delantera (especial para marcha lateral)

// --- Desplazamientos (Deltas) ---
const double PASO_X = 20.0;        // Longitud del paso hacia adelante/atrás
const double PASO_LADO_Y = 15.0;   // Longitud del paso lateral
const double APERTURA_GIRO = 10.0; // Cuanto se abre la pata para girar

// --- Trayectoria Adelante
// --- Posiciones de Inicio (Home) ---
Posicion3D ini_F = {X_FRONTAL, 0.0, Z_INICIO};
Posicion3D ini_B = {X_TRASERA, 0.0, Z_INICIO};

// --- Trayectoria Adelante (Forward) ---
// Frente: Se estira hacia adelante (-20 - 15 = -35)
Posicion3D s_1 = {X_FRONTAL, -15.0, Z_SUELO};
Posicion3D s_2 = {X_FRONTAL - PASO_X, -15.0, Z_AIRE};
Posicion3D s_3 = {X_FRONTAL - PASO_X, -15.0, Z_SUELO};

// Atrás: Se recoge hacia adelante (20 - 15 = 5)
Posicion3D s_1_atras = {X_TRASERA, -10.0, Z_SUELO};
Posicion3D s_2_atras = {X_TRASERA - PASO_X, -15.0, Z_AIRE};
Posicion3D s_3_atras = {X_TRASERA - PASO_X, -15.0, Z_SUELO};

// --- Trayectoria Atrás (Backward) ---
// Frente: Se recoge hacia atrás (-20 + 15 = -5)
Posicion3D s_1_back = {X_FRONTAL, 0.0, Z_SUELO};
Posicion3D s_2_back = {X_FRONTAL + PASO_X, 0.0, Z_AIRE};
Posicion3D s_3_back = {X_FRONTAL + PASO_X, 0.0, Z_SUELO};

// Atrás: Se estira hacia atrás (20 + 15 = 35)
Posicion3D s_1_atras_back = {X_TRASERA, 0.0, Z_SUELO};
Posicion3D s_2_atras_back = {X_TRASERA + PASO_X, 0.0, Z_AIRE};
Posicion3D s_3_atras_back = {X_TRASERA + PASO_X, 0.0, Z_SUELO};

// --- Trayectoria Lateral Izquierda ---
Posicion3D s_1_lado = {X_FRONT_LADO, 0.0, Z_SUELO};
Posicion3D s_2_lado = {X_FRONT_LADO, PASO_LADO_Y, Z_AIRE};
Posicion3D s_3_lado = {X_FRONT_LADO, PASO_LADO_Y, Z_SUELO};

Posicion3D s_1_atras_lado = {X_TRASERA, 0.0, Z_SUELO};
Posicion3D s_2_atras_lado = {X_TRASERA, PASO_LADO_Y, Z_AIRE};
Posicion3D s_3_atras_lado = {X_TRASERA, PASO_LADO_Y, Z_SUELO};

// --- Trayectoria Lateral Derecha (Invertimos Y) ---
Posicion3D s_1_lado_D = {X_FRONT_LADO, 0.0, Z_SUELO};
Posicion3D s_2_lado_D = {X_FRONT_LADO, -PASO_LADO_Y, Z_AIRE};
Posicion3D s_3_lado_D = {X_FRONT_LADO, -PASO_LADO_Y, Z_SUELO};

Posicion3D s_1_atras_lado_D = {X_TRASERA, 0.0, Z_SUELO};
Posicion3D s_2_atras_lado_D = {X_TRASERA, -PASO_LADO_Y, Z_AIRE};
Posicion3D s_3_atras_lado_D = {X_TRASERA, -PASO_LADO_Y, Z_SUELO};

// --- Trayectoria Giro Izquierda ---
// Combina paso hacia adelante + apertura lateral positiva
Posicion3D s_1_GI = {X_FRONTAL, 0.0, Z_SUELO};
Posicion3D s_2_GI = {X_FRONTAL - PASO_X, APERTURA_GIRO, Z_AIRE};
Posicion3D s_3_GI = {X_FRONTAL - PASO_X, APERTURA_GIRO, Z_SUELO};

Posicion3D s_1_atras_GI = {X_TRASERA, 0.0, Z_SUELO};
Posicion3D s_2_atras_GI = {X_TRASERA - PASO_X, APERTURA_GIRO, Z_AIRE};
Posicion3D s_3_atras_GI = {X_TRASERA - PASO_X, APERTURA_GIRO, Z_SUELO};

// Inversas para el giro (Back)
Posicion3D s_1_back_GI = {X_FRONTAL, 0.0, Z_SUELO};
Posicion3D s_2_back_GI = {X_FRONTAL + PASO_X, APERTURA_GIRO, Z_AIRE};
Posicion3D s_3_back_GI = {X_FRONTAL + PASO_X, APERTURA_GIRO, Z_SUELO};

Posicion3D s_1_atras_back_GI = {X_TRASERA, 0.0, Z_SUELO};
Posicion3D s_2_atras_back_GI = {X_TRASERA + PASO_X, APERTURA_GIRO, Z_AIRE};
Posicion3D s_3_atras_back_GI = {X_TRASERA + PASO_X, APERTURA_GIRO, Z_SUELO};

// --- Trayectoria Giro Derecha (Invertimos Y) ---
Posicion3D s_1_GD = {X_FRONTAL, 0.0, Z_SUELO};
Posicion3D s_2_GD = {X_FRONTAL - PASO_X, -APERTURA_GIRO, Z_AIRE};
Posicion3D s_3_GD = {X_FRONTAL - PASO_X, -APERTURA_GIRO, Z_SUELO};

Posicion3D s_1_atras_GD = {X_TRASERA, 0.0, Z_SUELO};
Posicion3D s_2_atras_GD = {X_TRASERA - PASO_X, -APERTURA_GIRO, Z_AIRE};
Posicion3D s_3_atras_GD = {X_TRASERA - PASO_X, -APERTURA_GIRO, Z_SUELO};

Posicion3D s_1_back_GD = {X_FRONTAL, 0.0, Z_SUELO};
Posicion3D s_2_back_GD = {X_FRONTAL + PASO_X, -APERTURA_GIRO, Z_AIRE};
Posicion3D s_3_back_GD = {X_FRONTAL + PASO_X, -APERTURA_GIRO, Z_SUELO};

Posicion3D s_1_atras_back_GD = {X_TRASERA, 0.0, Z_SUELO};
Posicion3D s_2_atras_back_GD = {X_TRASERA + PASO_X, -APERTURA_GIRO, Z_AIRE};
// NOTA: Aquí corregí un signo positivo que tenías en tu código original en la última línea
Posicion3D s_3_atras_back_GD = {X_TRASERA + PASO_X, -APERTURA_GIRO, Z_SUELO};

const int PATA_FL = 0; // Frontal Izquierda
const int PATA_FR = 1; // Frontal Derecha
const int PATA_BL = 2; // Trasera Izquierda
const int PATA_BR = 3; // Trasera Derecha

int pata_1 = PATA_FL;
int pata_2 = PATA_FR;
int pata_3 = PATA_BL;
int pata_4 = PATA_BR;

static enum Estado estado_actual = STOP;
static enum Estado estado_anterior = STOP;
bool fin_ciclo = false;
constexpr double SCALE = 1000.0 / 240.0;

std::array<std::array<double, 3>, 4>
calcular_angulos()
{
    std::array<std::array<double, 3>, 4> angulos;
    for (int leg = 0; leg < 4; leg++)
    {
        angulos[leg] = IK::kinematics_array(
            leg + 1,
            current_feet_positions[leg][0],
            current_feet_positions[leg][1],
            current_feet_positions[leg][2],
            current_body_orientation[0],
            current_body_orientation[1],
            current_body_orientation[2]);
    }
    return angulos;
}

bool movement_complete_flag = true;

// 🟢 Nueva función para abrir la memoria compartida de comandos
char *open_robot_command_shm(int &fd_shm)
{
    // Abrir o crear la memoria compartida
    fd_shm = shm_open(SHM_CMD_NAME, O_CREAT | O_RDWR, 0666);
    if (fd_shm == -1)
    {
        std::cerr << "❌ Error creando/abriendo SHM de comandos\n";
        return nullptr;
    }

    // Ajustar tamaño (solo 1 byte para el char)
    if (ftruncate(fd_shm, SHM_CMD_SIZE) == -1)
    {
        std::cerr << "❌ Error con ftruncate en SHM de comandos\n";
        return nullptr;
    }

    // Mapear memoria
    char *cmd_ptr = (char *)mmap(NULL, SHM_CMD_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_shm, 0);
    if (cmd_ptr == MAP_FAILED)
    {
        std::cerr << "❌ Error mapeando memoria compartida de comandos\n";
        return nullptr;
    }

    *cmd_ptr = 'k'; // Inicializar el estado en 'stop'
    std::cout << "📡 Memoria compartida de comandos lista. Comando inicial: 'k'\n";
    return cmd_ptr;
}

// Lógica de mover_servos (modificada para devolver el tiempo):
int mover_servos()
{
    const double POS_TOLERANCE = 0.5; // Tolerancia en milímetros
    int actual_max_time_ms = 0;

    for (int leg = 0; leg < 4; leg++)
    {
        // 1. CHEQUEO RÁPIDO: ¿Se ha movido esta pata?
        // Comparamos la posición actual deseada con la última enviada
        bool position_changed = false;

        if (std::abs(current_feet_positions[leg][0] - last_sent_feet_positions[leg][0]) > POS_TOLERANCE ||
            std::abs(current_feet_positions[leg][1] - last_sent_feet_positions[leg][1]) > POS_TOLERANCE ||
            std::abs(current_feet_positions[leg][2] - last_sent_feet_positions[leg][2]) > POS_TOLERANCE)
        {
            position_changed = true;
        }

        // 2. CÁLCULO Y ENVÍO (Solo si cambió)
        if (position_changed)
        {
            // A. Calcular Cinemática Inversa SOLO para esta pata
            // Nota: IK::kinematics_array devuelve array<double, 3>
            std::array<double, 3> angulos_pata = IK::kinematics_array(
                leg + 1, // ID de pata (1-4)
                current_feet_positions[leg][0],
                current_feet_positions[leg][1],
                current_feet_positions[leg][2],
                current_body_orientation[0],
                current_body_orientation[1],
                current_body_orientation[2]);

            // B. Preparar tiempo
            int current_move_time_ms = current_move_time_ms_per_leg[leg];
            current_move_time_ms = std::clamp(current_move_time_ms, 1, 1000);

            // C. Enviar a los 3 servos de la pata
            for (int j = 0; j < 3; j++)
            {
                int id = motores[leg].ids[j];
                int offset = motores[leg].offsets[j];

                // Convertir ángulo a pulsos
                int pulse = static_cast<int>(angulos_pata[j] * SCALE) + offset;
                pulse = std::clamp(pulse, 0, 1000);

                move_servo(id, pulse, current_move_time_ms);
            }

            // D. Actualizar historial y tiempo máximo
            if (current_move_time_ms > actual_max_time_ms)
            {
                actual_max_time_ms = current_move_time_ms;
            }

            // Guardamos que ya enviamos esta posición
            last_sent_feet_positions[leg] = current_feet_positions[leg];

            if (DEBUG)
            {
                std::cout << "-> [IK CALC] Pata " << motores[leg].leg << " movida.\n";
            }
        }
    }
    return actual_max_time_ms;
}
// 🟢 Función MODIFICADA (NO BLOQUEANTE)

void ejecutar_movimiento_y_esperar_tiempo()
{
    // 🔴 ANTES: auto target_angles = calcular_angulos(); <-- BORRAR ESTA LÍNEA (Ahorra mucho CPU)

    // 🟢 AHORA: Llamamos directamente a la versión optimizada sin argumentos
    int max_time_ms_sent = mover_servos();

    if (max_time_ms_sent > 0)
    {
        long delay_ms = max_time_ms_sent + 10;

        if (DEBUG)
            std::cout << "DEBUG: Esperando " << delay_ms << "ms.\n";

        movement_complete_flag = false;
        uv_timer_start(&movement_timer, on_movement_complete, delay_ms, 0);
    }
    else
    {
        // Si nadie se movió, avanzamos inmediatamente
        on_movement_complete(&movement_timer);
    }
}
// 🟢 Función que se llama cuando el tiempo de movimiento ha pasado.

void on_movement_complete(uv_timer_t *handle)
{
    if (DEBUG)
    {
        std::cout << "DEBUG: Tiempo de espera completado. Levantando FLAG.\n";
    }

    // 1. Detener el timer (es one-shot)
    uv_timer_stop(handle);

    // 2. ⭐ ESTABLECER EL FLAG A TRUE ⭐
    // Esto señala al bucle principal que el robot está listo para el siguiente paso.
    movement_complete_flag = true;

    // IMPORTANTE: Ya NO llamamos a ejecutar_movimiento_y_esperar_tiempo() aquí.
    // El bucle principal deberá leer este flag.
}

void sec_marcha_adelante();
void sec_marcha_izqueirda();
void sec_marcha_atras();
void reset_contadores_marcha();

/**
 * @brief Lógica que cambia el estado en función del comando de entrada.
 * @param command_char Carácter de entrada (ej: 'a', 'p', 'i').
 */
void gestor_estados(char command_char)
{
    switch (command_char)
    {
    case 'i':
        estado_actual = ADELANTE;
        printf("\n[COMANDO] 🟢 Estado cambiado a: adelante\n");
        break;
    case 'k':
        estado_actual = STOP;
        printf("\n[COMANDO] 🟠 Estado cambiado a: stop\n");
        break;
    case 'j':
        estado_actual = IZQUIERDA;
        printf("\n[COMANDO] 🟠 Estado cambiado a: izquierda\n");
        break;
    case 'l':
        estado_actual = DERECHA;
        printf("\n[COMANDO] 🟠 Estado cambiado a: atras\n");
        break;
    case 'm':
        estado_actual = ATRAS;
        break;
    case 'u':
        estado_actual = GIRO_IZQUIERDA;
        printf("\n[COMANDO] 🟠 Estado cambiado a: GIRO-izquierda\n");
        break;
    case 'o':
        estado_actual = GIRO_DERECHA;
        printf("\n[COMANDO] 🟠 Estado cambiado a: GIRO-derecha\n");
        break;
    default:
        printf("\n[COMANDO] ❌ Comando desconocido. Válidos: 'a', 'p', 'i'.\n");
        break;
    }
    if (estado_actual != estado_anterior)
    {
        estado_anterior = estado_actual;
        reset_contadores_marcha();
    }
    printf("Comandos: 'i-->adelante', 'k-->stop', 'j-->izquierda'. El [TIMER] se ejecuta cada 500ms.\n");
    printf("Ingrese comando > ");
    fflush(stdout); // Asegurar que el prompt se imprime inmediatamente
}

int paso_marcha_interno = 0;
int pata_actual_marcha = PATA_FL;
int ciclo_marcha_actual = 0;

void reset_contadores_marcha()
{
    paso_marcha_interno = 0;
    ciclo_marcha_actual = 0;
    pata_actual_marcha = PATA_FL; // O la pata con la que inicie tu secuencia
    if (DEBUG)
        std::cout << "🔄 Contadores de marcha reiniciados.\n";
}
int fase_vuelo = 120;
void sec_marcha_adelante()
{
    TIEMPO_BASE_MARCHA_MS = 130;
    tiempo_leg_move = 70;
    switch (paso_marcha_interno)
    {
    case 0:

        current_feet_positions[pata_1] = s_1;

        current_feet_positions[pata_2] = s_1;
        current_feet_positions[pata_2][1] *= -1.0; // Ejemplo: Multiplicar por 2

        current_feet_positions[pata_3] = s_1_atras;

        current_feet_positions[pata_4] = s_1_atras;
        current_feet_positions[pata_4][1] *= -1.0; // Ejemplo: Multiplicar por 2
        current_move_time_ms_per_leg.fill(200);
        pata_actual_marcha = pata_1;
        ciclo_marcha_actual = 0;
        paso_marcha_interno++;
        break;
    case 1: // fase de buelo

        switch (pata_actual_marcha)
        {
        case PATA_FL: // 0: Frontal Izquierda
            // Comportamiento normal (s_2)
            current_feet_positions[pata_actual_marcha] = s_2;
            break;

        case PATA_FR: // 1: Frontal Derecha
            current_feet_positions[pata_actual_marcha] = {s_2[0], s_2[1] * -1.0, s_2[2]};
            break;

        case PATA_BL: // 2: Trasera Izquierda
            current_feet_positions[pata_actual_marcha] = s_2_atras;
            break;

        case PATA_BR: // 3: Trasera Derecha
            current_feet_positions[pata_actual_marcha] = {s_2_atras[0], s_2_atras[1] * -1.0, s_2_atras[2]};
            break;
        }
        current_move_time_ms_per_leg.fill(fase_vuelo);
        paso_marcha_interno++;
        break;
    case 2:

        switch (pata_actual_marcha)
        {
        case PATA_FL: // 0: Frontal Izquierda
            // Comportamiento normal (s_2)
            current_feet_positions[pata_actual_marcha] = s_3;
            break;

        case PATA_FR: // 1: Frontal Derecha
            current_feet_positions[pata_actual_marcha] = {s_3[0], s_3[1] * -1.0, s_3[2]};
            break;

        case PATA_BL: // 2: Trasera Izquierda
            current_feet_positions[pata_actual_marcha] = s_3_atras;
            break;

        case PATA_BR: // 3: Trasera Derecha
            current_feet_positions[pata_actual_marcha] = {s_3_atras[0], s_3_atras[1] * -1.0, s_3_atras[2]};
            break;
        }
        current_move_time_ms_per_leg.fill(tiempo_leg_move);
        paso_marcha_interno++;
        break;
    case 3:

        switch (ciclo_marcha_actual)
        {
        case 0:
            pata_actual_marcha = pata_4;
            ciclo_marcha_actual++;
            break;
        case 1:
            pata_actual_marcha = pata_2;
            ciclo_marcha_actual++;
            break;
        case 2:
            current_feet_positions[pata_1] = s_1;

            current_feet_positions[pata_2] = s_1;
            current_feet_positions[pata_2][1] *= -1.0; // Ejemplo: Multiplicar por 2

            current_feet_positions[pata_4] = s_1_atras;
            current_feet_positions[pata_4][1] *= -1.0; // Ejemplo: Multiplicar por 2
            current_move_time_ms_per_leg.fill(TIEMPO_BASE_MARCHA_MS);
            pata_actual_marcha = pata_3;
            ciclo_marcha_actual++;
            break;
        case 3:
            current_feet_positions[pata_3] = s_1_atras;
            ciclo_marcha_actual = 0;
            current_move_time_ms_per_leg.fill(TIEMPO_BASE_MARCHA_MS);
            pata_actual_marcha = pata_1;
            break;
        }
        paso_marcha_interno = 1;
        break;
    }
}

void sec_marcha_izqueirda()
{
    TIEMPO_BASE_MARCHA_MS = 110;
    tiempo_leg_move = 180;
    switch (paso_marcha_interno)
    {
    case 0:
        std::cout << "\n--- MARCHA LATERAL INICIALIZACION (Paso 0) ---\n";
        current_feet_positions[PATA_FL] = s_1_lado;
        current_feet_positions[PATA_FR] = s_1_lado;
        current_feet_positions[PATA_BR] = s_1_atras_lado;
        current_feet_positions[PATA_BL] = s_1_atras_lado;
        current_move_time_ms_per_leg.fill(TIEMPO_BASE_MARCHA_MS);
        pata_actual_marcha = PATA_FL;
        ciclo_marcha_actual = 0;
        paso_marcha_interno++;
        break;
    case 1:
        std::cout << "--- PASO 1 (Levantar Pata " << motores[pata_actual_marcha].leg << ") ---\n";
        current_feet_positions[pata_actual_marcha] =
            (pata_actual_marcha == PATA_FL || pata_actual_marcha == PATA_FR) ? s_2_lado : s_2_atras_lado;
        current_move_time_ms_per_leg.fill(tiempo_leg_move);
        paso_marcha_interno++;
        break;
    case 2:
        std::cout << "--- PASO 2 (Mover Lado Pata " << motores[pata_actual_marcha].leg << ") ---\n";
        current_feet_positions[pata_actual_marcha] =
            (pata_actual_marcha == PATA_FL || pata_actual_marcha == PATA_FR) ? s_3_lado : s_3_atras_lado;
        current_move_time_ms_per_leg.fill(tiempo_leg_move);
        paso_marcha_interno++;
        break;
    case 3:
        std::cout << "--- PASO 3 (Lógica de Ciclo Original) ---\n";

        switch (ciclo_marcha_actual)
        {
        case 0:
            pata_actual_marcha = PATA_BR;
            ciclo_marcha_actual++;
            break;
        case 1:
            pata_actual_marcha = PATA_FR;
            ciclo_marcha_actual++;
            break;
        case 2:
            current_move_time_ms_per_leg.fill(tiempo_leg_move);
            pata_actual_marcha = PATA_BL;
            ciclo_marcha_actual++;
            break;
        case 3:
            current_feet_positions[PATA_BL] = s_1_atras_lado;
            current_feet_positions[PATA_FL] = s_1_lado;
            current_feet_positions[PATA_FR] = s_1_lado;
            current_feet_positions[PATA_BR] = s_1_atras_lado;
            pata_actual_marcha = PATA_FL;
            ciclo_marcha_actual = 0;
            current_move_time_ms_per_leg.fill(TIEMPO_BASE_MARCHA_MS);
            break;
        }
        paso_marcha_interno = 1;
        break;
    }
}

void sec_marcha_derecha()
{
    TIEMPO_BASE_MARCHA_MS = 200;
    tiempo_leg_move = 180;
    switch (paso_marcha_interno)
    {
    case 0:
        std::cout << "\n--- MARCHA LATERAL INICIALIZACION (Paso 0) ---\n";
        current_feet_positions[PATA_FL] = s_1_lado_D;
        current_feet_positions[PATA_FR] = s_1_lado_D;
        current_feet_positions[PATA_BR] = s_1_atras_lado_D;
        current_feet_positions[PATA_BL] = s_1_atras_lado_D;
        current_move_time_ms_per_leg.fill(TIEMPO_BASE_MARCHA_MS);
        pata_actual_marcha = PATA_FR;
        ciclo_marcha_actual = 0;
        paso_marcha_interno++;
        break;
    case 1:
        std::cout << "--- PASO 1 (Levantar Pata " << motores[pata_actual_marcha].leg << ") ---\n";
        current_feet_positions[pata_actual_marcha] =
            (pata_actual_marcha == PATA_FL || pata_actual_marcha == PATA_FR) ? s_2_lado_D : s_2_atras_lado_D;
        current_move_time_ms_per_leg.fill(tiempo_leg_move);
        paso_marcha_interno++;
        break;
    case 2:
        std::cout << "--- PASO 2 (Mover Lado Pata " << motores[pata_actual_marcha].leg << ") ---\n";
        current_feet_positions[pata_actual_marcha] =
            (pata_actual_marcha == PATA_FL || pata_actual_marcha == PATA_FR) ? s_3_lado_D : s_3_atras_lado_D;
        current_move_time_ms_per_leg.fill(tiempo_leg_move);
        paso_marcha_interno++;
        break;
    case 3:
        std::cout << "--- PASO 3 (Lógica de Ciclo Original) ---\n";

        switch (ciclo_marcha_actual)
        {
        case 0:
            pata_actual_marcha = PATA_BL;
            ciclo_marcha_actual++;
            break;
        case 1:
            pata_actual_marcha = PATA_FL;
            ciclo_marcha_actual++;
            break;
        case 2:
            current_move_time_ms_per_leg.fill(tiempo_leg_move);
            pata_actual_marcha = PATA_BR;
            ciclo_marcha_actual++;
            break;
        case 3:
            current_feet_positions[PATA_BL] = s_1_atras_lado_D;
            current_feet_positions[PATA_FL] = s_1_lado_D;
            current_feet_positions[PATA_FR] = s_1_lado_D;
            current_feet_positions[PATA_BR] = s_1_atras_lado_D;
            pata_actual_marcha = PATA_FR;
            ciclo_marcha_actual = 0;
            current_move_time_ms_per_leg.fill(TIEMPO_BASE_MARCHA_MS);
            break;
        }
        paso_marcha_interno = 1;
        break;
    }
}
// ======================================================
// 2. TAREA DE FONDO PERIÓDICA (Demostración "Tiempo Real")
// ======================================================
void sec_marcha_atras()
{
    TIEMPO_BASE_MARCHA_MS = 200;
    tiempo_leg_move = 110;
    switch (paso_marcha_interno)
    {
    case 0:
        std::cout << "\n--- MARCHA ATRÁS INICIALIZACION (Paso 0) ---\n";
        current_feet_positions[pata_4] = s_1_atras_back;
        current_feet_positions[pata_3] = s_1_atras_back;
        current_feet_positions[pata_2] = s_1_back;
        current_feet_positions[pata_1] = s_1_back;
        current_move_time_ms_per_leg.fill(TIEMPO_BASE_MARCHA_MS);
        pata_actual_marcha = pata_3;
        ciclo_marcha_actual = 0;
        paso_marcha_interno++;
        break;

    case 1:
        std::cout << "--- PASO 1 (Levantar Pata " << motores[pata_actual_marcha].leg << ") ---\n";
        current_feet_positions[pata_actual_marcha] =
            (pata_actual_marcha == pata_4 || pata_actual_marcha == pata_3) ? s_2_atras_back : s_2_back;
        current_move_time_ms_per_leg.fill(tiempo_leg_move);
        paso_marcha_interno++;
        break;

    case 2:
        std::cout << "--- PASO 2 (Mover Atrás Pata " << motores[pata_actual_marcha].leg << ") ---\n";
        current_feet_positions[pata_actual_marcha] =
            (pata_actual_marcha == pata_4 || pata_actual_marcha == pata_3) ? s_3_atras_back : s_3_atras;
        current_move_time_ms_per_leg.fill(tiempo_leg_move);
        paso_marcha_interno++;
        break;

    case 3:
        std::cout << "--- PASO 3 (Lógica de Ciclo Original ATRÁS) ---\n";
        switch (ciclo_marcha_actual)
        {
        case 0:
            pata_actual_marcha = pata_1;
            ciclo_marcha_actual++;
            break;
        case 1:
            pata_actual_marcha = pata_4;
            ciclo_marcha_actual++;
            break;
        case 2:
            current_feet_positions[pata_1] = s_1_back;
            current_feet_positions[pata_3] = s_1_atras_back;
            current_feet_positions[pata_4] = s_1_atras_back;
            current_move_time_ms_per_leg.fill(TIEMPO_BASE_MARCHA_MS);
            pata_actual_marcha = pata_2;
            ciclo_marcha_actual++;
            break;
        case 3:
            current_feet_positions[pata_2] = s_1_back;
            pata_actual_marcha = pata_3;
            ciclo_marcha_actual = 0;
            current_move_time_ms_per_leg.fill(tiempo_leg_move);
            break;
        }
        paso_marcha_interno = 1;
        break;
    }
}
void marcha_stop()
{
    current_feet_positions[pata_1] = ini_F;
    current_feet_positions[pata_2] = ini_F;
    current_feet_positions[pata_3] = ini_B;
    current_feet_positions[pata_4] = ini_B;
    current_move_time_ms_per_leg.fill(400);
}

void sec_giro_izquierda()
{
    TIEMPO_BASE_MARCHA_MS = 200;
    tiempo_leg_move = 150;
    switch (paso_marcha_interno)
    {
    case 0:
        std::cout << "\n--- MARCHA ADELANTE INICIALIZACION (Paso 0) ---\n";
        current_feet_positions[pata_1] = s_1_back_GI;       // FL
        current_feet_positions[pata_2] = s_1_GI;            // FR
        current_feet_positions[pata_3] = s_1_atras_back_GI; // BL
        current_feet_positions[pata_4] = s_1_atras_GI;      // BR
        current_move_time_ms_per_leg.fill(350);
        pata_actual_marcha = pata_1;
        ciclo_marcha_actual = 0;
        paso_marcha_interno++;
        break;
    case 1:
        std::cout << "--- PASO 1 (Levantar Pata " << motores[pata_actual_marcha].leg << ") ---\n";
        switch (pata_actual_marcha)
        {
        case PATA_FL:
            current_feet_positions[pata_actual_marcha] = s_2_back_GI;
            break;
        case PATA_FR:
            current_feet_positions[pata_actual_marcha] = s_2_GI;
            break;
        case PATA_BL:
            current_feet_positions[pata_actual_marcha] = s_2_atras_back_GI;
            break;
        case PATA_BR:
            current_feet_positions[pata_actual_marcha] = s_2_atras_GI;
            break;
        }

        current_move_time_ms_per_leg.fill(tiempo_leg_move);
        paso_marcha_interno++;
        break;
    case 2:
        std::cout << "--- PASO 2 (Mover Adelante Pata " << motores[pata_actual_marcha].leg << ") ---\n";
        switch (pata_actual_marcha)
        {
        case PATA_FL:
            current_feet_positions[pata_actual_marcha] = s_3_back_GI;
            break;
        case PATA_FR:
            current_feet_positions[pata_actual_marcha] = s_3_GI;
            break;
        case PATA_BL:
            current_feet_positions[pata_actual_marcha] = s_3_atras_back_GI;
            break;
        case PATA_BR:
            current_feet_positions[pata_actual_marcha] = s_3_atras_GI;
            break;
        }
        current_move_time_ms_per_leg.fill(tiempo_leg_move);
        paso_marcha_interno++;
        break;
    case 3:
        std::cout << "--- PASO 3 (Lógica de Ciclo Original) ---\n";
        switch (ciclo_marcha_actual)
        {
        case 0:
            pata_actual_marcha = pata_4;
            ciclo_marcha_actual++;
            break;
        case 1:
            pata_actual_marcha = pata_2;
            ciclo_marcha_actual++;
            break;
        case 2:
            current_move_time_ms_per_leg.fill(TIEMPO_BASE_MARCHA_MS);
            current_feet_positions[pata_1] = s_1_back_GI;
            current_feet_positions[pata_2] = s_1_GI;
            current_feet_positions[pata_4] = s_1_atras_back_GI;
            pata_actual_marcha = pata_3;
            ciclo_marcha_actual++;
            break;
        case 3:
            current_feet_positions[pata_3] = s_1_atras_GI;
            ciclo_marcha_actual = 0;
            current_move_time_ms_per_leg.fill(tiempo_leg_move);
            pata_actual_marcha = pata_1;
            break;
        }
        paso_marcha_interno = 1;
        break;
    }
}

void sec_giro_derecha()
{
    TIEMPO_BASE_MARCHA_MS = 130;
    int tiempo_leg_move = 150;
    switch (paso_marcha_interno)
    {
    case 0:
        std::cout << "\n--- MARCHA ADELANTE INICIALIZACION (Paso 0) ---\n";
        current_feet_positions[PATA_FL] = s_1_GD;            // FL
        current_feet_positions[PATA_FR] = s_1_back_GD;       // FR
        current_feet_positions[PATA_BR] = s_1_atras_GD;      // BL
        current_feet_positions[PATA_BL] = s_1_atras_back_GD; // BR
        current_move_time_ms_per_leg.fill(350);
        pata_actual_marcha = PATA_FR;
        ciclo_marcha_actual = 0;
        paso_marcha_interno++;
        break;
    case 1:
        std::cout << "--- PASO 1 (Levantar Pata " << motores[pata_actual_marcha].leg << ") ---\n";
        switch (pata_actual_marcha)
        {
        case PATA_FL:
            current_feet_positions[pata_actual_marcha] = s_2_GD;
            break;
        case PATA_FR:
            current_feet_positions[pata_actual_marcha] = s_2_back_GD;
            break;
        case PATA_BL:
            current_feet_positions[pata_actual_marcha] = s_2_atras_GD;
            break;
        case PATA_BR:
            current_feet_positions[pata_actual_marcha] = s_2_atras_back_GD;
            break;
        }

        current_move_time_ms_per_leg.fill(tiempo_leg_move);
        paso_marcha_interno++;
        break;
    case 2:
        std::cout << "--- PASO 2 (Mover Adelante Pata " << motores[pata_actual_marcha].leg << ") ---\n";
        switch (pata_actual_marcha)
        {
        case PATA_FL:
            current_feet_positions[pata_actual_marcha] = s_3_GD;
            break;
        case PATA_FR:
            current_feet_positions[pata_actual_marcha] = s_3_back_GD;
            break;
        case PATA_BL:
            current_feet_positions[pata_actual_marcha] = s_3_atras_GD;
            break;
        case PATA_BR:
            current_feet_positions[pata_actual_marcha] = s_3_atras_back_GD;
            break;
        }
        current_move_time_ms_per_leg.fill(tiempo_leg_move);
        paso_marcha_interno++;
        break;
    case 3:
        std::cout << "--- PASO 3 (Lógica de Ciclo Original) ---\n";
        switch (ciclo_marcha_actual)
        {
        case 0:
            pata_actual_marcha = PATA_BL;
            ciclo_marcha_actual++;
            break;
        case 1:
            pata_actual_marcha = PATA_FL;
            ciclo_marcha_actual++;
            break;
        case 2:
            current_move_time_ms_per_leg.fill(TIEMPO_BASE_MARCHA_MS);
            current_feet_positions[PATA_FL] = s_1_GD;
            current_feet_positions[PATA_FR] = s_1_back_GD;
            current_feet_positions[PATA_BL] = s_1_atras_back_GD;
            pata_actual_marcha = PATA_BR;
            ciclo_marcha_actual++;
            break;
        case 3:
            current_feet_positions[PATA_BR] = s_1_atras_GD;
            ciclo_marcha_actual = 0;
            current_move_time_ms_per_leg.fill(tiempo_leg_move);
            pata_actual_marcha = PATA_FR;
            break;
        }
        paso_marcha_interno = 1;
        break;
    }
}

/**
 * @brief Callback que se ejecuta cada 500ms para mostrar que el loop está activo.
 */
void timer_callback(uv_timer_t *handle)
{
    // 1. LECTURA Y LIMPIEZA DE MEMORIA COMPARTIDA
    if (shm_cmd_ptr != nullptr)
    {
        char command_char = *shm_cmd_ptr; // ⬅️ LECTURA PERIÓDICA AQUÍ

        if (command_char != '\0')
        {

            // 🟢 NUEVO: Imprimir el comando justo antes de procesar
            std::cout << "📥 Comando SHM detectado y procesado: '" << command_char << "'" << std::endl;

            // Se procesa si no está limpio
            gestor_estados(command_char);

            // Se limpia para evitar repetición
            *shm_cmd_ptr = '\0';
        }
    }

    if (movement_complete_flag)
    {

        movement_complete_flag = false; // Resetear el flag inmediatamente

        if (estado_actual == ADELANTE)
        {
            // 🚨 Si el robot está en ADELANTE, avanzar y programar el próximo delay.
            sec_marcha_adelante();
        }
        else if (estado_actual == STOP)
        {
            // 🚨 Si el robot está en STOP, ir a la posición final y DETENER el timer periódico.
            marcha_stop();
        }
        else if (estado_actual == IZQUIERDA)
        {
            sec_marcha_izqueirda();
        }
        else if (estado_actual == DERECHA)
        {
            sec_marcha_derecha();
        }
        else if (estado_actual == ATRAS)
        {
            sec_marcha_atras();
        }
        else if (estado_actual == GIRO_DERECHA)
        {
            sec_giro_derecha();
        }
        else if (estado_actual == GIRO_IZQUIERDA)
        {
            sec_giro_izquierda();
        }
        ejecutar_movimiento_y_esperar_tiempo();
    }
}
// ======================================================
// 3. LÓGICA DEL BUCLE DE EVENTOS (libuv)
// ======================================================

static char buffer_line[256];
static size_t buffer_pos = 0;

/**
 * @brief Se llama cuando hay datos listos en STDIN.
 */
void on_read(uv_stream_t *stream, ssize_t nread, const uv_buf_t *buf)
{
    if (nread < 0)
    {
        if (nread != UV_EOF)
        {
            fprintf(stderr, "Error de lectura: %s\n", uv_err_name(nread));
        }
        uv_close((uv_handle_t *)stream, NULL);
        return;
    }

    // Procesar los bytes leídos
    for (ssize_t i = 0; i < nread; ++i)
    {
        char c = buf->base[i];

        // Detectar Enter (o Nueva Línea)
        if (c == '\n' || c == '\r')
        {
            if (buffer_pos > 0)
            {
                // Solo enviamos el primer carácter (comando) al gestor
                gestor_estados(buffer_line[0]);
                buffer_pos = 0;
                memset(buffer_line, 0, sizeof(buffer_line));
            }
        }
        else if (buffer_pos < sizeof(buffer_line) - 1)
        {
            // Se lee un caracter y se guarda, pero no se imprime el prompt hasta Enter
            buffer_line[buffer_pos++] = c;
        }
    }
}

/**
 * @brief Función para asignar espacio al buffer de lectura (requerida por libuv).
 */
void alloc_buffer(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf)
{
    buf->base = buffer_line;
    buf->len = sizeof(buffer_line);
}

void ejemplo_obtener_voltaje()
{
    // 1. Define el ID del servo que quieres medir (ejemplo: ID 1)
    uint8_t servo_id = 1;

    // 2. Llama a la función para obtener el valor en milivoltios (mV)
    int raw_voltage_mv = get_servo_voltage(servo_id);

    if (raw_voltage_mv != -1)
    {
        // 3. Convierte a voltios para mostrar
        double voltage_v = raw_voltage_mv / 1000.0;

        std::cout << "🔋 Voltaje del Servo ID " << (int)servo_id
                  << ": " << std::fixed << std::setprecision(2)
                  << voltage_v << " V"
                  << " (" << raw_voltage_mv << " mV)" << std::endl;
    }
    else
    {
        std::cerr << "❌ Error: No se pudo leer el voltaje del Servo ID "
                  << (int)servo_id << ". (Timeout)" << std::endl;
    }
}
// ======================================================
// 4. MAIN
// ======================================================

int main()
{

    if (!serial_open("/dev/servos", 115200))
    { /* ... */
        return 1;
    }

    std::cout << "🚀 ATOM-51 listo.\n";
    ejemplo_obtener_voltaje();

    uv_tty_t stdin_t;
    uv_timer_t timer_h;
    int fd_cmd; // Descriptor para la memoria compartida

    // 🟢 PASO 1: Abrir Memoria Compartida y obtener puntero
    shm_cmd_ptr = open_robot_command_shm(fd_cmd);
    if (!shm_cmd_ptr)
        return -1;

    // --- 1. Inicializar Timers ---
    uv_timer_init(loop_global, &timer_h);
    uv_timer_init(loop_global, &movement_timer);
    uv_timer_start(&timer_h, timer_callback, 0, 20);

    printf("==========================================\n");
    printf("  CONTROL DE ESTADOS ASÍNCRONO (SHM)\n");
    printf("==========================================\n");
    printf("Comandos se leen de la memoria compartida: /robot_cmd\n");
    printf("Estado actual: STOP\n");
    // 🔴 Eliminar el prompt de entrada de terminal: fflush(stdout);

    return uv_run(loop_global, UV_RUN_DEFAULT);
}