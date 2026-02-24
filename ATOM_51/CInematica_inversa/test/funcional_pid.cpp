#include "BusServoControl.hpp"
#include "kinematics.hpp"
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <array>
#include <cmath>
#include <iomanip>
#include <atomic>
#include <mutex>
#include <chrono>

using namespace std;

// =============================
// Estructura para una pata
// =============================
struct MotorLeg {
    string leg;
    array<int, 3> ids;
    array<int, 3> offsets;
    array<array<int, 2>, 3> limits;
    array<double, 3> angles_theoretical;
    array<double, 3> angles_real;
};

// ✅ Mutex global para proteger acceso al bus serial
std::mutex bus_mutex;
const int MOVE_MS = 1;

// =======================================================
// 💡 Leer las posiciones reales (feedback) con protección
// =======================================================
void read_all_positions(vector<MotorLeg> &motores) {
    for (auto &leg : motores) {
        for (int i = 0; i < 3; ++i) {
            int id = leg.ids[i];
            int pos;

            {   // 🔒 Bloque protegido (lectura del bus)
                std::lock_guard<std::mutex> lock(bus_mutex);
                pos = get_servo_position(id);
            }

            if (pos != -1) {
                double angle_deg = (pos * 240.0 / 1000.0) + leg.offsets[i];
                leg.angles_real[i] = angle_deg;
            } else {
                cerr << "⚠️ Error leyendo servo ID " << id
                     << " (" << leg.leg << ")" << endl;
            }

            this_thread::sleep_for(chrono::milliseconds(5));
        }
    }
}

// =======================================================
// 🧠 Cálculo de ángulos (cinemática inversa)
// =======================================================
array<array<double, 3>, 4> calculo_angulos(
    const array<array<double, 3>, 4>& feet_positions,
    const array<double, 3>& rot_body,
    Kinematics& robot
) {
    double roll = rot_body[0];
    double pitch = rot_body[1];
    double yaw = rot_body[2];

    array<double, 4> y_offsets = {0.0, 0.0, -60.0, -60.0};
    array<array<double, 3>, 4> new_positions = feet_positions;
    for (int i = 0; i < 4; ++i)
        new_positions[i][1] += y_offsets[i];

    array<array<double, 3>, 4> angles_with_offsets = {0};

    for (int i = 0; i < 4; ++i) {
        IKResult ik_result = robot.leg_IK(
            Eigen::Vector3d(new_positions[i][0],
                            new_positions[i][1],
                            new_positions[i][2]),
            Eigen::Vector3d(roll, pitch, yaw),
            i, false
        );

        Eigen::Vector3d angles_rad = ik_result.angles;
        array<double, 3> angles = {
            angles_rad[0] * 180.0 / M_PI,
            angles_rad[1] * 180.0 / M_PI,
            angles_rad[2] * 180.0 / M_PI
        };

        switch (i) {
            case 0: angles_with_offsets[i] = {angles[0] + 121.0, angles[1] - 284.5, angles[2] + 103.98}; break;
            case 1: angles_with_offsets[i] = {-angles[0] + 121.0, angles[1] - 284.5, angles[2] + 103.98}; break;
            case 2: angles_with_offsets[i] = {angles[0] + 477.93, angles[1] + 530.47, -angles[2] + 133.96}; break;
            case 3: angles_with_offsets[i] = {-angles[0] - 236.93, angles[1] + 525.47, -angles[2] + 138.96}; break;
        }
    }

    return angles_with_offsets;
}

// =======================================================
// ⚙️ Controlador P simple
// =======================================================
struct PController {
    double Kp;

    PController(double kp = 0.05) : Kp(kp) {}

    double compute(double setpoint, double measured) {
        double error = setpoint - measured;
        double correction = Kp * error;
        correction = std::clamp(correction, -2.0, 2.0);
        return correction;
    }
};

// =======================================================
// 🔁 Bucle unificado (envío + control con temporización)
// =======================================================
void unified_loop(vector<MotorLeg> &motores,
                  vector<PController> &controllers,
                  atomic<bool> &running,
                  int send_period_ms,
                  int control_period_ms)
{
    using namespace chrono;
    auto next_send_time = steady_clock::now();
    auto last_control_time = steady_clock::now();

    const double error_min = 0.8; // 🔸 Umbral mínimo de error [grados]

    while (running) {
        auto now = steady_clock::now();

        // 🧠 Ejecutar control cada "control_period_ms"
        if (duration_cast<milliseconds>(now - last_control_time).count() >= control_period_ms) {
            read_all_positions(motores);

            for (int leg_index = 0; leg_index < motores.size(); ++leg_index) {
                auto &leg = motores[leg_index];

                for (int j = 0; j < 3; ++j) {
                    int ctrl_index = leg_index * 3 + j;
                    double setpoint = leg.angles_theoretical[j];
                    double measured = leg.angles_real[j];
                    double error = setpoint - measured;

                    // ✅ Solo aplica corrección si el error supera el umbral mínimo
                    if (fabs(error) > error_min) {
                        double correction = controllers[ctrl_index].compute(setpoint, measured);

                        double new_angle = std::clamp(
                            setpoint + correction,
                            (double)leg.limits[j][0],
                            (double)leg.limits[j][1]
                        );

                        leg.angles_theoretical[j] = new_angle;
                    }
                    // 🔇 Si el error es pequeño, no se hace nada
                }
            }

            last_control_time = now;
            cout << "⚙️ Control ejecutado cada " << control_period_ms << " ms\n";
        }

        // 🚀 Envío periódico condicionado por error
        for (auto &leg : motores) {
            for (int j = 0; j < 3; ++j) {
                double setpoint = leg.angles_theoretical[j];
                double measured = leg.angles_real[j];
                double error = setpoint - measured;

                // ✅ Solo enviar si hay error apreciable
                if (fabs(error) > error_min) {
                    double angle = leg.angles_theoretical[j];

                    // 🔢 Conversión a unidades del servo
                    int servo_units = static_cast<int>(
                        std::round((angle - leg.offsets[j]) * 1000.0 / 240.0)
                    );

                    // 🔒 Envío protegido
                    {
                        std::lock_guard<std::mutex> lock(bus_mutex);
                        move_servo(leg.ids[j], servo_units, MOVE_MS);
                    }
                }
                else {
                    // 🔇 No enviar si el error es insignificante
                }
            }
        }

        // ⏱️ Mantener ritmo de envío constante
        next_send_time += milliseconds(send_period_ms);
        this_thread::sleep_until(next_send_time);
    }

    cout << "🛑 Bucle unificado detenido.\n";
}

// =======================================================
// 🏁 MAIN
// =======================================================
int main() {
    if (!serial_open("/dev/ttyUSB0", 115200)) return -1;

    Kinematics robot;

    vector<MotorLeg> motores = {
        {"LF", {10,11,12}, {10,-2,-9},   {{{100,150},{50,140},{20,100}}}, {0,0,0}, {0,0,0}},
        {"LB", {7,8,9},    {-5,-5,-9},   {{{90,135},{50,140},{20,100}}}, {0,0,0}, {0,0,0}},
        {"RF", {1,2,3},    {12,9,5},     {{{80,140},{100,170},{140,235}}}, {0,0,0}, {0,0,0}},
        {"RB", {4,5,6},    {5,0,21},     {{{100,150},{100,170},{140,235}}}, {0,0,0}, {0,0,0}}
    };

    cout << "🔄 Sincronizando ángulos iniciales..." << endl;
    read_all_positions(motores);
    for (auto &leg : motores)
        leg.angles_theoretical = leg.angles_real;

    vector<PController> controllers(12, PController(0.035));
    atomic<bool> running(true);

    // 🕒 Configura los periodos
    int send_period_ms   = 50;   // Enviar comandos cada 50 ms
    int control_period_ms = 120; // Ejecutar control cada 100 ms

    // 🧠 Inicia el hilo unificado
    thread control_thread(unified_loop, ref(motores), ref(controllers),
                          ref(running), send_period_ms, control_period_ms);

    cout << "🎯 Bucle unificado: envío=" << send_period_ms
         << " ms | control=" << control_period_ms << " ms\n";

    // ================================================
    // 🦿 Movimiento incremental en Z
    // ================================================
    double z_start = -100.0;
    double z_end   = -160.0;
    double step    = 10.0;
    int delay_ms   = 80;

    for (double z = z_start; z >= z_end; z -= step) {
        array<array<double, 3>, 4> feet_positions = {{
            {0.0, 35.0, z},
            {0.0, 35.0, z},
            {0.0, 35.0, z},
            {0.0, 35.0, z}
        }};

        array<double, 3> rot_body = {0.0, 0.0, 0.0};
        auto angles = calculo_angulos(feet_positions, rot_body, robot);

        for (int i = 0; i < 4; ++i)
            motores[i].angles_theoretical = angles[i];

        cout << "🦶 Nueva referencia Z = " << z << " mm" << endl;
        this_thread::sleep_for(chrono::milliseconds(delay_ms));
    }

    cout << "✅ Movimiento incremental completado." << endl;

    // Espera unos segundos y detén el bucle
    this_thread::sleep_for(chrono::seconds(10));
    running = false;
    control_thread.join();

    serial_close();
    return 0;
}
