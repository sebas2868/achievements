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

            this_thread::sleep_for(chrono::milliseconds(3));
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
// ⚙️ Controlador P (Proporcional simple)
// =======================================================
struct PController {
    double Kp;

    PController(double kp = 0.05) : Kp(kp) {}

    double compute(double setpoint, double measured) {
        double error = setpoint - measured;
        double correction = Kp * error;
        correction = std::clamp(correction, -10.0, 10.0);
        return correction;
    }
};

// =======================================================
// 🔁 Bucle de control P (12 controladores independientes)
// =======================================================
void p_control_loop(vector<MotorLeg> &motores,
                    vector<PController> &controllers,
                    int duration_ms)
{
    read_all_positions(motores);

    for (int leg_index = 0; leg_index < motores.size(); ++leg_index) {
        auto &leg = motores[leg_index];

        for (int j = 0; j < 3; ++j) {
            int ctrl_index = leg_index * 3 + j;
            double setpoint = leg.angles_theoretical[j];
            double measured = leg.angles_real[j];
            double correction = controllers[ctrl_index].compute(setpoint, measured);

            double new_angle = std::clamp(setpoint + correction,
                                          (double)leg.limits[j][0],
                                          (double)leg.limits[j][1]);

            int servo_units = static_cast<int>(
                std::round((new_angle - leg.offsets[j]) * 1000.0 / 240.0)
            );

            {   // 🔒 Envío protegido
                std::lock_guard<std::mutex> lock(bus_mutex);
                move_servo(leg.ids[j], servo_units, duration_ms);
            }

            cout << fixed << setprecision(2)
                 << "[P] " << leg.leg
                 << " | Servo " << leg.ids[j]
                 << " | Set: " << setpoint
                 << " | Real: " << measured
                 << " | Corr: " << correction
                 << " | Out: " << new_angle << endl;
        }
    }
}

// =======================================================
// ⏱️ Controlador P periódico con periodo ajustable
// =======================================================
void run_p_periodic(vector<MotorLeg> &motores,
                    vector<PController> &controllers,
                    int duration_ms,
                    atomic<bool> &running,
                    int controller_period_ms) // 🔹 nuevo parámetro
{
    using namespace chrono;
    auto next_time = steady_clock::now();

    while (running) {
        p_control_loop(motores, controllers, duration_ms);
        next_time += milliseconds(controller_period_ms);
        this_thread::sleep_until(next_time);
    }

    cout << "🛑 Control P detenido." << endl;
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

    cout << "✅ Ángulos teóricos igualados a los reales." << endl;

    vector<PController> controllers;
    for (int i = 0; i < 12; ++i)
        controllers.push_back(PController(0.035)); // 🔹 12 controladores P suaves

    atomic<bool> running(true);

    // 🕒 Variable para controlar la frecuencia del controlador
    int controller_period_ms = 30; // 🔹 cada 100 ms (10 Hz)
    // puedes probar valores como 50 (20 Hz) o 200 (5 Hz)

    // 🧠 Hilo de control P
    thread p_thread(run_p_periodic, ref(motores), ref(controllers), 0, ref(running), controller_period_ms);
    cout << "🎯 Control P activo cada " << controller_period_ms << " ms..." << endl;

    // ================================================
    // 🦿 Movimiento incremental en Z
    // ================================================
    double z_start = -100.0;
    double z_end   = -160.0;
    double step    = 5.0;
    int delay_ms   = 50;

for (double z = z_start; z >= z_end; z -= step){
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

    cout << "✅ Movimiento incremental completado (Z = " << z_end << " mm)" << endl;

    this_thread::sleep_for(chrono::seconds(5));
    running = false;
    p_thread.join();

    serial_close();
    return 0;
}
