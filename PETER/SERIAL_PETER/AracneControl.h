
#ifndef ARACNECONTROL_H
#define ARACNECONTROL_H

#include <Adafruit_PWMServoDriver.h>
#include <Ticker.h>

// Constants for Servo PWM
#define SERVOMIN  170
#define SERVOMAX  595

// Servo pin assignments
extern const int servo_pin[4][3];
const float pi = 3.14159265358979323846; // Define pi constant
// Robot Modes
#define SPIDER  0
// Robot parameters (lengths in mm)
extern const float length_a;
extern const float length_b;
extern const float length_c;
extern const float length_side;
extern const float z_absolute;

// Constants for movement
extern const float z_default;
extern const float z_up;
extern const float z_boot;
extern const float x_default;
extern const float x_offset;
extern const float x_start;
extern const float x_step;
extern const float y_start;
extern const float y_step;
extern const float y_default;

// Variables for movement
extern volatile float site_now[4][3];
extern volatile float site_expect[4][3];
extern float temp_speed[4][3];
extern float move_speed;
extern float speed_multiple;
extern const float spot_turn_speed;
extern const float leg_move_speed;
extern const float body_move_speed;
extern const float stand_seat_speed;
extern volatile int rest_counter;
extern const float KEEP;

// Turn calculations
extern const float temp_a;
extern const float temp_b;
extern const float temp_c;
extern const float temp_alpha;
extern const float turn_x1;
extern const float turn_y1;
extern const float turn_x0;
extern const float turn_y0;

// Control variables
extern int mode;
extern bool servo_service_en;

// Function prototypes
void stand(void);
void step_forward(unsigned int step);
void step_back(unsigned int step);
void step_right(unsigned int step);
void step_left(unsigned int step);
void turn_left(unsigned int step);
void turn_right(unsigned int step);
void wait_all_reach(void);
void wait_reach(int leg);
void set_site(int leg, float x, float y, float z);

void inverseKinematics(volatile float &alpha, volatile float &beta, volatile float &gamma, 
                        volatile float x, volatile float y, volatile float z);
void anglesToLeg(int leg, int alpha, int beta, int gamma);
int angleToPulse(int angle);

#endif