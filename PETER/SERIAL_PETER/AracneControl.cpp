
#include "AracneControl.h"
#include <Arduino.h>

void inverseKinematics(volatile float &alpha, volatile float &beta, volatile float &gamma,float x, float y, float z) {
    // Calcular theta1 (rotación alrededor de la base en el plano x-y)
    float theta1 = atan2(y, x);

    // Calcular distancias intermedias r y s
    float r = sqrt(x * x + y * y) - length_a;
    float s = z;

    // Calcular d, la distancia desde el hombro hasta la muñeca
    float d = sqrt(r * r + s * s);

    // Verificar si la posición deseada está al alcance
    if (d > (length_c + length_b) || d < abs(length_c - length_b)) {
        Serial.println("Error: Posición fuera de alcance");
        return;
    }

    // Calcular theta2 usando la ley de cosenos y el arctangent corregido
    float cos_theta2 = (length_c * length_c + d * d - length_b * length_b) / (2 * length_c * d);
    cos_theta2 = constrain(cos_theta2, -1, 1);  // Asegurar que el valor esté entre -1 y 1
    float theta2 = atan2(s, r) - acos(cos_theta2);

    // Calcular theta3 usando la ley de cosenos con ajuste de ángulo interno
    float cos_theta3 = (length_c * length_c + length_b * length_b - d * d) / (2 * length_c * length_b);
    cos_theta3 = constrain(cos_theta3, -1, 1);  // Asegurar que el valor esté entre -1 y 1
    float theta3 = PI - acos(cos_theta3);

    // Convertir ángulos de radianes a grados para mayor claridad
    alpha = theta1 * 180.0 / PI + 45;
    beta = theta2 * 180.0 / PI + 45;
    gamma = theta3 * 180.0 / PI + 15;

    // Imprimir los resultados en el monitor serial
    /*Serial.println("Ángulos Calculados:");
    Serial.print("Alpha: "); Serial.print(int(alpha)); Serial.println("°");
    Serial.print("Beta: "); Serial.print(int(beta)); Serial.println("°");
    Serial.print("Gamma: "); Serial.print(int(gamma)); Serial.println("°");*/
}

// Define servo configurations
const int servo_pin[4][3] = {
    {0, 1, 2},
    {4, 5, 6},
    {8, 9, 10},
    {12, 13, 14}
};

// Robot parameters
const float length_a = 45.0;
const float length_c = 45.0;
const float length_b = 90.0;
const float length_side = 65.0;
const float z_absolute = 26.0;

// Movement constants
const float z_default = length_b+5;
const float z_up = 63;
const float z_boot = length_b+5;
const float x_default = sqrt(pow(length_a + length_c, 2) / 2)-2;
const float x_offset = 0;
const float y_start = 0;
const float x_start = 0;
const float y_step = x_default;
const float x_step = y_step;
const float y_default = x_default;

// Movement variables
volatile float site_now[4][3];
volatile float site_expect[4][3];
float temp_speed[4][3];
float move_speed = 8;
float speed_multiple = 1;
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;
volatile int rest_counter = 0;
const float KEEP = 255;

// Turn calculations
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / (2 * temp_a * temp_b));
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

// Control variables
int mode = SPIDER;
bool servo_service_en = true;

void stand(void){
  move_speed = stand_seat_speed; // Set movement speed to standing speed
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default); // Set each leg’s height to the default height (z_default)
  }
  wait_all_reach();

  //servo_service();
}

void step_forward(unsigned int step){
  //Serial.println("forward");
  move_speed = leg_move_speed;  // Set movement speed to leg movement speed
  while (step-- > 0)  // Repeat until the specified number of steps is complete
  {
    //Serial.print("entré");
    if (site_now[2][1] == y_start)  //PASO CON LA IZQUIERDA
    {
      //Serial.print("PASO 1");
      // Step 1: Lift and move legs 2 and 1 forward
      set_site(2, x_default + x_offset, y_start, z_up);  // Lift leg 2
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);  // Move leg 2 forward
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);  // Lower leg 2
      wait_all_reach();

      move_speed = body_move_speed;  // Slow down for body movement

      // Move the body forward by adjusting the grounded legs
      //=============IZQUIERDA=================
      //DELANTERA
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      //TRASERA
      set_site(1, x_default + x_offset, y_start + y_step, z_default);
      //===============DERECHA=================
      //DELANTERA
      set_site(3, x_default - x_offset, y_start, z_default);
      //TRASERA
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      
      wait_all_reach();

      move_speed = leg_move_speed;  // Restore speed for leg movement

      // Step 2: Lift and move leg 1 back to its starting position
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);  // Lift leg 1
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);  // Move leg 1 back
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);  // Lower leg 1
      wait_all_reach();
    }
    else  //PASO CON LA DERECHA
    {
      // Alternate step: Lift and move legs 0 and 3 forward
      //Serial.print("PASO 2");
      set_site(3, x_default + x_offset, y_start, z_up);  // Lift leg 0
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);  // Move leg 0 forward
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);  // Lower leg 0
      wait_all_reach();

      move_speed = body_move_speed;  // Slow down for body movement

      // Move the body forward by adjusting the grounded legs
      //===============DERECHA=================
      //DELANTERA
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      //TRASERA
      set_site(0, x_default + x_offset, y_start + y_step, z_default);
      //=============IZQUIERDA=================
      //DELANTERA
      set_site(2, x_default - x_offset, y_start, z_default);
      //TRASERA
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);

      wait_all_reach();

      move_speed = leg_move_speed;  // Restore speed for leg movement

      // Lift and move leg 3 back to its starting position
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);  // Lift leg 3
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);  // Move leg 3 back
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);  // Lower leg 3
      wait_all_reach();
    }
  }
}

void step_back(unsigned int step){
  move_speed = leg_move_speed;  // Set movement speed to leg movement speed

  while (step-- > 0)  // Repeat until the specified number of steps is complete
  {
    if (site_now[0][1] == y_start)
    {
      // Phase 1: Move legs 3 and 0
      set_site(0, x_default + x_offset, y_start, z_up);  // Lift leg 
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);  // Move leg  backward
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);  // Lower leg 
      wait_all_reach();

      move_speed = body_move_speed;  // Slow down for body movement

      // Adjust grounded legs to move the body forward
      //===============DERECHA=================
      //TRASERA
      set_site(0, x_default + x_offset, y_start + y_step, z_default);
      //DELANTERA
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
            //=============IZQUIERDA=================
      //TRASERA
      set_site(1, x_default + x_offset, y_start, z_default);
      //DELANTERA
      set_site(2, x_default - x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;  // Restore speed for leg movement

      // Move leg 0 back to its starting position
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);  // Lift leg 0
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);  // Move leg 0 backward
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);  // Lower leg 0
      wait_all_reach();
    }
    else
    {
      // Phase 2: Move legs 1 and 2
      set_site(1, x_default + x_offset, y_start, z_up);  // Lift leg 1
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);  // Move leg 1 backward
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);  // Lower leg 1
      wait_all_reach();

      move_speed = body_move_speed;  // Slow down for body movement

      // Adjust grounded legs to move the body forward
      //=============IZQUIERDA=================
      //TRASERA
      set_site(1, x_default + x_offset, y_start + y_step, z_default);
      //DELANTERA
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      //===============DERECHA=================
      //TRASERA
      set_site(0, x_default + x_offset, y_start, z_default);
      //DELANTERA
      set_site(3, x_default - x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;  // Restore speed for leg movement

      // Move leg 2 back to its starting position
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);  // Lift leg 2
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);  // Move leg 2 backward
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);  // Lower leg 2
      wait_all_reach();
    }
  }
}

void step_right(unsigned int step){
  move_speed = leg_move_speed;  // Set movement speed to leg movement speed

  while (step-- > 0)  // Repeat until the specified number of steps is complete
  {
    if (site_now[0][0] == x_start)
    {
      // Phase 1: Move legs 3 and 0
      set_site(0, x_start, y_default, z_up);  // Lift leg 
      wait_all_reach();
      set_site(0, x_start + 2 * x_step, y_default, z_up);  // Move leg  backward
      wait_all_reach();
      set_site(0, x_start + 2 * x_step, y_default, z_default);  // Lower leg 
      wait_all_reach();

      move_speed = body_move_speed;  // Slow down for body movement

      // Adjust grounded legs to move the body forward
      //===============DERECHA=================
      //TRASERA
      set_site(0, x_default, y_start + y_step, z_default);
      //DELANTERA
      set_site(3, x_start, y_start + y_step, z_default);
            //=============IZQUIERDA=================
      //TRASERA
      set_site(1, x_default, y_start + y_step, z_default);
      //DELANTERA
      set_site(2, x_start + 2 * x_step, y_default, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;  // Restore speed for leg movement

      // Move leg 0 back to its starting position
      set_site(2, x_start + 2 * x_step, y_default, z_up);  // Lift leg 0
      wait_all_reach();
      set_site(2, x_start, y_default, z_up);  // Move leg 0 backward
      wait_all_reach();
      set_site(2, x_start + x_offset, y_default, z_default);  // Lower leg 0
      wait_all_reach();
    }
    else
    {
      // Phase 1: Move legs 3 and 0
      set_site(3, x_start, y_default, z_up);  // Lift leg 
      wait_all_reach();
      set_site(3, x_start + 2 * x_step, y_default, z_up);  // Move leg  backward
      wait_all_reach();
      set_site(3, x_start + 2 * x_step, y_default, z_default);  // Lower leg 
      wait_all_reach();

      move_speed = body_move_speed;  // Slow down for body movement

      // Adjust grounded legs to move the body forward
      //===============DERECHA=================
      //TRASERA
      set_site(3, x_default, y_start + y_step, z_default);
      //DELANTERA
      set_site(0, x_start, y_start + y_step, z_default);
            //=============IZQUIERDA=================
      //TRASERA
      set_site(2, x_default, y_start + y_step, z_default);
      //DELANTERA
      set_site(1, x_start + 2 * x_step, y_default, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;  // Restore speed for leg movement

      // Move leg 0 back to its starting position
      set_site(1, x_start + 2 * x_step, y_default, z_up);  // Lift leg 0
      wait_all_reach();
      set_site(1, x_start, y_default, z_up);  // Move leg 0 backward
      wait_all_reach();
      set_site(1, x_start + x_offset, y_default, z_default);  // Lower leg 0
      wait_all_reach();
    }
  }
}

void step_left(unsigned int step){
  move_speed = leg_move_speed;  // Set movement speed to leg movement speed

  while (step-- > 0)  // Repeat until the specified number of steps is complete
  {
    if (site_now[2][0] == x_start)
    {
      // Phase 1: Move legs 3 and 0
      set_site(2, x_start, y_default, z_up);  // Lift leg 
      wait_all_reach();
      set_site(2, x_start + 2 * x_step, y_default, z_up);  // Move leg  backward
      wait_all_reach();
      set_site(2, x_start + 2 * x_step, y_default, z_default);  // Lower leg 
      wait_all_reach();

      move_speed = body_move_speed;  // Slow down for body movement

      // Adjust grounded legs to move the body forward
      //===============DERECHA=================
      //TRASERA
      set_site(2, x_default, y_start + y_step, z_default);
      //DELANTERA
      set_site(1, x_start, y_start + y_step, z_default);
            //=============IZQUIERDA=================
      //TRASERA
      set_site(3, x_default, y_start + y_step, z_default);
      //DELANTERA
      set_site(0, x_start + 2 * x_step, y_default, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;  // Restore speed for leg movement

      // Move leg 0 back to its starting position
      set_site(0, x_start + 2 * x_step, y_default, z_up);  // Lift leg 0
      wait_all_reach();
      set_site(0, x_start, y_default, z_up);  // Move leg 0 backward
      wait_all_reach();
      set_site(0, x_start + x_offset, y_default, z_default);  // Lower leg 0
      wait_all_reach();
    }
    else
    {
      // Phase 1: Move legs 3 and 0
      set_site(1, x_start, y_default, z_up);  // Lift leg 
      wait_all_reach();
      set_site(1, x_start + 2 * x_step, y_default, z_up);  // Move leg  backward
      wait_all_reach();
      set_site(1, x_start + 2 * x_step, y_default, z_default);  // Lower leg 
      wait_all_reach();

      move_speed = body_move_speed;  // Slow down for body movement

      // Adjust grounded legs to move the body forward
      //===============DERECHA=================
      //TRASERA
      set_site(1, x_default, y_start + y_step, z_default);
      //DELANTERA
      set_site(2, x_start, y_start + y_step, z_default);
            //=============IZQUIERDA=================
      //TRASERA
      set_site(0, x_default, y_start + y_step, z_default);
      //DELANTERA
      set_site(3, x_start + 2 * x_step, y_default, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;  // Restore speed for leg movement

      // Move leg 0 back to its starting position
      set_site(3, x_start + 2 * x_step, y_default, z_up);  // Lift leg 0
      wait_all_reach();
      set_site(3, x_start, y_default, z_up);  // Move leg 0 backward
      wait_all_reach();
      set_site(3, x_start + x_offset, y_default, z_default);  // Lower leg 0
      wait_all_reach();
    }
  }
}

void turn_left(unsigned int step){
  move_speed = spot_turn_speed;  // Set movement speed for turning
  while (step-- > 0)  // Repeat for the specified number of turning steps
  {
    if (site_now[1][1] == y_start)
    {
      // Phase 1: Move legs 3 and 1
      set_site(1, x_default + x_offset, y_start, z_up);  // Lift leg 3
      wait_all_reach();

      // Shift leg positions to begin the turn
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);  // Lower leg 3
      wait_all_reach();

      // Re-position legs to continue the turn
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);  // Lift leg 1
      wait_all_reach();

      // Reset leg positions to prepare for the next cycle
      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);  // Lower leg 1
      wait_all_reach();
    }
    else
    {
      // Phase 2: Move legs 0 and 2
      set_site(3, x_default + x_offset, y_start, z_up);  // Lift leg 0
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);  // Lower leg 0
      wait_all_reach();

      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);  // Lift leg 2
      wait_all_reach();

      // Reset leg positions to prepare for the next cycle
      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);  // Lower leg 2
      wait_all_reach();
    }
  }
}

void turn_right(unsigned int step){
  move_speed = spot_turn_speed;  // Set movement speed for turning

  while (step-- > 0)  // Repeat for the specified number of turning steps
  {
    if (site_now[2][1] == y_start)
    {
      // Phase 1: Move legs 2 and 0
      set_site(2, x_default + x_offset, y_start, z_up);  // Lift leg 2
      wait_all_reach();

      // Re-position legs to start the rightward turn
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);  // Lower leg 2
      wait_all_reach();

      // Re-position all legs to continue the turn
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);  // Lift leg 0
      wait_all_reach();

      // Reset leg positions to prepare for the next cycle
      set_site(3, x_default + x_offset, y_start, z_up);
      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);  // Lower leg 0
      wait_all_reach();
    }
    else
    {
      // Phase 2: Move legs 1 and 3
      set_site(0, x_default + x_offset, y_start, z_up);  // Lift leg 1
      wait_all_reach();

      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);  // Lower leg 1
      wait_all_reach();

      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);  // Lift leg 3
      wait_all_reach();

      // Reset leg positions to prepare for the next cycle
      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);  // Lower leg 3
      wait_all_reach();
    }
  }
}

// Also important Functions
void wait_all_reach(void){
  for (int i = 0; i < 4; i++)
    wait_reach(i); // Call wait_reach for each leg (0 to 3)
}
void wait_reach(int leg){
  while (1)
    if (site_now[leg][0] == site_expect[leg][0])  // Check x-coordinate
      if (site_now[leg][1] == site_expect[leg][1])  // Check y-coordinate
        if (site_now[leg][2] == site_expect[leg][2])  // Check z-coordinate
          break;  // Exit loop if all coordinates match
}
void set_site(int leg, float x, float y, float z){

  /*- Sets one of the endpoints' target positions.
  - This function updates temp_speed[4][3] with the calculated speeds.
  - Non-blocking function.*/

  float length_x = 0, length_y = 0, length_z = 0;

  // Calculate distance to target in each axis if a new target is provided (not KEEP)
  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  // Calculate the total distance to the new target position
  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  // Set speed for each axis, proportional to the distance in each direction
  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  // Update the target position (site_expect) if a new target is provided
  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;

  //Serial.println("set site");
}

int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}