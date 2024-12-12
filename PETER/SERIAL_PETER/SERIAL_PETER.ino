
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ticker.h>
#include <AracneControl.h>

Ticker ticker; // Create a Ticker object
// PCA9685 instance with I2C address 0x40
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
// Servo pulse range
#define SERVOMIN  125 // Minimum pulse length out of 4096
#define SERVOMAX  575 // Maximum pulse length out of 4096

// Number of legs and joints
#define NUM_LEGS 4
#define NUM_JOINTS 3
#define OMNI 1
#define CAR 2

/*const char* ssid = "Mi_PC";
const char* password = "123456789";*/

// Define the motor control pins
//PATA 0
const int motorPin00 = 2;  
const int motorPin01 = 3;  
//PATA 1
const int motorPin10 = 10;  
const int motorPin11 = 11;  
//PATA 2
const int motorPin20 = 13;  
const int motorPin21 = 12;
//PATA 3
const int motorPin30 = 8;  
const int motorPin31 = 9;
int state;
// Home position angles (adjust these as needed)
const int homePosition[NUM_JOINTS] = {90, 40, 120}; // Shoulder, Elbow, Wrist
const int flatPosition[NUM_LEGS][NUM_JOINTS] = {
  {90, 60, 0},
  {90, 70, 0},
  {90, 75, 0},
  {90, 80, 0}
}; // Shoulder, Elbow, Wrist
const int carPosition[NUM_LEGS][NUM_JOINTS] = {
  {145, 60, 0},
  {45, 70, 0},
  {150, 75, 0},
  {40, 80, 0}
}; // Shoulder, Elbow, Wrist

void anglesToLeg(int leg, int alpha, int beta, int gamma){
  board1.setPWM(servo_pin[leg][0], 0, angleToPulse(alpha)); //coxa
  board1.setPWM(servo_pin[leg][1], 0, angleToPulse(beta));  //femur
  board1.setPWM(servo_pin[leg][2], 0, angleToPulse(gamma)); //tibia
}

void tickerCallback(){
  /*- Microservos service function (called by timer interrupt at 50 Hz)
  - Moves each endpoint toward the expected site in a straight line.
  - `temp_speed[4][3]` should be set before setting the target (`site_expect`) 
    to ensure straight-line motion and determine movement speed.*/
  // Check if the servo service is enabled
  if (!servo_service_en){
    return;
  }
  static float alpha, beta, gamma;  // Temporary variables for joint angles
  //Serial.println("Ya estoy servicial");
  // Loop through each leg to update positions
  for (int i = 0; i < 4; i++)//imax=4
  {
    // Update each coordinate (x, y, z) based on speed
    for (int j = 0; j < 3; j++)
    {
      // If the distance to the target is greater than the incremental speed, move by temp_speed
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];  // Move incrementally
      else
        site_now[i][j] = site_expect[i][j];  // Snap to the target if close enough
    }
    //Serial.print("Leg:");Serial.println(i);
    // Convert the current Cartesian position to polar coordinates (joint angles)
    inverseKinematics(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    if ((i == 0)+(i == 2)){
      alpha = 180 - alpha;
    }
    if (i==0){
      beta = beta - 5;
      gamma = gamma + 5;
    }
    else if (i==3){
      beta = beta + 5;
      gamma = gamma + 10;
    }
    anglesToLeg(i,alpha,beta,gamma);
  }
}

void setup() {
   // Set the motor control pins as outputs
  pinMode(motorPin00, OUTPUT);
  pinMode(motorPin01, OUTPUT);
  pinMode(motorPin10, OUTPUT);
  pinMode(motorPin11, OUTPUT);
  pinMode(motorPin20, OUTPUT);
  pinMode(motorPin21, OUTPUT);
  pinMode(motorPin30, OUTPUT);
  pinMode(motorPin31, OUTPUT);
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Activado");
  
  setCenter();
   
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  move_speed = 8;
  mode = SPIDER;
  
  // Initialize I2C communication on SDA (21) and SCL (22) pins
  Wire.begin(5, 4);

  // Initialize PCA9685 board
  board1.begin();
  board1.setPWMFreq(60);  // Set PWM frequency for servo control (60 Hz)
  
  ticker.attach(0.02,tickerCallback);
}

void serial_comi(){
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Leer comando completo
    command.trim();  // Eliminar espacios o saltos de línea adicionales
    Serial.println("Serial_active");
    if (command == "up") {
      Serial.println("Comando recibido: ARRIBA");
      state = 1;
      
    } else if (command == "down") {
      Serial.println("Comando recibido: ABAJO");
      state = 2;

    } else if (command == "left") {
      Serial.println("Comando recibido: IZQUIERDA");
      state = 3;

    } else if (command == "right") {
      Serial.println("Comando recibido: DERECHA");
      state = 4;

    } else if (command == "up-right") {
      Serial.println("Comando recibido: ARRIBA-DERECHA");
      state = 5;

    } else if (command == "up-left") {
      Serial.println("Comando recibido: ARRIBA-IZQUIERDA");
      state = 6;

    } else if (command == "corner-right") {
      state = 7;

    } else if (command == "corner-left") {
      state = 8;

    } else if (command == "down-right") {
      state = 9;

    } else if (command == "down-left") {
      state = 10;

    } else if (command == "A") {
      Serial.println("Comando recibido: A");
      state = 11;

    } else if (command == "B") {
      Serial.println("Comando recibido: B");
      state = 12;

    } else if (command == "C") {
      Serial.println("Comando recibido: C");
      state = 13;
    } else if (command == "stop"){
      Serial.println("Comando recibido: STOP");
      state = 14;
    } else {
      Serial.println("Comando no reconocido");
    } 
}
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Leer comando completo
    command.trim();  // Eliminar espacios o saltos de línea adicionales

    Serial.print("Recibido: ");
    Serial.println(command);  // Respuesta al comando recibido

    // Responder según el comando
    if (command == "up") {
      Serial.println("Comando recibido: ARRIBA");
      state = 1;
      
    } else if (command == "down") {
      Serial.println("Comando recibido: ABAJO");
      state = 2;

    } else if (command == "left") {
      Serial.println("Comando recibido: IZQUIERDA");
      state = 3;

    } else if (command == "right") {
      Serial.println("Comando recibido: DERECHA");
      state = 4;

    } else if (command == "up-right") {
      Serial.println("Comando recibido: ARRIBA-DERECHA");
      state = 5;

    } else if (command == "up-left") {
      Serial.println("Comando recibido: ARRIBA-IZQUIERDA");
      state = 6;

    } else if (command == "corner-right") {
      state = 7;

    } else if (command == "corner-left") {
      state = 8;

    } else if (command == "down-right") {
      state = 9;

    } else if (command == "down-left") {
      state = 10;

    } else if (command == "A") {
      Serial.println("Comando recibido: A");
      state = 11;

    } else if (command == "B") {
      Serial.println("Comando recibido: B");
      state = 12;

    } else if (command == "C") {
      Serial.println("Comando recibido: C");
      state = 13;
    } else if (command == "stop"){
      Serial.println("Comando recibido: STOP");
      state = 14;
    } else {
      Serial.println("Comando no reconocido");
    }
  }
  //void serial_comi();
  if (state == 1) {
      if (mode== SPIDER){
        if (site_now[0][1] != y_start && site_now[1][1] != y_start) {
          setFrontStep();
        }
        servo_service_en = true;
        step_forward(1);
      }
      else {
        forwardCar();
      }
  } else if (state == 2) {
      if (mode== SPIDER){
        if (site_now[0][1] != y_start && site_now[1][1] != y_start) {
          setFrontStep();
        }
        servo_service_en = true;
        step_back(1);
      }
      else {
        backwardCar();
      }
  } else if (state == 3) {
      if (mode== SPIDER){
        if (site_now[0][0] != x_start && site_now[3][0] != x_start) {
          setSideStep();
        }
        servo_service_en = true;
        step_left(1);
      }
      else if (mode == OMNI){
        omniLeft();
      }
  } else if (state == 4) {
      if (mode== SPIDER){
        if (site_now[0][0] != x_start && site_now[3][0] != x_start) {
          setSideStep();
        }
        servo_service_en = true;
        step_right(1);
      }
      else if (mode == OMNI){
        omniRight();
      }
  } else if (state == 5) {
      if (mode== SPIDER){
        if (site_now[0][1] != y_start && site_now[1][1] != y_start) {
          setFrontStep();
        }
        servo_service_en = true;
        turn_right(1);
      }
      else {
        clockwiseTurn();
      }
  } else if (state == 6) {
      if (mode== SPIDER){
        if (site_now[0][1] != y_start && site_now[1][1] != y_start) {
          setFrontStep();
        }
        servo_service_en = true;
        turn_left(1);
      }
      else {
        counterClockwise();
      }
  } else if (state == 7) {
      if (mode==OMNI){
        diagRightForw();
      }
  } else if (state == 8) {
      if (mode==OMNI){
        diagLeftForw();
      }
  } else if (state == 9) {
      if (mode==OMNI){
        diagRightBack();
      }
  } else if (state == 10) {
      if (mode==OMNI){
        diagLeftBack();
      }
  } else if (state == 11) {
      stopVehicle();
      //moveToHomePosition();
      setCenter();
      mode = SPIDER;
      servo_service_en = true;
  } else if (state == 12) {
      servo_service_en = false;
      stopVehicle();
      move2carPosition();
      mode = CAR;
  } else if (state == 13) {
      servo_service_en = false;
      stopVehicle();
      move2flatPosition();
      mode = OMNI;
  } else if (state == 14){
      servo_service_en = false;
      stopVehicle();
  }
}

void moveToHomePosition() {
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int joint = 0; joint < NUM_JOINTS; joint++) {
      board1.setPWM(servo_pin[leg][joint], 0, angleToPulse(flatPosition[leg][joint]));
      //moveServo(leg, joint, homePosition[joint], 15);
    }
  }
}
void move2flatPosition() {
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int joint = 0; joint < NUM_JOINTS ; joint++) {
      board1.setPWM(servo_pin[leg][joint], 0, angleToPulse(flatPosition[leg][joint]));
      //moveServo(leg, joint, flatPosition[joint], 15);
    }
  }
}
void move2carPosition() {
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int joint = 0; joint < NUM_JOINTS ; joint++) {
      board1.setPWM(servo_pin[leg][joint], 0, angleToPulse(carPosition[leg][joint]));
      //moveServo(leg, joint, flatPosition[joint], 15);
    }
  }
}

void setCenter(){
  //LADO IZQUIERDO, PATAS 2(DELANTERA) Y 1(TRASERA)
  set_site(1, x_default, y_default , z_boot);
  set_site(2, x_default, y_default , z_boot);
  //LADO DERECHO, PATAS 3(DELANTERA) Y 0(TRASERA)
  set_site(0, x_default, y_default, z_boot);
  set_site(3, x_default, y_default, z_boot);
}  
void setFrontStep(){
  //LADO IZQUIERDO, PATAS 2(DELANTERA) Y 1(TRASERA)
  set_site(1, x_default - x_offset, y_start , z_boot);
  set_site(2, x_default + x_offset, y_start , z_boot);
  //LADO DERECHO, PATAS 3(DELANTERA) Y 0(TRASERA)
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(3, x_default + x_offset, y_start + y_step, z_boot);
}
void setSideStep(){
  //LADO IZQUIERDO, PATAS 2(DELANTERA) Y 1(TRASERA)
  set_site(1, x_start, y_default , z_boot);
  set_site(2, x_default + x_offset, y_default , z_boot);
  //LADO DERECHO, PATAS 3(DELANTERA) Y 0(TRASERA)
  set_site(0, x_start, y_default, z_boot);
  set_site(3, x_default + x_offset, y_default, z_boot);
}
const unsigned long intervalo = 1000;  // 1 segundos en milisegundos

void backwardCar(){
  //int tiempoInicio = millis();
  digitalWrite(motorPin00, HIGH);
  digitalWrite(motorPin01, LOW);

  digitalWrite(motorPin10, LOW);
  digitalWrite(motorPin11, HIGH);

  digitalWrite(motorPin20, LOW);
  digitalWrite(motorPin21, HIGH);
  
  digitalWrite(motorPin30, LOW);
  digitalWrite(motorPin31, HIGH);
  /*while(1){
    if (millis() - tiempoInicio >= intervalo) {
    digitalWrite(motorPin00, LOW);
    digitalWrite(motorPin01, LOW);
    digitalWrite(motorPin10, LOW);
    digitalWrite(motorPin11, LOW);
    digitalWrite(motorPin20, LOW);
    digitalWrite(motorPin21, LOW);
    digitalWrite(motorPin30, LOW);
    digitalWrite(motorPin31, LOW);
    break;
    }
  }*/
}

void clockwiseTurn(){
  //int tiempoInicio = millis();
  digitalWrite(motorPin00, LOW);
  digitalWrite(motorPin01, HIGH);

  digitalWrite(motorPin10, LOW);
  digitalWrite(motorPin11, HIGH);

  digitalWrite(motorPin20, HIGH);
  digitalWrite(motorPin21, LOW);
  
  digitalWrite(motorPin30, LOW);
  digitalWrite(motorPin31, HIGH);
}

void counterClockwise(){
  //int tiempoInicio = millis();
  digitalWrite(motorPin00, HIGH);
  digitalWrite(motorPin01, LOW);

  digitalWrite(motorPin10, HIGH);
  digitalWrite(motorPin11, LOW);

  digitalWrite(motorPin20, LOW);
  digitalWrite(motorPin21, HIGH);
  
  digitalWrite(motorPin30, HIGH);
  digitalWrite(motorPin31, LOW);
}

void forwardCar(){
  //int tiempoInicio = millis();
  digitalWrite(motorPin00, LOW);
  digitalWrite(motorPin01, HIGH);

  digitalWrite(motorPin10, HIGH);
  digitalWrite(motorPin11, LOW);

  digitalWrite(motorPin20, HIGH);
  digitalWrite(motorPin21, LOW);
  
  digitalWrite(motorPin30, HIGH);
  digitalWrite(motorPin31, LOW);
}
//OmniDireccional Derecha
void omniLeft(){
  //int tiempoInicio = millis();
  //PATA 0
  digitalWrite(motorPin00, LOW);
  digitalWrite(motorPin01, HIGH);
  //PATA 1
  digitalWrite(motorPin10, LOW);
  digitalWrite(motorPin11, HIGH);
  //PATA 2
  digitalWrite(motorPin20, LOW);
  digitalWrite(motorPin21, HIGH);
  //PATA 3
  digitalWrite(motorPin30, HIGH);
  digitalWrite(motorPin31, LOW);
}

//OmniDireccional Izquierda
void omniRight(){
  //int tiempoInicio = millis();
  //PATA 0
  digitalWrite(motorPin00, HIGH);
  digitalWrite(motorPin01, LOW);
  //PATA 1
  digitalWrite(motorPin10, HIGH);
  digitalWrite(motorPin11, LOW);
  //PATA 2
  digitalWrite(motorPin20, HIGH);
  digitalWrite(motorPin21, LOW);
  //PATA 3
  digitalWrite(motorPin30, LOW);
  digitalWrite(motorPin31, HIGH);
}

//OmniDireccional Diagonal DerechaFrente
void diagRightForw(){
  //int tiempoInicio = millis();
  //PATA 0
  digitalWrite(motorPin00, LOW);
  digitalWrite(motorPin01, HIGH);
  //PATA 1
  digitalWrite(motorPin10, LOW);
  digitalWrite(motorPin11, LOW);
  //PATA 2
  digitalWrite(motorPin20, LOW);
  digitalWrite(motorPin21, HIGH);
  //PATA 3
  digitalWrite(motorPin30, LOW);
  digitalWrite(motorPin31, LOW);
}

//OmniDireccional Diagonal IzquierdaFrente
void diagLeftForw(){
  //int tiempoInicio = millis();
  //PATA 0
  digitalWrite(motorPin00, LOW);
  digitalWrite(motorPin01, LOW);
  //PATA 1
  digitalWrite(motorPin10, LOW);
  digitalWrite(motorPin11, HIGH);
  //PATA 2
  digitalWrite(motorPin20, LOW);
  digitalWrite(motorPin21, LOW);
  //PATA 3
  digitalWrite(motorPin30, HIGH);
  digitalWrite(motorPin31, LOW);
}

//OmniDireccional Diagonal IzquierdaAtras
void diagLeftBack(){
  //int tiempoInicio = millis();
  //PATA 0
  digitalWrite(motorPin00, HIGH);
  digitalWrite(motorPin01, LOW);
  //PATA 1
  digitalWrite(motorPin10, LOW);
  digitalWrite(motorPin11, LOW);
  //PATA 2
  digitalWrite(motorPin20, HIGH);
  digitalWrite(motorPin21, LOW);
  //PATA 3
  digitalWrite(motorPin30, LOW);
  digitalWrite(motorPin31, LOW);
}

//OmniDireccional Diagonal DerechaFrente
void diagRightBack(){
  //int tiempoInicio = millis();
  //PATA 0
  digitalWrite(motorPin00, HIGH);
  digitalWrite(motorPin01, LOW);
  //PATA 1
  digitalWrite(motorPin10, LOW);
  digitalWrite(motorPin11, LOW);
  //PATA 2
  digitalWrite(motorPin20, LOW);
  digitalWrite(motorPin21, HIGH);
  //PATA 3
  digitalWrite(motorPin30, LOW);
  digitalWrite(motorPin31, LOW);
}

void stopVehicle(){
  //int tiempoInicio = millis();
  //PATA 0
  digitalWrite(motorPin00, LOW);
  digitalWrite(motorPin01, LOW);
  //PATA 1
  digitalWrite(motorPin10, LOW);
  digitalWrite(motorPin11, LOW);
  //PATA 2
  digitalWrite(motorPin20, LOW);
  digitalWrite(motorPin21, LOW);
  //PATA 3
  digitalWrite(motorPin30, LOW);
  digitalWrite(motorPin31, LOW);
}