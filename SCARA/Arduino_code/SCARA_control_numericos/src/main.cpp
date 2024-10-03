#include <i2c_module.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <AccelStepper.h>
#include <Servo.h>

#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define Y_STEP_PIN 3
#define Y_DIR_PIN 6
#define Z_STEP_PIN 4
#define Z_DIR_PIN 7
#define A_STEP_PIN 12
#define A_DIR_PIN 13
#define SERVO_PIN 22 // Pin para el servo

// Define nuevos límites para los ejes
#define X_MIN_POSITION 0
#define X_MAX_POSITION 10000 // Aumentado el rango máximo de X
#define Y_MIN_POSITION 0
#define Y_MAX_POSITION 6000 // Aumentado el rango máximo de Y
#define Z_MIN_POSITION 0
#define Z_MAX_POSITION 6000 // Aumentado el rango máximo de Z
#define A_MIN_POSITION 0
#define A_MAX_POSITION 10000 // Aumentado el rango máximo de A

// Factores de conversión de grados a pasos
#define X_STEPS_PER_DEGREE 14
#define Y_STEPS_PER_DEGREE 10.6
#define Z_STEPS_PER_DEGREE 3.2
#define A_STEPS_PER_DEGREE 25

#define rele 41
#define led 24
int boton=1;
unsigned long last_time = 0;
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepperA(AccelStepper::DRIVER, A_STEP_PIN, A_DIR_PIN);

Servo myServo; // Crea un objeto Servo


i2c_module sensorModule(0x70);
// Definición de pines para los motores y el servo

// Instanciar la clase con la dirección del TCA9548A

double promedios[4];
#define N 5 //LECTURAS A PROMEDIAR 

float lecturas[N][4];   // Array para almacenar las lecturas
int indice = 0;
float suma[4] = {0.0,0.0,0.0,0.0};
int sensores_values[4]; //{hombro, codo, muñeca, altura}

double q1,q2,q3,q4;

double a1 = 0.228;
double a2 = 0.1365;
double d1 = 0.045;
double d2 = 0.0575;
double d3 = 0.0811;

float sensores_mesure[4]; //[hombro, codo, muñeca ]
float scara_values[5]; //{theta1, z, theta2, phi, servo_angle}

double x,y,z, yaw, nx, ny;
double xd, yd, zd;
double pos_deseadas[6];
double datos[5] = {0, 0, 0, 0, 0};

void leer_sensores();
void promediar_sensores();

void k_directa(double q1 ,double q2, double q3){
  x = -0.1365*sin(q1)*sin(q3) + 0.1365*cos(q1)*cos(q3) + 0.228*cos(q1);
  y = 0.1365*sin(q1)*cos(q3) + 0.228*sin(q1) + 0.1365*sin(q3)*cos(q1);
  z = q2 - 0.0936;
}

void geometrico(){
  q2 = z - d1 + d2 + d3;
  double D = pow(x,2) + pow(y, 2);
  double c2 = (D - pow(a1,2) - pow(a2,2)) / (2 * a1 * a2);
  c2 = constrain(c2, -1.0, 1.0);
  q3 = atan2(sqrt(1-pow(c2,2)), c2);
  q1 = atan2(y, x) - atan2(a2*sin(q3), a1+a2*cos(q3));
  q4 = atan2(nx, ny) - q1 - q3;

}

void mth(){
 if (x==0){
  x = 0.00001;
 }
 q2 = z - d1 + d2 + d3;
 double D = pow(x,2) + pow(y,2);
 double c2 = (D - pow(a1,2) - pow(a2,2))/(2*a1*a2);
 c2 = constrain(c2, -1.0, 1.0);
 q3 = atan2(sqrt(1-pow(c2,2)), c2);
 double c1 = a2*sin(q3)*(y/x)+a2*cos(q3)+a1;
 c1 = c1/ (x + (pow(y,2)/x));
 c1 = constrain(c1, -1.0, 1.0);

 q1 = atan2(sqrt(1-pow(c1,2)), c1);
 q4 = atan2(nx,ny) - q1 - q3;
}

void algebraico(){
  q1 = atan2(y,x) + acos(constrain((53.67*(pow(x,2) + pow(y,2))+1.79)/(24.46*sqrt(pow(x,2) + pow(y,2))),-1,1));
  q3 = atan2(y*cos(q1)-x*sin(q1),x*cos(q1)+y*sin(q1)-0.228);
  q2 = z - d1 + d2 + d3;
  q4 = atan2(nx,ny) - q1 - q3;
}

void JT_calculo(double JT[3][3], double delta){
 k_directa(q1+delta, q2, q3);
  JT[0][0] = x;
  JT[0][1] = y;
  JT[0][2] = z;
  k_directa(q1, q2, q3);
  JT[0][0] -= x;
  JT[0][1] -= y;
  JT[0][2] -= z;

  k_directa(q1, q2+delta, q3);
  JT[1][0] = x;
  JT[1][1] = y;
  JT[1][2] = z;

  k_directa(q1, q2, q3);
  JT[1][0] -= x;
  JT[1][1] -= y;
  JT[1][2] -= z;

  k_directa(q1, q2, q3+delta);
  JT[2][0] = x;
  JT[2][1] = y;
  JT[2][2] = z;

  k_directa(q1, q2, q3);
  JT[2][0] -= x;
  JT[2][1] -= y;
  JT[2][2] -= z;

  for (int i = 0; i < 3; i++) {
  JT[i][0] = (1/delta)*JT[i][0];
  JT[i][1] = (1/delta)*JT[i][1];
  JT[i][2] = (1/delta)*JT[i][2];
  }

   
}

void transponerMatriz(double JT[3][3], double J[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      J[j][i] = JT[i][j];
    }
  }
}

double determinant3x3(double matrix[3][3]) {
  return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
       - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
       + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
}

void cofactor3x3(double matrix[3][3], double cofactorMatrix[3][3]) {
  cofactorMatrix[0][0] = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]);
  cofactorMatrix[0][1] = -(matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]);
  cofactorMatrix[0][2] = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

  cofactorMatrix[1][0] = -(matrix[0][1] * matrix[2][2] - matrix[0][2] * matrix[2][1]);
  cofactorMatrix[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]);
  cofactorMatrix[1][2] = -(matrix[0][0] * matrix[2][1] - matrix[0][1] * matrix[2][0]);

  cofactorMatrix[2][0] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]);
  cofactorMatrix[2][1] = -(matrix[0][0] * matrix[1][2] - matrix[0][2] * matrix[1][0]);
  cofactorMatrix[2][2] = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]);
}

void inverse3x3(double matrix[3][3], double inverseMatrix[3][3]) {
  // Calcular el determinante de la matriz
  double det = determinant3x3(matrix);

  if (det == 0) {
    // Si el determinante es 0, la matriz no tiene inversa
    Serial.println("La matriz no tiene inversa");
    return;
  }

  // Matriz de cofactores
  double cofactorMatrix[3][3];
  cofactor3x3(matrix, cofactorMatrix);

  // Matriz adjunta (transpuesta de la matriz de cofactores)
  double adjointMatrix[3][3];
  transponerMatriz(cofactorMatrix, adjointMatrix);

  // Calcular la inversa dividiendo la adjunta por el determinante
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      inverseMatrix[i][j] = adjointMatrix[i][j] / det;
    }
  }
}

void printMatrix3x3(double matrix[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(matrix[i][j], 4);  // 4 decimales
      Serial.print("\t");
    }
    Serial.println();
  }
}

void dotProduct3x3(double matrix[3][3], double vector[3], double result[3]) {
  for (int i = 0; i < 3; i++) {
    result[i] = 0;  // Inicializamos el resultado en 0
    for (int j = 0; j < 3; j++) {
      result[i] += matrix[i][j] * vector[j];  // Producto punto para cada fila
    }
  }
}

void printVector(double vector[3]) {
  for (int i = 0; i < 3; i++) {
    Serial.print("Elemento ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(vector[i], 4);  // Imprimir con 4 decimales
  }
}

double epsilon = 0.001; //los epsilon no pueden ser 0

void metodos_numericos(){
  xd = pos_deseadas[0];
  xd = pos_deseadas[0] + epsilon*(pos_deseadas[0]==0);

  yd = pos_deseadas[1];
  yd = pos_deseadas[1] + epsilon*(pos_deseadas[1]==0);

  zd = pos_deseadas[2];
  zd = pos_deseadas[3] + epsilon*(pos_deseadas[3]==0);

  double delta = 0.001;
  //double error;
  long int contador = 0;
  double alpha = 1.3;

  q1 = double(sensores_values[0])*PI/180;
  q1 = q1+epsilon*(q1==0);

  // q1 = double(sensores_values[])*PI/180;
  // q1 = q1+epsilon*(q1==0);

  q2 = sensores_values[3];
  q2 = q2 + epsilon*(q2==0);

  q3 = double(sensores_values[1])*PI/180;
  q3 = q3+epsilon*(q3==0);

  q4 = double(sensores_values[2])*PI/180;
  q4 = q4+epsilon*(q4==0);


  double JT[3][3]; 
  double J[3][3]; 
  while(true){
  JT_calculo(JT, delta);
  transponerMatriz(JT ,J);
  q4 = atan2(nx,ny) - q1 - q3;

  k_directa(q1, q2, q3);
  double error_local[3];
  error_local[0] = xd -x;
  error_local[1] = yd - y;
  error_local[2] = zd - z;
  double r_dot[3];
  double inv_j[3][3];

  if (pos_deseadas[5]==4){
  inverse3x3(J, inv_j);
  dotProduct3x3(inv_j, error_local, r_dot);
  }

  if (pos_deseadas[5]==5){
  dotProduct3x3(JT, error_local, r_dot);
  // printVector(r_dot);
  for (int i = 0; i < 3; i++) {
    r_dot[i] = alpha*r_dot[i];
  }
  }
  // Serial.println("Matriz Original:");
  // printVector(r_dot);
  q1 = q1 + r_dot[0];
  q2 = q2 + r_dot[1];
  q3  =q3 + r_dot[2];

  

 
  double condi_conv = sqrt(pow(error_local[0],2) + pow(error_local[1],2) + pow(error_local[2], 2));
  if ( condi_conv < epsilon){
    Serial.println(q1);
    Serial.println(q2);
    Serial.println(q3);
    Serial.println(q4);
    Serial.println(contador);
    Serial.println("le dio");
    break;
  }
  contador +=1;
  if (contador==800){
     Serial.println(q1);
    Serial.println(q2);
    Serial.println(q3);
    Serial.println(q4);
    Serial.println(condi_conv);
    Serial.println("no le dio");
    break;
  }
  // for (int i = 0; i < 3; i++) {
  //   Serial.print("Elemento ");
  //   Serial.print(i);
  //   Serial.print(": ");
  //   Serial.println(error_local[i]);
  // }
  // for (int i = 0; i < 3; i++) {
  //   for (int j = 0; j < 3; j++) {
  //     Serial.print(J[i][j]);
  //     Serial.print("\t"); // Para separar los valores con un tabulador
  //   }
  //   Serial.println(); // Salto de línea al final de cada fila
  // }
  //}
  }
}
// Función para pedir los valores al usuario


void esperar_valores(){
  while (true)
  {
  leer_sensores();
  if (Serial.available() == sizeof(float)* 6){
    float valores[6];
    Serial.readBytes((char *)valores, sizeof(valores));
    for (int i = 0; i < 6; i++) {
      pos_deseadas[i] = valores[i]; //(x,y,z,yaw)
      }
    Serial.print("Datos recibidos: ");
    Serial.println(pos_deseadas[5]);
    break;
  }
  }
  
  // x = pos_deseadas[0];
  // y = pos_deseadas[1];
  // z = pos_deseadas[2];
  // yaw = pos_deseadas[3];

  nx = cos(yaw);
  ny = -1*sin(yaw);

  if (pos_deseadas[5] == 1){
    geometrico();
    delay(10);
    Serial.println("Metodo geometrico");
  }
  else if (pos_deseadas[5] == 2){
    mth();
    delay(10);
    Serial.println("Metodo MTH");
  }
  else if (pos_deseadas[5]==3){
    algebraico();
    delay(10);
    Serial.println("Algebraico");
  }
  else if (pos_deseadas[5]==4 or pos_deseadas[5]==5){
  metodos_numericos();
  }
  
}

void setup() {
  pinMode(rele, OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  digitalWrite(rele, LOW);
  pinMode(19, INPUT_PULLUP);
  Serial.begin(115200);
  Serial2.begin(115200);
  sensorModule.begin();

   stepperX.setMaxSpeed(500);
  stepperX.setAcceleration(800);

  stepperY.setMaxSpeed(2000);
  stepperY.setAcceleration(700);

  stepperZ.setMaxSpeed(2000);
  stepperZ.setAcceleration(700);

  stepperA.setMaxSpeed(1000);
  stepperA.setAcceleration(1000);

  myServo.attach(SERVO_PIN); // Conecta el servo al pin definido
  myServo.write(0); // Inicializa el servo en l, a posición 0 grados

  //pedirValores();  // Llamar a la función para pedir los valores
  delay(10);
  EIMSK = (1<<INT2);      //Hab. Interrupción Externa
  EICRA = (1<<ISC21);     //Interrupción por flanco de bajada
  sei();
 
  for (int i = 0; i < N; i++) {
    leer_sensores();
  }
  
}



void loop() {
  esperar_valores();

  scara_values[0] = q1*180 / PI;

  scara_values[1] = q2*100-9;

  scara_values[2] = q3*180 / PI;
  scara_values[3] = q4*180 / PI;
  scara_values[4] =  pos_deseadas[4];

  // if (scara_values[0]>360){
  //   scara_values[0] = scara_values[0]-int(scara_values[0]/360)*360;
  // }
  // if (scara_values[0]<-360){
  //   scara_values[0] = scara_values[0]+int(abs(scara_values[0])/360)*360;
  // }
  // if (scara_values[2]>360){
  //   scara_values[2] = scara_values[2]-int(scara_values[2]/360)*360;
  // }
  // if (scara_values[2]<-360){
  //   scara_values[2] = scara_values[2]+int(abs(scara_values[2])/360)*360;
  // }

  Serial.println(scara_values[0]);
 Serial.println(scara_values[1]);
  Serial.println(scara_values[2]);

  stepperX.setCurrentPosition(sensores_values[0] * X_STEPS_PER_DEGREE);
  stepperA.setCurrentPosition((sensores_values[3]) * A_STEPS_PER_DEGREE);
  stepperY.setCurrentPosition(sensores_values[1] * Y_STEPS_PER_DEGREE);
  stepperZ.setCurrentPosition(sensores_values[2] * Z_STEPS_PER_DEGREE);




    long xSteps = scara_values[0] * X_STEPS_PER_DEGREE; //codo
    
    long aSteps = scara_values[1] * A_STEPS_PER_DEGREE; // Sin conversión para el eje A, asumiendo pasos directos
    long ySteps = scara_values[2] * Y_STEPS_PER_DEGREE; // prismatico
    long zSteps = scara_values[3] * Z_STEPS_PER_DEGREE;
    
    //Serial.println(aSteps);
    
    
    myServo.write(scara_values[4]);
  //   // Mueve los motores a la posición objetivo
    stepperX.moveTo(xSteps);
    stepperY.moveTo(ySteps);
    stepperZ.moveTo(zSteps);
    stepperA.moveTo(aSteps);
    
    while (stepperX.isRunning() || stepperY.isRunning() || stepperZ.isRunning() || stepperA.isRunning()) {
            stepperX.run();
            stepperY.run();
            stepperZ.run();
            stepperA.run();
     
  }

    
}


void leer_sensores(){
  int angle = sensorModule.readAS5600Angle(0); //hombro
  float angleInDegrees = angle * 0.08789;
  sensores_mesure[0] = angleInDegrees;
  // Serial.print(">Hombro:");
  // Serial.println(angleInDegrees);

  angle = sensorModule.readAS5600Angle(3); //codo 
  angleInDegrees = angle * 0.08789;
  sensores_mesure[1] = angleInDegrees;
  //Serial.print(">Codo:");
  //Serial.println(angleInDegrees);
  angle = sensorModule.readAS5600Angle(2); //muñeca
  angleInDegrees = angle * 0.08789;
  sensores_mesure[2] = angleInDegrees;
  //Serial.print(">Muñeca:");
  //Serial.println(angleInDegrees);

  if (sensorModule.readVL53L0XDistance(1) != -1) {
   Serial.print(">Z: ");

    sensores_mesure[3] = sensorModule.readVL53L0XDistance(1);
    Serial.println(sensores_mesure[3]/10);
  } 
  promediar_sensores();
  int entrada;
  
  if (promedios[0] <= 120){
    entrada = map(promedios[0],121,107 ,90,120);
  }
  else if (promedios[0] > 210){
    entrada = map(promedios[0],210,292 ,0,-90);
  }
  else if (promedios[0] > 292){
    entrada = map(promedios[0],292,331 ,-90,-120);
  }
  else{
    entrada = map(promedios[0],210,121 ,0,90);
  }
  // else{
  //   entrada = map(promedios[0],210, 331, 0, -120);
  // }
  entrada = constrain(entrada, -180, 180);
  sensores_values[0] = entrada;
  // Serial.print(">hombro_prom:"); //Revisar hombro por si falla inv_k
  // Serial.println(int(entrada));

  // Serial.print(">Codo_prom:"); //Revisar Codo error aparente de 10 grados

  entrada = promedios[1]-154;
  sensores_values[1] = entrada;
  //Serial.println(entrada);

  Serial.print(">muñeca_prom:"); //Revisar Codo error aparente de 10 grados
  entrada = (promedios[2] - 47);
  
  if (promedios[2] <= 46){
    entrada = map(promedios[2],46, 0, 0, -46);
  }
  if (promedios[2] > 230){
    entrada = map(promedios[2],360, 322, -46, -90);
  }
  // else{
  //    entrada = map(promedios[2],227, 360, 0,133);
  // }

  entrada = constrain(entrada, -180, 180);
  sensores_values[2] = -1*entrada;
  Serial.println(sensores_values[2]);
  
  Serial.print(">Z_prom:");
  int entrada_1 = (promedios[3]-98); // medida en mm //off set de
  entrada_1 = entrada_1*(entrada_1>0);
  sensores_values[3] = entrada_1;
  Serial.println(entrada_1);

}


void promediar_sensores(){
   // Restar la lectura más antigua de la suma
   for (int i = 0; i < 4; i++) {
    suma[i] = suma[i] - lecturas[indice][i];
    // Guardar la nueva lectura en el array
    lecturas[indice][i] = sensores_mesure[i];
    // Sumar la nueva lectura a la suma
    suma[i] = suma[i] + lecturas[indice][i];

   }

  
  // Incrementar el índice
  indice = indice + 1;
  // Si llegamos al final del array, volver al principio
  if (indice >= N) {
    indice = 0;
  }
  // Calcular el promedio
  for (int j = 0; j < 4; j++) {
    promedios[j] = suma[j] / N;
   }
  
}

ISR(INT2_vect){
  if (millis()-last_time > 300){
    if (boton==0){
      boton = 1;
      digitalWrite(led, LOW);
      digitalWrite(rele, LOW);
    }
    else {
      boton = 0;
      digitalWrite(led, HIGH);
      digitalWrite(rele, HIGH);

    }
  last_time = millis();
  }
  
}