#define F_CPU 16000000
#define BAUD 9600
#define BRC ((F_CPU/16/BAUD)-1)
#define RX_BUFFER_SIZE  4
#define DirectionPin  (10u)
#define BaudRate    (1000000ul) // Baud rate for serial communication with the AX-12A
#define ID1 (7u)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Servo.h>
#include <AX12A.h>
#include <avr/sleep.h>

int ENB = 4; //pwm motor
int bomba = 2; //bomba
volatile int state = 10; //variable de estado
volatile int state_ant = 0; //variable de estado
volatile int conteo = 0;
int pwm = 210;
int cuchara = 1;
Servo cereal;

volatile char opcion = '\0';
int cereal_cant = 0;
int leche_cant = 0;
volatile int estado = 0;

volatile int fin_conteo = 0;
volatile int cont = 0;
volatile int max_conteo = 0;

void ini_serial(){
  UBRR0H = (BRC>>8); // configura la velocidad de transmision
  UBRR0L = BRC; // a 9600 baudios
  UCSR0B = (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0) ; // Habilita la Transmision y recepcion
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); //configura tamaño de palabra 8 bits, sin paridad, 1 bit de parada
}

void anexSerial(int dato){
  UDR0 = dato;
  while (!(UCSR0A & (1 << UDRE0))) {} //esperar a que el bus se libere
}

void saltoLinea(){
  anexSerial(10);
  anexSerial(13);
}

void escribeSerial(const char *str){
  while(*str) anexSerial(*str++);
}

void contar_100ms(int intervalos) {
    fin_conteo = 0;
    cont = 0;
    max_conteo = intervalos;
    TCNT3 = 65536 - (F_CPU / 1024) / 10; // Cargar el valor para 100ms
    TIMSK3 |= (1 << TOIE3); // Habilitar interrupción de Timer 3
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();


    // Esperar hasta que se cumpla el final del conteo
    while (!fin_conteo) {
        sleep_cpu(); 
    }
    sleep_disable(); 
}

void menu_cereal(){
    saltoLinea();
    escribeSerial("Bienvenido a CEREAL OS");
    saltoLinea();
    escribeSerial("Selecciona la cantidad de cereal:  ");
    saltoLinea();
    escribeSerial("   1 - Poco cereal ");
    saltoLinea();
    escribeSerial("   2 - Mucho cereal  ");
    saltoLinea();
    estado = 0;
}

void menu_leche(){
    saltoLinea();
    escribeSerial("Selecciona la cantidad de leche:  ");
    saltoLinea();
    escribeSerial("   1 - Poca leche ");
    saltoLinea();
    escribeSerial("   2 - Mucha leche  ");
    saltoLinea();
}
//
void moveToInitialPosition() {
  ax12a.move(ID1, 0);
}

void setup() {
  ini_serial();
  ax12a.begin(BaudRate, DirectionPin, &Serial2); // Initialize AX-12A
  ax12a.setID(ID1, 7);
  ax12a.setEndless(ID1, OFF);
  moveToInitialPosition();
  pinMode (ENB, OUTPUT);
  DDRB |= (1 << PB6);
  cereal.attach(7); //servo 
  cereal.write(90);
  pinMode(21, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  TCCR3A = 0; // Modo normal
  TCCR3B = (1 << CS32) | (1 << CS30); // Prescaler 1024
  TCNT3 = 0; // Inicializar contador
  TIMSK3 = (1 << TOIE3); // Habilitar interrupción de desbordamiento
  EIMSK = (1<<INT0)|(1<<INT2); //Falling
  EICRA = (1<<ISC01)|(0<<ISC00)|(1<<ISC21)|(0<<ISC20);
  EICRB = (1<<ISC51)|(0<<ISC50);;
  contar_100ms(20);
  sei();
  menu_cereal();
}

void loop() {
  if (state==0){
  analogWrite (ENB, pwm); //Velocidad motor A
  }
  else if (state==1){
     saltoLinea();
     escribeSerial("-------SIRVIENDO CEREAL-----");
     saltoLinea();
    analogWrite (ENB, 0); //Velocidad motor A
    contar_100ms(1);
    if (cereal_cant==1){
      cereal.write(0);
      contar_100ms(8);
      cereal.write(90);
      contar_100ms(10);
    }
    else{
      cereal.write(0);
      contar_100ms(12);
      cereal.write(90);
      contar_100ms(10);
    }
    state = 0;
    }
   else if (state==2){
      saltoLinea();
      escribeSerial("-------SIRVIENDO LECHE-----");
      saltoLinea();
      contar_100ms(1);
      analogWrite (ENB, 0); //Velocidad motor A
      contar_100ms(10);
      if (leche_cant==1){
        digitalWrite(bomba, HIGH);
        contar_100ms(20);
        digitalWrite(bomba, LOW);
        contar_100ms(1);
    }
    else{
      digitalWrite(bomba, HIGH);
      contar_100ms(40);
      digitalWrite(bomba, LOW);
    }
    contar_100ms(1);
    EIMSK |= (1<<INT5);
    state = 0;
   }
   else if (state==3){
    EIMSK &= ~(1<<INT5);
    //contar_100ms(8);
    escribeSerial("-------COLOCANDO CUCHARA-----");
    analogWrite (ENB, 0); //Velocidad motor A
    if(cuchara == 1){
      moveToInitialPosition();
      contar_100ms(10);
      ax12a.setEndless(ID1, ON);
      ax12a.turn(ID1, RIGHT, 1000);
      contar_100ms(20);
      ax12a.turn(ID1, RIGHT, 0);
      contar_100ms(20);
      conteo = 0;
      analogWrite (ENB, pwm);
      contar_100ms(6);
      saltoLinea();
      escribeSerial("> PEDIDO LISTO !!!<");
      saltoLinea();
      escribeSerial("PARA VOLVER A PEDIR PRESIONE LA TECLA 'r'");
      saltoLinea();
      analogWrite (ENB, 0);
      cuchara = 2;
      state =10;
    }else if (cuchara == 2){
      ax12a.turn(ID1, RIGHT, 1000);
      contar_100ms(25);
      ax12a.turn(ID1, RIGHT, 0);
      contar_100ms(20);
      conteo = 0;
      analogWrite (ENB, pwm);
      contar_100ms(6);
      saltoLinea();
      escribeSerial("> PEDIDO LISTO !!!<");
      saltoLinea();
      escribeSerial("PARA VOLVER A PEDIR PRESIONE LA TECLA 'r'");
      saltoLinea();
      analogWrite (ENB, 0);
      cuchara = 3;
      state =10;
    }else if (cuchara == 3){
      ax12a.turn(ID1, RIGHT, 1000);
      contar_100ms(20);
      ax12a.turn(ID1, RIGHT, 0);
      contar_100ms(10);
      ax12a.turn(ID1, LEFT, 1000);
      contar_100ms(65);
      ax12a.turn(ID1, LEFT, 0);
      ax12a.setEndless(ID1, OFF);
      moveToInitialPosition();
      analogWrite (ENB, pwm);
      contar_100ms(6);
      saltoLinea();
      escribeSerial("> PEDIDO LISTO !!!<");
      saltoLinea();
      analogWrite (ENB, 0);
      cli();
      saltoLinea();
      escribeSerial("NO QUEDAN MAS CUCHARAS, PORFAVOR RECARGAR");
      saltoLinea();
      state = 10;
    }
    }else{
    analogWrite (ENB, 0); //Velocidad motor 
    }

}

ISR(INT0_vect){
 if (conteo==0){
 state = 1;
 conteo = 1;

  }
}

ISR(INT2_vect){
  if (conteo == 1){
    state = 2;
    conteo = 2;
  }

}

ISR(INT5_vect){
  if (conteo==2){
    state = 3;
  }
}

ISR(USART0_RX_vect){
    opcion = UDR0;
    if (opcion>='0' && opcion<='9'){
      if (estado == 0) {
          cereal_cant = opcion - '0'; // Convertir char a int
          if (cereal_cant == 1) {
              saltoLinea();
              escribeSerial("POCO CEREAL SELECCIONADO");
          } else if (cereal_cant == 2) {
              saltoLinea();
              escribeSerial("MUCHO CEREAL SELECCIONADO");
          }
          saltoLinea();
          estado = 1; // Cambiar al siguiente estado
          menu_leche();
          // Limpiar el buffer de recepción para evitar datos adicionales
        
      } else if (estado == 1) {
          leche_cant = opcion - '0'; // Convertir char a int
          if (leche_cant == 1) {
              saltoLinea();
              escribeSerial("POCA LECHE SELECCIONADA");
          } else if (leche_cant == 2) {
              saltoLinea();
              escribeSerial("MUCHA LECHE SELECCIONADA");
          }
          state = 0;
          saltoLinea();
      }
    }
    else if (opcion == 'r' && state==10){
      menu_cereal();
    }
}

ISR(TIMER3_OVF_vect) {
    TCNT3 = 65536 - (F_CPU / 1024) / 10; // Recargar el valor para 100ms
    cont++;
    if (cont >= max_conteo) {
        TIMSK3 &= ~(1 << TOIE3); // Deshabilitar interrupción de Timer 3
        fin_conteo = 1;
    }
}
