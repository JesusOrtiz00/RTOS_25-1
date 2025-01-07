#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "timers.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <array>
#include "semphr.h"


static const int led_pin = 14;
static const int button_pin_task1 = 12;
static TimerHandle_t timerTask1 = NULL;
static TimerHandle_t timerTask3 = NULL;

volatile bool start_timer_flag = false; 
volatile bool button_pressed_flag = false;
volatile bool paso_un_segundo = false;
volatile long double segundos = 0;

static const int led_pin_task3 = 15;
static const int button_pin_task3 = 13;
int period_task_5 = 250;

SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t UARTMutex;

//Declaración de variables para la pantalla LCD
#define SDA 0
#define SCL 1
#define COLUMNS 16
#define ROWS    2

LiquidCrystal_I2C lcd(0x27, COLUMNS, ROWS);

static const int LA = 26;
static const float refVoltage = 3.3; // Voltaje de referencia del ADC (3.3V para este caso)
static const float adcResolution = 1023.0; // Resolución del ADC (10 bits, valores entre 0 y 1023)


volatile bool  button_task1_pressed = false;
volatile float voltaje_ADC_task2 = 0;
volatile bool  button_task3_pressed = false;

#define led_pin_task5 16

#define BUTTTON_TASK6_PIN 6
#define PERIOD_TIMER 10

// Dirección del MPU6050
#define MPU6050_ADDR 0x68

// Registros del MPU6050
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define PWR_MGMT_1   0x6B

static TimerHandle_t timerTask6 = NULL;
String caras[] = {"UP", "FRONT", "DOWN", "BACK", "RIGHT", "LEFT"};
enum face {UP, FRONT, DOWN, BACK, RIGHT, LEFT};
enum face cara_actual = UP;
bool flag_button_t6 = false;
bool start_timer_T6 = false;
unsigned long face_times[6] = {0};
std::array<float, 3> accelArray = {0};

// Función para leer dos bytes y convertirlos a un valor de 16 bits
int16_t read16bit(uint8_t reg);
void init_MPU();
std::array<float, 3> read_IMU_XLR8();

//funcion para verificar cual cara es la actual de la tarea 6
void check_face();

// funcion para imprimir los tiempos de las caras de la tarea 6
void print_times();

/*****  Tarea 1  *****/

// interrupcion tarea 1
void buttonISR(){
  if(!digitalRead(button_pin_task1)){
    button_pressed_flag = true;  
  }else{
    button_pressed_flag = false;
  }
}

void onTimerTask1(TimerHandle_t xTimer){
  if(segundos < 3)  segundos++;
  paso_un_segundo = true;
}

void task1(void *parameters){
  int pin_state;
  
  while(1){

    if(button_pressed_flag){


      if(!start_timer_flag){
        xTimerStart(timerTask1,0);
        start_timer_flag = true;
        button_task1_pressed = true;
      }
      
      if(segundos < 3){ 
        digitalWrite(led_pin, HIGH);
      }

      if((segundos == 3) && (paso_un_segundo)){
        pin_state = digitalRead(led_pin);
        digitalWrite(led_pin, !pin_state);
        paso_un_segundo = false;
      }

    }else{

      digitalWrite(led_pin, LOW);
      if(start_timer_flag){ 
        xTimerStop(timerTask1,0);
        start_timer_flag = false;
        button_task3_pressed = false;
      }
      segundos = 0;

    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
void setup_timer_task1(){
  // create and start timer (num, divider, countUp)
  static const TickType_t dim_delay = 1000 / portTICK_PERIOD_MS;
  timerTask1 =  xTimerCreate(
                  "Timer tarea 1",
                  dim_delay,
                  pdTRUE,
                  (void*)0,
                  onTimerTask1);

  xTimerStop(timerTask1,0);
}

/*****  Tarea 2  *****/

//Tarea
void Tarea_ADC(void* parameters){
  const char* nombre_tarea = (const char*)parameters;

  while(1){
    int valor_ADC = analogRead(LA);  // Leer del ADC
    float voltaje = (valor_ADC * refVoltage) / adcResolution; // Calcular voltaje
    voltaje_ADC_task2 = voltaje;

    if (xSemaphoreTake(i2cMutex, 0) == pdTRUE){
        lcd.clear();  // Limpiar pantalla LCD antes de escribir nueva información

        lcd.setCursor(0, 0); // Primer renglón
        lcd.print("ADC: ");
        lcd.print(voltaje);
        lcd.print(" V");
        xSemaphoreGive(i2cMutex);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Esperar 1 segundo entre cada lectura
  }
}


/*****  Tarea 3  *****/


// Tarea: LED parpadeando si el botón está presionado
void TareaParpadeo(void *parameter) {
  while (1) {
    // Leer el estado del botón (LOW significa presionado)
    if (digitalRead(button_pin_task3) == LOW) {
      button_task3_pressed = true;
      // Si el botón está presionado, parpadear el LED
      digitalWrite(led_pin_task3, HIGH);
      vTaskDelay(1000 / portTICK_PERIOD_MS); // Encendido por 1 segundo
      digitalWrite(led_pin_task3, LOW);
      vTaskDelay(1000 / portTICK_PERIOD_MS); // Apagado por 1 segundo
    } else {
      button_task3_pressed = false;
      // Si el botón no está presionado, apagar el LED
      digitalWrite(led_pin_task3, LOW);
    }
    
  }
}

/*****  Tarea 4  *****/

void tareaEstados(void *parameters) {
  const size_t bufferSize = 32; // Tamaño máximo para el comando
  char inputBuffer[bufferSize] = {0}; // Buffer para la entrada serial
  char inputNum[bufferSize] = {0};
  const char* estadoT1 = "ET01";
  const char* estadoT2 = "ET02";
  const char* estadoT3 = "ET03";
  const char* delayT5 = "ET05";
  const char* estadoT6 = "ET06";
  const char* tarea7 = "ET07";

  bool number = false;
  int inicio_numeros = 0;

  while (1) {
    
    // Leer datos del puerto serial
    if (Serial.available() > 0) {
      size_t len = Serial.readBytesUntil('\n', inputBuffer, bufferSize - 1);
      inputBuffer[len] = '\0'; // Asegurar que termina en '\0'
      Serial.println("LEN: " + len);
      // Limpiar caracteres no deseados
      for (size_t i = 0; i < len; i++) {
        if (inputBuffer[i] == '\r') {
          inputBuffer[i] = '\0';
        }
        if(inputBuffer[i] == ':'){
          number = true;
          inicio_numeros = i;
        }
        if(number){
          inputNum[i-inicio_numeros] = inputBuffer[i+1];
          inputBuffer[i] = '\0';
        }
      }
      number = false;
      

      Serial.print("Comando recibido: ");
      Serial.println(inputBuffer);
     // Serial.print("Numero: ");
     // Serial.println(inputNum);

      // Comparar con comandos válidos
      if (strcmp(inputBuffer, estadoT1) == 0) {

        if (button_pressed_flag)
          Serial.println("Tarea 1: Botón presionado.");
        else
          Serial.println("Tarea 1: Botón no presionado.");

      } else if (strcmp(inputBuffer, estadoT2) == 0) {

        Serial.print("Tarea 2: Voltaje ADC = ");
        Serial.print(voltaje_ADC_task2);
        Serial.println(".");

      } else if (strcmp(inputBuffer, estadoT3) == 0) {

        if (button_task3_pressed)
          Serial.println("Tarea 3: Botón presionado.");
        else
          Serial.println("Tarea 3: Botón no presionado.");

      } else if (strcmp(inputBuffer, delayT5) == 0) {

        String delayTask5 = inputNum;
        period_task_5 = delayTask5.toInt();
        
        Serial.print("Periodo tarea 5: ");
        Serial.print(period_task_5);
        Serial.println(" ms");
        
        
      } else if (strcmp(inputBuffer, estadoT6) == 0) {
        if(flag_button_t6){
          Serial.print("Cara actual: ");
          Serial.println(caras[cara_actual]);
          print_times();
        }else{
          Serial.println("No se está contando el tiempo");
        }
      } else if (strcmp(inputBuffer, tarea7) == 0) {
        Serial.println("*** Tarea 7 ***");
        Serial.println("Ingrese el angulo en grados: ");
            float x1, x;
            int flag_T7 = 0;
              do{
                
                if (Serial.available() > 0) {
                  int index = 0;
                  while (Serial.available() && index < sizeof(inputBuffer) - 1) {
                    inputBuffer[index++] = Serial.read();
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                  }
                  inputBuffer[index] = '\0'; // Terminar el comando
                  flag_T7=1;
                }
              }while(flag_T7 == 0);
              flag_T7 = 0;
              x1 = atoi(inputBuffer);
              x=x1*3.14159265/180;
              float sen;
              sen = x - pow(x,3)/6 + pow(x,5)/120 - pow(x,7)/5040 + pow(x,9)/362880 - pow(x,11)/39916800 + pow(x,13)/6227020800 - (x,15)/1307674368000;
              
              Serial.print("Resultado: ");
              Serial.println(sen);
      } else {

        Serial.print("Comando no reconocido: ");
        Serial.println(inputBuffer);
        Serial.println("Intente de nuevo...");
        
      }

      memset(inputBuffer, 0, bufferSize); // Limpiar el buffer
    }
    
    // Retraso para evitar consumir demasiado CPU
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

/****** Tarea 5 ******/ 
#define UPPER_LIMIT 250
#define LOWER_LIMIT 0
void task5(void *parameters){
 
  bool incremento = true;
  int pwm = 0;

  while(1)
  {
    
    if(incremento)
      pwm += 25;
    else
      pwm -= 25;
    
    if((pwm == UPPER_LIMIT)&&(incremento)) incremento = false;
    if((pwm == LOWER_LIMIT)&&(!incremento)) incremento = true;
    
    analogWrite(led_pin_task5, pwm);

    vTaskDelay(period_task_5 / portTICK_PERIOD_MS);
  }
}

// interrupcion del boton de la tarea 6
void buttonT6ISR(){
  flag_button_t6 = !flag_button_t6;       
}
// funcion callback timer tarea 6
void onTimerTask6(TimerHandle_t xTimer){
  face_times[cara_actual]++;
}

/*****  Tarea 6  *****/
void task6(void *parameters){
  bool flag_print_times = false;

  while(1){
    /*
    accelArray = read_IMU_XLR8();
    // Mostrar los resultados en el monitor serial
    Serial.print("Acelerómetro (g): X=");
    Serial.print(accelArray[0]); // Escala del acelerómetro (+/-2g)
    Serial.print(" Y=");
    Serial.print(accelArray[1]);
    Serial.print(" Z=");
    Serial.println(accelArray[2]);
    */
    if(flag_button_t6){
      // iniciar timer
      if(!start_timer_T6){
        xTimerStart(timerTask6,0);
        start_timer_T6 = true;
        flag_print_times = false;
      }
      if (xSemaphoreTake(i2cMutex, 0) == pdTRUE){
        // leer aceleracion
        accelArray = read_IMU_XLR8();
        xSemaphoreGive(i2cMutex);
      }
      // verificar en cual cara se encuentra el cubo
      check_face();
     // Serial.print("Cara actual: ");
     // Serial.println(caras[cara_actual]);

    }else{
      // detener timer
      if(start_timer_T6){
        xTimerStop(timerTask6,0);
        start_timer_T6 = false;
      }
      // imprimir tiempos
      if(!flag_print_times){
        print_times();
        for(uint8_t i = 0; i < 6; i++){
          face_times[i] = 0;
        }
        flag_print_times = true;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}



void setup() {
  Serial.begin(115200);
  Serial.println("Configuracion.");
  
  i2cMutex = xSemaphoreCreateMutex();
  UARTMutex = xSemaphoreCreateMutex();
  /*****  Tarea 1  *****/
  // configurando pines s
  pinMode(button_pin_task1, INPUT_PULLUP);
  pinMode(led_pin, OUTPUT);
  // configuracion del timer
  setup_timer_task1();
  // interrupcion para el boton de la tarea 1
  attachInterrupt(digitalPinToInterrupt(button_pin_task1), buttonISR, CHANGE);
  // crear tarea 1
  xTaskCreate(task1, "Tarea 1", 1024, NULL, 1, NULL);
  
  /*****  Tarea 2  *****/
  pinMode(LA, INPUT);

  // Inicializar el LCD
  Wire.begin(); // SDA = GPIO4 y SCL = GPIO5
  lcd.init();             
  lcd.backlight();
  
  lcd.clear(); 
  lcd.print("Iniciando...");

  // Crear la tarea
  xTaskCreate(Tarea_ADC,
              "ADC",
              2048,
              (void*) "ADC",
              1,
              NULL
             );



  /*****  Tarea 3  *****/
  // Configurar el pin del LED como salida
  pinMode(led_pin_task3, OUTPUT);
  // Configurar el pin del botón como entrada con resistencia pull-up
  pinMode(button_pin_task3, INPUT_PULLUP);

  // Crear la tarea en el núcleo correspondiente
  xTaskCreate(
    TareaParpadeo,     // Función de la tarea
    "Tarea Parpadeo",  // Nombre de la tarea
    2048,              // Tamaño de la pila de la tarea (ejemplo)
    NULL,              // Parámetros de la tarea (ninguno en este caso)
    1,                 // Prioridad de la tarea
    NULL              // Manejador de la tarea (opcional)
    );

  /*****  Tarea 4  *****/
  
  xTaskCreate(
    tareaEstados,     // Función de la tarea
    "Tarea Estados",  // Nombre de la tarea
    2048,              // Tamaño de la pila de la tarea (ejemplo)
    NULL,              // Parámetros de la tarea (ninguno en este caso)
    1,                 // Prioridad de la tarea
    NULL              // Manejador de la tarea (opcional)
    );

  /*****  Tarea 5  *****/

  pinMode(led_pin_task5, OUTPUT);
  
  // Task to run forever
  xTaskCreate(task5,    
              "Tarea brillo variante", 
              2048,        
              NULL,         
              1,            
              NULL);

  /*****  Tarea 6  *****/

  setup_timer_task6();

  init_MPU();
  Serial.println("MPU6050 inicializado.");

  pinMode(BUTTTON_TASK6_PIN, INPUT_PULLUP);

  // interrupcion para el boton de la tarea 6
  attachInterrupt(digitalPinToInterrupt(BUTTTON_TASK6_PIN), buttonT6ISR, FALLING);
  // crear tarea 1
  xTaskCreate(task6, "Tarea 6", 1024, NULL, 1, NULL);
}

void loop() {
}


void print_times(){

  Serial.println("*** Tiempos ***");
  Serial.print("Cara UP: ");
  Serial.print(face_times[0]*PERIOD_TIMER/1000.0);
  Serial.print("  Cara FRONT: ");
  Serial.print(face_times[1]*PERIOD_TIMER/1000.0);
  Serial.print("  Cara DOWN: ");
  Serial.print(face_times[2]*PERIOD_TIMER/1000.0);
  Serial.print("  Cara BACK: ");
  Serial.print(face_times[3]*PERIOD_TIMER/1000.0);
  Serial.print("  Cara RIGHT: ");
  Serial.print(face_times[4]*PERIOD_TIMER/1000.0);
  Serial.print("  Cara LEFT: ");
  Serial.println(face_times[5]*PERIOD_TIMER/1000.0);

}
void check_face(){
   if(accelArray[0] >  0.8)
    cara_actual = UP;
  else if(accelArray[0] < -0.8) 
    cara_actual = DOWN;
  else if(accelArray[1] >  0.8) 
    cara_actual = LEFT;
  else if(accelArray[1] < -0.8)
    cara_actual = RIGHT;
  else if(accelArray[2] >  0.8)
    cara_actual = FRONT;
  else if(accelArray[2] < -0.8)
    cara_actual = BACK;
}
// Función para leer dos bytes y convertirlos a un valor de 16 bits
int16_t read16bit(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2);
  int16_t value = (Wire.read() << 8) | Wire.read();
  return value;
}
std::array<float, 3> read_IMU_XLR8() {
    // Leer datos del acelerómetro
    float accelX = read16bit(ACCEL_XOUT_H) / 16384.0;
    float accelY = read16bit(ACCEL_XOUT_H + 2) / 16384.0;
    float accelZ = read16bit(ACCEL_XOUT_H + 4) / 16384.0;

    return {accelX, accelY, accelZ};
}
void init_MPU(){
  
  Wire.begin();

  // Inicializar el MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); // Despertar el MPU6050
  Wire.endTransmission();
  
}
void setup_timer_task6(){
  // create and start timer (num, divider, countUp)
  static const TickType_t dim_delay = pdMS_TO_TICKS(PERIOD_TIMER);
  timerTask6 =  xTimerCreate(
                  "Timer tarea 6",
                  dim_delay,
                  pdTRUE,
                  (void*)0,
                  onTimerTask6);

  xTimerStop(timerTask6,0);
}
