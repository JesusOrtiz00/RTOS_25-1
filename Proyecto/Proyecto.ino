#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "timers.h"
#include <array>
#include "semphr.h"
#include <queue.h>
#include <Servo.h>

// Pines y configuraciones
#define PIN_VELOCIDAD_BANDA_PRINCIPAL 1
#define PIN_BANDA_PRINCIPAL 2
#define PIN_BANDA_1 3
#define PIN_BANDA_2 4
#define PIN_BANDA_3 5

#define PIN_SENSOR_PEQUENAS 6
#define PIN_SENSOR_MEDIANAS 7
#define PIN_SENSOR_GRANDES 8

#define PIN_SENSOR_BANDA_1 9
#define PIN_SENSOR_BANDA_2 10
#define PIN_SENSOR_BANDA_3 11

#define PIN_BOTON_ARRANQUE 12
#define PIN_BOTON_EMERGENCIA 13

#define PIN_SERVO_GRANDES 14
#define PIN_SERVO_MEDIANO 15

#define OBJETO_DETECTADO LOW

// Variables globales
volatile int velocidadBandaPrincipal = 128; // Velocidad inicial (0-255)
volatile bool paroEmergencia = true;
volatile int contadorPequenas = 0, contadorMedianas = 0, contadorGrandes = 0;
static QueueHandle_t queue;
TimerHandle_t timerBanda1, timerBanda2, timerBanda3;
Servo servoGrandes, servoMedianas;
TaskHandle_t handleControlBandaPrincipal, handleClasificacionCajas, handleControlBandasSecundarias, handleUART;


// Declaración de funciones
void tareaControlBandaPrincipal(void *pvParameters);
void tareaClasificacionCajas(void *pvParameters);
void tareaControlBandasSecundarias(void *pvParameters);
void tareaParoEmergencia(void *pvParameters);
void tareaActualizacionVelocidad(void *pvParameters);
void tareaContadorPaquetes(void *pvParameters);

// Prototipos de funciones de callback para temporizadores
void callbackTimerBanda1(TimerHandle_t xTimer);
void callbackTimerBanda2(TimerHandle_t xTimer);
void callbackTimerBanda3(TimerHandle_t xTimer);


void setup() {
  Serial.begin(115200);

  // Configuración de pines
  pinMode(PIN_VELOCIDAD_BANDA_PRINCIPAL, OUTPUT);
  pinMode(PIN_BANDA_PRINCIPAL, OUTPUT);
  pinMode(PIN_BANDA_1, OUTPUT);
  pinMode(PIN_BANDA_2, OUTPUT);
  pinMode(PIN_BANDA_3, OUTPUT);

  pinMode(PIN_SENSOR_PEQUENAS, INPUT_PULLUP);
  pinMode(PIN_SENSOR_MEDIANAS, INPUT_PULLUP);
  pinMode(PIN_SENSOR_GRANDES, INPUT_PULLUP);
  pinMode(PIN_SENSOR_BANDA_1, INPUT_PULLUP);
  pinMode(PIN_SENSOR_BANDA_2, INPUT_PULLUP);
  pinMode(PIN_SENSOR_BANDA_3, INPUT_PULLUP);

  pinMode(PIN_BOTON_ARRANQUE, INPUT_PULLUP);
  pinMode(PIN_BOTON_EMERGENCIA, INPUT_PULLUP);

  // Configuración de servos
  servoGrandes.attach(PIN_SERVO_GRANDES);
  servoMedianas.attach(PIN_SERVO_MEDIANO);

  // Crear temporizadores para las bandas
  timerBanda1 = xTimerCreate("TimerBanda1", pdMS_TO_TICKS(10000), pdFALSE, NULL, callbackTimerBanda1);
  timerBanda2 = xTimerCreate("TimerBanda2", pdMS_TO_TICKS(10000), pdFALSE, NULL, callbackTimerBanda2);
  timerBanda3 = xTimerCreate("TimerBanda3", pdMS_TO_TICKS(10000), pdFALSE, NULL, callbackTimerBanda3);

  if (timerBanda1 == NULL || timerBanda2 == NULL || timerBanda3 == NULL) {
    Serial.println("Error al crear temporizadores.");
    while (1);
  }

  // Crear tareas y guardar los manejadores
  xTaskCreate(tareaControlBandaPrincipal, "BandaPrincipal", 1024, NULL, 1, &handleControlBandaPrincipal);
  xTaskCreate(tareaClasificacionCajas, "Clasificacion", 1024, NULL, 1, &handleClasificacionCajas);
  xTaskCreate(tareaControlBandasSecundarias, "BandasSecundarias", 1024, NULL, 1, &handleControlBandasSecundarias);
  xTaskCreate(tareaParoEmergencia, "ParoEmergencia", 1024, NULL, 2, NULL);
  xTaskCreate(tareaUART, "UART", 1024, NULL, 1, &handleUART);

  Serial.println("Sistema inicializado.");

  // Detener todas las tareas
  vTaskSuspend(handleControlBandaPrincipal);
  vTaskSuspend(handleClasificacionCajas);
  vTaskSuspend(handleControlBandasSecundarias);
  vTaskSuspend(handleUART);
}

void loop() {
  // El bucle principal queda vacío porque FreeRTOS maneja las tareas
}

// Tarea para controlar la banda principal
void tareaControlBandaPrincipal(void *pvParameters) {
  while (1) {
    if (!paroEmergencia) {
      digitalWrite(PIN_BANDA_PRINCIPAL, HIGH);
      analogWrite(PIN_VELOCIDAD_BANDA_PRINCIPAL, velocidadBandaPrincipal);
    } else {
      
      digitalWrite(PIN_BANDA_PRINCIPAL, LOW);
      analogWrite(PIN_VELOCIDAD_BANDA_PRINCIPAL, 0);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // 10 ms
  }
}

// Tarea para clasificar cajas
void tareaClasificacionCajas(void *pvParameters) {
  while (1) {

    if (digitalRead(PIN_SENSOR_GRANDES) == OBJETO_DETECTADO) {
      contadorGrandes++;
      servoGrandes.write(90);
      vTaskDelay(pdMS_TO_TICKS(200));
      servoGrandes.write(0);
      //xQueueSend(queue, (void *)"Banda3", portMAX_DELAY);
    } else if (digitalRead(PIN_SENSOR_MEDIANAS) == OBJETO_DETECTADO) {
      contadorMedianas++;
      servoMedianas.write(90);
      vTaskDelay(pdMS_TO_TICKS(200));
      servoMedianas.write(0);
      //xQueueSend(queue, (void *)"Banda2", portMAX_DELAY);
    } else if (digitalRead(PIN_SENSOR_PEQUENAS) == OBJETO_DETECTADO) {
      contadorPequenas++;
      //xQueueSend(queue, (void *)"Banda1", portMAX_DELAY);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // 100 ms
  }
}

// Tarea para controlar las bandas secundarias
void tareaControlBandasSecundarias(void *pvParameters) {
  //char *banda;
  while (1) {
    
    if(!digitalRead(PIN_SENSOR_BANDA_1)){
      digitalWrite(PIN_BANDA_1, HIGH);
      xTimerStart(timerBanda1, 0);
    } else if(!digitalRead(PIN_SENSOR_BANDA_2)){
      digitalWrite(PIN_BANDA_2, HIGH);
      xTimerStart(timerBanda2, 0);
    } else if(!digitalRead(PIN_SENSOR_BANDA_3)){
      digitalWrite(PIN_BANDA_3, HIGH);
      xTimerStart(timerBanda3, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
  }
}

// Tarea para manejar el paro de emergencia
void tareaParoEmergencia(void *pvParameters) {
  while (1) {
    
    if (digitalRead(PIN_BOTON_EMERGENCIA) == LOW) {
      paroEmergencia = true;

       // Detener todas las tareas
      vTaskSuspend(handleControlBandaPrincipal);
      vTaskSuspend(handleClasificacionCajas);
      vTaskSuspend(handleControlBandasSecundarias);
      vTaskSuspend(handleUART);

      // Apagar motores
      digitalWrite(PIN_BANDA_PRINCIPAL, LOW);
      digitalWrite(PIN_BANDA_1, LOW);
      digitalWrite(PIN_BANDA_2, LOW);
      digitalWrite(PIN_BANDA_3, LOW);
      analogWrite(PIN_VELOCIDAD_BANDA_PRINCIPAL, 0);

      
      Serial.println("Paro de emergencia activado.");
    } else if (digitalRead(PIN_BOTON_ARRANQUE) == LOW && paroEmergencia) {
      paroEmergencia = false;

      // Reanudar todas las tareas
      vTaskResume(handleControlBandaPrincipal);
      vTaskResume(handleClasificacionCajas);
      vTaskResume(handleControlBandasSecundarias);
      vTaskResume(handleUART);

      Serial.println("Paro de emergencia desactivado. Sistema reanudado.");
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// Tarea para peticiones del usuario
void tareaUART(void *pvParameters) {
  const size_t bufferSize = 32; // Tamaño máximo para el comando
  char inputBuffer[bufferSize] = {0}; // Buffer para la entrada serial
  char inputNum[bufferSize] = {0};
  const char* comandoActualizarVelocidad = "VEL";
  const char* comandoConteo = "CONTEO";  
 
  while (1) {
    ;
    if (Serial.available()) {

      size_t len = Serial.readBytesUntil('\n', inputBuffer, bufferSize - 1);
      inputBuffer[len] = '\0';
      for (size_t i = 0; i < len; i++) {
        if (inputBuffer[i] == '\r') {
          inputBuffer[i] = '\0';
        }
      }
      
      if(strcmp(inputBuffer, comandoActualizarVelocidad) == 0){
        Serial.print("Introduzca un valor entre 0 y 255: ");

        bool banderaEsperandoVelocidad = true;

        do{
          if (Serial.available() > 0) {
            int index = 0;
            while (Serial.available() && index < sizeof(inputBuffer) - 1) {
              inputBuffer[index++] = Serial.read();
              vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            inputBuffer[index] = '\0'; // Terminar el comando
            banderaEsperandoVelocidad = false;
          }
          vTaskDelay(pdMS_TO_TICKS(50));
        }while(banderaEsperandoVelocidad);

        int nuevaVelocidad = atoi(inputBuffer);

        Serial.println("Velocidad actualizada.");

        if (nuevaVelocidad >= 0 && nuevaVelocidad <= 255)
          velocidadBandaPrincipal = nuevaVelocidad;
        else
          Serial.println("Valor no válido.");
      } else if(strcmp(inputBuffer, comandoConteo) == 0) {

        Serial.print("Pequeñas: ");
        Serial.println(contadorPequenas);
        Serial.print("Medianas: ");
        Serial.println(contadorMedianas);
        Serial.print("Grandes: ");
        Serial.println(contadorGrandes);
        
      } else {
        
        Serial.println("Comando no reconocido.");
      }
        
    }

    memset(inputBuffer, 0, bufferSize); // Limpiar el buffer

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void callbackTimerBanda1(TimerHandle_t xTimer){
  // apagar banda 1
  digitalWrite(PIN_BANDA_1, LOW);
}
void callbackTimerBanda2(TimerHandle_t xTimer){
  // apagar banda 2
  digitalWrite(PIN_BANDA_2, LOW);
}
void callbackTimerBanda3(TimerHandle_t xTimer){
  // apagar banda 3
  digitalWrite(PIN_BANDA_3, LOW);
}