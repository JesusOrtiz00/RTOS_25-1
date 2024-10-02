#if CONFIG_FREERTOS_UNICORE
 static const BaseType_t app_cpu = 0;
 #else
 static const BaseType_t app_cpu = 1;
 #endif

// Incluir la biblioteca de FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

 // Pines de los LEDs
static const int led_pin1 = 16;
static const int led_pin2 = 4;
static const int led_pin3 = 2;
static const int led_pin4 = 14;
static const int led_pin5 = 15;
static const int led_pin6 = 13;
static const int led_pin7 = 12;


 //Mensaje que enviara por consola
 const char msg1[] = "Unidad Profesional Interdisciplinaria de Ingenieria Campus Zacatecas IPN";
 const char msg2[] = "1";
 const char msg3[] = "2";
 const char msg4[] = "3";
 const char msg5[] = "4";
 const char msg6[] = "5";
 const char msg7[] = "6";

 //Gestión de tareas
 static TaskHandle_t tarea_1 = NULL;
 static TaskHandle_t tarea_2 = NULL;
 static TaskHandle_t tarea_3 = NULL;
 static TaskHandle_t tarea_4 = NULL;
 static TaskHandle_t tarea_5 = NULL;
 static TaskHandle_t tarea_6 = NULL;
 static TaskHandle_t tarea_7 = NULL;

 /*----------Tareas----------------*/

 //Tarea 01: Tarea de baja prioridad, se imprime el mensaje 
 void tarea01 (void *parameter){
  int msg_leng = strlen(msg1);
  while(1){
    digitalWrite(led_pin1, HIGH);
    for(int i=0; i<msg_leng;i++){
      Serial.print(msg1[i]);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    Serial.println();
    digitalWrite(led_pin1, LOW);
    vTaskDelay(300 / portTICK_PERIOD_MS);

  }
 }

 //Tarea 02: Imprimir mensaje 
 void tarea02 (void  *parameter) {
  while(1){
    digitalWrite(led_pin2, HIGH);
    Serial.print('1');
    digitalWrite(led_pin2, LOW);
    vTaskDelay(300/ portTICK_PERIOD_MS);

  }
 }

 //Tarea 03: Imprimir mensaje 
 void tarea03 (void  *parameter) {
  while(1){
    digitalWrite(led_pin3, HIGH);
    Serial.print('2');
    digitalWrite(led_pin3, LOW);
    vTaskDelay(300/ portTICK_PERIOD_MS);

  }
 }

 //Tarea 04: Imprimir mensaje 
 void tarea04 (void  *parameter) {
  while(1){
    digitalWrite(led_pin4, HIGH);
    Serial.print('3');
    digitalWrite(led_pin4, LOW);
    vTaskDelay(300/ portTICK_PERIOD_MS);

  }
 }

 //Tarea 05: Imprimir mensaje 
 void tarea05 (void  *parameter) {
  while(1){
     digitalWrite(led_pin5, HIGH);
    Serial.print('4');
    digitalWrite(led_pin5, LOW);
    vTaskDelay(300/ portTICK_PERIOD_MS);

  }
 }

 //Tarea 06: Imprimir mensaje 
 void tarea06 (void  *parameter) {
  while(1){
    digitalWrite(led_pin6, HIGH);
    Serial.print('5');
    digitalWrite(led_pin6, LOW);
    vTaskDelay(300/ portTICK_PERIOD_MS);

  }
 }

 //Tarea 07: Imprimir mensaje
 void tarea07 (void  *parameter) {
  while(1){
    digitalWrite(led_pin7, HIGH);
    Serial.print('6');
    digitalWrite(led_pin7, LOW);
    vTaskDelay(300/ portTICK_PERIOD_MS);

  }
 }
 
void setup(){
  Serial.begin(300);
  vTaskDelay(1000/ portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("Configuracion");

 // Configurar pines de los LEDs como salidas
  pinMode(led_pin1, OUTPUT);
  pinMode(led_pin2, OUTPUT);
  pinMode(led_pin3, OUTPUT);
  pinMode(led_pin4, OUTPUT);
  pinMode(led_pin5, OUTPUT);
  pinMode(led_pin6, OUTPUT);
  pinMode(led_pin7, OUTPUT);

  //  Se declaraan las tareas
  xTaskCreatePinnedToCore(tarea01,
  "Tarea 1",
   1024,
   NULL,
   1,
   &tarea_1,
   app_cpu);

    xTaskCreatePinnedToCore(tarea02,
  "Tarea 2",
   1024,
   NULL,
   2,
   &tarea_2,
   app_cpu);

   xTaskCreatePinnedToCore(tarea03,
  "Tarea 3",
   1024,
   NULL,
   3,
   &tarea_3,
   app_cpu);

   xTaskCreatePinnedToCore(tarea04,
  "Tarea 4",
   1024,
   NULL,
   4,
   &tarea_4,
   app_cpu);

     xTaskCreatePinnedToCore(tarea05,
  "Tarea 5",
   1024,
   NULL,
   3,
   &tarea_5,
   app_cpu);

     xTaskCreatePinnedToCore(tarea06,
  "Tarea 6",
   1024,
   NULL,
   2,
   &tarea_6,
   app_cpu);

     xTaskCreatePinnedToCore(tarea07,
  "Tarea 7",
   1024,
   NULL,
   1,
   &tarea_7,
   app_cpu);

}

void loop(){
 
}
 