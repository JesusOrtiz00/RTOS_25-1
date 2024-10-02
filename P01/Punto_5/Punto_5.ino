#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif



//Gestión de tareas
static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;
static TaskHandle_t task_3 = NULL;

//Pototipos de las tareas
void task1(void *pvParameters);
void task2(void *pvParameters);
void task3(void *pvParameters);


void setup() {
  Serial.begin(9600);  // Velocidad de transmisión libre
  Serial.println();
  Serial.println("Configuracion");

  // Se declaran las tareas
  xTaskCreatePinnedToCore(
                        task1,   // Función que implementa la tarea
                        "Task1", // Nombre de la tarea (para depuración)
                        1024,        // Tamaño de la pila (en palabras)
                        NULL,        // Parámetro de entrada
                        1,           // Prioridad de la tarea
                        &task_1,        // Handle de la tarea (no se necesita en este caso)
                        app_cpu      // Núcleo en el que se ejecutará la tarea (1 para CORE_1)
    );

  xTaskCreatePinnedToCore(
                        task2,   // Función que implementa la tarea
                        "Task2", // Nombre de la tarea (para depuración)
                        1024,        // Tamaño de la pila (en palabras)
                        NULL,        // Parámetro de entrada
                        1,           // Prioridad de la tarea
                        &task_2,        // Handle de la tarea (no se necesita en este caso)
                        app_cpu      // Núcleo en el que se ejecutará la tarea (1 para CORE_1)
    );
  
  xTaskCreatePinnedToCore(
                        task3,   // Función que implementa la tarea
                        "Task3", // Nombre de la tarea (para depuración)
                        1024,        // Tamaño de la pila (en palabras)
                        NULL,        // Parámetro de entrada
                        32,           // Prioridad de la tarea
                        &task_3,        // Handle de la tarea (no se necesita en este caso)
                        app_cpu      // Núcleo en el que se ejecutará la tarea (1 para CORE_1)
    );
}

void loop() {
 // si la tarea 1 ya se ejecutó se elimina
  if(task_1 != NULL){
    vTaskDelete(task_1);
    task_1 = NULL;
  } 
  // se supende la tarea 2 por 2 segundos, se reactivaa y se espera 2 segundos
  vTaskSuspend(task_2);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  vTaskResume(task_2);
  vTaskDelay(2000 / portTICK_PERIOD_MS);

}

void task1(void *pvParameters) {
  while(1){
    Serial.println("Tarea 1 ejecutada y eliminada");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
 }
}

void task2(void *pvParameters) {
  while (1) {
    Serial.println("Tarea 2 ejecutada");
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
  }
}

void task3(void *pvParameters) {
  while (1) {
    Serial.println("Tarea 3 ejecutándose indefinidamente");
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Ejecuta cada segundo
  }
}
