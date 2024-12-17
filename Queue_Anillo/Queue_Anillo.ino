#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

//delay para las tareas
#define DELAY 1000
// longitud de las colas
static const uint8_t msg_queue_len = 5;

static QueueHandle_t queue1;
static QueueHandle_t queue2;
static SemaphoreHandle_t mutex; 

void taskA(void *parameters)
{
  TaskHandle_t xTaskHandle = xTaskGetCurrentTaskHandle();
  const char *taskName = pcTaskGetName(xTaskHandle);

  int item = 0;
  int itemReceived;

  while (1)
  {
    // Enviar a queue1
    if (xQueueSend(queue1, (void *)&item, portMAX_DELAY))
    {
      Serial.print(taskName);
      Serial.print(" envió a queue1: ");
      Serial.println(item);
      item++;
    } else {
      Serial.println("Error enviando a queue1 desde taskA");
    }

    // Esperar hasta que tarea 2 procese el dato
    if (xSemaphoreTake(mutex, portMAX_DELAY))
    {
      // Recibir de queue2
      if (xQueueReceive(queue2, (void *)&itemReceived, portMAX_DELAY))
      {
        Serial.print(taskName);
        Serial.print(" recibió de queue2: ");
        Serial.println(itemReceived);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(DELAY));
  }
}

void taskB(void *parameters)
{
  TaskHandle_t xTaskHandle = xTaskGetCurrentTaskHandle();
  const char *taskName = pcTaskGetName(xTaskHandle);

  int item;

  while (1)
  {
    // Recibir de queue1
    if (xQueueReceive(queue1, (void *)&item, portMAX_DELAY))
    {
      Serial.print(taskName);
      Serial.print(" recibió de queue1: ");
      Serial.println(item);
    }

    // Procesar dato y enviar a queue2
    if (xQueueSend(queue2, (void *)&item, portMAX_DELAY))
    {
      Serial.print(taskName);
      Serial.print(" envió a queue2: ");
      Serial.println(item);

      // Liberar el semáforo para que tarea 1 continúe
      xSemaphoreGive(mutex);
    } else {
      Serial.println("Error enviando a queue2 desde taskB");
    }

    vTaskDelay(pdMS_TO_TICKS(DELAY)); 
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Inicio del programa");
  delay(500);

  // Crear las colas
  queue1 = xQueueCreate(msg_queue_len, sizeof(int));
  queue2 = xQueueCreate(msg_queue_len, sizeof(int));

  // Crear el semáforo binario
  mutex = xSemaphoreCreateBinary();

  if (mutex == NULL)
  {
    Serial.println("Error al crear el semáforo");
  }

  xSemaphoreTake(syncSemaphore, 0);

  // Crear las tareas
  xTaskCreatePinnedToCore(taskA, "Tarea A", 1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(taskB, "Tarea B", 1024, NULL, 1, NULL, app_cpu);
}

void loop()
{
  // Nada que hacer aquí, las tareas manejan todo
}
