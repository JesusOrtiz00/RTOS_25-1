// Tarea 1: LED parapadea a una frecuencia de 500 ms

// Tarea 2: LED parpadea a una frecuencia seleccionada por el usuario en ms.

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else 
  static const BaseType_t app_cpu = 1;
#endif

// definicion de los pines para los led y el boton
#define LED1 2
#define LED2 16
#define BUTTON 15

// Variables para el mensaje y los delays
int delay1 = 500;
int delay2 = 0;
String msg = "Frecuencia recibida: ";

// Tarea del parpadeo a frecuencia constante
void blinkTask(void *pvParameter) {
    // Configuración del pin como salida

    while (1) {
        // Enciende el LED
        digitalWrite(LED1, HIGH);
        vTaskDelay(delay1 / portTICK_PERIOD_MS);

        // Apaga el LED
        digitalWrite(LED1, LOW);
        vTaskDelay(delay1 / portTICK_PERIOD_MS);
    }
}

// Tarea del parpadeo con frecuencia variable desde el puerto serial
void blinkTaskSerial(void *vParameter){
   // int currentFreq = 0; // Frecuencia actual
  
  while(1) {
    // Revisa si hay datos en el buffer serial
    if (Serial.available() > 0) {
      // Lee el valor recibido
      int receivedValue = Serial.parseInt();  
      
      // imprimir mensaje caracter por caracter
      for(int i = 0; i < msg.length(); i++){
        Serial.print(msg[i]);
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
      // Imprimir el valor recibido
      Serial.println(receivedValue);
      
      // Solo se asigna si el valor es mayor que 0
      if (receivedValue > 0) {
        delay2 = receivedValue;
      }
    }

    // Verifica si la frecuencia es válida (mayor a 0)
    if (delay2 > 0) {
      // Enciende el LED
      digitalWrite(LED2, HIGH);
      vTaskDelay(delay2 / portTICK_PERIOD_MS);

      // Apaga el LED
      digitalWrite(LED2, LOW);
      vTaskDelay(delay2 / portTICK_PERIOD_MS);
    } else {
      // Si la frecuencia es 0 o menor, apaga el LED
      digitalWrite(LED2, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS); // Un pequeño delay para evitar saturación
    }
  }
}

// Tarea para la lectura del boton que cambia la frecuencia de parpadeo de ambos LED´s
void buttonDelay(void *vParameter){
  while(1){
    if((digitalRead(BUTTON) == 0)){
      delay1 = 1000;
      delay2 = 500;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // put your setup code here, to run once:
  // Se inicializa la comunicación serial
  Serial.begin(115200);

  // Se configura los pines de los LED´s como salidas
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  // Se configura el pin de botón como entrasda con pull-up
  pinMode(BUTTON, INPUT_PULLUP);

  // Se declaran las tareas
    xTaskCreatePinnedToCore(
                        blinkTask,   // Función que implementa la tarea
                        "BlinkTask", // Nombre de la tarea (para depuración)
                        1024,        // Tamaño de la pila (en palabras)
                        NULL,        // Parámetro de entrada
                        1,           // Prioridad de la tarea
                        NULL,        // Handle de la tarea (no se necesita en este caso)
                        app_cpu      // Núcleo en el que se ejecutará la tarea (1 para CORE_1)
    );

    xTaskCreatePinnedToCore(
                        blinkTaskSerial,   // Función que implementa la tarea
                        "BlinkTaskSerial", // Nombre de la tarea (para depuración)
                        1024,        // Tamaño de la pila (en palabras)
                        NULL,        // Parámetro de entrada
                        1,           // Prioridad de la tarea
                        NULL,        // Handle de la tarea (no se necesita en este caso)
                        app_cpu      // Núcleo en el que se ejecutará la tarea (1 para CORE_1)
    );

    xTaskCreatePinnedToCore(
                        buttonDelay,
                        "ButtonDelay",
                        1024,
                        NULL,
                        1,
                        NULL,
                        app_cpu);
}

void loop() {
  // put your main code here, to run repeatedly:

}
