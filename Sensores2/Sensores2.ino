#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#if CONFIG_FRERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else 
  static const BaseType_t app_cpu = 1;
#endif

//Declaración de variables para la pantalla LCD
#define SDA 15
#define SCL 14
#define COLUMNS 16
#define ROWS    2

LiquidCrystal_I2C lcd(0x27, COLUMNS, ROWS);

//Declaración de pines para los sensores de humedad y presencia 
static const int hum = 13;
static const int dis = 2; 

static int shared_var = 0;
static SemaphoreHandle_t mutex;

//Tarea
void Tarea_sen(void* parameters){
  int local_var;
  const char* nombre_tarea = (const char*)parameters;

  while(1){
    int valor_humedad = analogRead(hum);  // Leer el sensor de humedad
    int valor_infrarojo = digitalRead(dis);  // Leer el sensor de infrarrojo
    
    if(xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE){
      local_var = shared_var;
      local_var++;
      shared_var = local_var;

      lcd.clear();  // Limpiar pantalla LCD antes de escribir nueva información

      if (strcmp(nombre_tarea, "Humedad") == 0) {
        lcd.setCursor(0, 0); // Primer renglón
        lcd.print("Humedad: ");
        lcd.print(valor_humedad);
        lcd.print(" %");
        Serial.print("Humedad: ");
        Serial.print(valor_humedad);
        Serial.println(" %");
      } else if (strcmp(nombre_tarea, "Presencia") == 0) {
        lcd.setCursor(0, 1); // Segundo renglón
        lcd.print("Presencia: ");
        lcd.print(valor_infrarojo);
        Serial.print("Presencia: ");
        Serial.println(valor_infrarojo);
      }

      xSemaphoreGive(mutex);  // Liberar el mutex
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Esperar 1 segundo entre cada lectura
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(hum, INPUT);
  pinMode(dis, INPUT);

  // Inicializar el LCD
  Wire.begin(SDA, SCL);
  lcd.init();             
  lcd.backlight();
  
  lcd.clear(); 
  lcd.print("Iniciando...");

  // Crear mutex
  mutex = xSemaphoreCreateMutex();

  // Crear las tareas
  xTaskCreatePinnedToCore(Tarea_sen,
                          "Humedad",
                          2048,
                          (void*) "Humedad",
                          1,
                          NULL,
                          app_cpu
  );
  xTaskCreatePinnedToCore(Tarea_sen,
                          "Presencia",
                          2048,
                          (void*) "Presencia",
                          1,
                          NULL,
                          app_cpu
  );
}

void loop() {
  // El loop se mantiene vacío, las tareas están manejando todo
}