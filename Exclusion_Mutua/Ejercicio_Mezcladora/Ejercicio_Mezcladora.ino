#if CONFIG_FRERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

/*

  Estados:
    + Valvula A abierta 
    + Valvula B abierta 
    + Valvula 1 abierta 
    + Valvula 2 abierta 
    + Mezclado  
    + Drenado (opcional)
*/

enum EstadoSistema {VA, VB, V1, V2, MEZCLADO, DRENADO};
enum EstadoSistema estado = VA; // inicia por defecto en el primer estado

String texto_estado[] = {"VA+", "VA- VB+", "VB- V1+", "V1- V2+", "V2- M+", "M- D+"};

#define PIN_SENSOR_SP1        1
#define PIN_SENSOR_SP2        0
#define PIN_SENSOR_NIVEL      16
#define VALVULA_A             12
#define VALVULA_B             13
#define VALVULA_1             15
#define VALVULA_2             14
#define MOTOR                 2
#define VALVULA_DRENADO       3

// posicion de los sensores en el caracter
#define SP1           0
#define SP2           1
#define SN            2
#define FLAG_MEZCLADO 3
#define FLAG_DRENADO  4

// variables para guardar el estado de los sensores y actuadores
char sensores = 0;  // nibble bajo -> FLAG_DRENADO SN_L SN_H SP2 SP1

// Tarea para leer los sensores
void TareaLecturaSensores(void *parameters){
  while(1){

    if(!digitalRead(PIN_SENSOR_SP1))  sensores |= 0b0001;
    else                              sensores &= ~(0b0001);

    if(!digitalRead(PIN_SENSOR_SP2))  sensores |= 0b0010;
    else                              sensores &= ~(0b0010);

    if(!digitalRead(PIN_SENSOR_NIVEL)) sensores |= 0b0100;
    else                               sensores &= ~(0b0100);

    vTaskDelay(100  / portTICK_PERIOD_MS);
 }
}
// Tarea para el cambio de estado
void TareaManejoCambioEstado(void *parameters){

  while(1){

    // Estado VA -> VB
    if(estado == VA && (sensores & (1 << SP1))) estado = VB;
    // Estado VB -> V1
    else if(estado == VB && (sensores & (1 << SP2))) estado = V1;
    // Estado V1 -> V2
    else if(estado == V1 && (sensores & (1 << SN))) estado = V2;
    // Estado V2 -> MEZCLADO
    else if(estado == V2 && (sensores & (1 << FLAG_MEZCLADO))) estado = MEZCLADO;
    // Estado MEZCLADO -> DRENADO
    else if(estado == MEZCLADO && (sensores & (1 << FLAG_DRENADO))) estado = DRENADO;
    // estado DRENADO -> VA
    else if(estado == DRENADO && !(sensores & (1 << FLAG_DRENADO))) estado =  VA;

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

}
// tarea para activar y desactivar actuadores
void TareaManejoActuadores(void *parameters){

  static enum EstadoSistema estado_previo = VB;

  while(1){

    if(estado != estado_previo){
      estado_previo = estado;

      switch(estado){
        case VA:
          digitalWrite(VALVULA_A, HIGH);
          digitalWrite(VALVULA_B, LOW);
          digitalWrite(VALVULA_1, LOW);
          digitalWrite(VALVULA_2, LOW);
          digitalWrite(MOTOR, LOW);
          digitalWrite(VALVULA_DRENADO, LOW);
          break;
        case VB:
          digitalWrite(VALVULA_A, LOW);
          digitalWrite(VALVULA_B, HIGH);

          break;
        case V1:
          digitalWrite(VALVULA_B, LOW);
          digitalWrite(VALVULA_1, HIGH);

          break;
        case V2:
          digitalWrite(VALVULA_1, LOW);
          digitalWrite(VALVULA_2, HIGH);
          vTaskDelay(1000  / portTICK_PERIOD_MS);
          sensores |= (1<<FLAG_MEZCLADO);
          break;
        case MEZCLADO:
          
          digitalWrite(VALVULA_2, LOW);
          digitalWrite(MOTOR, HIGH);
          vTaskDelay(3000  / portTICK_PERIOD_MS);
          sensores &= ~(1<<FLAG_MEZCLADO)
          sensores |= (1<<FLAG_DRENADO);
          break;
        case DRENADO:
          digitalWrite(MOTOR, LOW);
          digitalWrite(VALVULA_DRENADO, HIGH);
          vTaskDelay(1000  / portTICK_PERIOD_MS);
          sensores &= ~(1<<FLAG_DRENADO);
          break;
      }
      Serial.println("Estado actual: " + texto_estado[estado]);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuracion");
  // Declarar entradas
  pinMode(PIN_SENSOR_SP1, INPUT_PULLUP);
  pinMode(PIN_SENSOR_SP2, INPUT_PULLUP);
  pinMode(PIN_SENSOR_NIVEL, INPUT_PULLUP);
  // Declarar salidas
  pinMode(VALVULA_A, OUTPUT);
  pinMode(VALVULA_B, OUTPUT);
  pinMode(VALVULA_1, OUTPUT);
  pinMode(VALVULA_2, OUTPUT);
  pinMode(MOTOR, OUTPUT);
  pinMode(VALVULA_DRENADO, OUTPUT);

  // Tarea para los sensores
  xTaskCreatePinnedToCore(
                        TareaLecturaSensores,   
                        "Tarea para leer los sensores", 
                        1024,      
                        NULL,       
                        1,          
                        NULL,       
                        app_cpu     
    );

  // Tarea para el cambio de estado
  xTaskCreatePinnedToCore(
                        TareaManejoCambioEstado,   
                        "Tarea para cambiar el estado",
                        1024,       
                        NULL,       
                        1,          
                        NULL,       
                        app_cpu      
    );

  // Tarea para los actuadores
  xTaskCreatePinnedToCore(
                        TareaManejoActuadores,   
                        "Tarea para manejar los actuadores", 
                        1024,       
                        NULL,       
                        1,          
                        NULL,       
                        app_cpu     
    );
}

void loop() {
  // put your main code here, to run repeatedly:

}
