#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// --- Melodía ---
#define NOTE_C4 262
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_F4 349
#define NOTE_E4 330
#define NOTE_D4 294
#define REST 0

int melody[] = {
  NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
  NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4,
  REST
};
int noteDurations[] = {
  4, 4, 4, 4, 4, 4, 2,
  4, 4, 4, 4, 4, 4, 2,
  2
};
int melodyLength = sizeof(melody) / sizeof(int);

// --- Configuración WiFi y MQTT ---
#define WIFI_SSID         "Micaela"
#define WIFI_PASSWORD     "Mica1611aaa"
#define MQTT_SERVER       "broker.hivemq.com"
#define MQTT_PORT         1883
#define MQTT_CLIENT_ID    "ESP32_CUNA"
#define MQTT_TOPIC_SONIDO "sensor/sonido"
#define MQTT_TOPIC_MOV    "sensor/movimiento"
#define MQTT_TOPIC_CONTROL "control/estado"

// --- Pines ---
#define BUZZER_PIN   18
#define LED_PIN      13
#define SOUND_PIN    32  
#define SERVO_PIN    23
#define PRESSURE_PIN 34  
#define BUTTON_PIN   4

// --- Umbrales y Constantes ---
#define UMBRAL_SONIDO        600
#define UMBRAL_MOV           500
#define SERVO_POS_OFF        0
#define SERVO_POS_ON         90
#define SERVO_MOVE_PERIOD_MS 500 

// --- Configuración FreeRTOS ---
#define QUEUE_SIZE           20
#define TASK_DELAY_SENS_MS   500
#define TASK_DELAY_BTN_MS    50
#define DEBOUNCE_DELAY_MS    200

// --- Variables Globales y Flags ---
int soundLevel = 0;
int movementLevel = 0;

// Flags de estado para desacoplar tareas (1 = ON, 0 = OFF)
int flagServo = 0;
int flagBuzzer = 0;
int flagLed = 0; // 1 = Sistema ON (no STANDBY), 0 = STANDBY

enum Estado {
    STANDBY,
    ACTIVO,
    LLANTO,
    MOVIMIENTO,
    MOVYLLA
};

enum Evento {
    EVENTO_SILENCIO,
    EVENTO_LLANTO,
    EVENTO_QUIETO,
    EVENTO_MOV,
    EVENTO_BOTON_ON,
    EVENTO_BOTON_OFF
};

Estado estado_actual = STANDBY;
QueueHandle_t colaEventos;
SemaphoreHandle_t mutexEstado; // Protege estado_actual Y todos los flags/niveles
Servo servoCuna;

WiFiClient espClient;
PubSubClient client(espClient);

// --- Funciones de Actuador (Hardware) ---
void encenderLED() {
    digitalWrite(LED_PIN, HIGH);
}

void apagarSistema() {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
    servoCuna.write(SERVO_POS_OFF); 
}

// --- Tareas de Actuadores (Basadas en Flags) ---

void TaskMelody(void *pvParameters) {
    while (1) {
        int localFlagBuzzer = 0; // Copia local del flag

        // 1. Verificar el flag (protegido por mutex)
        if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(50)) == pdTRUE) {
            localFlagBuzzer = flagBuzzer; // Lee el flag global
            xSemaphoreGive(mutexEstado);
        }

        // 2. Actuar según el flag
        if (localFlagBuzzer == 1) { // 1 = ON
            // Toca la melodía completa
            for (int i = 0; i < melodyLength; i++) {
                
                // Chequeo de aborto (cada 4 notas)
                if (i % 4 == 0) { 
                    if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(10)) == pdTRUE) {
                        // Si el flag global cambió a OFF mientras tocaba, parar.
                        if (flagBuzzer == 0) { 
                            xSemaphoreGive(mutexEstado);
                            break; // Salir del 'for' loop
                        }
                        xSemaphoreGive(mutexEstado);
                    }
                }

                // Tocar la nota
                int noteDuration = 1000 / noteDurations[i];
                tone(BUZZER_PIN, melody[i], noteDuration);
                int pauseBetweenNotes = noteDuration * 1.30;
                vTaskDelay(pdMS_TO_TICKS(pauseBetweenNotes));
                noTone(BUZZER_PIN);
            }
            vTaskDelay(pdMS_TO_TICKS(500)); // Pausa antes de repetir
        } else {
            // Si no debe sonar (flag == 0), apagar y dormir.
            noTone(BUZZER_PIN);
            vTaskDelay(pdMS_TO_TICKS(200)); // Chequear el flag cada 200ms
        }
    }
}

void TaskServo(void *pvParameters) {
    while(1) {
        int localFlagServo = 0; // Copia local del flag

        // 1. Verificar el flag (protegido por mutex)
        if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(50)) == pdTRUE) {
            localFlagServo = flagServo; // Lee el flag global
            xSemaphoreGive(mutexEstado);
        }

        // 2. Actuar según el flag
        if (localFlagServo == 1) { // 1 = ON
            // Realiza un ciclo de mecedora
            servoCuna.write(SERVO_POS_ON);
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_PERIOD_MS));
            
            // Chequeo de aborto (si el flag cambia a 0, parar)
            if (xSemaphoreTake(mutexEstado, 0) == pdTRUE) { 
               if (flagServo == 0) { // Si el flag global cambió a OFF
                   xSemaphoreGive(mutexEstado);
                   servoCuna.write(SERVO_POS_OFF);
                   continue; // Salta al inicio del while(1)
               }
               xSemaphoreGive(mutexEstado);
            }

            servoCuna.write(SERVO_POS_OFF);
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_PERIOD_MS));
        } else {
            // Si no debe moverse (flag == 0), asegurar OFF y dormir.
            servoCuna.write(SERVO_POS_OFF);
            vTaskDelay(pdMS_TO_TICKS(200)); // Chequear el flag cada 200ms
        }
    }
}

// --- Tareas de Sensores y Botón (Generadoras de Eventos) ---

void TaskSonido(void *pvParameters) {
    while (1) {
        int localSoundLevel = analogRead(SOUND_PIN);
        Serial.print("Sonido: ");
        Serial.println(localSoundLevel);
        
        Evento e = (localSoundLevel > UMBRAL_SONIDO) ? EVENTO_LLANTO : EVENTO_SILENCIO;
        xQueueSend(colaEventos, &e, 0); // Envía evento a la FSM
        
        // Actualiza el nivel global (protegido) para TaskPublish
        if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(50)) == pdTRUE) {
            soundLevel = localSoundLevel;
            xSemaphoreGive(mutexEstado);
        }
        
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_SENS_MS));
    }
}

void TaskMovimiento(void *pvParameters) {
    while (1) {
        int localMovementLevel = analogRead(PRESSURE_PIN);
        Serial.print("Presión: ");
        Serial.println(localMovementLevel);
        
        Evento e = (localMovementLevel > UMBRAL_MOV) ? EVENTO_MOV : EVENTO_QUIETO;
        xQueueSend(colaEventos, &e, 0); // Envía evento a la FSM
        
        // Actualiza el nivel global (protegido) para TaskPublish
        if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(50)) == pdTRUE) {
            movementLevel = localMovementLevel;
            xSemaphoreGive(mutexEstado);
        }
        
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_SENS_MS));
    }
}

void TaskBoton(void *pvParameters) {
    bool lastState = HIGH;
    while (1) {
        bool currentState = digitalRead(BUTTON_PIN);
        if (lastState == HIGH && currentState == LOW) { // Flanco de bajada
            
            Evento e;
            // Lee el estado actual (protegido) para decidir qué evento enviar
            if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(50)) == pdTRUE) {
                e = (estado_actual == STANDBY) ? EVENTO_BOTON_ON : EVENTO_BOTON_OFF;
                xSemaphoreGive(mutexEstado);
            } else {
                e = EVENTO_BOTON_OFF; // Default seguro
            }

            xQueueSend(colaEventos, &e, 0); // Envía evento a la FSM
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)); // Antirrebote
        }
        lastState = currentState;
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_BTN_MS)); // Frecuencia de chequeo
    }
}

// --- Tareas de Comunicación (Basadas en Flags) ---

void TaskPublish(void *pvParameters){
  while(1){
    char strMov[10];
    char strSon[10];
    int localFlagLed = 0; // Flag local (0=STANDBY, 1=ON)
    int localMov = 0;
    int localSon = 0;

    // Lee los flags y niveles (protegido)
    if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(50)) == pdTRUE) {
        localFlagLed = flagLed; 
        localMov = movementLevel;
        localSon = soundLevel;
        xSemaphoreGive(mutexEstado);
    } 
    
    // Solo publica si el sistema no está en STANDBY (flagLed == 1)
    if(localFlagLed == 1){ 
        sprintf(strMov, "%d", localMov); 
        client.publish(MQTT_TOPIC_MOV, strMov);
        sprintf(strSon, "%d", localSon); 
        client.publish(MQTT_TOPIC_SONIDO, strSon);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000)); // Publica cada 1 segundo
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String mensaje;
    for (unsigned int i = 0; i < length; i++) {
        mensaje += (char)payload[i];
    }
    
    Evento e;
    bool eventGenerated = false;

    if (String(topic) == MQTT_TOPIC_CONTROL) {
        if (mensaje == "STANDBY") {
            e = EVENTO_BOTON_OFF;
            eventGenerated = true;
        } else if (mensaje == "ACTIVO") {
            e = EVENTO_BOTON_ON;
            eventGenerated = true;
        } else if (mensaje == "MOVIMIENTO") {
            e = EVENTO_MOV;
            eventGenerated = true;
        }
        
        if (eventGenerated) {
          xQueueSend(colaEventos, &e, 0); // Envía evento a la FSM
        }
    }
}

// --- Tarea FSM (Máquina de Estados Finitos) ---
// * REFACTORIZADA CON LÓGICA DE ACCIÓN EN TRANSICIÓN *

void TaskFSM(void *pvParameters)
{
    Evento evento;
    // 'estado_anterior' ya no es necesario para esta lógica
    // Estado estado_anterior; 

    while (1)
    {
        // 1. Espera (bloqueado) hasta que llegue un evento a la cola
        if (xQueueReceive(colaEventos, &evento, portMAX_DELAY) == pdPASS)
        {
            // 2. Toma el mutex para proteger el 'estado_actual' y los 'flags'
            if (xSemaphoreTake(mutexEstado, portMAX_DELAY) == pdTRUE)
            {
                // estado_anterior = estado_actual; // Ya no es necesario

                // SWITCH EXTERNO: Basado en el ESTADO ACTUAL
                switch (estado_actual)
                {
                    case STANDBY:
                        // SWITCH INTERNO: Basado en el EVENTO RECIBIDO
                        switch (evento)
                        {
                            case EVENTO_BOTON_ON:
                                estado_actual = ACTIVO;
                                // --- Acción de Transición ---
                                flagLed = 1;
                                flagBuzzer = 0;
                                flagServo = 0;
                                encenderLED();
                                // -------------------------
                                break;
                            default: // Ignora otros eventos
                                break;
                        }
                        break; // Fin de case STANDBY

                    case ACTIVO:
                        switch (evento)
                        {
                            case EVENTO_BOTON_OFF:
                                estado_actual = STANDBY;
                                // --- Acción de Transición ---
                                flagLed = 0;
                                flagBuzzer = 0;
                                flagServo = 0;
                                apagarSistema();
                                // -------------------------
                                break;
                            case EVENTO_LLANTO:
                                estado_actual = LLANTO;
                                // --- Acción de Transición ---
                                flagLed = 1;
                                flagBuzzer = 1; // <-- BUZZER ON
                                flagServo = 0;
                                encenderLED();
                                // -------------------------
                                break;
                            case EVENTO_MOV:
                                estado_actual = MOVIMIENTO;
                                // --- Acción de Transición ---
                                flagLed = 1;
                                flagBuzzer = 0;
                                flagServo = 1; // <-- SERVO ON
                                encenderLED();
                                // -------------------------
                                break;
                            default:
                                break;
                        }
                        break; // Fin de case ACTIVO

                    case LLANTO:
                        switch (evento)
                        {
                            case EVENTO_BOTON_OFF:
                                estado_actual = STANDBY;
                                // --- Acción de Transición ---
                                flagLed = 0;
                                flagBuzzer = 0;
                                flagServo = 0;
                                apagarSistema();
                                // -------------------------
                                break;
                            case EVENTO_SILENCIO: // El bebé se calmó
                                estado_actual = ACTIVO;
                                // --- Acción de Transición ---
                                flagLed = 1;
                                flagBuzzer = 0;
                                flagServo = 0;
                                encenderLED();
                                // -------------------------
                                break;
                            case EVENTO_MOV: // Ahora llora Y se mueve
                                estado_actual = MOVYLLA;
                                // --- Acción de Transición ---
                                flagLed = 1;
                                flagBuzzer = 1;
                                flagServo = 1;
                                encenderLED();
                                // -------------------------
                                break;
                            default:
                                break;
                        }
                        break; // Fin de case LLANTO

                    case MOVIMIENTO:
                        switch (evento)
                        {
                            case EVENTO_BOTON_OFF:
                                estado_actual = STANDBY;
                                // --- Acción de Transición ---
                                flagLed = 0;
                                flagBuzzer = 0;
                                flagServo = 0;
                                apagarSistema();
                                // -------------------------
                                break;
                            case EVENTO_QUIETO: // El bebé dejó de moverse
                                estado_actual = ACTIVO;
                                // --- Acción de Transición ---
                                flagLed = 1;
                                flagBuzzer = 0;
                                flagServo = 0;
                                encenderLED();
                                // -------------------------
                                break;
                            case EVENTO_LLANTO: // Ahora se mueve Y llora
                                estado_actual = MOVYLLA;
                                // --- Acción de Transición ---
                                flagLed = 1;
                                flagBuzzer = 1;
                                flagServo = 1;
                                encenderLED();
                                // -------------------------
                                break;
                            default:
                                break;
                        }
                        break; // Fin de case MOVIMIENTO

                    case MOVYLLA:
                        switch (evento)
                        {
                            case EVENTO_BOTON_OFF:
                                estado_actual = STANDBY;
                                // --- Acción de Transición ---
                                flagLed = 0;
                                flagBuzzer = 0;
                                flagServo = 0;
                                apagarSistema();
                                // -------------------------
                                break;
                            case EVENTO_SILENCIO: // Dejó de llorar, pero sigue moviéndose
                                estado_actual = MOVIMIENTO;
                                // --- Acción de Transición ---
                                flagLed = 1;
                                flagBuzzer = 0;
                                flagServo = 1;
                                encenderLED();
                                // -------------------------
                                break;
                            case EVENTO_QUIETO: // Dejó de moverse, pero sigue llorando
                                estado_actual = LLANTO;
                                // --- Acción de Transición ---
                                flagLed = 1;
                                flagBuzzer = 1;
                                flagServo = 0;
                                encenderLED();
                                // -------------------------
                                break;
                            default:
                                break;
                        }
                        break; // Fin de case MOVYLLA
                }
                
                // 3. El bloque de "Acción de Entrada" (if estado_anterior != ...) ha sido eliminado
                //    y su lógica se ha movido a los 'case' del switch interno.
                
                // 4. Libera el mutex
                xSemaphoreGive(mutexEstado);
            }
        }
    }
}


void reconnectMQTT() {
    while (!client.connected()) {
        if (client.connect(MQTT_CLIENT_ID)) {
            client.subscribe(MQTT_TOPIC_CONTROL);
        } else {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

// --- Setup y Loop ---

void setup() {
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(SOUND_PIN, INPUT);
    pinMode(PRESSURE_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    servoCuna.attach(SERVO_PIN, 500, 2500);
    servoCuna.write(SERVO_POS_OFF);

    // Inicializa los mecanismos de FreeRTOS
    mutexEstado = xSemaphoreCreateMutex();
    colaEventos = xQueueCreate(QUEUE_SIZE, sizeof(Evento));

    // Conexión WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); 
    }

    // Configuración MQTT
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqttCallback);

    // Creación de Tareas
    xTaskCreate(TaskSonido, "TaskSonido", 4096, NULL, 1, NULL);
    xTaskCreate(TaskMovimiento, "TaskMovimiento", 4096, NULL, 1, NULL);
    xTaskCreate(TaskBoton, "TaskBoton", 2048, NULL, 1, NULL);
    xTaskCreate(TaskPublish, "TaskPublish", 2048, NULL, 1, NULL);
    xTaskCreate(TaskMelody, "TaskMelody", 4096, NULL, 1, NULL);
    xTaskCreate(TaskServo, "TaskServo", 4096, NULL, 1, NULL); 
    xTaskCreate(TaskFSM, "TaskFSM", 4096, NULL, 2, NULL); // Mayor prioridad
}

void loop() {
    // El loop principal solo maneja la conexión MQTT
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop(); // Permite al cliente MQTT procesar mensajes
    
    vTaskDelay(pdMS_TO_TICKS(100)); 
}