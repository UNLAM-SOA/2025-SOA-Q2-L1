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

// --- Variables Globales y Mecanismos de Sincronización ---

enum Estado { STANDBY, ACTIVO, LLANTO, MOVIMIENTO, MOVYLLA };
enum Evento { 
    EVENTO_SILENCIO, 
    EVENTO_LLANTO, 
    EVENTO_QUIETO, 
    EVENTO_MOV, 
    EVENTO_BOTON_TOGGLE // Evento único para el botón
};
enum TareaCmd { CMD_STOP = 0, CMD_START = 1 }; // Comandos para actuadores

Estado estado_actual = STANDBY; // ¡Solo la FSM toca esta variable!
QueueHandle_t colaEventos; // (Sensores -> FSM)

// --- ¡MECANISMOS DE "DESPERTADOR"! ---
// Semáforos Binarios para "Despertar" (Start)
SemaphoreHandle_t semMelodyStart;
SemaphoreHandle_t semServoStart;
SemaphoreHandle_t semPublishStart;
// Colas de Comandos para "Interrumpir" (Stop)
QueueHandle_t queueMelodyCmd;
QueueHandle_t queueServoCmd;
QueueHandle_t queuePublishCmd;


Servo servoCuna;
WiFiClient espClient;
PubSubClient client(espClient);

// --- Funciones de Actuador (Hardware) (sin cambios) ---
void encenderLED() { digitalWrite(LED_PIN, HIGH); }
void apagarSistema() {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
    servoCuna.write(SERVO_POS_OFF); 
}

// --- Tareas de Actuadores (Basadas en Señales/Comandos) ---

void TaskMelody(void *pvParameters) {
    // Inicializa cmd a un valor seguro (START)
    TareaCmd cmd = CMD_START; 
    
    while (1) {
        // 1. Duerme indefinidamente hasta que la FSM la "despierte"
        xSemaphoreTake(semMelodyStart, portMAX_DELAY); 

        // 2. Resetea el comando a START cada vez que se despierta.
        cmd = CMD_START; 

        while (1) { // Bucle de reproducción
            // 2.1. Revisa si la FSM envió un comando de STOP
            if (xQueueReceive(queueMelodyCmd, &cmd, 0) == pdPASS) {
                if (cmd == CMD_STOP) {
                    noTone(BUZZER_PIN);
                    break; // Vuelve a dormir (paso 1)
                }
            }

            // 2.2. Toca la melodía
            for (int i = 0; i < melodyLength; i++) {
                if (i % 4 == 0) { // Chequeo de aborto intermedio
                     if (xQueueReceive(queueMelodyCmd, &cmd, 0) == pdPASS) {
                         if (cmd == CMD_STOP) break;
                     }
                }
                if (cmd == CMD_STOP) break;

                int noteDuration = 1000 / noteDurations[i];
                tone(BUZZER_PIN, melody[i], noteDuration);
                vTaskDelay(pdMS_TO_TICKS(noteDuration * 1.30));
                noTone(BUZZER_PIN);
            }
            if (cmd == CMD_STOP) { 
                break; 
            }
            vTaskDelay(pdMS_TO_TICKS(500)); 
        }
    }
}

void TaskServo(void *pvParameters) {
    // Inicializa cmd a un valor seguro (STOP por defecto)
    TareaCmd cmd = CMD_STOP;
    while(1) {
        // 1. Duerme indefinidamente hasta que la FSM la "despierte"
        xSemaphoreTake(semServoStart, portMAX_DELAY);
        
        // 2. Resetea el comando a START cada vez que se despierta.
        cmd = CMD_START;

        while(1) { // Bucle de mecedora
            // 2.1. Revisa si la FSM envió un comando de STOP
            if (xQueueReceive(queueServoCmd, &cmd, 0) == pdPASS) {
                if (cmd == CMD_STOP) {
                    servoCuna.write(SERVO_POS_OFF);
                    break; // Vuelve a dormir (paso 1)
                }
            }
            
            // 2.2. Ejecuta un ciclo de mecedora
            servoCuna.write(SERVO_POS_ON);
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_PERIOD_MS));
            if (xQueueReceive(queueServoCmd, &cmd, 0) == pdPASS) {
                 if (cmd == CMD_STOP) break; 
            }
            servoCuna.write(SERVO_POS_OFF);
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_PERIOD_MS));
        }
    }
}

// --- Tareas de Sensores y Botón (Generadoras de Eventos) ---

void TaskSonido(void *pvParameters) {
    while (1) {
        int localSoundLevel = analogRead(SOUND_PIN);
        Serial.print("Sonido: "); Serial.println(localSoundLevel);
        
        // Solo envía eventos. No actualiza variables globales.
        Evento e = (localSoundLevel > UMBRAL_SONIDO) ? EVENTO_LLANTO : EVENTO_SILENCIO;
        xQueueSend(colaEventos, &e, 0); // Envía evento a la FSM
        
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_SENS_MS));
    }
}

void TaskMovimiento(void *pvParameters) {
    while (1) {
        int localMovementLevel = analogRead(PRESSURE_PIN);
        Serial.print("Presión: "); Serial.println(localMovementLevel);
        
        // Solo envía eventos. No actualiza variables globales.
        Evento e = (localMovementLevel > UMBRAL_MOV) ? EVENTO_MOV : EVENTO_QUIETO;
        xQueueSend(colaEventos, &e, 0); // Envía evento a la FSM
        
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_SENS_MS));
    }
}

// * REFACTORIZADO: Ya no consulta estado_actual ni usa Mutex *
void TaskBoton(void *pvParameters) {
    bool lastState = HIGH;
    while (1) {
        bool currentState = digitalRead(BUTTON_PIN);
        if (lastState == HIGH && currentState == LOW) { // Flanco de bajada
            
            // Envía un evento "TOGGLE" (alternar) simple.
            // La FSM decidirá qué hacer con él.
            Evento e = EVENTO_BOTON_TOGGLE; 
            xQueueSend(colaEventos, &e, 0); // Envía evento a la FSM
            
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)); // Antirrebote
        }
        lastState = currentState;
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_BTN_MS)); // Frecuencia de chequeo
    }
}

// --- Tareas de Comunicación ---

// * REFACTORIZADO: Ahora es controlado por señales y lee los sensores él mismo *
void TaskPublish(void *pvParameters){
  // Inicializa cmd a un valor seguro (STOP por defecto)
  TareaCmd cmd = CMD_STOP;
  while(1){
    
    // 1. Duerme indefinidamente hasta que la FSM la "despierte"
    xSemaphoreTake(semPublishStart, portMAX_DELAY);
    
    // 2. Resetea el comando a START cada vez que se despierta.
    cmd = CMD_START;

    // 2. Al despertar, comienza el bucle de publicación
    while(1) {
        // 2.1. Revisa (sin bloquear) si la FSM envió un comando de STOP
        if (xQueueReceive(queuePublishCmd, &cmd, 0) == pdPASS) {
            if (cmd == CMD_STOP) {
                break; // Sale del bucle de publicación y vuelve a dormir (paso 1)
            }
        }

        // 2.2. Lee los sensores AHORA (ya no depende de variables globales)
        int localSon = analogRead(SOUND_PIN);
        int localMov = analogRead(PRESSURE_PIN);
        
        char strMov[10], strSon[10];

        // 2.3. Publica los datos
        sprintf(strMov, "%d", localMov); 
        client.publish(MQTT_TOPIC_MOV, strMov);
        sprintf(strSon, "%d", localSon); 
        client.publish(MQTT_TOPIC_SONIDO, strSon);

        // 2.4. Espera 1 segundo
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String mensaje;
    for (unsigned int i = 0; i < length; i++) { mensaje += (char)payload[i]; }
    
    Evento e;
    bool eventGenerated = false;
    if (String(topic) == MQTT_TOPIC_CONTROL) {
        if (mensaje == "STANDBY" || mensaje == "ACTIVO") {
            // Ambos comandos remotos se traducen a un TOGGLE
            e = EVENTO_BOTON_TOGGLE; 
            eventGenerated = true;
        } else if (mensaje == "MOVIMIENTO") {
            e = EVENTO_MOV; 
            eventGenerated = true;
        }
        
        if (eventGenerated) { xQueueSend(colaEventos, &e, 0); } // Envía evento a la FSM
    }
}

// --- Tarea FSM (Cerebro Central) ---
// * REFACTORIZADA: Sin Mutex, controla Tareas de Actuador Y Publicación *

void TaskFSM(void *pvParameters)
{
    Evento evento;
    TareaCmd cmd_off = CMD_STOP; // Comando reutilizable

    while (1)
    {
        // 1. Espera (bloqueado) hasta que llegue un evento
        // (Ya no se necesita Mutex, FSM es la única que escribe 'estado_actual')
        if (xQueueReceive(colaEventos, &evento, portMAX_DELAY) == pdPASS)
        {
            // SWITCH EXTERNO: Basado en el ESTADO ACTUAL
            switch (estado_actual)
            {
                case STANDBY:
                    switch (evento)
                    {
                        case EVENTO_BOTON_TOGGLE: // ON (Físico o MQTT)
                            estado_actual = ACTIVO;
                            // --- Acción de Transición ---
                            encenderLED();
                            xSemaphoreGive(semPublishStart); // ¡Despierta TaskPublish!
                            // -------------------------
                            break;
                        default: break;
                    }
                    break; // Fin de case STANDBY

                case ACTIVO:
                    switch (evento)
                    {
                        case EVENTO_BOTON_TOGGLE: // OFF (Físico o MQTT)
                            estado_actual = STANDBY;
                            // --- Acción de Transición ---
                            xQueueSend(queuePublishCmd, &cmd_off, 0); // ¡Comando STOP!
                            apagarSistema();
                            // -------------------------
                            break;
                        case EVENTO_LLANTO:
                            estado_actual = LLANTO;
                            xSemaphoreGive(semMelodyStart); // ¡Despierta TaskMelody!
                            break;
                        case EVENTO_MOV:
                            estado_actual = MOVIMIENTO;
                            xSemaphoreGive(semServoStart); // ¡Despierta TaskServo!
                            break;
                        default: break;
                    }
                    break; // Fin de case ACTIVO

                case LLANTO:
                    switch (evento)
                    {
                        case EVENTO_BOTON_TOGGLE: // OFF
                            estado_actual = STANDBY;
                            xQueueSend(queuePublishCmd, &cmd_off, 0); // STOP Publish
                            xQueueSend(queueMelodyCmd, &cmd_off, 0); // STOP Melody
                            apagarSistema();
                            break;
                        case EVENTO_SILENCIO:
                            estado_actual = ACTIVO;
                            xQueueSend(queueMelodyCmd, &cmd_off, 0); // STOP Melody
                            break;
                        case EVENTO_MOV:
                            estado_actual = MOVYLLA;
                            xSemaphoreGive(semServoStart); // ¡Despierta TaskServo!
                            break;
                        default: break;
                    }
                    break; // Fin de case LLANTO

                case MOVIMIENTO:
                    switch (evento)
                    {
                        case EVENTO_BOTON_TOGGLE: // OFF
                            estado_actual = STANDBY;
                            xQueueSend(queuePublishCmd, &cmd_off, 0); // STOP Publish
                            xQueueSend(queueServoCmd, &cmd_off, 0); // STOP Servo
                            apagarSistema();
                            break;
                        case EVENTO_QUIETO:
                            estado_actual = ACTIVO;
                            xQueueSend(queueServoCmd, &cmd_off, 0); // STOP Servo
                            break;
                        case EVENTO_LLANTO:
                            estado_actual = MOVYLLA;
                            xSemaphoreGive(semMelodyStart); // ¡Despierta TaskMelody!
                            break;
                        default: break;
                    }
                    break; // Fin de case MOVIMIENTO

                case MOVYLLA:
                    switch (evento)
                    {
                        case EVENTO_BOTON_TOGGLE: // OFF
                            estado_actual = STANDBY;
                            xQueueSend(queuePublishCmd, &cmd_off, 0); // STOP Publish
                            xQueueSend(queueMelodyCmd, &cmd_off, 0); // STOP Melody
                            xQueueSend(queueServoCmd, &cmd_off, 0);  // STOP Servo
                            apagarSistema();
                            break;
                        case EVENTO_SILENCIO:
                            estado_actual = MOVIMIENTO;
                            xQueueSend(queueMelodyCmd, &cmd_off, 0); // STOP Melody
                            break;
                        case EVENTO_QUIETO:
                            estado_actual = LLANTO;
                            xQueueSend(queueServoCmd, &cmd_off, 0); // STOP Servo
                            break;
                        default: break;
                    }
                    break; // Fin de case MOVYLLA
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

    // --- Inicializa los mecanismos de FreeRTOS ---
    
    // (Mutex ya no es necesario para el estado, se eliminó)
    
    // Cola de eventos (Sensores -> FSM)
    colaEventos = xQueueCreate(QUEUE_SIZE, sizeof(Evento));

    // Semáforos Binarios para "Despertar" (Start)
    semMelodyStart = xSemaphoreCreateBinary();
    semServoStart = xSemaphoreCreateBinary();
    semPublishStart = xSemaphoreCreateBinary();

    // Colas de Comandos para "Interrumpir" (Stop)
    queueMelodyCmd = xQueueCreate(1, sizeof(TareaCmd)); // Cola de tamaño 1
    queueServoCmd = xQueueCreate(1, sizeof(TareaCmd));  // Cola de tamaño 1
    queuePublishCmd = xQueueCreate(1, sizeof(TareaCmd)); // Cola de tamaño 1

    // Conexión WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(500); }

    // Configuración MQTT
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqttCallback);

    // Creación de Tareas
    xTaskCreate(TaskSonido, "TaskSonido", 4096, NULL, 1, NULL);
    xTaskCreate(TaskMovimiento, "TaskMovimiento", 4096, NULL, 1, NULL);
    xTaskCreate(TaskBoton, "TaskBoton", 2048, NULL, 1, NULL);
    xTaskCreate(TaskPublish, "TaskPublish", 4096, NULL, 1, NULL); // Aumentado stack por lecturas
    xTaskCreate(TaskMelody, "TaskMelody", 4096, NULL, 1, NULL);
    xTaskCreate(TaskServo, "TaskServo", 4096, NULL, 1, NULL); 
    xTaskCreate(TaskFSM, "TaskFSM", 4096, NULL, 2, NULL); // Mayor prioridad
}

void loop() {
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(100)); 
}