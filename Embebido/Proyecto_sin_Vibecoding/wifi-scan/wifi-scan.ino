#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>


#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define REST      0 

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


#define WIFI_SSID           "Micaela"
#define WIFI_PASSWORD       "Mica1611aaa"
#define MQTT_SERVER         "broker.hivemq.com"
#define MQTT_PORT           1883
#define MQTT_CLIENT_ID      "ESP32_CUNA"
#define MQTT_TOPIC_SONIDO   "sensor/sonido"
#define MQTT_TOPIC_MOV      "sensor/movimiento"
#define MQTT_TOPIC_ESTADO   "sensor/estado"
#define MQTT_TOPIC_CONTROL  "control/estado"

#define BUZZER_PIN     18
#define LED_PIN        13
#define SOUND_PIN      32  
#define SERVO_PIN      23
#define PRESSURE_PIN   34  
#define BUTTON_PIN     4

#define UMBRAL_SONIDO       600
#define UMBRAL_MOV          500
#define SERVO_POS_OFF       0
#define SERVO_POS_ON        90
#define SERVO_MOVE_PERIOD_MS 500 

#define QUEUE_SIZE          20
#define TASK_DELAY_SENS_MS  500
#define TASK_DELAY_BTN_MS   50
#define DEBOUNCE_DELAY_MS   200

int soundLevel = 0;
int movementLevel = 0;

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
SemaphoreHandle_t mutexEstado;
Servo servoCuna;


WiFiClient espClient;
PubSubClient client(espClient);

void encenderLED() {
    digitalWrite(LED_PIN, HIGH);
}


void apagarSistema() {
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
    servoCuna.write(SERVO_POS_OFF); 
}

void TaskMelody(void *pvParameters) {
    while (1) {
        bool shouldPlay = false;

        // 1. Verificar el estado (protegido por mutex)
        if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (estado_actual == LLANTO || estado_actual == MOVYLLA) {
                shouldPlay = true;
            }
            xSemaphoreGive(mutexEstado);
        }

        // 2. Actuar según el estado
        if (shouldPlay) {
            // Toca la melodía completa (bloqueando esta tarea, que está bien)
            for (int i = 0; i < melodyLength; i++) {
                
                // Chequeo de aborto (simplificado):
                // Si el estado cambia a mitad de la melodía, queremos parar.
                // Chequeamos solo cada 4 notas para no sobrecargar el mutex.
                if (i % 4 == 0) { 
                     if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(10)) == pdTRUE) {
                        if (estado_actual != LLANTO && estado_actual != MOVYLLA) {
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
            // Si no debe sonar, apagar y dormir.
            noTone(BUZZER_PIN);
            vTaskDelay(pdMS_TO_TICKS(200)); // Chequear el estado cada 200ms
        }
    }
}

void TaskServo(void *pvParameters) {
    while(1) {
        bool shouldMove = false;

        // 1. Verificar el estado (protegido por mutex)
        if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (estado_actual == MOVIMIENTO || estado_actual == MOVYLLA) {
                shouldMove = true;
            }
            xSemaphoreGive(mutexEstado);
        }

        // 2. Actuar según el estado
        if (shouldMove) {
            // Realiza un ciclo de mecedora (bloqueando esta tarea)
            servoCuna.write(SERVO_POS_ON);
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_PERIOD_MS));
            
            // Chequeo de aborto (simplificado):
            // Si el estado cambia, parar inmediatamente.
            if (xSemaphoreTake(mutexEstado, 0) == pdTRUE) { 
               if (estado_actual != MOVIMIENTO && estado_actual != MOVYLLA) {
                   xSemaphoreGive(mutexEstado);
                   servoCuna.write(SERVO_POS_OFF);
                   continue; // Salta al inicio del while(1)
               }
               xSemaphoreGive(mutexEstado);
            }

            servoCuna.write(SERVO_POS_OFF);
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_PERIOD_MS));
        } else {
            // Si no debe moverse, asegurarse de que está en OFF y dormir.
            servoCuna.write(SERVO_POS_OFF);
            vTaskDelay(pdMS_TO_TICKS(200)); // Chequear el estado cada 200ms
        }
    }
}


void TaskSonido(void *pvParameters) {
    while (1) {
        int localSoundLevel = analogRead(SOUND_PIN);
        Serial.print("Sonido: ");
        Serial.println(localSoundLevel);
        
        Evento e = (localSoundLevel > UMBRAL_SONIDO) ? EVENTO_LLANTO : EVENTO_SILENCIO;
        xQueueSend(colaEventos, &e, 0);
        
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
        xQueueSend(colaEventos, &e, 0);
        
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
        if (lastState == HIGH && currentState == LOW) {
            
            Evento e;
            if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(50)) == pdTRUE) {
                e = (estado_actual == STANDBY) ? EVENTO_BOTON_ON : EVENTO_BOTON_OFF;
                xSemaphoreGive(mutexEstado);
            } else {
                e = EVENTO_BOTON_OFF;
            }

            xQueueSend(colaEventos, &e, 0);
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
        }
        lastState = currentState;
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_BTN_MS));
    }
}

void TaskPublish(void *pvParameters){
  while(1){
    char strMov[10];
    char strSon[10];
    Estado estadoLocal;
    int localMov = 0;
    int localSon = 0;

    if (xSemaphoreTake(mutexEstado, pdMS_TO_TICKS(50)) == pdTRUE) {
        estadoLocal = estado_actual;
        localMov = movementLevel;
        localSon = soundLevel;
        xSemaphoreGive(mutexEstado);
    } else {
        estadoLocal = STANDBY;
    }

    if(estadoLocal != STANDBY){
        sprintf(strMov, "%d", localMov); 
        client.publish(MQTT_TOPIC_MOV, strMov);
        sprintf(strSon, "%d", localSon); 
        client.publish(MQTT_TOPIC_SONIDO, strSon);
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
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
          xQueueSend(colaEventos, &e, 0);
        }
    }
}

void TaskFSM(void *pvParameters)
{
    Evento evento;
    Estado estado_anterior;

    while (1)
    {
        if (xQueueReceive(colaEventos, &evento, portMAX_DELAY) == pdPASS)
        {
            if (xSemaphoreTake(mutexEstado, portMAX_DELAY) == pdTRUE)
            {
                estado_anterior = estado_actual;

                switch (estado_actual)
                {
                    case STANDBY:
                        if (evento == EVENTO_BOTON_ON) {
                            estado_actual = ACTIVO;
                        }
                        break;

                    case ACTIVO:
                        if (evento == EVENTO_BOTON_OFF) {
                            estado_actual = STANDBY;
                        } else if (evento == EVENTO_LLANTO) {
                            estado_actual = LLANTO;
                        } else if (evento == EVENTO_MOV) {
                            estado_actual = MOVIMIENTO;
                        }
                        break;

                    case LLANTO:
                        if (evento == EVENTO_BOTON_OFF) {
                            estado_actual = STANDBY;
                        } else if (evento == EVENTO_SILENCIO) {
                            estado_actual = ACTIVO;
                        } else if (evento == EVENTO_MOV) {
                            estado_actual = MOVYLLA;
                        }
                        break;

                    case MOVIMIENTO:
                        if (evento == EVENTO_BOTON_OFF) {
                            estado_actual = STANDBY;
                        } else if (evento == EVENTO_QUIETO) {
                            estado_actual = ACTIVO;
                        } else if (evento == EVENTO_LLANTO) {
                            estado_actual = MOVYLLA;
                        }
                        break;

                    case MOVYLLA:
                        if (evento == EVENTO_BOTON_OFF) {
                            estado_actual = STANDBY;
                        } else if (evento == EVENTO_SILENCIO) {
                            estado_actual = MOVIMIENTO;
                        } else if (evento == EVENTO_QUIETO) {
                            estado_actual = LLANTO;
                        }
                        break;
                }
                
                if (estado_anterior != estado_actual) {
                   switch (estado_actual) {
                        case STANDBY:
                            apagarSistema();
                            break;
                        case ACTIVO:
                        case LLANTO:
                        case MOVIMIENTO:
                        case MOVYLLA:
                            encenderLED();
                            break;
                   }
                }
                
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

void setup() {
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(SOUND_PIN, INPUT);
    pinMode(PRESSURE_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    servoCuna.attach(SERVO_PIN, 500, 2500);
    servoCuna.write(SERVO_POS_OFF);

    mutexEstado = xSemaphoreCreateMutex();
    
    colaEventos = xQueueCreate(QUEUE_SIZE, sizeof(Evento));

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int i = 0;
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print("Conectando");
        i=i+1;
        Serial.println(i);
        delay(500); 
    }

    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqttCallback);

    xTaskCreate(TaskSonido, "TaskSonido", 4096, NULL, 1, NULL);
    xTaskCreate(TaskMovimiento, "TaskMovimiento", 4096, NULL, 1, NULL);
    xTaskCreate(TaskBoton, "TaskBoton", 2048, NULL, 1, NULL);
    xTaskCreate(TaskPublish, "TaskPublish", 2048, NULL, 1, NULL);
    xTaskCreate(TaskMelody, "TaskMelody", 4096, NULL, 1, NULL);
    xTaskCreate(TaskServo, "TaskServo", 4096, NULL, 1, NULL); 
    xTaskCreate(TaskFSM, "TaskFSM", 4096, NULL, 2, NULL);

}

void loop() {
    if (!client.connected()) {
        reconnectMQTT();
        Serial.print("Reintentar...");
    }
    client.loop();
    vTaskDelay(pdMS_TO_TICKS(100)); 
}


