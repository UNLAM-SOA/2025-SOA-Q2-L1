/*
 * FIRMWARE CUNA INTELIGENTE v1.0
 * ---------------------------------
 * Plataforma: ESP32 (Compatible con Wokwi)
 * OS: FreeRTOS
 * Arquitectura: Máquina de Estados (FSM) basada en eventos.
 * * Tareas:
 * - 1x Tarea FSM (cerebro)
 * - 3x Tareas de Sensores (Botón, Sonido, Presión)
 * - 3x Tareas de Actuadores (LED, Buzzer, Servo)
 */

// --- Includes ---
#include <Arduino.h>
#include <ESP32Servo.h> // Librería Servo para ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <Metrics.h>
// "pitches.h" ha sido eliminado.

// --- Definición de Frecuencias (reemplaza a pitches.h) ---
// Frecuencias en Hz para las notas de "Estrellita"
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440

// --- Pines (Compatibles con Wokwi) ---

// --- Pines (Compatibles con Wokwi) ---
#define LED_PIN 13         // LED indicador de sistema ON
#define BUTTON_PIN 4    // Botón de encendido/apagado
#define SOUND_SENSOR_PIN 32 // Sensor de sonido (Analógico)
#define PRESSURE_SENSOR_PIN 34 // Sensor de presión (Analógico)
#define BUZZER_PIN 18     // Buzzer
#define SERVO_PIN 23      // Servomotor

// --- Umbrales de Sensores ---
#define SOUND_THRESHOLD 3000   // Umbral para detectar llanto (0-4095)
#define PRESSURE_THRESHOLD 300 // Umbral para detectar movimiento (0-4095)

//int flag = 0;
unsigned long statsIni = 0;
// --- Definición de la Máquina de Estados (FSM) ---

// 1. Estados del Sistema
typedef enum {
  STATE_OFF,
  STATE_IDLE,
  STATE_CRYING,
  STATE_MOVING,
  STATE_CRYING_AND_MOVING
} SystemState;

// 2. Eventos que los sensores pueden enviar
typedef enum {
  EVENT_BUTTON_PRESS,
  EVENT_CRY_DETECTED,
  EVENT_CRY_STOPPED,
  EVENT_MOVE_DETECTED,
  EVENT_MOVE_STOPPED
} FsmEvent;

// --- Variables Globales y Handlers de FreeRTOS ---

// Variable de estado global. Protegida por un Mutex.
volatile SystemState g_currentState = STATE_OFF;

// Mutex para proteger g_currentState
SemaphoreHandle_t stateMutex;

// Cola para enviar eventos desde los sensores a la FSM
QueueHandle_t eventQueue;

// Objeto Servo
Servo servo;

// --- Melodía (Estrellita) ---
int melody[] = {
  NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
  NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4
};
int noteDurations[] = {
  4, 4, 4, 4, 4, 4, 2,
  4, 4, 4, 4, 4, 4, 2
};

// --- Prototipos de Tareas ---
void taskFSM(void *pvParameters);
void taskButtonReader(void *pvParameters);
void taskSoundSensor(void *pvParameters);
void taskPressureSensor(void *pvParameters);
void taskLedControl(void *pvParameters);
void taskBuzzerControl(void *pvParameters);
void taskServoControl(void *pvParameters);

// --- Tareas de Actuadores (Helper) ---
void playMelody();
SystemState getCurrentState(); // Helper para leer el estado de forma segura

// =================================================================
//                      SETUP
// =================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando Sistema de Cuna Inteligente...");

  // 1. Configurar Pines
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SOUND_SENSOR_PIN, INPUT);
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // 2. Configurar Servo
  // Permite la asignación de pines GPIO y temporizadores para el servo en ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50); // Frecuencia estándar para servos
  servo.attach(SERVO_PIN, 500, 2500); // Pin, ancho de pulso min/max

  // 3. Crear Mutex y Cola
  stateMutex = xSemaphoreCreateMutex();
  eventQueue = xQueueCreate(10, sizeof(FsmEvent)); // Cola de 10 eventos

  // 4. Crear Tareas
  // Se pinchan las tareas al Core 1 (Core 0 maneja WiFi/BT por defecto)
  
  // Tarea "Cerebro"
  xTaskCreatePinnedToCore(
    taskFSM, "TaskFSM", 4096, NULL, 5, NULL, 1);

  // Tareas de Sensores (Entradas)
  xTaskCreatePinnedToCore(
    taskButtonReader, "TaskButton", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(
    taskSoundSensor, "TaskSound", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(
    taskPressureSensor, "TaskPressure", 2048, NULL, 3, NULL, 1);

  // Tareas de Actuadores (Salidas)
  xTaskCreatePinnedToCore(
    taskLedControl, "TaskLED", 1024, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(
    taskBuzzerControl, "TaskBuzzer", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(
    taskServoControl, "TaskServo", 2048, NULL, 2, NULL, 1);

  Serial.println("Sistema iniciado. Tareas creadas.");

  
    initStats();
    statsIni = millis();
	
}

// =================================================================
//                      LOOP (Vacío)
// =================================================================
void loop() {
  // El loop de Arduino es ahora una tarea de baja prioridad.
  // Dejamos que el planificador de FreeRTOS maneje todo.
 //if (millis() - statsIni >= 10000 && flag == 0){
   //     flag = 1;
    //}
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// =================================================================
//                 TAREA 1: MÁQUINA DE ESTADOS (FSM)
// =================================================================
/*
 * Esta es la tarea "cerebro".
 * 1. Espera (bloqueada) a que llegue un evento a la 'eventQueue'.
 * 2. Cuando llega un evento, despierta.
 * 3. Bloquea el 'stateMutex' para cambiar el estado global.
 * 4. Implementa el patrón de diseño "switch anidado".
 * 5. Libera el 'stateMutex'.
 * 6. Vuelve a dormir esperando el próximo evento.
 */
void taskFSM(void *pvParameters) {
  FsmEvent event;

  while (1) {
    // 1. Espera por un evento de la cola
    if (xQueueReceive(eventQueue, &event, portMAX_DELAY) == pdPASS) {
      
      // 2. Evento recibido. Tomar el mutex para modificar el estado.
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      
      // Serial.print("FSM: Recibido Evento: ");
      // Serial.print(event);
      // Serial.print(" | Estado Actual: ");
      // Serial.println(g_currentState);

      // 3. Switch de Estado (Outer)
      switch (g_currentState) {
        
        case STATE_OFF:
          // 4. Switch de Evento (Inner)
          switch (event) {
            case EVENT_BUTTON_PRESS:
              g_currentState = STATE_IDLE; // Encender
              Serial.println("FSM: OFF -> IDLE");
              break;
            default: // Ignorar otros eventos si está apagado
              break;
          }
          break;

        case STATE_IDLE:
          switch (event) {
            case EVENT_BUTTON_PRESS:
              g_currentState = STATE_OFF; // Apagar
              Serial.println("FSM: IDLE -> OFF");
              break;
            case EVENT_CRY_DETECTED:
              g_currentState = STATE_CRYING;
              Serial.println("FSM: IDLE -> CRYING");
              break;
            case EVENT_MOVE_DETECTED:
              g_currentState = STATE_MOVING;
              Serial.println("FSM: IDLE -> MOVING");
              break;
            default: // Ignorar "stopped" events
              break;
          }
          break;

        case STATE_CRYING:
          switch (event) {
            case EVENT_BUTTON_PRESS:
              g_currentState = STATE_OFF; // Apagar
              Serial.println("FSM: CRYING -> OFF");
              break;
            case EVENT_MOVE_DETECTED:
              g_currentState = STATE_CRYING_AND_MOVING; // Estado combinado
              Serial.println("FSM: CRYING -> CRYING_AND_MOVING");
              break;
            case EVENT_CRY_STOPPED:
              g_currentState = STATE_IDLE; // Volver a reposo
              Serial.println("FSM: CRYING -> IDLE");
              break;
            default:
              break;
          }
          break;

        case STATE_MOVING:
          switch (event) {
            case EVENT_BUTTON_PRESS:
              g_currentState = STATE_OFF; // Apagar
              Serial.println("FSM: MOVING -> OFF");
              break;
            case EVENT_CRY_DETECTED:
              g_currentState = STATE_CRYING_AND_MOVING; // Estado combinado
              Serial.println("FSM: MOVING -> CRYING_AND_MOVING");
              break;
            case EVENT_MOVE_STOPPED:
              g_currentState = STATE_IDLE; // Volver a reposo
              Serial.println("FSM: MOVING -> IDLE");
              break;
            default:
              break;
          }
          break;

        case STATE_CRYING_AND_MOVING:
          switch (event) {
            case EVENT_BUTTON_PRESS:
              g_currentState = STATE_OFF; // Apagar
              Serial.println("FSM: COMBO -> OFF");
              break;
            case EVENT_CRY_STOPPED:
              g_currentState = STATE_MOVING; // Solo se mueve
              Serial.println("FSM: COMBO -> MOVING");
              break;
            case EVENT_MOVE_STOPPED:
              g_currentState = STATE_CRYING; // Solo llora
              Serial.println("FSM: COMBO -> CRYING");
              break;
            default:
              break;
          }
          break;
      }
      
      // 5. Liberar el mutex
      xSemaphoreGive(stateMutex);
    }
  }
}

// =================================================================
//                 TAREAS DE SENSORES (ENTRADAS)
// =================================================================

/*
 * Tarea 2: Lector de Botón
 * - Revisa el botón con debounce simple.
 * - Envía un evento a la cola cuando se presiona.
 */
void taskButtonReader(void *pvParameters) {
  bool buttonPressed = false;
  while (1) {
    if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) {
      buttonPressed = true;
      FsmEvent event = EVENT_BUTTON_PRESS;
      xQueueSend(eventQueue, &event, portMAX_DELAY);
    } else if (digitalRead(BUTTON_PIN) == HIGH) {
      buttonPressed = false;
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Debounce/Polling
  }
}

/*
 * Tarea 3: Sensor de Sonido
 * - Lee el sensor analógico.
 * - Mantiene su propio estado (isCrying) para evitar spam de eventos.
 * - Envía "CRY_DETECTED" solo en la transición de silencio a llanto.
 * - Envía "CRY_STOPPED" solo en la transición de llanto a silencio.
 */

/*
 * Tarea 3: Sensor de Sonido (Corregida)
 * - Añade una comprobación del estado global.
 * - Si el sistema está OFF, resetea su estado local 'isCrying' a false.
 */
void taskSoundSensor(void *pvParameters) {
  bool isCrying = false;
  FsmEvent event;

  while (1) {
    // Leemos el estado global PRIMERO
    SystemState state = getCurrentState();
    int value = analogRead(SOUND_SENSOR_PIN);

    // Si el sistema está encendido, procesamos la lógica de eventos
    if (state != STATE_OFF) {
      if (value > SOUND_THRESHOLD && !isCrying) {
        isCrying = true;
        event = EVENT_CRY_DETECTED;
        xQueueSend(eventQueue, &event, 0); 
      } else if (value < SOUND_THRESHOLD && isCrying) {
        isCrying = false;
        event = EVENT_CRY_STOPPED;
        xQueueSend(eventQueue, &event, 0);
      }
    } else {
      // Si el sistema está APAGADO (STATE_OFF),
      // reseteamos nuestro estado interno.
      isCrying = false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Polling
  }
}

/*
 * Tarea 4: Sensor de Presión (Corregida)
 * - Exactamente la misma lógica de corrección.
 * - Si el sistema está OFF, resetea 'isMoving' a false.
 */
void taskPressureSensor(void *pvParameters) {
  bool isMoving = false;
  FsmEvent event;

  while (1) {
    // Leemos el estado global PRIMERO
    SystemState state = getCurrentState();
    int value = analogRead(PRESSURE_SENSOR_PIN);

    // Si el sistema está encendido, procesamos la lógica de eventos
    if (state != STATE_OFF) {
      if (value > PRESSURE_THRESHOLD && !isMoving) {
        isMoving = true;
        event = EVENT_MOVE_DETECTED;
        xQueueSend(eventQueue, &event, 0);
      } else if (value < PRESSURE_THRESHOLD && isMoving) {
        isMoving = false;
        event = EVENT_MOVE_STOPPED;
        xQueueSend(eventQueue, &event, 0);
      }
    } else {
      // Si el sistema está APAGADO (STATE_OFF),
      // reseteamos nuestro estado interno.
      isMoving = false;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Polling
  }
}

// =================================================================
//                 TAREAS DE ACTUADORES (SALIDAS)
// =================================================================

/*
 * Tarea 5: Control del LED
 * - Tarea "tonta". Solo obedece.
 * - Lee el estado global (con Mutex).
 * - Enciende el LED si el estado NO es OFF.
 */
void taskLedControl(void *pvParameters) {
  while (1) {
    SystemState state = getCurrentState();
    
    if (state == STATE_OFF) {
      digitalWrite(LED_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, HIGH);
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Polling
  }
}

/*
 * Tarea 6: Control del Buzzer
 * - Lee el estado global.
 * - Si el estado es CRYING o CRYING_AND_MOVING, llama a playMelody().
 * - playMelody() está diseñada para ser NO BLOQUEANTE y reactiva.
 */
void taskBuzzerControl(void *pvParameters) {
  while (1) {
    SystemState state = getCurrentState();

    if (state == STATE_CRYING || state == STATE_CRYING_AND_MOVING) {
      playMelody(); // Esta función se encarga de los vTaskDelay
    } else {
      noTone(BUZZER_PIN);
      vTaskDelay(pdMS_TO_TICKS(200)); // Esperar si no hay nada que hacer
    }
  }
}

/*
 * Tarea 7: Control del Servo
 * - Lee el estado global.
 * - Si el estado es MOVING o CRYING_AND_MOVING, ejecuta el vaivén.
 * - Es reactivo: chequea el estado *antes* de cada movimiento.
 */
void taskServoControl(void *pvParameters) {
  while (1) {
    SystemState state = getCurrentState();

    if (state == STATE_MOVING || state == STATE_CRYING_AND_MOVING) {
      // Mover a 90 grados
      servo.write(90);
      finishStats();
      vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 seg

      // RE-CHEQUEAR el estado. Si el bebé paró, no volver.
      state = getCurrentState();
      if (state != STATE_MOVING && state != STATE_CRYING_AND_MOVING) {
        continue; // Volver al inicio del loop
      }

      // Mover a 0 grados
      servo.write(0);
      vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 seg

    } else {
      // Si no debe moverse, ir a la posición de reposo (0)
      servo.write(0);
      vTaskDelay(pdMS_TO_TICKS(200)); // Esperar
    }
  }
}

// =================================================================
//                 FUNCIONES AUXILIARES
// =================================================================

/*
 * Lee el estado global g_currentState de forma segura usando el Mutex.
 * Esta es la única forma en que las tareas de actuadores deben leer el estado.
 */
SystemState getCurrentState() {
  SystemState tempState;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  tempState = g_currentState;
  xSemaphoreGive(stateMutex);
  return tempState;
}

/*
 * Reproductor de melodía NO BLOQUEANTE.
 * - Toca una nota.
 * - Usa vTaskDelay() en lugar de delay().
 * - DESPUÉS de cada nota, re-chequea el estado.
 * - Si el estado cambia (bebé se calmó), para la melodía inmediatamente.
 */
void playMelody() {
  int melodyLength = sizeof(melody) / sizeof(melody[0]);
  
  for (int thisNote = 0; thisNote < melodyLength; thisNote++) {
    
    // 1. RE-CHEQUEAR ESTADO ANTES DE TOCAR LA SIGUIENTE NOTA
    SystemState state = getCurrentState();
    if (state != STATE_CRYING && state != STATE_CRYING_AND_MOVING) {
      noTone(BUZZER_PIN);
      break; // Salir del loop de melodía
    }

    // 2. Tocar la nota
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);

    // 3. Pausa no bloqueante (vTaskDelay)
    int pauseBetweenNotes = noteDuration * 1.30;
    vTaskDelay(pdMS_TO_TICKS(pauseBetweenNotes));

    // 4. Parar el tono
    noTone(BUZZER_PIN);
  }
}