#include <Arduino.h> 
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
}
#include <cstdint> 
#include <cstddef> 
#include <cstring> 

#include "esp_timer.h"
#include "Metrics.h"
// ------------------ variables globales ------------------
size_t heapTotal = 0;
size_t heapLibre = 0;
size_t heapUsado = 0;

float pctCoreTotal[2] = { 0, 0 };
float pctIdleTotal[2] = { 0, 0 };
float pctBusyTotal[2] = { 0, 0 };

float sumCoreTotal[2] = { 0, 0 };
float sumIdleTotal[2] = { 0, 0 };
float sumBusyTotal[2] = { 0, 0 };

uint64_t sumHeapTotal = 0;
uint64_t sumHeapLibre = 0;
uint64_t sumHeapUsado = 0;

//snapshots previos
static uint64_t prevTotalCore[2] = { 0, 0 };
static uint64_t prevIdleCore[2] = { 0, 0 };
static uint32_t prevTotalRuntime = 0;
static bool havePrevSnapshot = false;

int gSamples = 0;

uint64_t initialTime = 0;

static TaskHandle_t gStatsTask = nullptr;

static portMUX_TYPE sMux = portMUX_INITIALIZER_UNLOCKED;

// ------------------ Prototipos de funciones ------------------
static void calculateStats();
static void printCpuStats();
static void printMemoryStats();
static void printCpuAverageStats();
static void printMemoryAverageStats();
static void StatsTask(void *pv);

static void printCpuUseForTask(uint64_t totalCore[], TaskStatus_t *taskArray, UBaseType_t taskCount) {
  Serial.println("\n=== Uso de CPU por núcleo y tarea ===");
  
  for (UBaseType_t i = 0; i < taskCount; i++) {
    uint32_t core = taskArray[i].xCoreID;
 
    if (core < 2 && totalCore[core] > 0) {
      float pct = 100.0f * (float)taskArray[i].ulRunTimeCounter / (float)totalCore[core];
      Serial.printf("Core %u | %-16s | %6.2f%%\n", core, taskArray[i].pcTaskName, pct);
    }
  }
}

static void calculateStats() {
  UBaseType_t maxTasks = uxTaskGetNumberOfTasks();
  TaskStatus_t *taskArray = (TaskStatus_t *)pvPortMalloc(maxTasks * sizeof(TaskStatus_t));

  if (!taskArray) {
    Serial.println("[Stats] Error de memoria");
    return;
  }

  uint32_t totalRuntime = 0;
  UBaseType_t taskCount = uxTaskGetSystemState(taskArray, maxTasks, &totalRuntime);

  uint64_t totalCore[2] = { 0, 0 };
  uint64_t idleCore[2] = { 0, 0 };

  // Acumulo tiempos por core y detecto IDLE0/IDLE1
  for (UBaseType_t i = 0; i < taskCount; i++) {
    if (taskArray[i].xCoreID < 2) {
      totalCore[taskArray[i].xCoreID] += taskArray[i].ulRunTimeCounter;

      const char *name = taskArray[i].pcTaskName;
      if (name && (strncmp(name, "IDLE0", 5) == 0 || strncmp(name, "IDLE1", 5) == 0)) {
        idleCore[taskArray[i].xCoreID] += taskArray[i].ulRunTimeCounter;
      }
    }
  }

  //si esta habilitado se imprimen el uso de cpu por cada tarea
  if (SHOW_INTERMEDIATE_STATS) {
    printCpuUseForTask(totalCore, taskArray, taskCount);
  }

  if (totalRuntime > 0) {
    for (int core = 0; core < 2; core++) {

      // Calculo el total/idle/busy del núcleo
      pctCoreTotal[core] = 100.0f * (float)totalCore[core] / (float)totalRuntime;
      pctIdleTotal[core] = 100.0f * (float)idleCore[core] / (float)totalRuntime;
      pctBusyTotal[core] = pctCoreTotal[core] - pctIdleTotal[core];

      // Acumulo para promedio
      sumCoreTotal[core] += pctCoreTotal[core];
      sumIdleTotal[core] += pctIdleTotal[core];
      sumBusyTotal[core] += pctBusyTotal[core];
    }

    // Obtengo el uso de la Memoria heap
    heapTotal = ESP.getHeapSize();
    heapLibre = ESP.getFreeHeap();
    heapUsado = heapTotal - heapLibre;

    //llevo la cuenta de la cantidad de muestras para poder despues sacar el promedio
    sumHeapTotal += heapTotal;
    sumHeapLibre += heapLibre;
    sumHeapUsado += heapUsado;

    gSamples++;
    
  } else {
    Serial.println("(totalRuntime = 0: aún no hay muestras suficientes. No se pudo calcular las métricas)");
  }

  vPortFree(taskArray);
}


static void printCpuStats() {
  for (int core = 0; core < 2; core++) {
    Serial.printf("Core %d -> Total: %6.2f%% | Ocupado: %6.2f%% | Libre (IDLE): %6.2f%%\n",
                  core, pctCoreTotal[core], pctBusyTotal[core], pctIdleTotal[core]);
  }
}

static void printMemoryStats() {
  Serial.println("\n=== Estado de la memoria en ESP32 ===");
  Serial.println("=== Memoria interna (Heap) ===");
  Serial.printf("Heap total : %lu bytes\n", (unsigned long)heapTotal);
  Serial.printf("Heap libre : %lu bytes\n", (unsigned long)heapLibre);
  Serial.printf("Heap usado : %lu bytes\n", (unsigned long)heapUsado);
  Serial.printf("Uso        : %.2f %%\n\n",
                heapTotal ? (heapUsado * 100.0) / heapTotal : 0.0);
}

static void printCpuAverageStats() {
  if (gSamples <= 0) {
    Serial.println("Sin muestras para promediar.");
    return;
  }

  Serial.println("\n==================================================");
  Serial.println("====== Estado Promedio del Uso de CPU en ESP32 ===");
  Serial.println("==================================================");

  for (int core = 0; core < 2; core++) {
    const float avgTotal = sumCoreTotal[core] / gSamples;
    const float avgBusy = sumBusyTotal[core] / gSamples;
    const float avgIdle = sumIdleTotal[core] / gSamples;

    Serial.printf("Core %d -> Total: %6.2f%% | Ocupado: %6.2f%% | Libre (IDLE): %6.2f%%\n",
                  core, avgTotal, avgBusy, avgIdle);
  }
}

static void printMemoryAverageStats() {
  if (gSamples <= 0) {
    Serial.println("Sin muestras para promediar.");
    return;
  }

  unsigned long averageHeapTotal = (unsigned long)(sumHeapTotal / (uint64_t)gSamples);
  unsigned long averageHeapLibre = (unsigned long)(sumHeapLibre / (uint64_t)gSamples);
  unsigned long averageHeapUsado = (unsigned long)(sumHeapUsado / (uint64_t)gSamples);

  Serial.println("\n==================================================");
  Serial.println("====== Estado Promedio de la memoria en ESP32 ====");
  Serial.println("============= Memoria interna (Heap) =============");
  Serial.println("==================================================");

  Serial.printf("Heap total : %lu bytes\n", averageHeapTotal);
  Serial.printf("Heap libre : %lu bytes\n", averageHeapLibre);
  Serial.printf("Heap usado : %lu bytes\n", averageHeapUsado);
  Serial.printf("Uso        : %.2f %%\n\n",
                averageHeapTotal ? (averageHeapUsado * 100.0) / averageHeapTotal : 0.0);
}

static void StatsTask(void *pv) {
  initialTime = esp_timer_get_time();

  while(1) {

    //imprime la metricas normales
    calculateStats();

    //se muestran las estadisticas intermedias
    if (SHOW_INTERMEDIATE_STATS) {
      Serial.println("\n=== Contribución al total del sistema ===");
      printCpuStats();
      printMemoryStats();
    }

    /* Se fija si le llego una notificacion de finalización de ejecucion por parte del padre.
        -Si le llego, muestra el promedio de las metricas y termina y borra esta tarea
        -Si no le llego, espera (bloquea)la cantidad de milisegundos inidicado en TIME_BETWEEN_PRINTS_TASK, 
        de manera tal que se espere esa cantidad de tiempo para volver hacer el calculo de la stats
    */
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TIME_BETWEEN_PRINTS_TASK)) > 0) {

      uint64_t finalTime = esp_timer_get_time() - initialTime;
      double samplingTime = finalTime / 1e6;

      Serial.println("\n=== Contribución Promedio al total del sistema ===");
      Serial.printf("=== Tiempo de muestreado: %6.0lf% (segundos) ===\n", samplingTime);

      printCpuAverageStats();
      printMemoryAverageStats();

      Serial.println("Muestreo de Metricas finalizado");

      gStatsTask = nullptr;
      vTaskDelete(nullptr);
    }
  }
}

// ------------------ APIS ------------------
void initStats() {
#if (configGENERATE_RUN_TIME_STATS == 1)

  Serial.printf("\nMuestreo de Metricas Iniciado...\n");

  taskENTER_CRITICAL(&sMux);
  bool already = (gStatsTask != nullptr);
  taskEXIT_CRITICAL(&sMux);
  if (already) return;

  // Reseteo los variables utilizadas para el promedio
  for (int i = 0; i < 2; i++) {
    pctCoreTotal[i] = 0;
    pctIdleTotal[i] = 0;
    pctBusyTotal[i] = 0;
    sumCoreTotal[i] = 0;
    sumIdleTotal[i] = 0;
    sumBusyTotal[i] = 0;
  }
  sumHeapTotal = 0; 
  sumHeapLibre = 0;
  sumHeapUsado = 0;
  gSamples = 0;

  //creo la tarea
  BaseType_t ok = xTaskCreatePinnedToCore(StatsTask, "StatsTask", 4096, nullptr, 1, &gStatsTask, 1);

  if (ok != pdPASS) {
    Serial.println("[Stats] No se pudo crear la tarea StatsTask");

    taskENTER_CRITICAL(&sMux);
    gStatsTask = nullptr;
    taskEXIT_CRITICAL(&sMux);
  }
#else
  Serial.println("\n=== Stats de CPU no disponibles ===");
  Serial.println("Habilitá configGENERATE_RUN_TIME_STATS=1 en FreeRTOSConfig.h");
#endif
}

void finishStats() {
  TaskHandle_t h;

  taskENTER_CRITICAL(&sMux);
  h = gStatsTask;
  taskEXIT_CRITICAL(&sMux);

  if (!h)
    return;
  
  //le notifico a la tarea que finalice saliendo del loop
  xTaskNotifyGive(h);
}