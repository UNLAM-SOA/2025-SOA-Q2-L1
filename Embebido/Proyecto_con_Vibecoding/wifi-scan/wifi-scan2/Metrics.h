#ifndef METRICS_H
#define METRICS_H

//habilitar esta variable para ver las estadisticas intermedias
#define SHOW_INTERMEDIATE_STATS false

//Tiempo en milisegundos que indica cada cuanto se realiza el muestreo y las impresiones de las metricas
#define TIME_BETWEEN_PRINTS_TASK 300

// APIs públicas
void initStats();
void finishStats();

#endif // METRICS_H

