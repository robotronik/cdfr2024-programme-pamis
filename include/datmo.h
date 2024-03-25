#include <vl53l7cx_class.h>
#include <Arduino.h>

#define RES VL53L7CX_RESOLUTION_4X4

#define FREQUENCY_HZ 60
#define THRESHOLD 10

uint16_t minValue(VL53L7CX_ResultsData* Results);
void performClustering(VL53L7CX_ResultsData* Results, uint16_t cluster_threshold);