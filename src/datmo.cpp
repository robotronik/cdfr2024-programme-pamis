#include "datmo.h"

uint16_t minValue(VL53L7CX_ResultsData* Results){
    int zones_per_line = (RES == VL53L7CX_RESOLUTION_8X8) ? 8 : 4;

    uint16_t minDistance = 10000;

    for (int c = 0; c<RES; c++){

          //On prend en compte uniquement les zones où le mesure est valide (status = 5 ou 9)
          uint8_t zoneStatus = Results->target_status[c];
          if (zoneStatus == 5 || zoneStatus == 9){
            uint16_t distance = Results->distance_mm[c];
            if (distance < minDistance){
              minDistance = distance;
            }
          }
    }
    return minDistance;
}

void perfromClustering(VL53L7CX_ResultsData* Results, uint16_t cluster_threshold){

    int zones_per_line = (RES == VL53L7CX_RESOLUTION_8X8) ? 8 : 4;
    int clusters[RES][RES];
    int clusterSize[RES];

    int nb_clusters = 0;

    for (int c = 0; c<RES; c++){

        int cluster_size = 0;
        
        //On prend en compte uniquement les zones où le mesure est valide (status = 5 ou 9)
        uint8_t zoneStatus = Results->target_status[c];
        if (zoneStatus == 5 || zoneStatus == 9){
            uint16_t distance = Results->distance_mm[c] - Results->distance_mm[c + 1];
            if (distance > cluster_threshold){
             nb_clusters++;
            }
            else{
                clusterSize[nb_clusters]++;
            }
            clusters[nb_clusters][cluster_size] = c;
        }
    }
      
    
}