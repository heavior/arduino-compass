#ifndef STORAGE_H
#define STORAGE_H

#define NANO33BLE_FS_SIZE_KB        256 // 512 kb of data per file 

#include <FS_Nano33BLE.h> 
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "compassData.pb.h"

#define MAX_MAP_POINTS (NANO33BLE_FS_SIZE_KB * 1024 / (compass_MapPoint_size + 20))
// checking how many points can we support in a file. not a presice estimation, but will do for now
// roughly - 1000 points, can reduce name length to accomodate more. 
// can also store numbers more efficiently

FileSystem_MBED *myFS; 

void storage_begin(){
  myFS = new FileSystem_MBED(); 
  if (!myFS->init()) 
  { 
    Serial.println("FS Mount Failed"); 
  } 
}

void write_map_points(const char *filename, compass_MapPoint* points, size_t num_points) {
    // Open the file for writing
    FILE *file = fopen(filename, "w");

    if(!file){
      Serial.println("can't save file"); 
      return;
    }

    // Write the CSV header
    char header[] = "id,name,latitude,longitude,radius,visited\n";
    fwrite(header, 1, sizeof(header)-1,file);
    if(num_points > num_points > MAX_MAP_POINTS){ 
      num_points = MAX_MAP_POINTS;
    }
    // Write each point to the file
    for (size_t i = 0; i < num_points; ++i) {
        char buffer[512];
        int visited = points[i].visited ? 1 : 0;  // Convert bool to int
        snprintf(buffer, sizeof(buffer), "%u,%s,%lf,%lf,%u,%d\n",
            points[i].id, points[i].name, points[i].coordinates.latitude, 
            points[i].coordinates.longitude, points[i].radius, visited);

        fwrite(buffer, 1, strlen(buffer),file);
    }

    // Close the file
    fclose(file);
}
compass_MapPoint* read_map_points(const char *filename, size_t *num_points) {
    // Open the file for reading
    FILE *file = fopen(filename, "r");
    if(!file){
      return NULL;
    }

    // Determine the number of lines in the file (ignoring the header)
    *num_points = 0;
    while (!feof(file)) {
        char c = fgetc(file);
        if (c == '\n') {
            (*num_points)++;
        }
    }
    (*num_points)--;  // Subtract one for the header line

    // Rewind the file and skip the header line
    fseek(file,0,SEEK_SET);
    char header[256];
    fgets(header, sizeof(header),file);

    // Allocate memory for the points
    compass_MapPoint* points = (compass_MapPoint*)malloc(*num_points * sizeof(compass_MapPoint));

    // Read each line from the file
    for (size_t i = 0; i < *num_points; ++i) {
        char line[512];
        fgets(line, sizeof(line),file);
        // Parse the line into a compass_MapPoint
        sscanf(line, "%u,%255[^,],%lf,%lf,%u,%d\n",
            &points[i].id, points[i].name, &points[i].coordinates.latitude, 
            &points[i].coordinates.longitude, &points[i].radius, (int*)&points[i].visited);
    }

    // Close the file
    fclose(file);

    return points;
}

  

#endif