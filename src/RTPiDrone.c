#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include "RTPiDrone.h"
#define LENGTH 128

typedef enum {
    vanilla,
    preemptRT,
    xenomai
} kernelType;

struct str_drone{
//    bool    debugMode;
    char    logfileName[LENGTH];
};

static void getTimeString(char*);
static int getKernelString(char*);
static kernelType getKernelType(char*);
static int generateFileName(char*);

int Drone_init(Drone** rpiDrone) {
/*
#ifdef DEBUGMODE
    rpiDrone->debugMode = true;
#else
    rpiDrone->debugMode = false;
#endif
*/
    *rpiDrone = (Drone*) malloc(sizeof(Drone));
    if (generateFileName((*rpiDrone)->logfileName)) {
        perror("Cannot decide log file Name");
    }

//    if (rpiDrone->debugMode) {
        printf("%s\n", (*rpiDrone)->logfileName);
//    }

    return 0;
}

int Drone_Calibration(Drone* rpiDrone){
    return 0;
}

void Drone_Start(Drone* rpiDrone){
    puts("Start Test");
}

int Drone_End(Drone** rpiDrone){
    free(*rpiDrone);
    *rpiDrone = NULL;
    puts("End Test");
    
    return 0;
}

static void getTimeString(char* timeStr){
    time_t  timer = time(NULL);
    struct tm *ptr=localtime(&timer);
    strftime(timeStr,LENGTH,"%y%m%d_%H%M%S",ptr);
}

static int getKernelString(char* kStr) {
    FILE *fkernel = fopen("/proc/version", "r");
    if (!fkernel) {
        perror("fopen( \"/proc/version\", \"r\" )");
        return -1;
    }
    char buf[LENGTH];
    if (!fgets(buf, LENGTH, fkernel)) {
        perror("fgets error!");
        return -2;
    }
    sscanf(buf, "Linux version %s (", kStr);
    fclose(fkernel);
    return 0;
}

static kernelType getKernelType(char* kStr){
    if (strstr(kStr, "rt")) return preemptRT;
    if (strstr(kStr, "ipipe")) return xenomai;
    return vanilla;
}

static int generateFileName(char* fileName){
    char kStr[LENGTH], timeStr[LENGTH]; 
    getTimeString(timeStr);
    if (getKernelString(kStr)) return -1;
    kernelType kType = getKernelType(kStr);
    const char head[] = "VPX";
    strcpy(fileName, &head[kType]);
    strcpy(fileName+1, kStr);
    strcat(fileName, "__");
    strcat(fileName, timeStr);
    strcat(fileName, ".log");

    return 0;
}
