#include "RTPiDrone_DataExchange.h"
#include "Common.h"
#include <string.h>
#include <math.h>
#include <pthread.h>
#define LINESIZE        512
#define LINETEMPSIZE    128

typedef struct __DataChain {
    Drone_DataExchange data;
    struct __DataChain *next;
} DataChain;

static char LINE[LINESIZE], LINETEMP[LINETEMPSIZE];
static float T_temp;
static DataChain *dataStart, *dataEnd;
static pthread_mutex_t  mutex;
static pthread_cond_t   cond;
static pthread_t        pid;
static int              global_thread;
static int              iStop;
static void* writeData(void*);

int Drone_DataExchange_Init(Drone_DataExchange** data, FILE* f)
{
    *data = (Drone_DataExchange*)calloc(1, sizeof(Drone_DataExchange));
    dataStart = (DataChain*) calloc(1, sizeof(DataChain));
    dataEnd = dataStart;
    pthread_cond_init(&cond,NULL);
    pthread_mutex_init(&mutex,NULL);
    pthread_create(&pid, NULL, writeData, (void*)f);
    return 0;
}

void Drone_DataExchange_End(Drone_DataExchange** data)
{
    iStop = -1;
    pthread_cond_signal(&cond);
    pthread_join(pid,NULL);
    pthread_cond_destroy(&cond);
    pthread_mutex_destroy(&mutex);
    free(dataStart);
    dataStart = dataEnd = NULL;
    free(*data);
    data = NULL;
}

void Drone_DataExchange_Print(Drone_DataExchange* data)
{
    printf("Acc: %f, %f, %f\n", data->acc[0], data->acc[1], data->acc[2]);
    printf("Gyr: %f, %f, %f\n", data->gyr[0], data->gyr[1], data->gyr[2]);
    printf("Mag: %f, %f, %f\n", data->mag[0], data->mag[1], data->mag[2]);
    printf("Attitide: %f, temperature: %f, pressure: %f\n", data->attitude, data->temperature, data->pressure);
    printf("dt: %f\n", data->dt);
}

void Drone_DataExchange_PrintAngle(Drone_DataExchange* data)
{
    printf("T = %f, Roll: %f, Pitch: %f, Yaw: %f\n", data->T, data->angle[0], data->angle[1], data->angle[2]);
}

void Drone_DataExchange_PrintTextFile(Drone_DataExchange* data, FILE *fp)
{
    sprintf(LINE, "%f\t%f\t%f\t%f\t", data->T, data->dt, data->T-T_temp, data->dt_accu);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->angle[0], data->angle[1], data->angle[2]);
    strcat(LINE, LINETEMP);

    float norm = getSqrt(data->acc, 3);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->acc[0]/norm, data->acc[1]/norm, data->acc[2]/norm);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->gyr[0], data->gyr[1], data->gyr[2]);
    strcat(LINE, LINETEMP);

    norm = getSqrt(data->mag, 3);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->mag[0]/norm, data->mag[1]/norm, data->mag[2]/norm);
    strcat(LINE, LINETEMP);

    norm = getSqrt(data->acc_est, 3);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->acc_est[0]/norm, data->acc_est[1]/norm, data->acc_est[2]/norm);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->gyr_est[0], data->gyr_est[1], data->gyr_est[2]);
    strcat(LINE, LINETEMP);

    norm = getSqrt(data->mag_est, 3);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->mag_est[0]/norm, data->mag_est[1]/norm, data->mag_est[2]/norm);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%f\t%f\t", data->attitude, data->att_est);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%f\t%f\t", data->attitudeHT, data->attHT_est);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%u\t", data->comm.power);
    strcat(LINE, LINETEMP);
    sprintf(LINETEMP, "%f\t%f\t%f\t", data->comm.angle_expect[0], data->comm.angle_expect[1], data->comm.angle_expect[2]);
    strcat(LINE, LINETEMP);
    for (int i=0; i<4; ++i) {
        sprintf(LINETEMP, "%d\t", data->comm.control[i]);
        strcat(LINE, LINETEMP);
    }
    //sprintf(LINETEMP, "%f\t%f\t%f\t%f\t", data->comm.control[0], data->comm.control[1], data->comm.control[2], data->comm.control[3]);
    sprintf(LINETEMP, "%u\t%u\t%u\t%u\t", data->power[0], data->power[1], data->power[2], data->power[3]);
    strcat(LINE, LINETEMP);
    fprintf(fp, "%s\n", LINE);
    T_temp = data->T ;
}

void Drone_DataExchange_SaveFile(Drone_DataExchange* data)
{
    dataEnd->data = *data;
    dataEnd->next = (DataChain*) calloc(1, sizeof(DataChain));
    dataEnd = dataEnd->next;
    global_thread = 1;
    pthread_cond_signal(&cond);
}

void Drone_DataExchange_PrintFile(Drone_DataExchange* data, FILE *fp)
{
    fwrite(data, sizeof(Drone_DataExchange), 1, fp);
}

static void* writeData(void* fp)
{
    FILE* f = (FILE*)fp;
    while (!iStop) {
        pthread_mutex_lock(&mutex);
        if (!global_thread) pthread_cond_wait(&cond, &mutex);
    
        while (dataStart != dataEnd) {
            Drone_DataExchange_PrintFile(&dataStart->data, f);
            DataChain *dataTemp = dataStart->next;
            free(dataStart);
            dataStart = dataTemp;        
        }
        global_thread = 0;
        pthread_mutex_unlock(&mutex);
    }
    fflush(f);
    pthread_exit(NULL);
}

