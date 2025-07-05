
#include <stdint.h>
#include "i2c.h"

#define FUNCTION_ACCEL 1
#define FUNCTION_GYRO 2
#define FUNCTION_MAG 3
#define FUNCTION_NONE 0

// BMX055 structure
typedef struct
{
    float Ax;
    float Ay;
    float Az;

    float Gx;
    float Gy;
    float Gz;

    float Mx;
    float My;
    float Mz;

    float Temperature;
    float Temperature_mag;
} BMX055_t;

extern volatile uint8_t CurrentFunction; // Объявление переменной

uint8_t BMX055_BMA_Init(I2C_HandleTypeDef *I2Cx);

void setFastOffset_BMA(I2C_HandleTypeDef *I2Cx);

void BMX055_Read_Accel(I2C_HandleTypeDef *I2Cx, BMX055_t *DataStruct);

uint8_t BMX055_BMG_Init(I2C_HandleTypeDef *I2Cx);

void setFastOffset_BMG(I2C_HandleTypeDef *I2Cx);

void BMX055_Read_Gyro(I2C_HandleTypeDef *I2Cx, BMX055_t *DataStruct);

uint8_t BMX055_BMM_Init(I2C_HandleTypeDef *I2Cx);

void readADC(I2C_HandleTypeDef *I2Cx);

void setFastOffset_BMM(I2C_HandleTypeDef *I2Cx, uint8_t r);

void BMX055_Read_Mag(I2C_HandleTypeDef *I2Cx, BMX055_t *DataStruct);

void BMX055_Read_Accel_DMA(I2C_HandleTypeDef *I2Cx, uint8_t *data);


void BMX055_Read_Gyro_DMA(I2C_HandleTypeDef *I2Cx, uint8_t *data);

void BMX055_Read_Mag_DMA(I2C_HandleTypeDef *I2Cx, float *data[4]);
void readADC_dma(I2C_HandleTypeDef *I2Cx);

