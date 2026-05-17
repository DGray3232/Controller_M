
#include <math.h>
#include "bmx055.h"
#include "gpio.h"

//	АДРЕС И ID ЧИПА Акселерометра:						
#define	BMA_ADDRES 0x19 << 1	// Адрес датчика на шине I2C
#define	BMA_ID 0xFA	// ID датчика Акселерометра
//	РЕГИСТРЫ ЧИПА Акселерометра:							
#define	REG_BMA_CHIPID 0x00	// ID чипа	
#define REG_BMA_RESET 0x14 // Программная перезагрузка
#define REG_BMA_LPW	0x11 //	Выбор основных режимов питания	
#define REG_BMA_RANGE 0x0F // Диапазон измеряемого углового ускорения	
#define REG_BMA_BW 0x10	// Полоса пропускания (Bandwidths)
#define REG_BMA_D_HBW 0x13 // Выбор типа данных и их затенение
#define REG_BMA_OFC_CTRL 0x36 // Управлен. вычислением компенс. смещения
#define REG_BMA_OFC_SETTING	0x37 // Настройки вычисления компенс. смещения
#define REG_BMA_X_LSB 0x02 // Данные углового ускорения по оси X		
#define REG_BMA_X_MSB 0x03 // Данные углового ускорения по оси X		
#define REG_BMA_Y_LSB 0x04 // Данные углового ускорения по оси Y	
#define REG_BMA_Y_MSB 0x05 // Данные углового ускорения по оси Y	
#define REG_BMA_Z_LSB 0x06 // Данные углового ускорения по оси Z	
#define REG_BMA_Z_MSB 0x07 // Данные углового ускорения по оси Z	
#define REG_BMA_TEMP 0x08 // Данные температуры чипа	в °C			


//	АДРЕС И ID ЧИПА Гироскопа:					
#define	BMG_ADDRES 0x69 << 1 // Адрес датчика на шине I2C
#define	BMG_ID 0x0F // ID датчика
//	РЕГИСТРЫ ЧИПА Гироскопа:									
#define	REG_BMG_CHIPID 0x00 // ID чипа
#define REG_BMG_RESET 0x14 // Программная перезагрузка
#define REG_BMG_LPM1 0x11 // Выбор основных режимов питания
#define REG_BMG_RANGE 0x0F // Диапазон измеряемой угловой скорости
#define REG_BMG_BW 0x10 // Полоса пропускания (Bandwidths)
#define REG_BMG_D_HBW 0x13 // Выбор типа данных и их затенение
#define REG_BMG_AOFS_FOFS 0x32 // Настр-ки авто/быстр компенсации смещения
#define	REG_BMG_X_LSB 0x02 // Данные угловой скорости по оси X


//	АДРЕС И ID ЧИПА МАГНИТОМЕТРА:
#define	BMM_ADDRES 0x13 << 1  // Адрес датчика на шине I2C
#define	BMM_ID 0x32 // ID датчика
//	РЕГИСТРЫ ЧИПА МАГНИТОМЕТРА:
#define	REG_BMM_CHIPID 0x40 // ID чипа
#define	REG_BMM_CTRL_0 0x4B // Управление питанием и перезагрузка
#define	REG_BMM_CTRL_1 0x4C // Режимы работы, скорость и тестирование
#define	REG_BMM_REP_XY 0x51 // Количество выборок для осей XY
#define	REG_BMM_REP_Z 0x52 // Количество выборок для оси  Z
#define REG_BMM_DIG_X1 0x5D // Значения корректировки
#define REG_BMM_DIG_Y1 0x5E // Значения корректировки
#define REG_BMM_DIG_Z4_LSB 0x62 // Значения корректировки
#define REG_BMM_DIG_Z4_MSB 0x63 //	Значения корректировки
#define REG_BMM_DIG_X2 0x64 // Значения корректировки
#define REG_BMM_DIG_Y2 0x65 // Значения корректировки
#define REG_BMM_DIG_Z2_LSB 0x68 // Значения корректировки
#define REG_BMM_DIG_Z2_MSB 0x69 // Значения корректировки
#define REG_BMM_DIG_Z1_LSB 0x6A // Значения корректировки
#define REG_BMM_DIG_Z1_MSB 0x6B // Значения корректировки
#define REG_BMM_DIG_XYZ1_LSB 0x6C // Значения корректировки
#define REG_BMM_DIG_XYZ1_MSB 0x6D // Значения корректировки
#define REG_BMM_DIG_Z3_LSB 0x6E // Значения корректировки
#define REG_BMM_DIG_Z3_MSB 0x6F // Значения корректировки
#define REG_BMM_DIG_XY2 0x70 // Значения корректировки
#define REG_BMM_DIG_XY1	0x71 //	Значения корректировки
#define	REG_BMM_X_LSB 0x42 // Данные магнитного поля по оси X

uint8_t dig_x1; //	Объявляем переменную для хранеия корректировочного значения
uint8_t dig_y1; //	Объявляем переменную для хранеия корректировочного значения
uint8_t dig_x2; //	Объявляем переменную для хранеия корректировочного значения
uint8_t dig_y2; //	Объявляем переменную для хранеия корректировочного значения
uint16_t dig_z1; //	Объявляем переменную для хранеия корректировочного значения
uint16_t dig_z2;	//	Объявляем переменную для хранеия корректировочного значения
uint16_t dig_z3;	// 	Объявляем переменную для хранеия корректировочного значения
uint16_t dig_z4;	//	Объявляем переменную для хранеия корректировочного значения
uint8_t	dig_xy1; //	Объявляем переменную для хранеия корректировочного значения
uint8_t dig_xy2;	 //	Объявляем переменную для хранеия корректировочного значения
uint16_t dig_xyz1; //	Объявляем переменную для хранеия корректировочного значения
uint8_t mag_adc[4]={0,0,0,0}; // Определяем массив для хранения прочитанных показаний АЦП по осям XYZ и температуры
float magBias[3]={0,0,0}; // Определяем массив для хранения смещения показаний магнитометра в мГс для осей XYZ на
float varQuantum =	1.0/1.6; // 1.0/1.6 мГс  = 1.0/16.0 мкТл  Определяем переменную для хранения шага квантования
const uint16_t i2c_timeout_bmx = 500;


volatile uint8_t CurrentFunction = FUNCTION_NONE; // Определение переменной

uint8_t BMX055_BMA_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check = 0;;
    uint8_t Data;
    uint8_t i = 0;

    while(check != BMA_ID){
          HAL_I2C_Mem_Read(I2Cx, BMA_ADDRES, REG_BMA_CHIPID, 1, &check, 1, i2c_timeout_bmx);
          HAL_Delay(100);
    	   i++;
    	   if(i>10){
    		  return 1;
    	   }
    }
    //HAL_I2C_Mem_Read(I2Cx, BMA_ADDRES, REG_BMA_CHIPID, 1, &check, 1, i2c_timeout_bmx);
    //HAL_Delay(100);
    if (check == BMA_ID)
    {
        Data = 0xB6;
        HAL_I2C_Mem_Write(I2Cx, BMA_ADDRES, REG_BMA_RESET, 1, &Data, 1, i2c_timeout_bmx); // Выполняем программную перезагрузку (записываем значение 0xB6 в регистр REG_BMA_RESET)
        HAL_Delay(50);
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, BMA_ADDRES, REG_BMA_LPW, 1, &Data, 1, i2c_timeout_bmx); // Переводим акселеромерт в нормальный режим питания (сбрасываем флаги suspend, lowpower_en и deep_suspend)
        HAL_Delay(50);
        Data = 0x03; // 2G=0x03 2.0/2048.0, 4G=0x05 4.0/2048.0, 8G=0x08 8.0/2048.0, 16G=0x0C 16.0/2048.0
        HAL_I2C_Mem_Write(I2Cx, BMA_ADDRES, REG_BMA_RANGE, 1, &Data, 1, i2c_timeout_bmx); // Устанавливаем диапазон измерений акселерометра (указывая предел измеряемого углового ускорения)
        HAL_Delay(50);
        Data = 0x0D; // 16Hz=0x09, 31Hz=0x0A, 63Hz=0x0B, 125Hz=0x0C, 250Hz=0x0D, 500Hz=0x0E, 1000Hz=0x0F
        HAL_I2C_Mem_Write(I2Cx, BMA_ADDRES, REG_BMA_BW, 1, &Data, 1, i2c_timeout_bmx); // Устанавливаем пропускную способность акселерометра (указывая частоту обновления фильтрованных данных)
        HAL_Delay(50);
        Data = 0x00;                                                                            
        HAL_I2C_Mem_Write(I2Cx, BMA_ADDRES, REG_BMA_D_HBW, 1, &Data, 1, i2c_timeout_bmx);  // Указываем акселерометру, что требуется выводить отфильтрованные данные (dataHigh_bw=0)
                                                                                           // с механизмом их затенения во время чтения (shadow_dis=0).
                                                                                           // Затенение осначает что значение регистров данных не будет изменяться от начала и до конца пакетного чтения
        return 0;
    }
    return 1;
}

// Выполнение быстрой компенсации смещения данных:
void setFastOffset_BMA(I2C_HandleTypeDef *I2Cx) { 

    uint8_t Data;
    uint8_t i;

    // Сбрасываем все значения регистров компенсаций смещения в 0 (установкой флага offset\_reset)
    Data = 0x80; 
    HAL_I2C_Mem_Write(I2Cx, BMA_ADDRES, REG_BMA_OFC_CTRL, 1, &Data, 1, i2c_timeout_bmx);
    // Устанавливаем значения компенсации смещения в X=0g, Y=0g, Z=+1g
    Data = 0x20; 
    HAL_I2C_Mem_Write(I2Cx, BMA_ADDRES, REG_BMA_OFC_SETTING, 1, &Data, 1, i2c_timeout_bmx);
    // Вычисляем смещение для оси X
    Data = 0x20; 
    HAL_I2C_Mem_Write(I2Cx, BMA_ADDRES, REG_BMA_OFC_CTRL, 1, &Data, 1, i2c_timeout_bmx);
    do {
        HAL_I2C_Mem_Read(I2Cx, BMA_ADDRES, REG_BMA_OFC_CTRL, 1, &i, 1, i2c_timeout_bmx);
        HAL_Delay(50);
    } while ((i & 0x10) == 0);
    // Вычисляем смещение для оси Y
    Data = 0x40; 
    HAL_I2C_Mem_Write(I2Cx, BMA_ADDRES, REG_BMA_OFC_CTRL, 1, &Data, 1, i2c_timeout_bmx);
    do {
        HAL_I2C_Mem_Read(I2Cx, BMA_ADDRES, REG_BMA_OFC_CTRL, 1, &i, 1, i2c_timeout_bmx);
        HAL_Delay(50);
    } while ((i & 0x10) == 0);
    // Вычисляем смещение для оси Z
    Data = 0x60; 
    HAL_I2C_Mem_Write(I2Cx, BMA_ADDRES, REG_BMA_OFC_CTRL, 1, &Data, 1, i2c_timeout_bmx);
    do {
        HAL_I2C_Mem_Read(I2Cx, BMA_ADDRES, REG_BMA_OFC_CTRL, 1, &i, 1, i2c_timeout_bmx);
        HAL_Delay(50);
    } while ((i & 0x10) == 0);
}

void BMX055_Read_Accel(I2C_HandleTypeDef *I2Cx, BMX055_t *DataStruct)
{
    uint8_t Rec_Data[7];
    HAL_I2C_Mem_Read(I2Cx, BMA_ADDRES, REG_BMA_X_LSB, 1, Rec_Data, 7, i2c_timeout_bmx);

    DataStruct->Ax = (float)((int16_t)(((uint16_t)Rec_Data[1]<<8) | (uint16_t)Rec_Data[0])>>4) * (2.0/2048.0);
    DataStruct->Ay = (float)((int16_t)(((uint16_t)Rec_Data[3]<<8) | (uint16_t)Rec_Data[2])>>4) * (2.0/2048.0);
    DataStruct->Az = (float)((int16_t)(((uint16_t)Rec_Data[5]<<8) | (uint16_t)Rec_Data[4])>>4) * (2.0/2048.0);
    DataStruct->Temperature  = (float)((int16_t)((uint16_t)Rec_Data[6]<<8)>>8) * 0.5 + 23.0;
}
HAL_StatusTypeDef BMX055_Read_Accel_DMA(I2C_HandleTypeDef *I2Cx, uint8_t *data)
{
    CurrentFunction = FUNCTION_ACCEL;
    return HAL_I2C_Mem_Read_DMA(I2Cx, BMA_ADDRES, REG_BMA_X_LSB, 1, data, 7);
}

uint8_t BMX055_BMG_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check = 0;;
    uint8_t Data;
    uint8_t i = 0;

    while(check != BMG_ID){
          HAL_I2C_Mem_Read(I2Cx, BMG_ADDRES, REG_BMG_CHIPID, 1, &check, 1, i2c_timeout_bmx);
          HAL_Delay(50);
    	   i++;
    	   if(i>10){
    		  return 1;
    	   }
    }
    //HAL_I2C_Mem_Read(I2Cx, BMG_ADDRES, REG_BMG_CHIPID, 1, &check, 1, i2c_timeout_bmx);
    //HAL_Delay(50);
    if (check == BMG_ID)
    {
        Data = 0xB6;
        HAL_I2C_Mem_Write(I2Cx, BMG_ADDRES, REG_BMG_RESET, 1, &Data, 1, i2c_timeout_bmx); // Выполняем программную перезагрузку (записываем значение 0xB6 в регистр REG_BMG_RESET)
        HAL_Delay(50);
        Data = 0x00;
        while (HAL_I2C_Mem_Write(I2Cx, BMG_ADDRES, REG_BMG_LPM1, 1, &Data, 1, i2c_timeout_bmx) != HAL_OK);
        //HAL_I2C_Mem_Write(I2Cx, BMG_ADDRES, REG_BMG_LPM1, 1, &Data, 1, i2c_timeout_bmx); //	Переводим гироскоп в нормальный режим питания (сбрасываем флаги suspend и deep_suspend)
        HAL_Delay(50);
        Data = 0x00; // 125DPS=0x04 124.87/32768.0, 250DPS=0x03 249.75/32768.0, 500DPS=0x02 499.5 /32768.0, 1000DPS=0x01 999.0 /32768.0, 2000DPS=0x00 1998.0/32768.0
        HAL_I2C_Mem_Write(I2Cx, BMG_ADDRES, REG_BMG_RANGE, 1, &Data, 1, i2c_timeout_bmx); // Устанавливаем диапазон измерений гироскопа (указывая предел измеряемой угловой скорости)
        HAL_Delay(50);
        Data = 0x00; // 23Hz=0x04, 12Hz=0x05, 32Hz=0x07, 47Hz=0x03, 64Hz=0x06, 116Hz=0x02, 230Hz=0x01, 523Hz=0x00
        HAL_I2C_Mem_Write(I2Cx, BMG_ADDRES, REG_BMG_BW, 1, &Data, 1, i2c_timeout_bmx); // Устанавливаем пропускную способность гироскопа (указывая частоту обновления фильтрованных данных)
        HAL_Delay(50);
        Data = 0x00;                                                                            
        HAL_I2C_Mem_Write(I2Cx, BMG_ADDRES, REG_BMG_D_HBW, 1, &Data, 1, i2c_timeout_bmx); // Указываем гироскопу, что требуется выводить отфильтрованные данные (dataHigh_bw=0)
                                                                                          // с механизмом их затенения во время чтения (shadow_dis=0).
                                                                                          // Затенение осначает что значение регистров данных не будет изменяться от начала и до конца пакетного чтения
        return 0;
    }
    return 1;
}

// Выполнение быстрой компенсации смещения данных:
void setFastOffset_BMG(I2C_HandleTypeDef *I2Cx) { 

    uint8_t Data;
    uint8_t check;

    // Выполняем быструю компенсацию смещения для всех осей,
    // установив все биты регистра REG_BMG_AOFS_FOFS в «1»
    Data = 0xFF; 
    HAL_I2C_Mem_Write(I2Cx, BMG_ADDRES, REG_BMG_AOFS_FOFS, 1, &Data, 1, i2c_timeout_bmx);
    // Ждём завершения быстрой компенсации смещения
    // (ждём пока сбросится флаг FOffsetEn)
    do {
        HAL_I2C_Mem_Read(I2Cx, BMG_ADDRES, REG_BMG_AOFS_FOFS, 1, &check, 1, i2c_timeout_bmx);
    } while ((check & 0x08) != 0);
}

void BMX055_Read_Gyro(I2C_HandleTypeDef *I2Cx, BMX055_t *DataStruct)
{
    uint8_t Rec_Data[6];
    HAL_I2C_Mem_Read(I2Cx, BMG_ADDRES, REG_BMG_X_LSB, 1, Rec_Data, 6, i2c_timeout_bmx);

    DataStruct->Gx = (float)((int16_t)(((uint16_t)Rec_Data[1]<<8) | (uint16_t)Rec_Data[0])) * (1998.0/32768.0) * 1.0f;
    DataStruct->Gy = (float)((int16_t)(((uint16_t)Rec_Data[3]<<8) | (uint16_t)Rec_Data[2])) * (1998.0/32768.0) * 1.0f;
    DataStruct->Gz = (float)((int16_t)(((uint16_t)Rec_Data[5]<<8) | (uint16_t)Rec_Data[4])) * (1998.0/32768.0) * 1.0f;
}
HAL_StatusTypeDef BMX055_Read_Gyro_DMA(I2C_HandleTypeDef *I2Cx, uint8_t *data)
{
    CurrentFunction = FUNCTION_GYRO;
    return HAL_I2C_Mem_Read_DMA(I2Cx, BMG_ADDRES, REG_BMG_X_LSB, 1, data, 6);
}

uint8_t BMX055_BMM_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check = 0;;
    uint8_t Data;
    uint8_t j[2];
    uint8_t i = 0;
// Выходим из режима приостановки (suspend) и устанавливаем рабочие параметры
 while(check != BMM_ID){
	   HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_CHIPID, 1, &check, 1, i2c_timeout_bmx);
	   Data = 0x82;
	   HAL_I2C_Mem_Write(I2Cx, BMM_ADDRES, REG_BMM_CTRL_0, 1, &Data, 1, i2c_timeout_bmx); // Выполняем программную перезагрузку установив оба бита softreset<1:0> регистра REG_BMM_CTRL_0
	   HAL_Delay(100);
	   Data = 0x01;
	   HAL_I2C_Mem_Write(I2Cx, BMM_ADDRES, REG_BMM_CTRL_0, 1, &Data, 1, i2c_timeout_bmx); // Выходим из режима приостановки (suspend) в спящий (Sleep) режим, установив бит pwr_control регистра REG_BMM_CTRL_0
	   HAL_Delay(100);;
	   i++;
	   if(i>10){
		  return 1;
	   }
 }
        Data = 0x07 << 3; // 10Hz=0x00 2Hz=0x01 6Hz=0x02 8Hz=0x03 15Hz=0x04 20Hz=0x05 25Hz=0x06 30Hz=0x07
        HAL_I2C_Mem_Write(I2Cx, BMM_ADDRES, REG_BMM_CTRL_1, 1, &Data, 1, i2c_timeout_bmx); // Установка полосы пропускания для фильтрованных данных:
        HAL_Delay(50);
        Data = 3; // xy=3 10Hz xy=9 10Hz xy=15 10Hz xy=47 20Hz
        Data  = (Data-1)/2;
        HAL_I2C_Mem_Write(I2Cx, BMM_ADDRES, REG_BMM_REP_XY, 1, &Data, 1, i2c_timeout_bmx); // Устанавливаем количество выборок по осям XY
        HAL_Delay(50);
        Data = 3; // z=3 10Hz z=15 10Hz z=27 10Hz z=83 20Hz
        Data  = (Data-1);
        HAL_I2C_Mem_Write(I2Cx, BMM_ADDRES, REG_BMM_REP_Z, 1, &Data, 1, i2c_timeout_bmx); // Устанавливаем количество выборок по оси Z
        HAL_Delay(50);
        //	Читаем корректировочные значения
        HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_DIG_X1, 1, &dig_x1, 1, i2c_timeout_bmx); // Читаем один байт корректировочных значений из регистра  REG_BMM_DIG_X1
        HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_DIG_X2, 1, &dig_x2, 1, i2c_timeout_bmx); // Читаем один байт корректировочных значений из регистра  REG_BMM_DIG_X2
        HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_DIG_Y1, 1, &dig_y1, 1, i2c_timeout_bmx); // Читаем один байт корректировочных значений из регистра  REG_BMM_DIG_Y1
        HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_DIG_Y2, 1, &dig_y2, 1, i2c_timeout_bmx);// Читаем один байт корректировочных значений из регистра  REG_BMM_DIG_Y2
        HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_DIG_XY1, 1, &dig_xy1, 1, i2c_timeout_bmx);//	Читаем один байт корректировочных значений из регистра  REG_BMM_DIG_XY1
        HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_DIG_XY2, 1, &dig_xy2, 1, i2c_timeout_bmx);//	Читаем один байт корректировочных значений из регистра  REG_BMM_DIG_XY2

        HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_DIG_Z1_LSB, 1, j, 2, i2c_timeout_bmx); // Читаем два байта корректировочных значений из регистров REG_BMM_DIG_Z1_LSB и REG_BMM_DIG_Z1_MSB
        dig_z1   = ((uint16_t)j[1]<<8) | j[0];
        HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_DIG_Z2_LSB, 1, j, 2, i2c_timeout_bmx); // Читаем два байта корректировочных значений из регистров REG_BMM_DIG_Z2_LSB и REG_BMM_DIG_Z2_MSB
        dig_z2   = ((uint16_t)j[1]<<8) | j[0];
        HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_DIG_Z3_LSB, 1, j, 2, i2c_timeout_bmx); // Читаем два байта корректировочных значений из регистров REG_BMM_DIG_Z3_LSB и REG_BMM_DIG_Z3_MSB
        dig_z3   = ((uint16_t)j[1]<<8) | j[0];
        HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_DIG_Z4_LSB, 1, j, 2, i2c_timeout_bmx); // Читаем два байта корректировочных значений из регистров REG_BMM_DIG_Z4_LSB и REG_BMM_DIG_Z4_MSB
        dig_z4   = ((uint16_t)j[1]<<8) | j[0];
        HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_DIG_XYZ1_LSB, 1, j, 2, i2c_timeout_bmx); //	Читаем два байта корректировочных значений из регистров REG_BMM_DIG_XYZ1_LSB и REG_BMM_DIG_XYZ1_MSB
        dig_xyz1   = ((uint16_t)j[1]<<8) | j[0];
    return 0;
}

//	Чтение показаний АЦП магнитометра с корректировкой прочитанных значений:
void readADC(I2C_HandleTypeDef *I2Cx) {
    uint8_t i[8];
    // Читаем 8 байт регистров АЦП начиная с регистра REG_BMM_X_LSB в массив i
    HAL_I2C_Mem_Read(I2Cx, BMM_ADDRES, REG_BMM_X_LSB, 1, i, 8, i2c_timeout_bmx);
    // Получаем сопротивление для расчета температуры
    uint16_t j = (((uint16_t)i[7] << 8) | (uint16_t)i[6]) >> 2;
    // Рассчитываем температуру по полученному сопротивлению
    mag_adc[3] = (int16_t)(((int32_t)dig_xyz1 << 14) / (j != 0 ? j : dig_xyz1)) - (int16_t)0x4000;
    // Получаем значение АЦП для оси X и корректируем его
    mag_adc[0] = (int16_t)(dig_x1 << 3) +
                 (int16_t)((((int32_t)((int16_t)(((uint16_t)i[1] << 8) | (uint16_t)i[0]) >> 3) *
                 (((((int32_t)dig_xy2 * (((int32_t)mag_adc[3] * (int32_t)mag_adc[3]) >> 7) +
                 (int32_t)mag_adc[3] * (int32_t)((int16_t)dig_xy1 << 7)) >> 9) +
                 (int32_t)0x100000) * (int32_t)((int16_t)dig_x2 + (int16_t)0xA0)) >> 12)) >> 13);
    // Получаем значение АЦП для оси Y и корректируем его
    mag_adc[1] = (int16_t)(dig_y1 << 3) +
                 (int16_t)((((int32_t)((int16_t)(((uint16_t)i[3] << 8) | (uint16_t)i[2]) >> 3) *
                 (((((int32_t)dig_xy2 * (((int32_t)mag_adc[3] * (int32_t)mag_adc[3]) >> 7) +
                 (int32_t)mag_adc[3] * (int32_t)((int16_t)dig_xy1 << 7)) >> 9) +
                 (int32_t)0x100000) * (int32_t)((int16_t)dig_y2 + (int16_t)0xA0)) >> 12)) >> 13);
    // Получаем значение АЦП для оси Z и корректируем его
    mag_adc[2] = (int16_t)((((int32_t)(((int16_t)(((uint16_t)i[5] << 8) | (uint16_t)i[4]) >> 1) - dig_z4) << 15) -
                 (((int32_t)dig_z3 * (int32_t)((int16_t)j - (int16_t)dig_xyz1)) >> 2)) /
                 (int32_t)(dig_z2 + ((int16_t)((((int32_t)dig_z1 * (int32_t)((int16_t)j << 1)) + (1 << 15)) >> 16))));
}

//Выполнение компенсации смещения данных:Усреднённые значения на момент выполнения компенсации заносятся в массив magBias и потом будут вычитаться из реальных показаний
void setFastOffset_BMM(I2C_HandleTypeDef *I2Cx, uint8_t r) { //	Аргумент: 0 - сброс накопленных показаний,  1 - накопление показаний смещения.

	 static int16_t mag_max[3] = {-32768,-32768,-32768}; //	Определяем массив для хранения максимальных показаний АЦП по осям XYZ
	 static int16_t mag_min[3] = { 32767, 32767, 32767}; //	Определяем массив для хранения минимальных  показаний АЦП по осям XYZ
	 if (r == 1){
	 //	Накапливаем данные для определения лимитов:
     readADC(I2Cx);	//	Читаем АЦП.
	 for(uint8_t i=0; i<3; i++){		//	Проходим по данным трёх осей (X/Y/Z)
	     if (mag_max[i]<mag_adc[i]){
             mag_max[i]=mag_adc[i]; //	Определяем максимум для оси mag_max[i] из текущих прочитанных данных оси mag_adc[i].
         }
	     if (mag_min[i]>mag_adc[i]){
             mag_min[i]=mag_adc[i]; //	Определяем минимум  для оси mag_min[i] из текущих прочитанных данных оси mag_adc[i].
         }
     }
	 //	Определяем смещение показаний:
	 magBias[0] = (float)((int32_t)(mag_max[0] + mag_min[0]) / 2) * varQuantum;//среднее значение показаний АЦП по оси X умножаем на шаг квантования varQuantum и получаем смещение показаний оси в мГс
	 magBias[1] = (float)((int32_t)(mag_max[1] + mag_min[1]) / 2) * varQuantum;//среднее значение показаний АЦП по оси Y умножаем на шаг квантования varQuantum и получаем смещение показаний оси в мГс
	 magBias[2] = (float)((int32_t)(mag_max[2] + mag_min[2]) / 2) * varQuantum;//среднее значение показаний АЦП по оси Z умножаем на шаг квантования varQuantum и получаем смещение показаний оси в мГс
	 }
	 //	Сбрасываем накопленные значения:
	 if (r == 0){
	 magBias[0] = 0; magBias[1] = 0; magBias[2] = 0;
	 mag_max[0] = -32768; mag_max[1] = -32768; mag_max[2] =- 32768;
	 mag_min[0] = 32768; mag_min[1] = 32768; mag_min[2] = 32768;
	 }
}

void BMX055_Read_Mag(I2C_HandleTypeDef *I2Cx, BMX055_t *DataStruct){

     readADC(I2Cx);

     DataStruct->Mx = -(((float) mag_adc[1] * varQuantum - magBias[1]) * 0.1f);
     DataStruct->My = ((float) mag_adc[0] * varQuantum - magBias[0]) * 0.1f;
     DataStruct->Mz = ((float) mag_adc[2] * varQuantum - magBias[2]) * 0.1f;
     DataStruct->Temperature_mag = (float) mag_adc[3] * 273.15 / 10000;
}

//	Чтение показаний АЦП магнитометра с корректировкой прочитанных значений:
void readADC_dma(I2C_HandleTypeDef *I2Cx) {
    uint8_t i[8];
    // Читаем 8 байт регистров АЦП начиная с регистра REG_BMM_X_LSB в массив i
    HAL_I2C_Mem_Read_DMA(I2Cx, BMM_ADDRES, REG_BMM_X_LSB, 1, i, 8);
    // Получаем сопротивление для расчета температуры
    uint16_t j = (((uint16_t)i[7] << 8) | (uint16_t)i[6]) >> 2;
    // Рассчитываем температуру по полученному сопротивлению
    mag_adc[3] = (int16_t)(((int32_t)dig_xyz1 << 14) / (j != 0 ? j : dig_xyz1)) - (int16_t)0x4000;
    // Получаем значение АЦП для оси X и корректируем его
    mag_adc[0] = (int16_t)(dig_x1 << 3) +
                 (int16_t)((((int32_t)((int16_t)(((uint16_t)i[1] << 8) | (uint16_t)i[0]) >> 3) *
                 (((((int32_t)dig_xy2 * (((int32_t)mag_adc[3] * (int32_t)mag_adc[3]) >> 7) +
                 (int32_t)mag_adc[3] * (int32_t)((int16_t)dig_xy1 << 7)) >> 9) +
                 (int32_t)0x100000) * (int32_t)((int16_t)dig_x2 + (int16_t)0xA0)) >> 12)) >> 13);
    // Получаем значение АЦП для оси Y и корректируем его
    mag_adc[1] = (int16_t)(dig_y1 << 3) +
                 (int16_t)((((int32_t)((int16_t)(((uint16_t)i[3] << 8) | (uint16_t)i[2]) >> 3) *
                 (((((int32_t)dig_xy2 * (((int32_t)mag_adc[3] * (int32_t)mag_adc[3]) >> 7) +
                 (int32_t)mag_adc[3] * (int32_t)((int16_t)dig_xy1 << 7)) >> 9) +
                 (int32_t)0x100000) * (int32_t)((int16_t)dig_y2 + (int16_t)0xA0)) >> 12)) >> 13);
    // Получаем значение АЦП для оси Z и корректируем его
    mag_adc[2] = (int16_t)((((int32_t)(((int16_t)(((uint16_t)i[5] << 8) | (uint16_t)i[4]) >> 1) - dig_z4) << 15) -
                 (((int32_t)dig_z3 * (int32_t)((int16_t)j - (int16_t)dig_xyz1)) >> 2)) /
                 (int32_t)(dig_z2 + ((int16_t)((((int32_t)dig_z1 * (int32_t)((int16_t)j << 1)) + (1 << 15)) >> 16))));
}
void BMX055_Read_Mag_DMA(I2C_HandleTypeDef *I2Cx, float *data[4]){

	CurrentFunction = FUNCTION_MAG; // Установка текущей функции
	readADC_dma(I2Cx);

	*data[0] = -(((float) mag_adc[1] * varQuantum - magBias[1]) * 0.1f);
	*data[1] = ((float) mag_adc[0] * varQuantum - magBias[0]) * 0.1f;
	*data[2] = ((float) mag_adc[2] * varQuantum - magBias[2]) * 0.1f;
	*data[3] = (float) mag_adc[3] * 273.15 / 10000;
}


//Парсинг сырых данных Акселерометра
void BMX055_Process_Accel_Raw(BMX055_t *DataStruct, uint8_t *buffer)
{
    // buffer[0] = X_LSB, buffer[1] = X_MSB, и т.д.
    DataStruct->Ax = (float)((int16_t)(((uint16_t)buffer[1]<<8) | (uint16_t)buffer[0])>>4) * (2.0f/2048.0f);
    DataStruct->Ay = (float)((int16_t)(((uint16_t)buffer[3]<<8) | (uint16_t)buffer[2])>>4) * (2.0f/2048.0f);
    DataStruct->Az = (float)((int16_t)(((uint16_t)buffer[5]<<8) | (uint16_t)buffer[4])>>4) * (2.0f/2048.0f);
    DataStruct->Temperature = (float)((int16_t)buffer[6]) * 0.5f + 23.0f;
}
//Парсинг сырых данных Гироскопа
void BMX055_Process_Gyro_Raw(BMX055_t *DataStruct, uint8_t *buffer)
{
    DataStruct->Gx = (float)((int16_t)(((uint16_t)buffer[1]<<8) | (uint16_t)buffer[0])) * (1998.0f/32768.0f);
    DataStruct->Gy = (float)((int16_t)(((uint16_t)buffer[3]<<8) | (uint16_t)buffer[2])) * (1998.0f/32768.0f);
    DataStruct->Gz = (float)((int16_t)(((uint16_t)buffer[5]<<8) | (uint16_t)buffer[4])) * (1998.0f/32768.0f);
}
