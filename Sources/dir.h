/*
библиотека для работы с ДИР
для неё необходим доступ к отправке/приему данных по ВШ, доступ к включению/выключению ДИР 
*/
#ifndef _dir_H_
#define _dir_H_

#include <string.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "mbkap.h"
#include "timers.h"
#include "uarts.h"
#include "mko.h"
//
#pragma pack(1)
/* ДИР */
typedef struct // структура данных ДИР от Дорошкина
{
    uint16_t dir_0;         //+0
    uint16_t dir;         //+2
    uint16_t tmpr;         //+4
}typeDIRRecord; //6 байт

typedef struct // структура управления ДИР 
{
    typeDIRRecord DIRRec[4]; 
}typeDIRData; //24 байт

typedef  struct // структура с кадром ДИР
{
	//обязательная часть - заголовок
    uint16_t label;              //+0 0
    uint16_t definer;           //+2 1
    uint16_t num;               //+4 2
    uint32_t time;               //+6 3-4
	//
    typeDIRData dir_data[2];   //+10 7-28
    //
	uint8_t reserv[4]; //+58 29-30
	//обязательная часть - контрольная сумма
    uint16_t crc16; //+62 31
}typeDIRFrame;

typedef  struct // структура с управляющими переменными ДИР
{
	uint16_t frame_definer; 	//определитель информационного кадра 
	uint8_t sub_addr; 		//субадрес для выкладывания данного кадра
	uint16_t id;			//id на внутренней шине	
	uint8_t meas_num; //количество считанных измерений с последнего формирования кадра
	uint8_t mode;  //0 - не запускаем, 1 - 24V, 2 - 30V
}typeDIRControl;

typedef  struct // структура  ДИР
{
    typeDIRFrame frame;       
    typeDIRControl ctrl;          
    typeDIRData data;
}typeDIRDevice;

/* ДНТ */
typedef struct //кадр с данными от ДНТ с ПА30 !!!нельзя делать 32-битные поля, иначе потом есть проблемы с выкладыванием на подадрес (меняются местами части по 16бит)!!!
{
	//обязательная часть - заголовок
    uint16_t label;  //+0
    uint16_t definer; //+2  - определитель по формуту 1
    uint16_t num; //+4
    uint32_t time; //+6 3-4
    //
    uint16_t current; //+10
	uint16_t signal; //+12
	uint16_t zero; //+14
	//
	uint16_t temperature; //+16
    uint16_t shut_off_grid_voltage; //+18 
    uint16_t dnt_state; //+20
    //резерв
	uint16_t reserved[20]; //+22
	//обязательная часть - контрольная сумма
    uint16_t crc16; //+62
}typeDNTDataFrame;

typedef struct //единичное измерение ДНТ
{
	//обязательная часть - заголовок
    uint16_t current;  //+0
    uint8_t temp; //+2  //важно! поля по 8 бит меняются местами
    uint8_t ku; //+3
}typeDNTRec; //4

typedef struct //кадр с данными для использования  в ЦМ
{
	//обязательная часть - заголовок
    uint16_t label;  //+0
    uint16_t definer; //+2 
    uint16_t num; //+4
    uint32_t time;  //+6 3-4
    //
	typeDNTRec data[12]; //+10 5-29
	//
	uint8_t number; //+58
	uint8_t shut_off_grid_voltage; //+59
	uint16_t meas_int; //+60
	//обязательная часть - контрольная сумма
    uint16_t crc16; //+62
}typeDNTFrame;

typedef  struct // структура  данных для обмена через МКО через ДИР
{
    uint16_t CmdWord;       
    uint16_t Data[33];       
}typeMKOPacket;

typedef  struct // структура данных для управления обменом по мко через ДИР
{
    uint16_t Run;       
    uint16_t BSIStat;       
    typeMKOPacket packet;       
}typeDIRMKOData;

typedef  struct // структура с управляющими переменными ДНТ
{
	uint16_t frame_definer; 	//определитель информационного кадра для ДНТ
	uint8_t sub_addr; 		//субадрес для выкладывания данного кадра
	//
	uint8_t dir_id;			//адрес ДИР на внутренней шине (с ДНТ общение через ДИР)
	uint8_t mko_addr;			//адрес МКО ДНТ - 22
	uint16_t dev_frame_definer; 	//определитель кадра пришедшего от ДНТ: сопадает - ок, не совпадает - данные не верны
	uint16_t dev_frame_num; 	//номер кадра для проверки принятых данных на уникальность (изменился номер - значит новые данные пришли; не изменился - старые данные)
	typeDIRMKOData mko;
	//
	uint8_t meas_num; //количество считанных измерений с последнего формирования кадра
	uint8_t mode;  //1 - единичный запуск, 2 - циклический запуск
	uint16_t last_measure_interval;
}typeDNTControl;

typedef  struct // структура  ДИР
{
    typeDNTFrame frame;       
    typeDNTControl ctrl;          
    typeDNTDataFrame dev_frame;
}typeDNTDevice;

/* ADII */
typedef struct // структура с кадром данных АДИИ
{
    uint16_t label;              //+0 0
    uint16_t definer;           //+2 1
    uint16_t num;               //+4 2
    uint32_t time;               //+6 3-4
    //
    uint8_t cnt_val[52]; //+10 5-30
    //
    uint16_t crc16; //+62 31
}typeADIIFrame;

typedef struct // структура с данными АДИИ для общения по ВШ
{
    uint16_t Run;
	uint16_t RxLeng;
	uint8_t Data[64];
}typeADIIData;

typedef  struct // структура с управляющими переменными АДИИ
{
	uint16_t frame_definer; 	//определитель информационного кадра для ДНТ
	uint8_t sub_addr; 		//субадрес для выкладывания данного кадра
	//
	uint8_t dir_id;			//адрес ДИР на внутренней шине (с ДНТ общение через ДИР)
	//
	uint8_t mode; // 0 - режим измерения, 1 - тестовый режим
	typeADIIData data;
}typeADIIControl;

typedef  struct // структура  АДИИ
{
    typeADIIFrame frame;       
    typeADIIControl ctrl;          
}typeADIIDevice;

//*** ДИР
void DIR_Init(typeDIRDevice *dir_ptr, uint16_t frame_definer, uint8_t sub_addr, uint8_t id);
void DIR_constatnt_mode(uint8_t mode); // широковещательная; mode: 1 - on; 0 - off;
void DIR_Start_Measurement(typeDIRDevice *dir_ptr); //измрение идет долго, предлагается забирать старые при запуске ДИР. В итоге будет задержка в один изм. интервал
void DIR_Data_Get(typeDIRDevice *dir_ptr, typeCMParameters* cm_ptr);
/* формирование кадров */
void DIR_Frame_Init(typeDIRDevice *dir_ptr);
void DIR_Frame_Build(typeDIRDevice *dir_ptr, typeCMParameters* cm_ptr);
/* функции для внутреннего использования */
void _dir_struct_rev(typeDIRData* dir_struct_ptr);
//*** ДНТ
void DNT_Init(typeDNTDevice *dnt_ptr, uint16_t frame_definer, uint16_t dev_frame_definer, uint8_t sub_addr, uint8_t mko_addr, uint8_t dir_id);
void DNT_MKO_Read_Initiate(typeDNTDevice *dnt_ptr);
void DNT_MKO_Read_Finish(typeDNTDevice *dnt_ptr, typeCMParameters* cm_ptr);
void DNT_MKO_Measure_Initiate(typeDNTDevice *dnt_ptr);
void DNT_MKO_Measure_Finish(typeDNTDevice *dnt_ptr, typeCMParameters* cm_ptr);
/* формирование кадров */
void DNT_Frame_Init(typeDNTDevice *dnt_ptr);
void DNT_Frame_Build(typeDNTDevice *dnt_ptr, typeCMParameters* cm_ptr);
void DNT_Frame_Write_to_SA(typeDNTDevice *dnt_ptr, typeCMParameters* cm_ptr);
//*** АДИИ
void ADII_Init(typeADIIDevice *adii_ptr, uint16_t frame_definer, uint8_t sub_addr, uint8_t dir_id);
void ADII_Meas_Start(typeADIIDevice *adii_ptr);  //управление командой происходит переменной adii.ctrl.mode: 1 - режим тестирования, 0 - нормальный режим
void ADII_Read_Data(typeADIIDevice *adii_ptr, typeCMParameters* cm_ptr);
/* формирование кадров */
void ADII_Frame_Init(typeADIIDevice *adii_ptr);
void ADII_Frame_Build(typeADIIDevice *adii_ptr, typeCMParameters* cm_ptr);
/* функции для внутреннего использования */
uint8_t _adii_crc_check(uint8_t* buff, uint8_t leng); //проверка контрольной суммы для АДИИ !!! требуются тесты
#endif
