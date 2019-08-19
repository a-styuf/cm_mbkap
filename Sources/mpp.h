/*
библиотека для работы с МПП
для неё необходим доступ к отправке/приему данных по ВШ, доступ к включению/выключению МПП 
*/
#ifndef _MPP_H_
#define _MPP_H_

#include <string.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "mbkap.h"
#include "timers.h"
#include "uarts.h"
#include "mko.h"

typedef struct // структура МПП от Дорошкина
{
    uint32_t AcqTime_s;         //+0
    uint32_t AcqTime_us;         //+4
    uint32_t WidhtTime;     //+8
    uint16_t ZeroCount;     //+12
    uint16_t Peak;              //+14
    uint32_t Power;            //+16
    uint16_t Mean;              //+20
    uint16_t Noise;              //+22
}typeMPPRec;

typedef  struct // структура с кадром МПП
{
    uint16_t label;              //+0 0
    uint16_t definer;           //+2 1
    uint16_t num;               //+4 2
    uint32_t time;               //+6 3-4
	//
    uint16_t arch_count;    //+10 5
    uint16_t offset;    //+12 6
    typeMPPRec mpp_rec[2];   //+14 7-30
    //
    uint16_t crc16; //+62 31
}typeMPPFrame;

typedef  struct // структура с управляющими переменными МПП
{
	uint8_t frame_definer; 	//определитель информационного кадра 
	uint8_t sub_addr; 		//субадрес для выкладывания данного кадра
	uint16_t id;			//id на внутренней шине	
	uint16_t forced_start_timeout; //таймаут на принудительный запуск
	uint8_t frame_pulse_cnt; //количество считанных помех с последнего формирования кадра
}typeMPPControl;

typedef  struct // структура  МПП
{
    typeMPPFrame frame;       
    typeMPPControl ctrl;          
}typeMPPDevice;

/*Инициализация устойства*/
void MPP_Init(typeMPPDevice *mpp_ptr, uint16_t frame_definer, uint8_t sub_addr, uint8_t id, uint16_t offset);
/* Общение с МПП по ВШ */
void MPP_time_set(void); // широковещательная
void MPP_constatnt_mode(uint8_t mode);  // широковещательная; mode: 1 - on; 0 - off
void MPP_On(typeMPPDevice *mpp_ptr);
void MPP_Off(typeMPPDevice *mpp_ptr);
void MPP_Offset_Set(typeMPPDevice *mpp_ptr, uint16_t offset);
void MPP_arch_count_offset_get(typeMPPDevice *mpp_ptr);
void MPP_struct_request(typeMPPDevice *mpp_ptr);
void MPP_struct_get(typeMPPDevice *mpp_ptr, typeCMParameters* cm_ptr);
/* формирование кадров */
void MPP_Frame_Init(typeMPPDevice *mpp_ptr);
void MPP_Frame_Build(typeMPPDevice *mpp_ptr, typeCMParameters* cm_ptr);
void MPP_mem_init(typeMPPDevice *mpp_ptr);
void MPP_forced_start(typeMPPDevice *mpp_ptr);
/* функции для внутреннего использования */
uint8_t _mpp_num_from_id(uint8_t mpp_id); //функция взятия порядкового номера МПП из его ID. Пригодится при неочевидном переводе
void _mpp_struct_rev(typeMPPRec* mpp_struct_ptr);

#endif
