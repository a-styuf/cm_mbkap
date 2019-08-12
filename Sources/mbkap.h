#ifndef _MBKAP_H_
#define _MBKAP_H_

#include "1986ve8_lib/cm4ikmcu.h"
//***Параметры программы для ЦМ МБКАП
// номер устройства
#define DEV_NUM 0 //todo: уточнить номер устройства у Игоря
// параметры МКО
#define MKO_ID  22
// времянные параметры интервалов и слотов
#define CM_PARAM_SAVE_PERIOD_S   1
#define DEFAULT_SYS_INTERVAL_S 10
#define DEFAULT_MEAS_INTERVAL_S 10
#define SLOT_TIME_MS 100
// уровни срабатывания токовой защиты мА
#define CM_BOUND 0 
#define MPP27_BOUND 0
#define MPP100_BOUND 0
#define DIR_BOUND 0
#define DNT_BOUND 0
#define ADII_BOUND 0
//таймаут для определения неответа для внутренней шины
#define UART_TIMEOUT_MS 6
//номера кадров для МКО/МПИ
#define SYS_FRAME_NUM 0x0F
#define mpp27_frame_num 0x01
#define mpp100_frame_num 0x02
#define dir_frame_num 0x03
#define dnt_frame_num 0x04
#define adii_frame_num 0x05
#define arch_frame 0x0E
#define command 0x11
#define arch_data 0x12
#define tech_comm 0x1E
#define time_comm 0x1D
//id modules
#define mpp27_id 2 
#define mpp100_id 3
#define dir_id 4
//
#pragma pack(1)
// ВАЖНО: последовательные однобайтовые параметры при копировании в кадр меняются местами

//структуры кадров
typedef struct // ситстемный кадр
{
    uint16_t label;  //+0 0
    uint16_t definer; //+2 1 
    uint16_t num; //+4 2
    uint32_t time; //+6 4
    //
    uint16_t currents[7]; //+10 5-10
	//
    uint16_t read_ptr; //+24 12
    uint16_t write_ptr; //+26 13
	//
	uint8_t mko_error_cnt; //+28 14
    uint8_t mko_error; //+29 14
    uint16_t rst_cnt; //+30 15
    //
    int32_t diff_time_s; //+32 16-17
    // важно: данные параметры восле копирования в кадр будут стоять в обратном порядке
    uint8_t sync_num; //+36 18
    uint8_t diff_time_low; //+37 18
    //
    uint8_t bus_nans_cnt; //+38 19
    uint8_t bus_error_cnt; //+39 19
    //
    uint8_t bus_nans_status; //+40 20
    uint8_t bus_error_status; //+41 20
    //
    int16_t temp; //+42 21
    //
    uint32_t operating_time; //+44 22-23
    //
    uint16_t measure_interval;//+48 24
    uint16_t sys_interval;//+50 25
	//
	uint32_t sync_time_s;//+52 26-27
	uint8_t stm_val;//+56 28
    uint8_t sync_time_low;//+57 28
    //
	uint8_t pwr_status; //+58 29
    uint8_t pwr_state; //+59 29
	//
    uint8_t reserved[2]; //+60
    //
    uint16_t crc16; //+62 31
}typeSysFrames;

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
    typeMPPRec mpp_frame_1;   //+14 7-18
    typeMPPRec mpp_frame_2;   //+38 19-30
    //
    uint16_t crc16; //+62 31
}typeMPPFrame;

typedef struct // одиночное измерение ДНТ
{
	int16_t current;              //+0 0
	// важно: данные параметры восле копирования в кадр будут стоять в обратном порядке
    uint8_t temperature;           //+2 1
    uint8_t ku;               //+3 1
}typeDNTMeasurement;

typedef struct // структура с кадром данных ДНТ (12 измерений)
{
    uint16_t label;              //+0 0
    uint16_t definer;           //+2 1
    uint16_t num;               //+4 2
    uint32_t time;               //+6 3-4
    //
    typeDNTMeasurement dnt_measurement[12]; //+10-46 5-23
	uint16_t change_int_num; //+48 24
    uint16_t changed_meas_interv; //+50 25
    //
    uint8_t reserved[10]; //+52
    //
    uint16_t crc16; //+62 31
}typeDNTFrame;

typedef struct //одиночное измерение ДИР
{
	uint16_t value;              //+0
    uint16_t temperature;   //+2
}typeDIRMeasurement;

typedef struct // измерения всех ДИР-ов (5 штук)
{
	typeDIRMeasurement measurements[5];
}typeAllDIRMeasurements;

typedef struct // структура с кадром данных ДИР (2 измерений)
{
    uint16_t label;              //+0 0
    uint16_t definer;           //+2 1
    uint16_t num;               //+4 2
    uint32_t time;               //+6 3-4
    //
    typeAllDIRMeasurements dnt_measurements[2]; //+10-50 5-25
    //
    uint8_t reserved[10]; //+52
    //
    uint16_t crc16; //+62 31
}typeDIRFrame;

typedef struct // структура с кадром данных АДИИ
{
    uint16_t label;              //+0 0
    uint16_t definer;           //+2 1
    uint16_t num;               //+4 2
    uint32_t time;               //+6 3-4
    //
    uint8_t adii_array[52]; //+10-62
    //
    uint16_t crc16; //+62 31
}typeADIIFrame;

//структуры управления
typedef struct  // структура состояния СТМ: если поля больше 0, то выставяем замкнуто, если меньше нуля - то разомкнуто
{
	// изменяемая часть от прибора к прибору
    int8_t NKPBE;
    int8_t AMKO;
	// обязательная часть
	int8_t block; //блокируется изменение СТМ
}typeSTMstruct;

typedef struct  // структура с счетсиками состояний для различных устройств - изначально инициализируем числом, затем числа уменьшаются до нуля: нуль - отдыхаем
{
	uint8_t MPP_cnt[2]; //массив с флагами для МПП1-6
	uint8_t DNT_cnt; //массив с флагами для ДНТ
	uint8_t DIR_cnt; //массив с флагами для ДИР
}typePerepherrialControlPollingstruct;

typedef struct // параетры ЦМ для управления и сохранения в память !!! занимает два кадра!
{
    uint16_t label; //+0
    uint32_t time; //+2
    uint16_t frame_number; //+6
    uint16_t rst_cnt; //+8
    uint16_t read_ptr; //+10
    uint16_t write_ptr; //+12
    //важно: данные параметры (однобитные) восле копирования в кадр будут стоять в обратном порядке
    uint8_t bus_error_cnt; //+14
    uint8_t bus_nans_cnt; //+15
    uint8_t bus_error_status; //+16
    uint8_t bus_nans_status; //+17
    uint8_t pwr_status; //+18 
    uint8_t pwr_state; //+19 
    int32_t diff_time_s; //+20
    int8_t diff_time_low; //+24
    uint8_t sync_num; //+25
    uint32_t operating_time; //+26
    uint16_t measure_interval;//+30
	uint16_t sys_interval; //+32
    uint16_t currents[7]; //+34       
	uint8_t mko_error_cnt;//+48
	uint8_t mko_error;//+49
	uint16_t speed_mode_state; //+50 состояния нахождения генерации типов кадров по режиму 1 ("0") или 2 ("1"): 0-1 - МПП1-2, 2 - ДНТ, 3 - ДИР, 4 - АДИИ
	uint16_t speed_mode_timeout; //+52 таймаут на переход в ускоренный режим
	uint16_t pwr_bounds[7];//+54 граница срабатывания токовой защиты периферии
	uint32_t sync_time_s;//+68
	uint8_t sync_time_low;//+72
	uint8_t stm_val;//+73
    uint8_t reserved[52];//+74
    uint16_t crc16; //+126
}typeCMParameters;


// функция для работы с памятью
int8_t Save_Data_Frame(uint8_t* frame, typeCMParameters* cm_ptr);  // сохранение кадра с данными в архивную память
int8_t Load_Data_Frame(typeCMParameters* cm_ptr);  // загрузка кадра с данными из памяти с выкладыванием на подадрес
int8_t Write_Parameters(typeCMParameters* cm_ptr);   // загрузка параметров в структуру: параметры хранятся в начале и конце памяти
int8_t Read_Parameters(typeCMParameters* cm_ptr);  // сохранение параметров из структуры в память
int8_t _read_cm_parameters_frame_with_crc16_check(uint16_t addr, uint8_t* frame); //чтение 128-байтового кадра с параметрами
// функции для работы со структурой управления ЦМ typeCMParameters
void CM_Parame_Full_Init(typeCMParameters* cm_ptr); //функция инициализации структуры, зануляет все кроме наработки
void _cm_params_set_default(typeCMParameters* cm_ptr);
void CM_Parame_Start_Init(typeCMParameters* cm_ptr); //функция инициализации структуры, зануляет все, что нет необходимости хранить
void CM_Parame_Operating_Time_Init(uint32_t op_time, typeCMParameters* cm_ptr); //функция, которая устанавливает наработку
void CM_Parameters_Write(typeCMParameters* parameters);
int8_t CM_Parameters_Read(typeCMParameters* parameters);
// формирование системного кадра
void Sys_Frame_Init(typeSysFrames *sys_frame); //инициализируются известные поля для системного кадра и выкладывается на подадрес
void Sys_Frame_Build(typeSysFrames *sys_frame, typeCMParameters* cm_ptr);
// работа с МКО
uint8_t get_mko_addr(uint8_t def_addr);
// управление питанием
void  Pwr_current_process(typeCMParameters* cm);
// отладочный интерфейс
int8_t Debug_Get_Packet (uint16_t* reg_addr, uint16_t* data, uint8_t* leng);
// Внутрениие рабочие функции
uint32_t _rev_u32 (volatile uint32_t val); //перестановка по 16-ти битным словам
void _buff_rev16(uint16_t *buff, uint8_t leng_16);
uint8_t uint16_to_log2_uint8_t(uint16_t var);
#endif
