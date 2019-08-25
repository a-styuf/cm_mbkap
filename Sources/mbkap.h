#ifndef _MBKAP_H_
#define _MBKAP_H_

#include "1986ve8_lib/cm4ikmcu.h"
#include <stdio.h>
#include <string.h>
#include "gpio.h"
#include "uarts.h"
#include "eerom.h"
#include "mko.h"
#include "crc16.h"
#include "timers.h"
#include "power_management.h"

//***Параметры программы для ЦМ МБКАП
// номер устройства
#define DEV_NUM 0 //todo: уточнить номер устройства у Игоря
// параметры МКО
#define MKO_ID  22
// времянные параметры интервалов и слотов
#define CM_PARAM_SAVE_PERIOD_S   1
#define DEFAULT_SYS_INTERVAL_S 9
#define DEFAULT_MEAS_INTERVAL_S 10
#define DEFAULT_ADII_INTERVAL_S 20
#define SLOT_TIME_MS 100
// уровни срабатывания токовой защиты мА
#define CM_BOUND 0 
#define MPP27_BOUND 0
#define MPP100_BOUND 0
#define DIR_BOUND 0
#define DNT_BOUND 0
#define ADII_BOUND 0
// настройки МПП
#define MPP27_DEF_OFFSET 0  // уровень срабатывания МПП27
#define MPP100_DEF_OFFSET 0  // уровень срабатывания МПП100
// таймаут для определения неответа для внутренней шины
#define UART_TIMEOUT_MS 6
// номера кадров для МКО/МПИ
// оперативные кадры
#define SYS_FRAME_NUM 0x0F
#define MPP27_FRAME_NUM 0x01
#define MPP100_FRAME_NUM 0x02
#define DIR_FRAME_NUM 0x03
#define DNT_FRAME_NUM 0x04
#define ADII_FRAME_NUM 0x05
// остальные
#define ARCH_FRAME_REQ_SA 0x12
#define COMMAND_MESSAGE_SA 0x11
#define ARCH_FRAME_DATA_SA 0x0E
#define TECH_COMMAND_SA 0x1E
// id modules
#define MPP27_ID 2 
#define MPP100_ID 3
#define DIR_ID 4
// параметры переферийных устройств
#define DNT_MKO_ADDR 22
#define DNT_DEV_NUM 2  // тип аппаратуры: мелкосерийный ДНТ (определяет Игорь Щепихин)
#define DNT_MKO_ZAV_NUM 5
// параметры защищенной области
#define DEFEND_VOLUME 100 //в кадрах
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
    int16_t diff_time_s; //+32 16
    // важно: данные параметры восле копирования в кадр будут стоять в обратном порядке
    uint8_t sync_num; //+34 17
    uint8_t diff_time_low; //+35 17
	//
	uint32_t sync_time_s;//+36 18-19
	uint8_t stm_val;//+40 20
    uint8_t sync_time_low;//+41 20
    //
    uint8_t bus_nans_cnt; //+42 21
    uint8_t bus_error_cnt; //+43 21
    //
    uint8_t bus_nans_status; //+44 22
    uint8_t bus_error_status; //+45 22
    //
    int16_t temp; //+46 23
    //
    uint32_t operating_time; //+48 24-25
    //
    uint16_t measure_interval;//+52 26
    uint16_t sys_interval;//+54 27
    //
	uint8_t pwr_status; //+56 28
    uint8_t pwr_state; //+57 28
	//
	uint8_t adii_mode; //+59 29
	uint8_t adii_fk; //+58 29
	uint16_t adii_interval; //+60 30
    //
    uint16_t crc16; //+62 31
}typeSysFrames;

//структуры управления
typedef struct  // структура с счетсиками состояний для различных устройств - изначально инициализируем числом, затем числа уменьшаются до нуля: нуль - отдыхаем
{
	uint8_t MPP_cnt[2]; //массив с флагами для МПП1-6
	uint8_t DNT_cnt; //массив с флагами для ДНТ
	uint8_t DIR_cnt; //массив с флагами для ДИР
}typePerepherrialControlPollingstruct;

typedef struct // параетры ЦМ для управления и сохранения в память !!! занимает два кадра !!!
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
    int16_t diff_time_s; //+20
    int8_t diff_time_low; //+22
    uint8_t sync_num; //+23
    uint32_t operating_time; //+24
    uint16_t measure_interval;//+28
	uint16_t sys_interval; //+30
    uint16_t currents[7]; //+32       
	uint8_t mko_error_cnt;//+46
	uint8_t mko_error;//+47
	uint16_t normal_mode_state; //+48 флаги запусков получния кадров: 0-1 - МПП1-2, 2 - ДНТ, 3 - ДИР, 4 - АДИИ
	uint16_t speed_mode_state; //+50 состояния нахождения генерации типов кадров по режиму 1 ("0") или 2 ("1"): 0-1 - МПП1-2, 2 - ДИР, 3 - ДНТ, 4 - АДИИ
	uint16_t speed_mode_timeout; //+52 таймаут на переход в ускоренный режим
	uint16_t pwr_bounds[7];//+54 граница срабатывания токовой защиты периферии
	uint32_t sync_time_s;//+68
	uint8_t sync_time_low;//+72
	uint8_t stm_val;//+73
	//
	uint8_t adii_mode; //+74
	uint8_t adii_fk; //+75
	uint16_t adii_interval; //+76
	//
	uint8_t defend_mem; //+78
	uint8_t debug; //+79
	//
    uint8_t reserved[46];//+80
    uint16_t crc16; //+126
}typeCMParameters;

typedef struct // структура информационного поля в стартовой строке переферийного устройства
{
	uint8_t number;
	uint8_t leng;
	uint8_t string[16];
}typeDevInformationField;

typedef struct // структура со стартовой информацией о включенной переферии
{
	uint8_t id;
	uint8_t fixed_field[6];
	uint8_t inf_filed_number;
	typeDevInformationField inf_field_arr[8];
}typeDevStartInformation;

// функция для работы с памятью
int8_t Save_Data_Frame(uint8_t* frame, typeCMParameters* cm_ptr);  // сохранение кадра с данными в архивную память
uint16_t _calc_defended_mem_addr(typeCMParameters* cm_ptr);
void Move_Read_Ptr_To_Defended_Mem(typeCMParameters* cm_ptr);
int8_t Load_Data_Frame(typeCMParameters* cm_ptr);  // загрузка кадра с данными из памяти с выкладыванием на подадрес
int8_t Write_Parameters(typeCMParameters* cm_ptr);  // загрузка параметров в структуру: параметры хранятся в начале и конце памяти
int8_t Read_Parameters(typeCMParameters* cm_ptr);  // сохранение параметров из структуры в память
int8_t _read_cm_parameters_frame_with_crc16_check(uint16_t addr, uint8_t* frame); //чтение 128-байтового кадра с параметрами
// функции для работы со структурой управления ЦМ typeCMParameters
void CM_Parame_Full_Init(typeCMParameters* cm_ptr);  // функция инициализации структуры, зануляет все кроме наработки
void _cm_params_set_default(typeCMParameters* cm_ptr);
void CM_Parame_Start_Init(typeCMParameters* cm_ptr);  // функция инициализации структуры, зануляет все, что нет необходимости хранить
void CM_Parame_Command_Init(typeCMParameters* cm_ptr);  // функция инициализации структуры по командному сообщению, зануляет все
void CM_Parame_Operating_Time_Init(uint32_t op_time, typeCMParameters* cm_ptr); //функция, которая устанавливает наработку
void CM_Parameters_Write(typeCMParameters* parameters);
int8_t CM_Parameters_Read(typeCMParameters* parameters);
// общие функции для работы с кадрами
uint16_t _frame_definer(uint8_t frame_modification, uint16_t device_number,  uint16_t fabrication_num, uint8_t frame_type);
// формирование системного кадра
void Sys_Frame_Init(typeSysFrames *sys_frame); //инициализируются известные поля для системного кадра и выкладывается на подадрес
void Sys_Frame_Build(typeSysFrames *sys_frame, typeCMParameters* cm_ptr);
// работа с МКО
uint8_t get_mko_addr(uint8_t def_addr);
// управление питанием
void  Pwr_current_process(typeCMParameters* cm_ptr);
// отладочный интерфейс
int8_t Debug_Get_Packet (uint16_t* reg_addr, uint16_t* data, uint8_t* leng);
// ВШ
int8_t F_Trans(typeCMParameters* cm_ptr, uint8_t code, uint8_t dev_id, uint16_t start_addr, uint16_t cnt, uint16_t * data_arr); //функция, позволяющая отправлять стандартизованные ModBus запросы/ответы
uint16_t Tech_SA_Transaction(uint16_t *data_arr); // функция для трансляции данных из МКО в ВШ
//получение идентификационной строки для переферии
int8_t Pereph_On_and_Get_ID_Frame(uint8_t dev_num, typeDevStartInformation* dev_init_inf_ptr); //включаем переферии и получаем от нее идентификационный пакет
// Внутрениие рабочие функции
uint32_t _rev_u32 (volatile uint32_t val); //перестановка по 16-ти битным словам
void _buff_rev16(uint16_t *buff, uint8_t leng_16);
uint8_t uint16_to_log2_uint8_t(uint16_t var);
uint16_t get_val_from_bound(uint16_t val, uint16_t min, uint16_t max); //если число внутри границ - используется оно, если нет, то ближайшая граница
#endif
