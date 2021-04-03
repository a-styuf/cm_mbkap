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
#define DEV_NUM 208 //206 - МБКАП_зн02; 207 - МБКАП_зн03; 208 - МБКАП_зн04; 209 - МБКАП_зн05;
// параметры МКО
#define MKO_ID  0  // 0 - адрес берется с разъема, не 0 - адрес МКО
// времянные параметры интервалов и слотов
#define CM_PARAM_SAVE_PERIOD_S   1
	//
#define DEFAULT_SYS_INTERVAL_S 1800
#define DEFAULT_MEAS_INTERVAL_S 360
#define DEFAULT_DIR_INTERVAL_S 60  
#define DEFAULT_ADII_MEAS_INTERVAL_S 360
#define DEFAULT_ADII_DEPOL_INTERVAL_S 60
	//
#define DEFAULT_SPEEDY_MODE_TIME_S 3600
#define SLOT_TIME_MS 100
// уровни срабатывания токовой защиты мА - трехкратное превышение нормального потребления
#define MBKAP_BOUND 3*350
#define CM_BOUND 3*55
#define MPP27_BOUND 3*36
#define MPP100_BOUND 3*36
#define DIR_BOUND 3*50
#define DNT_BOUND 3*31
#define ADII_BOUND 3*125
// настройки МПП
#define MPP27_DEF_OFFSET 10 // уровень срабатывания МПП27, кв. АЦП
#define MPP100_DEF_OFFSET 6  // уровень срабатывания МПП100, кв. АЦП
#define MPP100_DEF_BOUND 2285  // 85 V // уровень срабатывания МПП100 [кв. АЦП] при котором происходит отключение реле питания по величине 100В (входное напряжение РМПЕ 80-100)
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
#define NU_ID 15
// параметры переферийных устройств
#define DNT_MKO_ADDR 19
#define DNT_DEV_NUM 2  // тип аппаратуры: мелкосерийный ДНТ (определяет Игорь Щепихин)
#define DNT_DEV_FRAME_NUM 0
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
    uint16_t currents[7]; //+10 5-12
	//
    uint8_t rst_cnt; //+24 12
    int8_t temperature; //+25 12
	//
    uint16_t read_ptr; //+26 13
    uint16_t write_ptr; //+28 14
    //
	uint8_t mko_error_cnt; //+30 15
    uint8_t mko_error; //+31 15
    //
    uint8_t bus_nans_cnt; //+32 16
    uint8_t bus_nans_status; //+33 16
    //
	uint8_t bus_error_cnt; //+34 17
    uint8_t bus_error_status; //+35 17
	//
    uint8_t pwr_state; //+36 18
	uint8_t pwr_status; //+37 18
	 //
    uint16_t measure_interval;//+38 19
    uint16_t sys_interval;//+40 20
	//
    int16_t diff_time_s; //+42 21
    // важно: данные параметры восле копирования в кадр будут стоять в обратном порядке
    uint8_t sync_num; //+44 22
    uint8_t diff_time_low; //+45 22
	//
	uint32_t sync_time_s;//+46 23-24
	uint8_t stm_val;//+50 25
    uint8_t sync_time_low;//+51 25
    //
    uint32_t operating_time; //+52 25-26
	//
	uint8_t adii_mode; //+56 27
	uint8_t adii_fk; //+57 27
	uint16_t adii_measure_interval; //+58 28
	uint16_t adii_depol_interval; //+60 28
    //
	//uint8_t reserved[2];//+62 0
	//
    uint16_t crc16; //+62 30
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
    uint16_t currents[7]; //+28       
	uint16_t pwr_bounds[7];//+42 граница срабатывания токовой защиты периферии
	uint8_t mko_error_cnt;//+56
	uint8_t mko_error;//+57
	//
	uint32_t sync_time_s;//+58
	uint8_t sync_time_low;//+62
	uint8_t stm_val;//+63
	//
	uint16_t parame_interval; //+64 таймер на измерительный интервал
	uint16_t parame_timeout; //+66 таймер на измерительный интервал
	//
	uint16_t sys_interval; //+68
	uint16_t sys_timeout; //+70 таймер на измерительный интервал
	//
	uint16_t measure_interval;//+72
	uint16_t measure_state; //+74 флаги запусков получния кадров: 0-1 - МПП1-2, 2 - ДНТ, 3 - ДИР, 4 - АДИИ
	uint16_t measure_timeout; //+76 таймер на измерительный интервал
	//
	uint16_t speed_mode_state; //+78 состояния нахождения генерации типов кадров по режиму 1 ("0") или 2 ("1"): 0-1 - МПП1-2, 2 - ДИР, 3 - ДНТ
	uint16_t speed_mode_timeout; //+80 таймаут на переход в ускоренный режим
	//
	uint16_t dir_interval; //+82
	uint16_t dir_timeout; //+84
	//
	uint8_t adii_mode; //+86
	uint8_t adii_fk; //+87
	uint16_t adii_measure_interval; //+88
	uint16_t adii_depol_interval; //+90
	uint16_t adii_timeout; //+93
	//
	uint8_t defend_mem; //+94 // если 1 - запись остановлена, т.к. указатель чтения достиг защищенную область (указатель записи - размер защищенной области)
	uint8_t debug; //+95
	//
	uint16_t additional_sys_frame_flags; //+96 //каждый бит соответствует определенному событиию, по которому сгенерировался дополнительный кадр
	int16_t temperature; //+98
	//
    uint8_t reserved[26];//+100
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

// обработка информации о потреблении токов
uint8_t  Pwr_current_process(typeCMParameters* cm_ptr);
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
void CM_Parame_Start_Init(typeCMParameters* cm_ptr, typeCMParameters* cm_old_ptr);  // функция инициализации структуры, зануляет все, что нет необходимости хранить
void CM_Parame_Command_Init(typeCMParameters* cm_ptr);  // функция инициализации структуры по командному сообщению, зануляет все
void CM_Parame_Operating_Time_Init(uint32_t op_time, typeCMParameters* cm_ptr); //функция, которая устанавливает наработку
uint8_t CM_Parame_Comparison(typeCMParameters* cm_ptr, typeCMParameters* cm_old_ptr); //сравнивает новую и старую структуру с параметрами ЦМ, выдавая результат в  виде флагов несовпадения
uint8_t CM_Parame_Processor_1s(typeCMParameters* cm_ptr);
//работа с измерительным интервалом и ускоренным режимом
uint16_t Set_Speedy_Mode(typeCMParameters* cm_ptr, uint16_t on, uint16_t state, uint16_t speedy_mode_time);
uint8_t Speed_Mode_Processor_1s(typeCMParameters* cm_ptr);
uint8_t Measurment_Processor_1s(typeCMParameters* cm_ptr); //включаем структуру ДНТ, т.к. ускоренный режим ДНТ отличается от обычного
uint8_t ADII_Meas_Processor_1s(typeCMParameters* cm_ptr);
uint8_t DIR_Meas_Processor_1s(typeCMParameters* cm_ptr);
// общие функции для работы с кадрами
uint16_t _frame_definer(uint8_t frame_modification, uint16_t device_number,  uint16_t fabrication_num, uint8_t frame_type);
// формирование системного кадра
void Sys_Frame_Init(typeSysFrames *sys_frame); //инициализируются известные поля для системного кадра и выкладывается на подадрес
void Sys_Frames_Additional_Build(typeSysFrames *sys_frame, typeCMParameters* cm_ptr, typeCMParameters* cm_old_ptr);
void Sys_Frames_Interval_Build(typeSysFrames *sys_frame, typeCMParameters* cm_ptr);
void Sys_Frame_Build(typeSysFrames *sys_frame, typeCMParameters* cm_ptr);
uint8_t Sys_Frame_Processor_1s(typeSysFrames* sys_ptr, typeCMParameters* cm_ptr, typeCMParameters* cm_old_ptr);  //формирование системного кадра
// работа с МКО
uint8_t get_mko_addr(uint8_t def_addr);
// отладочный интерфейс
int8_t Debug_Get_Packet (uint16_t* reg_addr, uint16_t* data, uint8_t* leng);
// ВШ
int8_t F_Trans(typeCMParameters* cm_ptr, uint8_t code, uint8_t dev_id, uint16_t start_addr, uint16_t cnt, uint16_t * data_arr); //функция, позволяющая отправлять стандартизованные ModBus запросы/ответы
void F16_IB_data_transfer(typeCMParameters* cm_ptr, uint8_t dev_id, uint16_t start_addr, uint16_t cnt, uint16_t * data_arr); //функция, позволяющая отправлять дополнительные данные в ModBus
uint16_t Tech_SA_Transaction(uint16_t *data_arr); // функция для трансляции данных из МКО в ВШ
//получение идентификационной строки для переферии
int8_t Pereph_On_and_Get_ID_Frame(uint8_t dev_num, typeDevStartInformation* dev_init_inf_ptr); //включаем переферии и получаем от нее идентификационный пакет
// Внутрениие рабочие функции
uint32_t _rev_u32 (volatile uint32_t val); //перестановка по 16-ти битным словам
void _buff_rev16(uint16_t *buff, uint8_t leng_16);
uint8_t uint16_to_log2_uint8_t(uint16_t var);
uint16_t get_val_from_bound(uint16_t val, uint16_t min, uint16_t max); //если число внутри границ - используется оно, если нет, то ближайшая граница
#endif
