#include "1986ve8_lib/cm4ikmcu.h"
#include <string.h>
#include <stdio.h>
#include "sysinit.h"
#include "eerom.h"
#include "mko.h"
#include "adc.h"
#include "wdt.h"
#include "uarts.h"
#include "timers.h"
#include "mbkap.h"
#include "stm.h"
#include "gpio.h"
#include "power_management.h"
#include "mpp.h"
#include "dir.h"
//Настройки программы в файле mbkap.c

//
extern typeCMParameters cm, cm_old;
extern typeSysFrames sys_frame;
extern typeSTMstruct stm;
// Структуры для управления периферией
typeMPPDevice mpp27; 
typeMPPDevice mpp100; 
typeDIRDevice dir; 
typeDNTDevice dnt; 
typeADIIDevice adii; 

typeDevStartInformation mpp27_init_inf;
typeDevStartInformation mpp100_init_inf;
typeDevStartInformation dir_init_inf;

extern typeMKOControl mko_dev;
extern uint16_t ADCData[];
//
uint8_t Buff[256];
uint8_t leng, dbg_status;
uint16_t reg_addr, dbg_data[32];
uint8_t uint8_val;
uint16_t uint16_val, uint16_arr[32];
uint64_t uint64_val;


int main() {
	uint8_t time_slot_count = 0; //счетчики для отслеживанияинтервалов
	// инициализация переферии
	System_Init();
	MKO_Init(get_mko_addr(MKO_ID)); // установить 0 для работы только от адреса, задаваемого соединителем
	ADC_Init();
	UART0_Init();
	Timers_Init();
	Init_STM(&stm, &cm);
	// инициализация структур управления ЦМ
	CM_Parame_Start_Init(&cm, &cm_old);
	Sys_Frame_Init(&sys_frame);
	//включение питания всей периферии  !!!важно - следить за тем, что бы модули включались не противореча параметру cm.pwr_state !!!todo: будет время - переделать на параметризованное включение
	Pwr_Off_All_Devices();
	//
	Pereph_On_and_Get_ID_Frame(2, &mpp27_init_inf);
	Pereph_On_and_Get_ID_Frame(3, &mpp100_init_inf);
	Pereph_On_and_Get_ID_Frame(4, &dir_init_inf);
	Pwr_Perepherial_Devices_On(); //включаем ДНТ и АДИИ
	// запускаем работу периферии
	MPP_Init(&mpp27, _frame_definer(0, DEV_NUM, 0, MPP27_FRAME_NUM), MPP27_FRAME_NUM, MPP27_ID, MPP27_DEF_OFFSET, &cm);
	MPP_Init(&mpp100, _frame_definer(0, DEV_NUM, 0, MPP100_FRAME_NUM), MPP100_FRAME_NUM, MPP100_ID, MPP100_DEF_OFFSET, &cm);
	DIR_Init(&dir, _frame_definer(0, DEV_NUM, 0, DIR_FRAME_NUM), DIR_FRAME_NUM, DIR_ID, &cm);
	DNT_Init(&dnt, _frame_definer(0, DEV_NUM, 0, DNT_FRAME_NUM), _frame_definer(1, DNT_DEV_NUM, 5, DNT_DEV_FRAME_NUM), DNT_FRAME_NUM, DNT_MKO_ADDR, DIR_ID, &cm);
	ADII_Init(&adii, _frame_definer(0, DEV_NUM, 0, ADII_FRAME_NUM), ADII_FRAME_NUM, DIR_ID, &cm);
	// запускаем вотчдог
	WDT_Init();	
	// запускаем таймер для таймслотов
	Timers_Start(0, SLOT_TIME_MS);
	//
	while(1) {
		WDRST;
		//Циклограмма для опроса и формирования системного кадра: используем счетчик секунд для отсчета интервала, что бы не занимать таймер
		if (Timers_Status(0))
		{   
			Timers_Start(0, SLOT_TIME_MS); // перезапускаем таймер для формирования слота времени (возможная проблема - пропуск слота)
			//формируем всевдо счетчик по 1 с
			time_slot_count += 1;
			if (time_slot_count >= 10){ //обработка событий требующая интервала одна секудна
				time_slot_count = 0;
				//***Обрабока СТМ
				STM_1s_step(&stm, &cm);
				//***формирование и запись параметров в память для их использования в случае выключения питания
				CM_Parame_Processor_1s(&cm);
				//***формируем системный кадр
				Sys_Frame_Processor_1s(&sys_frame, &cm, &cm_old);
				// обрабока состояния ускоренного режима
				Speed_Mode_Processor_1s(&cm);
				//***формирование флагов на запуск измерений по измерительному интервалу
				Measurment_Processor_1s(&cm);
				//***формирование флагов на запуск измерений по измерительному интервалу ДИР
				DIR_Meas_Processor_1s(&cm);
				//***формирование флагов на запуск измерений по измерительному интервалу АДИИ
				ADII_Meas_Processor_1s(&cm);
			}
			if ((time_slot_count % 5) == 0){  //обработка событий требующая скорости больше, чем одна секудна
				Pwr_current_process(&cm); // работа с потреблением
			}
			//***обработка тайм слотов
			switch(time_slot_count%10){  //используя переменную meas_interval организуем 10 таймслотов по 100мс
				case 0:  // МПП: запрос 2х структур
					if (cm.measure_state & (0x1 << MPP27_FRAME_NUM)) MPP_struct_request(&mpp27, &cm);
					if (cm.measure_state & (0x1 << MPP100_FRAME_NUM)) MPP_struct_request(&mpp100, &cm);
					break;
				case 1:  // запрос количества помех в архиве и границы срабатывания
					if (cm.measure_state & (0x1 << MPP27_FRAME_NUM)) MPP_arch_count_offset_get(&mpp27, &cm);
					if (cm.measure_state & (0x1 << MPP100_FRAME_NUM)) MPP_arch_count_offset_get(&mpp100, &cm);
					break;
				case 2: // МПП: забор 2х структур 
					if (cm.measure_state & (0x1 << MPP27_FRAME_NUM)) MPP_struct_get(&mpp27, &cm);
					if (cm.measure_state & (0x1 << MPP100_FRAME_NUM)) MPP_struct_get(&mpp100, &cm);
					break;
				case 3: // МПП: принудительный запуск при необходимости
					if (cm.measure_state & (0x1 << MPP27_FRAME_NUM)) {
						MPP_forced_start(&mpp27, &cm);
						cm.measure_state &= ~(1<<MPP27_FRAME_NUM);
					}
					if (cm.measure_state & (0x1 << MPP100_FRAME_NUM)) {
						MPP_forced_start(&mpp100, &cm);
						cm.measure_state &= ~(1<<MPP100_FRAME_NUM);
					}
					break;
				case 4:  // ДИР: чтение резуьтата и запуск измерения
					if (cm.measure_state & (0x1 << DIR_FRAME_NUM)) {
						DIR_Data_Get(&dir, &cm);
						DIR_Start_Measurement(&dir, &cm);
						cm.measure_state &= ~(1<<DIR_FRAME_NUM);
					}
					break;
				case 5: // ДНТ: чтение результата запроса запуска измерения и отправка запроса на чтение результата
					if (cm.measure_state & (0x1 << DNT_FRAME_NUM)){
						DNT_MKO_Measure_Finish(&dnt, &cm);
						DNT_MKO_Read_Initiate(&dnt, &cm);
					}
					break;
				case 6: // ДНТ: чтение результата запроса чтения результатов и отправка запроса на запуск измерения
					if (cm.measure_state & (0x1 << DNT_FRAME_NUM)){
						DNT_MKO_Read_Finish(&dnt, &cm);
						DNT_MKO_Measure_Initiate(&dnt, &cm);
						//
						cm.measure_state &= ~(1<<DNT_FRAME_NUM);
					}
					break;
				case 7: // АДИИ: 
					if (cm.measure_state & (0x1 << ADII_FRAME_NUM)){
						ADII_Meas_Start(&adii, &cm);  //управление командой происходит переменной adii.ctrl.mode: 1 - режим тестирования, 0 - нормальный режим
						//
					}
					break;
				case 8: // Отправка системного кадра в ВШ
						F16_IB_data_transfer(&cm, NU_ID, 0x00, 32, (uint16_t*)&sys_frame);
					break;
				case 9:
					if (cm.measure_state & (0x1 << ADII_FRAME_NUM)){
						ADII_Read_Data(&adii, &cm); //чтение данных АДИИ переставляем дальше от запуска, иначе ДИР не отвечает
						//
						cm.measure_state &= ~(1<<ADII_FRAME_NUM);
					}
					break;
			}
		}
		//***Прием команд по МКО
		if (MKO_IVect(&cm.mko_error, &cm.mko_error_cnt) != 0x0000){
			if (mko_dev.subaddr == COMMAND_MESSAGE_SA) {  //обработка командных сообщений
				if (mko_dev.data[0] == 0x0001) {  //  синхронизация времени
					Get_Time_sec_parts(&cm.sync_time_s, &cm.sync_time_low);
					uint64_val = ((uint64_t)mko_dev.data[1] << 32) + ((uint64_t)mko_dev.data[2] << 16); // + ((uint64_t)mko_dev.data[3] << 0)) & 0xFFFFFFFFFFFF; часть для дробной синхронизации
					Time_Set(uint64_val, &cm.diff_time_s, &cm.diff_time_low);
					MPP_time_set();
					cm.sync_num += 1;
				}
				else if (mko_dev.data[0] == 0x0002) {  //  инициализация ЦМ
					// перевключение питания
					Pwr_Off_All_Devices();  // 1000 мс
					// инициализируем память с привязкой к номеру ячейки
					Format_Mem();  // мс
					// последовательно включаем все питание
					Pereph_On_and_Get_ID_Frame(2, &mpp27_init_inf); // 500 мс
					Pereph_On_and_Get_ID_Frame(3, &mpp100_init_inf); // 500 мс
					Pereph_On_and_Get_ID_Frame(4, &dir_init_inf); // 500 мс
					Pwr_Perepherial_Devices_On(); //включаем ДНТ и АДИИ - 4000мс
					//инициализация структур переферии //500 мс
					MPP_Init(&mpp27, _frame_definer(0, DEV_NUM, 0, MPP27_FRAME_NUM), MPP27_FRAME_NUM, MPP27_ID, MPP27_DEF_OFFSET, &cm);
					MPP_Init(&mpp100, _frame_definer(0, DEV_NUM, 0, MPP100_FRAME_NUM), MPP100_FRAME_NUM, MPP100_ID, MPP100_DEF_OFFSET, &cm);
					DIR_Init(&dir, _frame_definer(0, DEV_NUM, 0, DIR_FRAME_NUM), DIR_FRAME_NUM, DIR_ID, &cm);
					DNT_Init(&dnt, _frame_definer(0, DEV_NUM, 0, DNT_FRAME_NUM), _frame_definer(1, DNT_DEV_NUM, 5, DNT_DEV_FRAME_NUM), DNT_FRAME_NUM, DNT_MKO_ADDR, DIR_ID, &cm);
					ADII_Init(&adii, _frame_definer(0, DEV_NUM, 0, ADII_FRAME_NUM), ADII_FRAME_NUM, DIR_ID, &cm);
					//инициализация памяти МПП
					MPP_mem_init(&mpp27, &cm);
					MPP_mem_init(&mpp100, &cm);
					//обнуляем время
					Time_Set(0, &cm.diff_time_s, &cm.diff_time_low);
					//инициализация структуры управления
					CM_Parame_Command_Init(&cm);
				}
				else if (mko_dev.data[0] == 0x0003) {  //  установка измерительного интервала
					if (mko_dev.data[1] == 0){  // измерительный интервал
						cm.measure_interval = get_val_from_bound(mko_dev.data[2], 10, 3600);
					}
					else if (mko_dev.data[1] == 1){  // системный интервал
						cm.sys_interval = get_val_from_bound(mko_dev.data[2], 10, 3600);
					}
					else if (mko_dev.data[1] == 2){  // интервал опроса АДИИ
						cm.adii_measure_interval = get_val_from_bound(mko_dev.data[2], 10, 3600);
					}
					else if (mko_dev.data[1] == 3){  // интервал опроса АДИИ
						cm.adii_depol_interval = get_val_from_bound(mko_dev.data[2], 10, 3600);
					}
				}
				else if (mko_dev.data[0] == 0x0004) {  // установка указателя чтения ЗУ
					cm.read_ptr = get_val_from_bound( mko_dev.data[3], 0, Get_Max_Data_Frame_Num());
				}
				else if (mko_dev.data[0] == 0x0005) {  // переход в защищенную область
					if (cm.defend_mem) Move_Read_Ptr_To_Defended_Mem(&cm);
				}
				else if (mko_dev.data[0] == 0x0006) {  // установка отсечки для МПП
					if (mko_dev.data[1] == 1) MPP_Offset_Set(&mpp27, mko_dev.data[2], &cm);
					else if (mko_dev.data[1] == 2) MPP_Offset_Set(&mpp100, mko_dev.data[2], &cm);
				}
				else if (mko_dev.data[0] == 0x0007) {  // управление АДИИ
					if (mko_dev.data[1] == 0){
							adii.ctrl.mode = 1;
							// ADII_Meas_Start(&adii);  //возможно, лучше сразу запустить АДИИ, а не ждать следующего измерительного интервала - требуется проверка
					}
				}
				else if (mko_dev.data[0] == 0x0008) {  // включение режима констант
					MPP_constatnt_mode(mko_dev.data[1] & 0x01);  // широковещательная; mode: 1 - on; 0 - off
					//
					DNT_MKO_Constant_Mode(&dnt, &cm, 0x01 & mko_dev.data[1]);
				}
				else if (mko_dev.data[0] == 0x0009) {  // установка уровня токовой защиты
					if (mko_dev.data[1] <= 6) cm.pwr_bounds[mko_dev.data[1]] = mko_dev.data[2];
				}
				else if (mko_dev.data[0] == 0x000A) {  // установка состояния питания модулей
					Set_Pwr_State(&cm.pwr_state, &cm.pwr_status, mko_dev.data[1] & 0xFF, mko_dev.data[2] & 0x01);
				}
				else if (mko_dev.data[0] == 0x000B) {  // запуск ускоренного режима
					Set_Speedy_Mode(&cm, mko_dev.data[1], mko_dev.data[2], mko_dev.data[3]);
				}
				else if (mko_dev.data[0] == 0x000C) {  // управление порогом отключения МБКАП по МПП100
					MPP_Pwr_Off_Bound_Set(&mpp100, mko_dev.data[1], &cm);
				}
				else if (mko_dev.data[0] == 0x000D) {  // управление напряжением работы ДИР
					dnt.ctrl.mode = mko_dev.data[1];
				}
			}
			else if(mko_dev.subaddr == ARCH_FRAME_REQ_SA){  // обновление кадра на ПА из ЗУ ЦМ
				Load_Data_Frame(&cm);
			}
			else if(mko_dev.subaddr == TECH_COMMAND_SA){  // технологический ПА
				// копируем содержимое, но меняем в первом слове 8й бит на 1
				Read_from_SubAddr(mko_dev.subaddr, dbg_data);
				dbg_data[0]  |= 0x0100;
				// разбираем команды
				if  (mko_dev.data[0] == 0x0001) { // зеркало для проверки связи
				}
				else if(mko_dev.data[0] == 0x0002){  // чтение произвольного блока памяти ЗУ ЦМ (в том числеи и с параметрами ЦМ) с выкладыванием на 14-й подадрес
					uint16_val = mko_dev.data[1]; //адрес
					dbg_data[2] = Read_Frame(uint16_val,  (uint8_t*)uint16_arr); 
					Write_to_SubAddr(ARCH_FRAME_DATA_SA, uint16_arr);
				}
				else if(mko_dev.data[0] == 0x0003){  // неразрушающая проверка памяти
					dbg_data[1] = Check_Mem(); 
				}
				else if(mko_dev.data[0] == 0x0004){  // отправка команды по ВШ
					Tech_SA_Transaction((uint16_t*)&dbg_data[0]);
				}
				else if(mko_dev.data[0] == 0x0005){  // установка СТМ на 20 секунд
					Set_STM_from_uint16_val(&stm, &cm, mko_dev.data[1], 20);
				}
				// todo: доделать тех подадрес
				Write_to_SubAddr(TECH_COMMAND_SA, dbg_data);
			}
		}
		//***Отладочный порт //работает по команде 0х10 !!!
		if(Debug_Get_Packet(&reg_addr, dbg_data, &leng) == 0x01) {
			if((reg_addr == 0x00) & (leng >= 2)){  // проверка адресации
				cm.debug = ((dbg_data[1] >> 8) & 0x01) ^ 0x01;
			}
			else if ((reg_addr == 0x01) & (leng >= 2)) {
				GPIO_Pwr((dbg_data[0] >> 8), (dbg_data[1] >> 8) & 0x01);
			}
		}
	}
}

