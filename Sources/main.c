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
extern typeCMParameters cm;
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
//
uint8_t Buff[256];
extern uint16_t ADCData[];
uint8_t leng, dbg_status;
uint16_t reg_addr, dbg_data[32];
uint64_t uint64_val;

int main() {
	uint16_t cm_param_count = 0, sys_frame_count = 0, meas_interv_count = 0, adii_interv_count = 0; //счетчики для отслеживанияинтервалов
	// инициализация переферии
	System_Init();
	MKO_Init(get_mko_addr(MKO_ID)); //установить 0 для работы только от адреса, задаваемого соединителем
	ADC_Init();
	UART0_Init();
	Timers_Init();
	Init_STM(&stm, &cm);
	// инициализация структур управления ЦМ
	CM_Parame_Start_Init(&cm);
	Sys_Frame_Init(&sys_frame);
	//включение питания периферии
	Pereph_On_and_Get_ID_Frame(2, &mpp27_init_inf);
	Pereph_On_and_Get_ID_Frame(3, &mpp100_init_inf);
	Pereph_On_and_Get_ID_Frame(4, &dir_init_inf);
	Pwr_Perepherial_Devices_On(); //включаем ДНТ и АДИИ
	// запускаем работу периферии
	MPP_Init(&mpp27, _frame_definer(0, DEV_NUM, 0, MPP27_FRAME_NUM), MPP27_FRAME_NUM, MPP27_ID, MPP27_DEF_OFFSET);
	MPP_Init(&mpp100, _frame_definer(0, DEV_NUM, 0, MPP100_FRAME_NUM), MPP100_FRAME_NUM, MPP100_ID, MPP100_DEF_OFFSET);
	DIR_Init(&dir, _frame_definer(0, DEV_NUM, 0, DIR_FRAME_NUM), DIR_FRAME_NUM, DIR_ID);
	DNT_Init(&dnt, _frame_definer(0, DEV_NUM, 0, DNT_FRAME_NUM), _frame_definer(0, DNT_DEV_NUM, 5, DNT_FRAME_NUM), DNT_FRAME_NUM, DNT_MKO_ADDR, DIR_ID);
	ADII_Init(&adii, _frame_definer(0, DEV_NUM, 0, ADII_FRAME_NUM), ADII_FRAME_NUM, DIR_ID);
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
			//***формирование и запись параметров в память для их использования в случае выключения питания
            cm_param_count += 1;
            if (cm_param_count >= CM_PARAM_SAVE_PERIOD_S*10)
            {
				cm_param_count = 0;
				// работа с потреблением
				Pwr_current_process(&cm);
				Pwr_Ctrl_by_State(cm.pwr_state);
				//работа со временем ЦМ
                cm.operating_time += CM_PARAM_SAVE_PERIOD_S; //todo: возможная проблема - расхождения времени и времени наработки из-за пропусков секундных интервалов
                cm.time = Get_Time_s();
				//не забываем сохранять структуру в память
                Write_Parameters(&cm);
				//выставляем СТМ
				stm.NKPBE = (GPIO_Get_CM_Id() != 0) ? 10: 0;
				STM_1s_step(&stm, &cm);
            }
			//***формируем системный кадр
            sys_frame_count += 1;  
            if (sys_frame_count >= cm.sys_interval*10) 
            {
				sys_frame_count = 0;
				//
                Sys_Frame_Build(&sys_frame, &cm);
            }
			//***формирование флагов на запуск измерений по измерительному интервалу
			meas_interv_count += 1;
			if (meas_interv_count >= cm.measure_interval*10) 
            {
				meas_interv_count = 0;
				//
				cm.normal_mode_state = 	(1<<MPP27_FRAME_NUM)|
															(1<<MPP100_FRAME_NUM)|
															(1<<DIR_FRAME_NUM)|
															(1<<DNT_FRAME_NUM);
            }			
			//***формирование флагов на запуск измерений по измерительному интервалу
			adii_interv_count += 1;
			if (adii_interv_count >= cm.adii_interval*10) 
            {
				adii_interv_count = 0;
				//
				cm.normal_mode_state = (1<<ADII_FRAME_NUM);
            }			
			//***обработка тайм слотов
			switch(meas_interv_count%10){  //используя переменную meas_interval организуем 10 таймслотов по 100мс
				case 0:  // МПП: запрос 2х структур
					if (cm.normal_mode_state & (0x1 << MPP27_FRAME_NUM)) MPP_struct_request(&mpp27);
					if (cm.normal_mode_state & (0x1 << MPP100_FRAME_NUM)) MPP_struct_request(&mpp100);
					break;
				case 1:  // запрос количества помех в архиве и границы срабатывания
					if (cm.normal_mode_state & (0x1 << MPP27_FRAME_NUM)) MPP_arch_count_offset_get(&mpp27);
					if (cm.normal_mode_state & (0x1 << MPP100_FRAME_NUM)) MPP_arch_count_offset_get(&mpp100);
					break;
				case 2: // МПП: забор 2х структур 
					if (cm.normal_mode_state & (0x1 << MPP27_FRAME_NUM)) {
						MPP_struct_get(&mpp27, &cm);
					}
					if (cm.normal_mode_state & (0x1 << MPP100_FRAME_NUM)) {
						MPP_struct_get(&mpp100, &cm);
					}
					cm.normal_mode_state &= ~((1<<MPP27_FRAME_NUM)|(1<<MPP100_FRAME_NUM));
					break;
				case 3:  // ДИР: чтение резуьтата и запуск измерения
					if (cm.normal_mode_state & (0x1 << DIR_FRAME_NUM)) {
						DIR_Data_Get(&dir, &cm);
						DIR_Start_Measurement(&dir);
						cm.normal_mode_state &= ~(1<<DIR_FRAME_NUM);
					}
					break;
				case 4: // ДНТ: чтение результата запроса запуска измерения и отправка запроса на чтение результата
					if (cm.normal_mode_state & (0x1 << DNT_FRAME_NUM)){
						DNT_MKO_Measure_Finish(&dnt, &cm);
						DNT_MKO_Read_Initiate(&dnt);
					}
					break;
				case 5: // ДНТ: чтение результата запроса чтения результатов и отправка запроса на запуск измерения
					if (cm.normal_mode_state & (0x1 << DNT_FRAME_NUM)){
						DNT_MKO_Read_Finish(&dnt, &cm);
						DNT_MKO_Measure_Initiate(&dnt);
						//
						cm.normal_mode_state &= ~(1<<DNT_FRAME_NUM);
					}
					break;
				case 6: // АДИИ: 
					if (cm.normal_mode_state & (0x1 << ADII_FRAME_NUM)){
						ADII_Read_Data(&adii, &cm);
						ADII_Meas_Start(&adii);  //управление командой происходит переменной adii.ctrl.mode: 1 - режим тестирования, 0 - нормальный режим
						//
						cm.normal_mode_state &= ~(1<<ADII_FRAME_NUM);
					}
					break;
				case 7:
					break;
				case 8:
					break;
				case 9:
					break;
			}
		}
		//***Прием команд по МКО
        if (MKO_IVect(&cm.mko_error, &cm.mko_error_cnt) != 0x0000){
			if (mko_dev.subaddr == 0x11) {  //обработка командных сообщений
                if (mko_dev.data[0] == 0x0001) {  //  синхронизация времени
					Get_Time_sec_parts(&cm.sync_time_s, &cm.sync_time_low);
					uint64_val = ((uint64_t)mko_dev.data[1] << 32) + ((uint64_t)mko_dev.data[2] << 16); // + ((uint64_t)mko_dev.data[3] << 0)) & 0xFFFFFFFFFFFF; часть для дробной синхронизации
					Time_Set(uint64_val, &cm.diff_time_s, &cm.diff_time_low);
					cm.sync_num += 1;
                }
				else if (mko_dev.data[0] == 0x0002) {  //  инициализация ЦМ
					// перевключение питания
					Pwr_Off_All_Devices();
					// инициализируем память с привязкой к номеру ячейки
					Format_Mem();
					// последовательно включаем все питание
					Pereph_On_and_Get_ID_Frame(2, &mpp27_init_inf);
					Pereph_On_and_Get_ID_Frame(3, &mpp100_init_inf);
					Pereph_On_and_Get_ID_Frame(4, &dir_init_inf);
					Pwr_Perepherial_Devices_On(); //включаем ДНТ и АДИИ
					//инициализация структур переферии
					MPP_Init(&mpp27, _frame_definer(0, DEV_NUM, 0, MPP27_FRAME_NUM), MPP27_FRAME_NUM, MPP27_ID, MPP27_DEF_OFFSET);
					MPP_Init(&mpp100, _frame_definer(0, DEV_NUM, 0, MPP100_FRAME_NUM), MPP100_FRAME_NUM, MPP100_ID, MPP100_DEF_OFFSET);
					DIR_Init(&dir, _frame_definer(0, DEV_NUM, 0, DIR_FRAME_NUM), DIR_FRAME_NUM, DIR_ID);
					DNT_Init(&dnt, _frame_definer(0, DEV_NUM, 0, DNT_FRAME_NUM), _frame_definer(0, DNT_DEV_NUM, 5, DNT_FRAME_NUM), DNT_FRAME_NUM, DNT_MKO_ADDR, DIR_ID);
					ADII_Init(&adii, _frame_definer(0, DEV_NUM, 0, ADII_FRAME_NUM), ADII_FRAME_NUM, DIR_ID);
					//инициализация памяти МПП
					MPP_mem_init(&mpp27);
					MPP_mem_init(&mpp100);
					//обнуляем время
					Time_Set(0, &cm.diff_time_s, &cm.diff_time_low);
					//инициализация структуры управления
					CM_Parame_Command_Init(&cm);
                }
			}    
        }
		//***Отладочный порт //работает по команде 0х10 !!!
        if(Debug_Get_Packet(&reg_addr, dbg_data, &leng) == 0x01) 
        {
            if((reg_addr == 0x00) & (leng >= 2)){  // проверка адресации
				dbg_status = ((dbg_data[1] >> 8) & 0x01) ^ 0x01;
			}
			else if ((reg_addr == 0x01) & (leng >= 2)) {
				GPIO_Pwr((dbg_data[0] >> 8), (dbg_data[1] >> 8) & 0x01);
			}
        }
	}
}

