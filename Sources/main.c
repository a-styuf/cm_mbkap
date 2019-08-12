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
//Настройки программы в файле mbkap.c

//
extern typeCMParameters cm;
extern typeSysFrames sys_frame;
extern typeSTMstruct stm;
//
uint8_t Buff[256];
extern uint16_t ADCData[];
uint8_t leng, dbg_status;
uint16_t reg_addr, dbg_data[32];

int main() {
	uint8_t leng;
	uint16_t cm_param_count = 0, sys_frame_count = 0, meas_interv_count = 0; //счетчики для отслеживанияинтервалов
	// инициализация переферии
	System_Init();
	MKO_Init(get_mko_addr(MKO_ID)); //установить 0 для работы только от адреса, задаваемого соединителем
	ADC_Init();
	UART0_Init();
	Timers_Init();
	Init_STM(&stm, &cm);
	// инициализация структур
	CM_Parame_Start_Init(&cm);
	Sys_Frame_Init(&sys_frame);
	//
	Pwr_All_Perepherial_Devices_On();
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
			//***формируем системный кадр
			meas_interv_count += 1;
			if (meas_interv_count >= cm.measure_interval*10) 
            {
				meas_interv_count = 0;
				//
				                
            }			
		}
		//Прием команд по МКО
        if (MKO_IVect(&cm.mko_error, &cm.mko_error_cnt) != 0x0000){			
		}
		//Отладочный порт //работает по команде 0х10 !!!
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

