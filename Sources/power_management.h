#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_

#include "adc.h"
#include "gpio.h"
#include "timers.h"

uint8_t Get_Modules_Current(uint16_t *currents, uint16_t *pwr_bounds);
void Pwr_On_All_Devices(void);
void Pwr_Off_All_Devices(void); // GPIO_Pwr(uint8_t pwr_num, uint8_t on)  //pwr_num: 0-CM, 1-MPP27, 2-MPP100, 3-DIR, 4-DNT, 5-ADII 
void Pwr_Perepherial_Devices_On(void);
void Pwr_Ctrl_by_State(uint8_t pwr_state);
uint8_t Set_Pwr_State(uint8_t *pwr_state_ptr, uint8_t *pwr_status_ptr, uint8_t dev_num, uint8_t on); //функция, через которую необходимо устанавливать состояние питания модулей, простая установка переменный cm.state не принесет результата

#endif
