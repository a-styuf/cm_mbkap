#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_

uint8_t Get_Modules_Current(uint16_t *currents, uint16_t *pwr_bounds);
void Pwr_All_Perepherial_Devices_On(void);
void Pwr_Perepherial_Devices_On(void);
void Pwr_Ctrl_by_State(uint8_t pwr_state);

#endif
