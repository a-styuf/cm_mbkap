#ifndef _GPIO_H_
#define _GPIO_H_

uint8_t GPIO_Get_CM_Id(void);
void GPIO_Gen(uint8_t on);
void GPIO_TM(uint8_t tm_num, uint8_t on);   //tm_num = 0,1: NKBE, AMKO
void GPIO_Pwr(uint8_t pwr_num, uint8_t on);   //pwr_num: 0-CM, 1-MPP27, 2-MPP100, 3-DIR, 4-DNT, 5-ADII 
uint8_t GPIO_MKO_Id(void);

#endif


