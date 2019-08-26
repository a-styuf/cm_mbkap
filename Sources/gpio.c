#include "gpio.h"

uint8_t GPIO_Get_CM_Id(void) 
{
  return (uint8_t)((PORTB->RXTX) & 0x01);
}

void GPIO_TM(uint8_t tm_num, uint8_t on)   //tm_num = 0,1: NKBE, AMKO
{ 
  if(tm_num <= 1) {
    if(on) PORTE->CRXTX = (1<<(30+tm_num));
    else   PORTE->SRXTX = (1<<(30+tm_num));
    }
}

void GPIO_Pwr(uint8_t pwr_num, uint8_t on)  //pwr_num: 0-MBKAP, 1-CM, 2-MPP27, 3-MPP100, 4-DIR, 5-DNT, 6-ADII 
 {  
	switch(pwr_num){
		case 0: // MBKAP
			//
			break;
		case 1: // CM
			if (on) PORTE->CRXTX = (1<<16);
			else PORTE->SRXTX = (1<<16);
			break;
		case 2:  // MPP27
			if (on) PORTE->CRXTX = (1<<17);
			else PORTE->SRXTX = (1<<17);
			break;
		case 3:  // MPP100
			if (on) PORTE->CRXTX = (1<<19);
			else PORTE->SRXTX = (1<<19);
			break;
		case 4:  // DIR
			if (on) PORTE->CRXTX = (1<<18);
			else PORTE->SRXTX = (1<<18);
			break;
		case 5: // DNT
			if (on){
				PORTE->SRXTX = (1<<22);
				// PORTE->CRXTX = (1<<23);
			} 
			else {
				PORTE->SRXTX = (1<<23);
				// PORTE->CRXTX = (1<<22);
			}
			Timers_Start(1, 20); 
			while (Timers_Status(1) == 0) {};
			PORTE->CRXTX = (1<<22);
			PORTE->CRXTX = (1<<23);
			break;
		case 6: // ADII
			if (on){
				PORTE->SRXTX = (1<<20);
				// PORTE->CRXTX = (1<<21);
			} 
			else {
				PORTE->SRXTX = (1<<21);
				// PORTE->CRXTX = (1<<20);
			}
			Timers_Start(1, 20); 
			while (Timers_Status(1) == 0) {};
			PORTE->CRXTX = (1<<20);
			PORTE->CRXTX = (1<<21);
			break;
	}
}

uint8_t GPIO_MKO_Id(void) 
{
	return (uint8_t)((PORTB->RXTX >> 9) & 0x3F);
}



