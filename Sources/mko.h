#ifndef _MKO_H_
#define _MKO_H_


#define IRQn_MIL_STD_15531 (IRQn_Type)117


void MKO_Init(uint16_t mko_addr);
uint16_t MKO_IVect(void);


#endif

