#ifndef _UARTS_H_
#define _UARTS_H_


void UART0_Init(void);
void UART0_SendPacket(uint8_t *buff, uint8_t leng, uint8_t en_crc_flg);
int8_t UART0_GetPacket(uint8_t *buff, uint8_t *leng);


#endif

