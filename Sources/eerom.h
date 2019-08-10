#ifndef _EEROM_H_
#define _EEROM_H_

#define MEM1_OFFSET 0x50000000 // согласно описанию на 1986У
#define MEM2_OFFSET 0x60000000

#define MEM1_SIZE 0x00020000 // 1Mb
#define MEM2_SIZE 0x00020000 // 1Mb

#define MEM1_FRAME_SIZE 0x0800 // 2048 кадров
#define MEM2_FRAME_SIZE 0x0800 //2048 кадров

#define MAX_FRAME_SIZE 0x0800 // 2048 кадров - для работы с одной памятью 3DMR1M08

int8_t Write_Frame(uint16_t address, uint8_t* frame);
int8_t Read_Frame(uint16_t address, uint8_t* frame);
uint16_t Get_Max_Data_Frame_Num(void);

#endif
