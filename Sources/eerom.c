#include "1986ve8_lib/cm4ikmcu.h"
#include "eerom.h"
#include "string.h"


int8_t Write_Frame(uint16_t address, uint8_t* frame)
{
	if (address < MAX_FRAME_SIZE){
		if ((address >= 0) && (address < MEM1_FRAME_SIZE)){
			memcpy((uint8_t*)(MEM1_OFFSET + address*64), frame, 64);
			return 0;
		}
		else if ((address >= MEM1_FRAME_SIZE) && (address < MEM1_FRAME_SIZE + MEM2_FRAME_SIZE)){
			memcpy((uint8_t*)(MEM2_OFFSET + address*64), frame, 64);
			return 0;
		}
		else{
			return -1; // если пытаемся вылезти за пределы двух памятей
		}	
	}
	else{
		return -1; // если пытаемся вылезти за пределы отвыеденной памяти
	}
}

int8_t Read_Frame(uint16_t address, uint8_t* frame)
{
	if (address < MAX_FRAME_SIZE){
		if ((address >= 0) && (address < MEM1_FRAME_SIZE)){
			memcpy(frame, (uint8_t*)(MEM1_OFFSET + address*64), 64);
			return 0;
		}
		else if ((address >= MEM1_FRAME_SIZE) && (address < MEM1_FRAME_SIZE + MEM2_FRAME_SIZE)){
			memcpy(frame, (uint8_t*)(MEM2_OFFSET + address*64), 64);
			return 0;
		}
		else{
			return -1; // если пытаемся вылезти за пределы двух памятей
		}	
	}
	else{
		return -1; // если пытаемся вылезти за пределы отвыеденной памяти
	}
}

uint16_t Get_Max_Data_Frame_Num(void)
{
	return (MAX_FRAME_SIZE > MEM1_FRAME_SIZE) ? (MAX_FRAME_SIZE-8) : (MAX_FRAME_SIZE-4); // если используем только одну память, то -2, если две - -4
}
