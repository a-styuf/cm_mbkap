#include "1986ve8_lib/cm4ikmcu.h"
#include "eerom.h"
#include "string.h"
#include "wdt.h"

int8_t Write_Frame(uint16_t address, uint8_t* frame)
{
	if (address < MAX_FRAME_SIZE){
		if ((address >= 0) && (address < MEM1_FRAME_SIZE)){
			memcpy((uint8_t*)(MEM1_OFFSET + address*64), frame, 64);
			return 1;
		}
		else if ((address >= MEM1_FRAME_SIZE) && (address < MEM1_FRAME_SIZE + MEM2_FRAME_SIZE)){
			memcpy((uint8_t*)(MEM2_OFFSET + (address - MEM1_FRAME_SIZE)*64), frame, 64);
			return 1;
		}
		else{
			return 0; // если пытаемся вылезти за пределы двух памятей
		}	
	}
	else{
		return 0; // если пытаемся вылезти за пределы отвыеденной памяти
	}
}

int8_t Read_Frame(uint16_t address, uint8_t* frame)
{
	if (address < MAX_FRAME_SIZE){
		if ((address >= 0) && (address < MEM1_FRAME_SIZE)){
			memcpy(frame, (uint8_t*)(MEM1_OFFSET + address*64), 64);
			return 1;
		}
		else if ((address >= MEM1_FRAME_SIZE) && (address < MEM1_FRAME_SIZE + MEM2_FRAME_SIZE)){
			memcpy(frame, (uint8_t*)(MEM2_OFFSET + (address - MEM1_FRAME_SIZE)*64), 64);
			return 1;
		}
		else{
			return 0; // если пытаемся вылезти за пределы двух памятей
		}	
	}
	else{
		return 0; // если пытаемся вылезти за пределы отвыеденной памяти
	}
}

uint16_t Get_Max_Data_Frame_Num(void)
{
	return (MAX_FRAME_SIZE > MEM1_FRAME_SIZE) ? (MAX_FRAME_SIZE-8) : (MAX_FRAME_SIZE-4); // если используем только одну память, то -2, если две - -4
}

int8_t Format_Mem(void) // функция инициализации памяти 0xFEFE, но с адресом в начале каждого блока из 64 байт
{
	uint16_t i;
	uint8_t block[64];
	memset(block, 0xFE, 64);
	for (i=0; i<MAX_FRAME_SIZE; i++)
	{
		WDRST;
		*(uint16_t*)&block[0] = i;
		Write_Frame(i, block);
	}
	return 0;
}

uint16_t Check_Mem(void)
{
	uint16_t i, mem_state = 0;
	uint8_t test_frame[64] = {0}, read_frame[64] = {0}, mem_frame[64] = {0};
	//создаем тестовые сигналы
	for (i=0; i<64; i++) {
		test_frame[i] = i & 0xFF;
	}
    for (i=1; i < MAX_FRAME_SIZE; i++)
    {
        WDRST;
		//сохраняем данные из памяти и пишем тестовый блок во все 4 памяти
		Read_Frame(i, mem_frame);
		Write_Frame(i, test_frame);
		Read_Frame(i, read_frame);
		if (memcmp(read_frame, test_frame, 64) != 0) {
			mem_state = i;
			break;
		}
		Write_Frame(i, mem_frame);
		}
	return mem_state;
}

