#include "stm.h"
#include "gpio.h"

typeSTMstruct stm;

void Init_STM(typeSTMstruct* stm_ptr, typeCMParameters *cm_ptr)
{
	// ставим для КПБЕ и НормЦМ включение на 20 секунд
    stm_ptr->NKPBE = 20*GPIO_Get_CM_Id();
    stm_ptr->AMKO = 0;
	stm_ptr->block = 0;
	Set_STM(stm_ptr, cm_ptr);
}

void STM_1s_step(typeSTMstruct* stm_ptr, typeCMParameters *cm_ptr)
{
	if (stm_ptr->block == 0){ // изменение состояния stm в случае отсутствия блокировки
		if (stm_ptr->NKPBE != 0) {
			stm_ptr->NKPBE += ((stm_ptr->NKPBE < 0) ? 1 : -1); 
		}
		if (stm_ptr->AMKO != 0) {
			stm_ptr->AMKO += ((stm_ptr->AMKO < 0) ? 1 : -1); 
		}
		Set_STM(stm_ptr, cm_ptr);
	}
	else {  // уменьшение времени блокировки
		stm_ptr->block += ((stm_ptr->block < 0) ? 1 : -1); 
	}
}

void Set_STM(typeSTMstruct* stm_ptr, typeCMParameters *cm_ptr)
{
	cm_ptr->stm_val = (((stm_ptr->NKPBE > 0) ? 0x01 : 0x00) << 0) + (((stm_ptr->AMKO > 0) ? 0x01 : 0x00) << 1);
    GPIO_TM(0, cm_ptr->stm_val & 0x01); // 0: NKBE
    GPIO_TM(1, cm_ptr->stm_val & 0x02); // 1: AMKO
}

void Set_STM_from_uint16_val(typeSTMstruct* stm_ptr, typeCMParameters *cm_ptr, uint16_t stm_val, uint8_t time_out)
{
	stm_ptr->NKPBE = (stm_val >> 1) & 0x01;
    stm_ptr->AMKO = (stm_val >> 0) & 0x01;
	stm_ptr->block = time_out;
	Set_STM(stm_ptr, cm_ptr);
}
