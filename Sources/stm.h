#ifndef _STM_H_
#define _STM_H_


#include "1986ve8_lib/cm4ikmcu.h"
#include "mbkap.h"

typedef struct  // структура состояния СТМ: если поля больше 0, то выставяем замкнуто, если меньше нуля - то разомкнуто
{
	// изменяемая часть от прибора к прибору
    int8_t NKPBE;
    int8_t AMKO;
	// обязательная часть
	int8_t block; //блокируется изменение СТМ
}typeSTMstruct;

void Init_STM(typeSTMstruct* stm_ptr, typeCMParameters *cm_ptr);
void STM_1s_step(typeSTMstruct* stm_ptr, typeCMParameters *cm_ptr);
void Set_STM(typeSTMstruct* stm_ptr, typeCMParameters *cm_ptr);
void Set_STM_from_uint16_val(typeSTMstruct* stm_ptr, typeCMParameters *cm_ptr, uint16_t stm_val, uint8_t time_out);

#endif
