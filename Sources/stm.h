#ifndef _STM_H_
#define _STM_H_


#include "1986ve8_lib/cm4ikmcu.h"
#include "mbkap.h"

void Init_STM(typeSTMstruct* stm_ptr, typeCMParameters *cm);
void STM_1s_step(typeSTMstruct* stm_ptr, typeCMParameters *cm_ptr);
void Set_STM(typeSTMstruct* stm_ptr, typeCMParameters *cm_ptr);

#endif
