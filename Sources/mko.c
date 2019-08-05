#include <string.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "mko.h"


uint16_t MKOIVect;


void MKO_Init(uint16_t mko_addr) {
  uint32_t saved_reg;
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->PER1_CLK |= (1<<19);  //enable clock for MIL0
  MIL_STD_15531->CONTROL = 1;  //reset
  MIL_STD_15531->CONTROL = (1<<20)|(40<<11)|(3<<4)|(2<<2)|((mko_addr & 0x1F)<<6);
  MIL_STD_15531->StatusWord1 = ((mko_addr & 0x1F)<<11);
  saved_reg = CLK_CNTR->PER0_CLK;
  CLK_CNTR->PER0_CLK |= (1<<13);  //clock for PORTA
  PORTA->KEY = _KEY_;
  PORTA->CFUNC[2]  = 0xF0FF0000;
  PORTA->CFUNC[3]  = 0x00FF0FFF;
  PORTA->SFUNC[2]  = 0xC0CC0000;
  PORTA->SFUNC[3]  = 0x00CC0CCC;
  PORTA->SANALOG   = 0x37B00000;
  PORTA->SPWR[1]   = 0x003C0F00;
  PORTA->SPULLDOWN = 0x31800000;
  CLK_CNTR->PER0_CLK = saved_reg;
  /*enable interrupt*/
  MIL_STD_15531->INTEN = 0x0004;  //valmess int
  NVIC_EnableIRQ(IRQn_MIL_STD_15531);
}


uint16_t MKO_IVect() {
  uint16_t ret;
  NVIC_DisableIRQ(IRQn_MIL_STD_15531);  
  ret = MKOIVect;
  MKOIVect = 0;
  NVIC_EnableIRQ(IRQn_MIL_STD_15531);
  return ret;
}


void INT_MIL0_Handler(void) {
  uint16_t cmd_word;
  cmd_word = MIL_STD_15531->CommandWord1;
  if(MIL_STD_15531->ERROR == 0) {
    MKOIVect = (uint16_t)cmd_word;
    if((cmd_word & 0x07E0) == 0x03C0) {    /* write to subaddr=30  ? */
      cmd_word &= 0x1F;
      if(cmd_word == 0) cmd_word = 32;
      memcpy((char*)&MIL_STD_15531->DATA[30*32], (char*)&MIL_STD_15531->DATA[30*32], cmd_word << 2);
      }
    }
}


