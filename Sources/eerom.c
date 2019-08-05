#include "1986ve8_lib/cm4ikmcu.h"
#include "eerom.h"

#define NULL (void*)0

typeEEProgHeader EEProgHeader;


void SPI_Init() {
  CLK_CNTR->PER1_CLK |= 4;  //enable clock for SSP0
  CLK_CNTR->SSP0_CLK = 0x10010000;
  MDR_SSP0->CR0 = 0x407;  // rate=1MHz,  SPH=0, SPO=0, Motorola, 8bit
  MDR_SSP0->CPSR = 2;  // rate=1MHz
  CLK_CNTR->PER0_CLK |= 0x00002000;
  PORTA->KEY = _KEY_;
  PORTA->CFUNC[0] = 0xF0000000;
  PORTA->CFUNC[1] = 0x00000FFF;
  PORTA->SFUNC[0] = 0x60000000;
  PORTA->SFUNC[1] = 0x00000606;
  PORTA->SANALOG =  0x00000780;
  PORTA->SRXTX = 0x00000200;
  PORTA->SOE =   0x00000200;
  PORTA->SPWR[0] = 0x000A8000;
  MDR_SSP0->CR1 = 2;  //enable SSP0
}


uint8_t SPI_Exchange(uint8_t data) {
//  PORTA->CRXTX = 0x00000200;
  MDR_SSP0->DR = data;
  while((MDR_SSP0->SR & 0x10) != 0);  //while busy
//  PORTA->SRXTX = 0x00000200;  
  return (uint8_t)MDR_SSP0->DR;
}


void SPI_MExchange(uint8_t *data_in, uint8_t *data_out, int len) {
  int i;
  if(data_in == NULL) {
    for(i=0; i<len; i++) SPI_Exchange(data_out[i]);
    }
  else if(data_out == NULL) {
    for(i=0; i<len; i++) data_in[i] = SPI_Exchange(0);
    }
  else {
    for(i=0; i<len; i++) data_in[i] = SPI_Exchange(data_out[i]);
    }
}


void EE_Write(uint32_t ee_addr, uint8_t *data_ptr, uint16_t leng) {
  PORTA->CRXTX = 0x00000200;
  SPI_Exchange(0x06);  //WREN
  PORTA->SRXTX = 0x00000200;
  PORTA->CRXTX = 0x00000200;
  SPI_Exchange(0x02);  //WRITE
  SPI_Exchange(*((uint8_t*)&ee_addr+2));
  SPI_Exchange(*((uint8_t*)&ee_addr+1));
  SPI_Exchange(*((uint8_t*)&ee_addr));  
  SPI_MExchange(NULL, data_ptr, leng);
  PORTA->SRXTX = 0x00000200;
}


void EE_Read(uint32_t ee_addr, uint8_t *data_ptr, uint16_t leng) {
  PORTA->CRXTX = 0x00000200;
  SPI_Exchange(0x03);  //READ
  SPI_Exchange(*((uint8_t*)&ee_addr+2));
  SPI_Exchange(*((uint8_t*)&ee_addr+1));
  SPI_Exchange(*((uint8_t*)&ee_addr));  
  SPI_MExchange(data_ptr, NULL, leng);
  PORTA->SRXTX = 0x00000200;
}


uint16_t crc16_update(uint16_t crc, uint8_t *buff, uint32_t leng) {
  int i, j;
  for(j=0; j<leng; j++) {
    crc ^= buff[j];
    for(i=0; i<8; ++i) {
      if(crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc = (crc >> 1);
      }
    }
  return crc;
}


void EE_Prog() {
  uint16_t crc;
  SPI_Init();
  EEProgHeader.offs = 4;
  EEProgHeader.sram_addr = 0x20000000 + IROM_OFFS;
  EEProgHeader.start_addr = *((uint32_t*)EEProgHeader.sram_addr+1);
  EEProgHeader.prog_len = (0x8000 - IROM_OFFS) / 4;
  EEProgHeader.crc_cnt_len = (0x8000 - IROM_OFFS) / 4;
  EEProgHeader.crc = crc16_update(0xFFFF, (uint8_t*)&EEProgHeader, sizeof(EEProgHeader)-2);
  EE_Write(0, (uint8_t*)&EEProgHeader, sizeof(EEProgHeader));
  EE_Write(sizeof(EEProgHeader), (uint8_t*)(0x20000000 + IROM_OFFS), 0x8000 - IROM_OFFS);
  crc = crc16_update(0xFFFF, (uint8_t*)(0x20000000 + IROM_OFFS), 0x8000 - IROM_OFFS);
  EE_Write(sizeof(EEProgHeader)+(0x8000 - IROM_OFFS), (uint8_t*)&crc, 2);
}

