#ifndef _EEROM_H_
#define _EEROM_H_

#define IROM_OFFS     0x0


#pragma pack(2)

typedef struct {
  uint32_t offs;
  uint32_t sram_addr;
  uint32_t start_addr;
  uint32_t prog_len;
  uint16_t crc_cnt_len;
  uint16_t crc;
  }typeEEProgHeader;


void SPI_Init(void);
uint8_t SPI_Exchange(uint8_t data);
void SPI_MExchange(uint8_t *data_in, uint8_t *data_out, int len);
void EE_Write(uint32_t ee_addr, uint8_t *data_ptr, uint16_t leng);
void EE_Read(uint32_t ee_addr, uint8_t *data_ptr, uint16_t leng);
void EE_Prog(void);
  

#endif
