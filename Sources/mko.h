#ifndef _MKO_H_
#define _MKO_H_


#define IRQn_MIL_STD_15531 (IRQn_Type)117
#define mko_default_addr 13

typedef struct  //  max 62 - параетры ЦМ для сохоранения
{
	uint16_t cw; //+0
	uint16_t aw; //+2
    uint8_t addr; //+4
    uint8_t r_w; //+5
    uint8_t subaddr; //+6
    uint8_t leng; //+7
	uint16_t msg; //+8
	uint8_t num; //+10
	uint8_t rcv_a;  //+10
	uint8_t rcv_b;  //+10
	uint8_t error; //+11
	uint8_t error_cnt; //+12
    uint16_t data[32]; //+13
}typeMKOControl;


void MKO_Init(uint8_t mko_addr);
void BlockMKOTransmitter(void);
void Set_Busy(void);
void Set_Release(void);
void Write_to_SubAddr(uint8_t subaddr, uint16_t* data);
void Read_from_SubAddr(uint8_t subaddr, uint16_t* data);
uint16_t MKO_IVect(uint8_t* error, uint8_t* error_cnt);
void Get_MKO_error(uint8_t* error, uint8_t* error_cnt);



#endif

