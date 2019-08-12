/*
библиотека для МКО/МПИ 1986ВЕ8Т
Реализовано:
 - КУ 2, 4, 5, 8;
 - отключение работы передатчика;
 - выставление АМКО;
 - установка сигнала занято.
*/
#include <string.h>
#include "1986ve8_lib/cm4ikmcu.h"
#include "mko.h"
#include "mbkap.h"


uint16_t MKOIVect;
typeMKOControl mko_dev;
extern typeSTMstruct stm;

void MKO_Init(uint8_t mko_addr) 
{
	uint32_t saved_reg;
	CLK_CNTR->KEY = _KEY_;
	CLK_CNTR->PER1_CLK |= (1<<19);  //enable clock for MIL0
	CLK_CNTR->PER0_CLK |= (1<<13);  //clock for PORTA
	// saved_reg = CLK_CNTR->PER0_CLK;
	//
	MIL_STD_15531->CONTROL = 1;  //reset
	MIL_STD_15531->CONTROL = (1<<20)|(40<<11)|(3<<4)|(2<<2)|((mko_addr & 0x1F)<<6);
	MIL_STD_15531->StatusWord1 = ((mko_addr & 0x1F)<<11);
	//
	PORTA->KEY = _KEY_;
	PORTA->SANALOG =   0xFFFFFFFF;
	PORTA->CFUNC[2]  = 0xF0FF0000;
	PORTA->SFUNC[2] =  0xC0CC0000;
	PORTA->CFUNC[3]  = 0x00FF0FFF;
	PORTA->SFUNC[3] =  0x00CC0CCC;
	PORTA->SPWR[1] =   0x003C0F00;
	PORTA->SPULLDOWN = 0xFFFFFFFF;
	//
	//CLK_CNTR->PER0_CLK = saved_reg;
	/*enable interrupt*/
	MIL_STD_15531->INTEN = 0x0004;  //valmess int
	NVIC_EnableIRQ(IRQn_MIL_STD_15531);
	//
	mko_dev.addr = mko_addr;
	mko_dev.error_cnt = 0;
	mko_dev.error = 0;
	if (mko_addr==0){ //отключаем МКО
		MIL_STD_15531->CONTROL = 1;  //reset
	}
}

uint16_t MKO_IVect(uint8_t* error, uint8_t* error_cnt) 
{
    uint8_t i=0;
    uint16_t ret;
    NVIC_DisableIRQ(IRQn_MIL_STD_15531);  
    ret = MKOIVect;
    if (ret != 0) {
        mko_dev.addr = ((ret >> 11) & 0x1F);
        mko_dev.r_w = ((ret >> 10) & 0x01);
        //mko_cw.subaddr = ((ret >> 5) & 0x1F);
        //mko_cw.leng = ret & 0x1F;
        if (mko_dev.r_w == 0x00) { //работаем только с приемом данных
            for (i=0; i<32; i++) {
                mko_dev.data[i] = (uint16_t)MIL_STD_15531->DATA[mko_dev.subaddr*32 + i];
            }
        }
		else {
			ret = 0;
		}
    }
    MKOIVect = 0;
	*error = mko_dev.error;
	*error_cnt = mko_dev.error_cnt;
    NVIC_EnableIRQ(IRQn_MIL_STD_15531);  
    return ret;
}

void Set_Busy(void)
{
    MIL_STD_15531->StatusWord1 = ((mko_dev.addr & 0x1F)<<11)|(0x01 << 3);
}

void Release_Busy(void)
{
    MIL_STD_15531->StatusWord1 = ((mko_dev.addr & 0x1F)<<11)|(0x00 << 3);
}

void Write_to_SubAddr(uint8_t subaddr, uint16_t* data)
{    
    uint8_t i;
    Set_Busy();
    for (i=0; i<32; i++)
    {
        MIL_STD_15531->DATA[subaddr*32 + i] = (uint32_t)data[i];
    }
    Release_Busy();
}

void BlockMKOTransmitter(void)
{
	MIL_STD_15531->CONTROL = 1;  //reset
}

void INT_MIL0_Handler(void) 
{
	mko_dev.cw = MIL_STD_15531->CommandWord1;
	mko_dev.msg = MIL_STD_15531->MSG;
	mko_dev.leng = mko_dev.cw & 0x1F;
	mko_dev.rcv_a = ((MIL_STD_15531->STATUS >> 9) & 0x01);
	mko_dev.rcv_b = ((MIL_STD_15531->STATUS >> 10) & 0x01);
	MIL_STD_15531->STATUS  &= ~(3 << 9); // обнуляем флаг активности каналов МКО
	stm.AMKO = 21; //запускаем АМКО на 20 секунд + запас в 1 секунду
	mko_dev.error = (MIL_STD_15531->ERROR & 0xFFFF);
	if (MIL_STD_15531->ERROR == 0) {
		MKOIVect = (uint16_t)mko_dev.cw;
		if (mko_dev.msg == 0x0410) {
			switch (mko_dev.leng){
				case 2: //передать ответное слово
					// ничего не делаем, ответное слово передается ядром
					break;
				case 4: //блокировать передатчик
					if (mko_dev.rcv_a){
						MIL_STD_15531->CONTROL &= ~(1 << 5); //блокируем передатчик Б
					}
					else if(mko_dev.rcv_b){
						MIL_STD_15531->CONTROL &= ~(1 << 4); //блокируем передатчик А
					}
					break;
				case 5: //разблокировать передатчик
					if (mko_dev.rcv_a){
						MIL_STD_15531->CONTROL |= (1 << 5); //разблокируем передатчик Б
					}
					else if(mko_dev.rcv_b){
						MIL_STD_15531->CONTROL |= (1 << 4); //разблокируем передатчик А
					}
					break;
				case 8: //разблокировать передатчик
					MIL_STD_15531->CONTROL |= 1;  //reset
					MIL_STD_15531->CONTROL &= ~1; //clear reset
					break;
			}
		}
		else	{
			if (((mko_dev.cw >> 10) & 0x01) != 0) return;  //работаем только с приемом
			mko_dev.subaddr = ((mko_dev.cw >> 5) & 0x1F);
			switch (mko_dev.subaddr) {
					case 17: // командное сообщение
					case 18: // запрос кадра из ЗУ ЦМ
					case 19: // запрос кадра из ЗУ МПП
					case 29: // команда на синхронзацию времени
					case 30: // технологический подадрес
						if (mko_dev.leng == 0) mko_dev.leng = 32;
						memcpy((char*)&MIL_STD_15531->DATA[mko_dev.subaddr*32], (char*)&MIL_STD_15531->DATA[mko_dev.subaddr*32], mko_dev.leng << 2);
						break;
			}
		}
	}
	else{
		mko_dev.error_cnt ++;
	}
}

void Get_MKO_error(uint8_t* error, uint8_t* error_cnt)
{
	*error = mko_dev.error;
	*error_cnt = mko_dev.error_cnt;
}
