#include "1986ve8_lib/cm4ikmcu.h"
#include "adc.h"

uint16_t ADCData[ADC0_CHAN_NUM];
uint16_t ADCDataSm[ADC0_CHAN_NUM];
uint8_t ADCDataCnt[ADC0_CHAN_NUM];


void ADC_Init() {
  // PC[8]   ADC2    ch0
  // PC[9]   ADC3    ch1
  // PC[10]  ADC4    ch2
  // PC[11]  ADC5    ch3
  // PC[12]  ADC6    ch4
  // PC[13]  ADC7    ch5
  // PC[14]  ADC8    ch6
  int i, sm;
  CLK_CNTR->KEY = _KEY_;
  CLK_CNTR->PER1_CLK |= (1<<23);  //clock ADC0
  ADC0->KEY = _KEY_;
  ADC0->CONFIG1 = (255<<12)|(511<<3);  //вот такая хренотень
  /* калибровка */
  ADC0->FIFOEN1 = 0xF0000000;
  ADC0->CONFIG0 = 1;  //enable ADC0
  sm = 0;
  for(i=0; i<4; i++) {
    ADC0->CONTROL = (60 + i)<<8 | 1;
    while((ADC0->STATUS & 1)==0);
    sm = sm + (ADC0->RESULT & 0xFFF);
    }
  sm = sm >> 2;
  ADC0->FIFOEN1 = 0;
  ADC0->CONFIG1 = (ADC0->CONFIG1 & 0xFFFFF) | (sm << 20);
  /**/
  for(i=0; i<ADC0_CHAN_NUM; i++) ADCDataCnt[i] = 0;
  ADC0->CONFIG2 = 0x01009004;  //enable interrupt, temp.sensor on
  ADC0->CHSEL0 =  0x00FFFFFF; //0x0080007F;
  ADC0->FIFOEN0 = 0x00FFFFFF; //0x0080007F;
  ADC0->CONFIG0 = 0x00000025;  //enable ADC0, SELMODE=1, continues mode
  NVIC_EnableIRQ(IRQn_ADC0);
}


void INT_ADC0_Handler(void) {   //period ??? ms
  uint32_t rslt;
  uint16_t chn, val;
  while(ADC0->STATUS & 1) {
    rslt = ADC0->RESULT;
    chn = *((uint16_t*)&rslt + 1);
    val = *((uint16_t*)&rslt);
    if(chn < ADC0_CHAN_NUM) {
      ADCDataSm[chn] += val;
      ADCDataCnt[chn]++;
      if(ADCDataCnt[chn] >= 8) {
        ADCData[chn] = ADCDataSm[chn]>>3;
        ADCDataCnt[chn] = 0;
        ADCDataSm[chn] = 0;
        }
      }
    }
}

