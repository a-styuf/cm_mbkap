#include "power_management.h"

extern uint16_t ADCData[ADC0_CHAN_NUM];

//функции для формирования системных данных
uint8_t Get_Modules_Current(uint16_t *currents, uint16_t *pwr_bounds)
{   
    float cal_a[7] = {0.322, 0.017, 0.017, 0.017, 0.017, 0.322, 0.322};  // калибровки для 7-ми каналов измерения токов [МБКАП, ЦМ, МПП27, МПП100, ДИР, ДНТ, АДИИ]
    float cal_b[7] = {0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000};  // насчитано исходя из: 1) Vref = 3.3, АЦП-12бит.
    int8_t i = 0, status = 0x00;
    uint16_t adc_data[7];
    NVIC_DisableIRQ(IRQn_ADC0);
    //memcpy(adc_data, ADCData, sizeof(adc_data));  // adc_data: 0-DNT, 1-ADII, 2 - MPP_12, 3 - CM
	adc_data[0] = ADCData[2];  // MBKAP
	adc_data[1] = ADCData[6];  // CM
	adc_data[2] = ADCData[5];  // MPP27
	adc_data[3] = ADCData[3];  // MPP100
	adc_data[4] = ADCData[4];  // ДИР
	adc_data[5] = ADCData[0];  // ДНТ
	adc_data[6] = ADCData[1];  // АДИИ
    NVIC_EnableIRQ(IRQn_ADC0);
    for (i=0; i<7; i++)
    {
        currents[i] = cal_a[i] * adc_data[i] + cal_b[i];
        if ((currents[i] > pwr_bounds[i]) && (pwr_bounds[i] != 0)) status |= (1 << i);
    }
    return status;
}

int16_t Get_MCU_Temp(void)
{
	float temp_fp;
	temp_fp = ((-ADCData[ADC0_CHAN_NUM-1] + FACTORY_ADC_TEMP25)/FACTORY_ADC_AVG_SLOPE + FACTORY_TEMP25)*256.;
	return (int16_t)(floor(temp_fp));
}

void Pwr_On_All_Devices(void) 
{
	Timers_Start(1, 200); 
    while (Timers_Status(1) == 0);
	GPIO_Pwr(2, 0x1); //МПП27
    Timers_Start(1, 200); 
    while (Timers_Status(1) == 0);
	GPIO_Pwr(3, 0x1); //МПП100
    Timers_Start(1, 200); 
    while (Timers_Status(1) == 0);
	GPIO_Pwr(4, 0x1); //DIR
	Timers_Start(1, 200); 
    while (Timers_Status(1) == 0);
	GPIO_Pwr(5, 0x1); //DNT
	Timers_Start(1, 200); 
    while (Timers_Status(1) == 0);
	GPIO_Pwr(6, 0x1); //ADII
    Timers_Start(1, 1000); 
    while (Timers_Status(1) == 0);
};

void Pwr_Off_All_Devices(void) 
{
	GPIO_Pwr(2, 0x0); //МПП27
    Timers_Start(1, 100); 
    while (Timers_Status(1) == 0);
	GPIO_Pwr(3, 0x0); //МПП100
    Timers_Start(1, 100); 
    while (Timers_Status(1) == 0);
	GPIO_Pwr(4, 0x0); //DIR
	Timers_Start(1, 100); 
    while (Timers_Status(1) == 0);
	GPIO_Pwr(5, 0x0); //DNT
	Timers_Start(1, 100); 
    while (Timers_Status(1) == 0);
	GPIO_Pwr(6, 0x0); //ADII
    Timers_Start(1, 100); 
    while (Timers_Status(1) == 0);
};

void Pwr_Perepherial_Devices_On(void) 
{
	Timers_Start(1, 250); 
    while (Timers_Status(1) == 0);
	GPIO_Pwr(5, 0x1); //DNT
	Timers_Start(1, 250); 
    while (Timers_Status(1) == 0);
	GPIO_Pwr(6, 0x1); //ADII
    Timers_Start(1, 500); 
    while (Timers_Status(1) == 0);
};

void  Pwr_Ctrl_by_State(uint8_t pwr_state) 
{
	GPIO_Pwr(0, (pwr_state>>0) & 0x01); //MBKAP :todo - пока ПО ЦМ не может отключить весь МБКАП
	GPIO_Pwr(1, (pwr_state>>1) & 0x01); //cm
    GPIO_Pwr(2, (pwr_state>>2) & 0x01); //mpp27
    GPIO_Pwr(3, (pwr_state>>3) & 0x01); //mpp100
    GPIO_Pwr(4, (pwr_state>>4) & 0x01); //dir
    GPIO_Pwr(5, (pwr_state>>5) & 0x01); //dnt 
    GPIO_Pwr(6, (pwr_state>>6) & 0x01); //adii 
}

uint8_t Set_Pwr_State(uint8_t *pwr_state_ptr, uint8_t *pwr_status_ptr, uint8_t dev_num, uint8_t on) //функция, через которую необходимо устанавливать состояние питания модулей, простая установка переменный cm.state не принесет результата
{
	uint8_t new_pwr_state = *pwr_state_ptr;
	if (dev_num <= 6) {
		if (on) new_pwr_state |= (1<<dev_num);
		else new_pwr_state &= ~(1<<dev_num);
		*pwr_status_ptr &= ~(1<<dev_num);
	}
	if (*pwr_state_ptr != new_pwr_state){
		Pwr_Ctrl_by_State(new_pwr_state);
		*pwr_state_ptr = new_pwr_state;
		return 1;
	}
	return 0;
}

