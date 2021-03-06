/*
	Важно - все фукнции в данной библиотеке - это по сути драйвера к отдельному МПП!!! В таком стиле и надо дополнять.
*/
#include "mpp.h"

//общие переменные
extern uint8_t in_buff[256];
extern uint8_t out_buff[256];

/*Инициализация устойства*/
void MPP_Init(typeMPPDevice *mpp_ptr, uint16_t frame_definer, uint8_t sub_addr, uint8_t id, uint16_t offset, typeCMParameters* cm_ptr)
{
	// Инициализируем структуру урпавления
	mpp_ptr->ctrl.frame_definer = frame_definer;
	mpp_ptr->ctrl.sub_addr = sub_addr;
	mpp_ptr->ctrl.id = id;
	// Включение МПП
	MPP_On(mpp_ptr, cm_ptr);
	// Иницилизация кадра
	MPP_Frame_Init(mpp_ptr);
	// Установка отсечки
	MPP_Offset_Set(mpp_ptr, offset, cm_ptr);
	// Установка порога срабатывания отключения
	MPP_Pwr_Off_Bound_Set(mpp_ptr, MPP100_DEF_BOUND, cm_ptr);
	//
	Timers_Start(1, 100); 
    while (Timers_Status(1) == 0);
}

/* Общение с МПП по ВШ */
void MPP_time_set(void) // широковещательная
{
	uint8_t out_buff[8], i;
    uint32_t time = 0;
	//
	time = Get_Time_s();
	//
	out_buff[0] = 0xFF;  // широковещательная команд
	out_buff[1] = 111;
	out_buff[2] = (time >> 24)  & 0xFF;
	out_buff[3] = (time >> 16)  & 0xFF;
	out_buff[4] = (time >> 8)  & 0xFF;
	out_buff[5] = (time >> 0)  & 0xFF;
	//
	for (i=0; i<3; i++){ //отправляем три раза для надежности
		UART0_SendPacket(out_buff, 6, 1);
		Timers_Start(1, 3); // дополнительный таймаут для разделения пакетов
		while (Timers_Status(1) == 0) {};
	}
}

void MPP_constatnt_mode(uint8_t mode)  // широковещательная; mode: 1 - on; 0 - off;
{
	uint8_t out_buff[8], i;
    uint8_t mode_mask = 0;
	if (mode == 1) mode_mask = 0xFF;
    //
    out_buff[0] = 0xFF;  // широковещательная команд
    out_buff[1] = 106;  // формат, что бы случайно не послать такую команду
    out_buff[2] = 0x55;
    out_buff[3] = 0x55;
    out_buff[4] = 0xAA & mode_mask; //clear data if mode 0
    out_buff[5] = 0xAA & mode_mask;
    //
	for (i=0; i<3; i++){
		UART0_SendPacket(out_buff, 6, 1);
		Timers_Start(1, 3); // дополнительный таймаут для разделения пакетов
		while (Timers_Status(1) == 0) {};
	}
}

void MPP_On(typeMPPDevice *mpp_ptr, typeCMParameters* cm_ptr)
{
    uint16_t data[2];
    data[0] = (0x02 << 8); // 0x02 - команда на включение канала мпп на регистрацию
    data[1] = 0x0100; 
	F_Trans(cm_ptr, 16, mpp_ptr->ctrl.id, 0, 2, data);
}

void MPP_Off(typeMPPDevice *mpp_ptr, typeCMParameters* cm_ptr)
{
    uint16_t data[2];
    data[0] = (0x02 << 8); // 0x02 - команда на включение канала мпп на регистрацию
    data[1] = 0x0000; 
	F_Trans(cm_ptr, 16, mpp_ptr->ctrl.id, 0, 2, data);
}

void MPP_Offset_Set(typeMPPDevice *mpp_ptr, uint16_t offset, typeCMParameters* cm_ptr)
{
	uint16_t data[2];
	data[0] = (0x01 << 8); // 0х01 - команда на установку уставки
	data[1] = __REV16(offset & 0xFFFF);
	F_Trans(cm_ptr, 16, mpp_ptr->ctrl.id, 0, 2, data);
}

void MPP_Pwr_Off_Bound_Set(typeMPPDevice *mpp_ptr, uint16_t bound, typeCMParameters* cm_ptr)
{
	uint16_t data[2];
	data[0] = (0x5A << 8); // 0х01 - команда на установку уставки
	data[1] = __REV16(bound & 0xFFFF);
	F_Trans(cm_ptr, 16, mpp_ptr->ctrl.id, 0, 2, data);
}

void MPP_arch_count_offset_get(typeMPPDevice *mpp_ptr, typeCMParameters* cm_ptr)
{
	uint8_t in_data[8];
	F_Trans(cm_ptr, 3, mpp_ptr->ctrl.id, 119, 4, (uint16_t*)in_data);
	mpp_ptr->frame.arch_count = (in_data[0] << 8) + in_data[1];
	mpp_ptr->frame.offset = (in_data[4] << 8) + in_data[5];
}

void MPP_struct_request(typeMPPDevice *mpp_ptr, typeCMParameters* cm_ptr)
{
    uint16_t data[8];
    data[0] = (0x04 << 8); // формируем регистр команд
    data[1] = 0x0100; // указываем количество помех - 1
	F_Trans(cm_ptr, 16, mpp_ptr->ctrl.id, 0, 2, data);
}

void MPP_struct_get(typeMPPDevice *mpp_ptr, typeCMParameters* cm_ptr)
{
	uint8_t i, in_data[64];
	uint8_t forced_start_flag = 1;
	typeMPPRec rec[2];
	
	if (F_Trans(cm_ptr, 3, mpp_ptr->ctrl.id, 7, 26, (uint16_t*)in_data) == 1) { //вычитываем две структуры
		// сохраняем полученные структуры в свои переменные, но отбрасываем номер канала
		memcpy((uint8_t*)&rec[0], in_data+2, sizeof(typeMPPRec));
		memcpy((uint8_t*)&rec[1], in_data+2+24+2, sizeof(typeMPPRec));
		// приводим порядок байт к используемому в МК
		_mpp_struct_rev(&rec[0]);
		_mpp_struct_rev(&rec[1]);
		for (i=0; i<2; i++){ 
			if((rec[i].AcqTime_s != 0) || (rec[i].AcqTime_us != 0)){ // проверяем есть ли измерение в прочитанных данных
				forced_start_flag = 0; //отменяем принудительный запуск, так как  была получена структура помехи
				//
				mpp_ptr->frame.mpp_rec[mpp_ptr->ctrl.frame_pulse_cnt] = rec[i];  // копируем структуру, так как она не пустая
				//
				mpp_ptr->ctrl.frame_pulse_cnt += 1;
				if (mpp_ptr->ctrl.frame_pulse_cnt >= 2){ //в кадре уже 2 помехи
					mpp_ptr->ctrl.frame_pulse_cnt = 0;
					MPP_Frame_Build(mpp_ptr, cm_ptr);  //выкладываем на подадрем и в ЗУ
				}
				else{
					//кадр не полностью заполнен помехами
				}
				//
			} 
		}
	}
	mpp_ptr->ctrl.forced_start_flag = forced_start_flag;
}

/* формирование кадров */
void MPP_Frame_Init(typeMPPDevice *mpp_ptr)
{
    uint16_t frame[32] = {0};
	memset(&mpp_ptr->frame, 0xFEFE, sizeof(typeMPPFrame));
	mpp_ptr->frame.label = 0x0FF1;
	mpp_ptr->frame.definer = mpp_ptr->ctrl.frame_definer;
	mpp_ptr->frame.time = _rev_u32(Get_Time_s());
	mpp_ptr->frame.crc16 = crc16_ccitt((uint8_t*)&mpp_ptr->frame, 62);
	memcpy((uint8_t *)frame, (uint8_t *)(&mpp_ptr->frame), sizeof(typeMPPFrame));
	//
	Write_to_SubAddr(mpp_ptr->ctrl.sub_addr, frame);
}

void MPP_Frame_Build(typeMPPDevice *mpp_ptr, typeCMParameters* cm_ptr) 
{
    uint16_t frame[32] = {0};
	mpp_ptr->frame.label = 0x0FF1;
	mpp_ptr->frame.definer = mpp_ptr->ctrl.frame_definer;
	mpp_ptr->frame.num = cm_ptr->frame_number;
	mpp_ptr->frame.time = _rev_u32(Get_Time_s());
	mpp_ptr->frame.crc16 = crc16_ccitt((uint8_t*)&mpp_ptr->frame, 62);
	cm_ptr->frame_number += 1;
	memcpy((uint8_t *)frame, (uint8_t *)(&mpp_ptr->frame), sizeof(typeMPPFrame));
	//
	Write_to_SubAddr(mpp_ptr->ctrl.sub_addr, frame);
	Save_Data_Frame((uint8_t*)frame, cm_ptr);
}

void MPP_mem_init(typeMPPDevice *mpp_ptr, typeCMParameters* cm_ptr)
{
    uint16_t data[8];
    data[0] = (0x54 << 8); // формируем регистр команд
	F_Trans(cm_ptr, 16, mpp_ptr->ctrl.id, 0, 1, data);
    Timers_Start(1, 200); 
    while (Timers_Status(1) == 0);
}

void MPP_forced_start(typeMPPDevice *mpp_ptr, typeCMParameters* cm_ptr) // происходит только в случае установки флага mpp_ptr->forced_start_flag
{
    uint16_t data[8];
	if (mpp_ptr->ctrl.forced_start_flag) { //если есть необходимость запустить измреение МПП принудительно - запускаем
		mpp_ptr->ctrl.forced_start_flag = 0;
		data[0] = (0x51 << 8); // формируем регистр команд
		data[1] = 0x01 << 8; // формируем регистр команд
		F_Trans(cm_ptr, 16, mpp_ptr->ctrl.id, 0, 2, data);
	} 
	else MPP_On(mpp_ptr, cm_ptr); // если необходимости нет - включаем регистрацию, т.к. МПП мог отключиться по питанию
}

/* функции для внутреннего использования */
void _mpp_struct_rev(typeMPPRec* mpp_struct_ptr)
{  
    uint16_t data[32];
    uint8_t i = 0;
    memcpy((uint8_t *)data, (uint8_t *)mpp_struct_ptr, sizeof(typeMPPRec));
    for (i=0; i<(sizeof(typeMPPRec)/2); i++) {
        data[i] = __REV16(data[i]);
    }
    memcpy((uint8_t *)mpp_struct_ptr, (uint8_t *)data, sizeof(typeMPPRec));
}

uint8_t _mpp_num_from_id(uint8_t mpp_id) //функция взятия порядкового номера МПП из его ID. Пригодится при неочевидном переводе
{
	uint8_t mpp_num;
	mpp_num = mpp_id - 2;
	return mpp_num;
}
