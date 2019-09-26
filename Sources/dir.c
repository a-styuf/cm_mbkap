/*
	Важно - все фукнции в данной библиотеке - это по сути драйвера к отдельному устройству переферии!!! В таком стиле и надо дополнять.
	Если общение с дополнительным устройством происходит через ДИР: добавлять библиотеку в этот файл
*/
#include "dir.h"

#define INIT_TIMEOUT_MS 100

//общие переменные
extern uint8_t in_buff[256];
extern uint8_t out_buff[256];

/*** ДИР ***/
/*Инициализация устойства*/
void DIR_Init(typeDIRDevice *dir_ptr, uint16_t frame_definer, uint8_t sub_addr, uint8_t id, typeCMParameters* cm_ptr)
{
	// Инициализируем структуру урпавления
	dir_ptr->ctrl.frame_definer = frame_definer;
	dir_ptr->ctrl.sub_addr = sub_addr;
	dir_ptr->ctrl.id = id;
	// Параметры для запуска измерения
	dir_ptr->ctrl.mode = 1;
	dir_ptr->ctrl.meas_num = 0;
	//
	
	// Иницилизация кадра
	DIR_Frame_Init(dir_ptr);
	// Предварительный запуск ДИР
	DIR_Start_Measurement(dir_ptr, cm_ptr);
	//
	Timers_Start(1, INIT_TIMEOUT_MS); 
    while (Timers_Status(1) == 0);
}

void DIR_constatnt_mode(uint8_t mode)  // широковещательная; mode: 1 - on; 0 - off;
{
	uint8_t out_buff[8];
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
    UART0_SendPacket(out_buff, 6, 1);
}

void DIR_Start_Measurement(typeDIRDevice *dir_ptr, typeCMParameters* cm_ptr) //измрение идет долго, предлагается забирать старые при запуске ДИР. В итоге будет задержка в один изм. интервал
{
	uint16_t data[8];
	if (dir_ptr->ctrl.mode <= 2){ //значения валидные
		data[0] = dir_ptr->ctrl.mode;
	}
	else{  //значение не валидное - скидываем на 0
		dir_ptr->ctrl.mode = 0x01;
		data[0] = 0x01;
	}
	F_Trans(cm_ptr, 6, dir_ptr->ctrl.id, 0, NULL, data);
}

void DIR_Data_Get(typeDIRDevice *dir_ptr, typeCMParameters* cm_ptr)
{
	uint8_t in_data[32];
	if (F_Trans(cm_ptr, 3, dir_ptr->ctrl.id, 1, 12, (uint16_t*)in_data) == 1) { //вычитываем по одному измерению для каждого канала ДИР
		memcpy(&dir_ptr->frame.dir_data[dir_ptr->ctrl.meas_num], in_data, sizeof(typeDIRData));
		_dir_struct_rev(&dir_ptr->frame.dir_data[dir_ptr->ctrl.meas_num]);
		//
		dir_ptr->ctrl.meas_num += 1;
		if (dir_ptr->ctrl.meas_num >= 2){ // если кадр уже заполнен - выкладываем на ПА и в ЗУ
			dir_ptr->ctrl.meas_num = 0;
			dir_ptr->frame.mode = dir_ptr->ctrl.mode;
			DIR_Frame_Build(dir_ptr, cm_ptr);
		}
	}
}
/* формирование кадров */
void DIR_Frame_Init(typeDIRDevice *dir_ptr)
{
    uint16_t frame[32] = {0};
	memset(&dir_ptr->frame, 0xFEFE, sizeof(typeDIRFrame));
	dir_ptr->frame.label = 0x0FF1;
	dir_ptr->frame.definer = dir_ptr->ctrl.frame_definer;
	dir_ptr->frame.time = _rev_u32(Get_Time_s());
	dir_ptr->frame.crc16 = crc16_ccitt((uint8_t*)&dir_ptr->frame, 62);
	memcpy((uint8_t *)frame, (uint8_t *)(&dir_ptr->frame), sizeof(typeDIRFrame));
	//
	Write_to_SubAddr(dir_ptr->ctrl.sub_addr, frame);
}

void DIR_Frame_Build(typeDIRDevice *dir_ptr, typeCMParameters* cm_ptr) 
{
    uint16_t frame[32] = {0};
	dir_ptr->frame.label = 0x0FF1;
	dir_ptr->frame.definer = dir_ptr->ctrl.frame_definer;
	dir_ptr->frame.num = cm_ptr->frame_number;
	dir_ptr->frame.time = _rev_u32(Get_Time_s());
	dir_ptr->frame.crc16 = crc16_ccitt((uint8_t*)&dir_ptr->frame, 62);
	cm_ptr->frame_number += 1;
	memcpy((uint8_t *)frame, (uint8_t *)(&dir_ptr->frame), sizeof(typeDIRFrame));
	//
	Write_to_SubAddr(dir_ptr->ctrl.sub_addr, frame);
	Save_Data_Frame((uint8_t*)frame, cm_ptr);
}
/* функции для внутреннего использования */
void _dir_struct_rev(typeDIRData* dir_struct_ptr)
{  
    uint16_t data[32];
    uint8_t i = 0;
    memcpy((uint8_t *)data, (uint8_t *)dir_struct_ptr, sizeof(typeDIRData));
    for (i=0; i<(sizeof(typeDIRData)/2); i++) {
        data[i] = __REV16(data[i]);
    }
    memcpy((uint8_t *)dir_struct_ptr, (uint8_t *)data, sizeof(typeDIRData));
}

/*** ДНТ: работает через ДИР ***/ 
/*Инициализация устойства*/
void DNT_Init(typeDNTDevice *dnt_ptr, uint16_t frame_definer, uint16_t dev_frame_definer, uint8_t sub_addr, uint8_t mko_addr, uint8_t dir_id, typeCMParameters* cm_ptr)
{
	// Инициализируем структуру урпавления
	dnt_ptr->ctrl.frame_definer = frame_definer;
	dnt_ptr->ctrl.sub_addr = sub_addr;
	//
	dnt_ptr->ctrl.mko_addr = mko_addr;
	dnt_ptr->ctrl.dev_frame_definer = dev_frame_definer;
	dnt_ptr->ctrl.dev_frame_num = 0;
	dnt_ptr->ctrl.dir_id = dir_id;
	// Параметры работы прибора
	dnt_ptr->ctrl.mode = 0x02;
	dnt_ptr->ctrl.meas_num = 0;
	// Иницилизация кадра
	DNT_Frame_Init(dnt_ptr);
	// Предварительный запуск ДИР
	DNT_MKO_Measure_Initiate(dnt_ptr, cm_ptr);
	//
	Timers_Start(1, INIT_TIMEOUT_MS);
    while (Timers_Status(1) == 0);
}

void DNT_MKO_Read_Initiate(typeDNTDevice *dnt_ptr, typeCMParameters* cm_ptr)
{
	typeDIRMKOData mko_data;
	dnt_ptr->ctrl.mko.Run = 0x0001;
	dnt_ptr->ctrl.mko.BSIStat = 0x0000;
	memset((uint8_t*)&dnt_ptr->ctrl.mko.packet.Data[0], 0xFE, 66);
	dnt_ptr->ctrl.mko.packet.CmdWord = 	((dnt_ptr->ctrl.mko_addr & 0x1F) << 11) +  //адрес мко
																(0x01 << 10) + //направление передачи: 0 - КК->ОУ, 1 - ОУ->КК
																((30 & 0x1F) << 5) + //подадрес: 30 - данные ДНТ
																(32 & 0x1F);  //длина: читаем по 32 слова
	// переставляем байты в 16-битных словах
	mko_data = dnt_ptr->ctrl.mko;
	_dir_mko_data_struct_rev(&mko_data);
	//
	F_Trans(cm_ptr, 16, dnt_ptr->ctrl.dir_id,  0x0100, sizeof(typeDIRMKOData)/2, (uint16_t*)&mko_data);
}

void DNT_MKO_Read_Finish(typeDNTDevice *dnt_ptr, typeCMParameters* cm_ptr)
{
	// Читаем данные из ДИР из структуры управления МКО
	F_Trans(cm_ptr, 3, dnt_ptr->ctrl.dir_id, 0x0100, sizeof(typeDIRMKOData)/2, (uint16_t*)&dnt_ptr->ctrl.mko);
	_dir_mko_data_struct_rev(&dnt_ptr->ctrl.mko);
	//перекладываем данные в структуру с подадресом, формируемым ДНТ
	memcpy((uint8_t*)&dnt_ptr->dev_frame, (uint8_t *) &dnt_ptr->ctrl.mko.packet.Data[1], 64);
	// вяческие проверки
	if ((dnt_ptr->ctrl.mko.Run) || (dnt_ptr->ctrl.mko.BSIStat & 0x8000)){  //ошибка транзакции МКО в ДИР
		cm_ptr->bus_nans_cnt += 1;
		cm_ptr->bus_nans_status |= 1<<5;
	}
	else if ((dnt_ptr->ctrl.mko.packet.Data[0]  & 0xF800) != (dnt_ptr->ctrl.mko.packet.CmdWord & 0xF800)){ //проверка совпадения адресов МКО в КС и ОС
		cm_ptr->bus_nans_cnt += 1;
		cm_ptr->bus_nans_status |= 1<<5;
	}
	else if ((dnt_ptr->dev_frame.label != 0x0FF1) && ((dnt_ptr->dev_frame.definer & 0xFC07) != (dnt_ptr->ctrl.dev_frame_definer & 0xFC07))){ //Проверка совпадение метки и определителя кадра из ДНТ без учета заводского номера
		cm_ptr->bus_error_cnt += 1;
		cm_ptr->bus_error_status |= 1<<5;
	}
	else if (dnt_ptr->dev_frame.num == dnt_ptr->ctrl.dev_frame_num){ // проверка на изменение номерка кадра (что бы определить читаем ли мы старый кадр, или новый)
		cm_ptr->bus_error_cnt += 1;
		cm_ptr->bus_error_status |= 1<<5;
	}
	// несмотря на ошибки разбираем данные: игнорируем их, т.к. нет смысла отбрасывать данные - валидность проверится по ошибкам
	dnt_ptr->frame.data[dnt_ptr->ctrl.meas_num].current = dnt_ptr->dev_frame.current;
	dnt_ptr->frame.data[dnt_ptr->ctrl.meas_num].temp = (dnt_ptr->dev_frame.temperature >> 8) & 0xFF;
	dnt_ptr->frame.data[dnt_ptr->ctrl.meas_num].ku = dnt_ptr->dev_frame.dnt_state & 0x03;
	// запоминаем время, при котором сменился измерительный интервал
	if ((dnt_ptr->ctrl.last_measure_interval != cm_ptr->measure_interval) || dnt_ptr->ctrl.meas_num == 0){
		dnt_ptr->frame.shut_off_grid_voltage = (dnt_ptr->dev_frame.shut_off_grid_voltage >> 8) & 0xFF;
		dnt_ptr->frame.number = dnt_ptr->ctrl.meas_num;
		dnt_ptr->frame.meas_int = cm_ptr->measure_interval;
		//
		dnt_ptr->ctrl.last_measure_interval = cm_ptr->measure_interval;
	}		
	//
	dnt_ptr->ctrl.meas_num += 1;
	if (dnt_ptr->ctrl.meas_num >= 12){ // когда намерили 12 измерений  сохраняем в ЗУ и выкладываем на ПА
		dnt_ptr->ctrl.meas_num = 0;
		DNT_Frame_Build(dnt_ptr, cm_ptr);
	}
	else{ //если память еще кадр не заполнен до конца просто выкладываем на ПА (для удобства последующего тестирования)
		DNT_Frame_Write_to_SA(dnt_ptr, cm_ptr);
	}
}

void DNT_MKO_Measure_Initiate(typeDNTDevice *dnt_ptr, typeCMParameters* cm_ptr)
{
	typeDIRMKOData mko_data;
	dnt_ptr->ctrl.mko.Run = 0x0001;
	dnt_ptr->ctrl.mko.BSIStat = 0x0000;
	memset((uint8_t*)&dnt_ptr->ctrl.mko.packet.Data[0], 0xFE, 66);
	//
	dnt_ptr->ctrl.mko.packet.CmdWord = 	((dnt_ptr->ctrl.mko_addr & 0x1F) << 11) +  //адрес мко
																(0x00 << 10) + //направление передачи: 0 - КК->ОУ, 1 - ОУ->КК
																((29 & 0x1F) << 5) + //подадрес: 30 - данные ДНТ
																(32 & 0x1F);  //длина: читаем/пишем по 32 слова
	dnt_ptr->ctrl.mko.packet.Data[0] = 0x0FF1;
	dnt_ptr->ctrl.mko.packet.Data[1] = dnt_ptr->ctrl.dev_frame_definer + 1; //у кадар с данными тип 0, у ПА управления - 1
	dnt_ptr->ctrl.mko.packet.Data[2] = 1; //время измерения в с
	dnt_ptr->ctrl.mko.packet.Data[3] = 100; //мертвое время измерения в мс
	dnt_ptr->ctrl.mko.packet.Data[4] = 0; // тип для осциллограммы: 0 - ток, 1 - нуль
	dnt_ptr->ctrl.mko.packet.Data[5] = 0; // ку для осциллограммы
	// проверка на ускоренный режим
	if ((cm_ptr->speed_mode_state & (1<<DNT_FRAME_NUM)) && (cm_ptr->speed_mode_timeout)) dnt_ptr->ctrl.mode = 0x04;
	else dnt_ptr->ctrl.mode = 0x02;
	//
	dnt_ptr->ctrl.mko.packet.Data[6] = dnt_ptr->ctrl.mode;
	// переставляем байты в 16-битных словах
	mko_data = dnt_ptr->ctrl.mko;
	_dir_mko_data_struct_rev(&mko_data);
	//
	F_Trans(cm_ptr, 16, dnt_ptr->ctrl.dir_id,  0x0100, sizeof(typeDIRMKOData)/2, (uint16_t*)&mko_data);
}

void DNT_MKO_Measure_Finish(typeDNTDevice *dnt_ptr, typeCMParameters* cm_ptr)
{
	// Читаем данные из ДИР из структуры управления МКО
	F_Trans(cm_ptr, 3, dnt_ptr->ctrl.dir_id,  0x0100, sizeof(typeDIRMKOData)/2, (uint16_t*)&dnt_ptr->ctrl.mko);
	_dir_mko_data_struct_rev(&dnt_ptr->ctrl.mko);
	// вяческие проверки
	if ((dnt_ptr->ctrl.mko.Run) || (dnt_ptr->ctrl.mko.BSIStat & 0x8000)){  //ошибка транзакции МКО в ДИР
		cm_ptr->bus_nans_cnt += 1;
		cm_ptr->bus_nans_status |= 1<<5;
	}
	else if ((dnt_ptr->ctrl.mko.packet.Data[32]  & 0xF800) != (dnt_ptr->ctrl.mko.packet.CmdWord & 0xF800)){ //проверка совпадения адресов МКО в КС и ОС
		cm_ptr->bus_nans_cnt += 1;
		cm_ptr->bus_nans_status |= 1<<5;
	}
}

void DNT_MKO_Constant_Mode(typeDNTDevice *dnt_ptr, typeCMParameters* cm_ptr, uint8_t on)
{
	dnt_ptr->ctrl.mko.Run = 0x0001;
	dnt_ptr->ctrl.mko.BSIStat = 0x0000;
	memset((uint8_t*)&dnt_ptr->ctrl.mko.packet.Data[0], 0xFE, 66);
	//
	dnt_ptr->ctrl.mko.packet.CmdWord = 	((dnt_ptr->ctrl.mko_addr & 0x1F) << 11) +  //адрес мко
																(0x00 << 10) + //направление передачи: 0 - КК->ОУ, 1 - ОУ->КК
																((29 & 0x1F) << 5) + //подадрес: 30 - данные ДНТ
																(32 & 0x1F);  //длина: читаем/пишем по 32 слова
	dnt_ptr->ctrl.mko.packet.Data[0] = 0x0FF1;
	dnt_ptr->ctrl.mko.packet.Data[1] = dnt_ptr->ctrl.dev_frame_definer + 1; //у кадар с данными тип 0, у ПА управления - 1
	dnt_ptr->ctrl.mko.packet.Data[2] = 1; //время измерения в с
	dnt_ptr->ctrl.mko.packet.Data[3] = 100; //мертвое время измерения в мс
	dnt_ptr->ctrl.mko.packet.Data[4] = 0; // тип для осциллограммы: 0 - ток, 1 - нуль
	dnt_ptr->ctrl.mko.packet.Data[5] = 0; // ку для осциллограммы
	if (on)  dnt_ptr->ctrl.mode |= 0x10;
	else dnt_ptr->ctrl.mko.packet.Data[6] = dnt_ptr->ctrl.mode &= (~0x10);
	dnt_ptr->ctrl.mko.packet.Data[6] = dnt_ptr->ctrl.mode;
	F_Trans(cm_ptr, 16, dnt_ptr->ctrl.dir_id,  0x0100, sizeof(typeDIRMKOData)/2, (uint16_t*)&dnt_ptr->ctrl.mko);
}
/* формирование кадров */
void DNT_Frame_Init(typeDNTDevice *dnt_ptr)
{
    uint16_t frame[32] = {0};
	memset(&dnt_ptr->frame, 0xFEFE, sizeof(typeDNTFrame));
	dnt_ptr->frame.label = 0x0FF1;
	dnt_ptr->frame.definer = dnt_ptr->ctrl.frame_definer;
	dnt_ptr->frame.time = _rev_u32(Get_Time_s());
	dnt_ptr->frame.crc16 = crc16_ccitt((uint8_t*)&dnt_ptr->frame, 62);
	memcpy((uint8_t *)frame, (uint8_t *)(&dnt_ptr->frame), sizeof(typeDNTFrame));
	//
	Write_to_SubAddr(dnt_ptr->ctrl.sub_addr, frame);
}

void DNT_Frame_Build(typeDNTDevice *dnt_ptr, typeCMParameters* cm_ptr) 
{
    uint16_t frame[32] = {0};
	dnt_ptr->frame.label = 0x0FF1;
	dnt_ptr->frame.definer = dnt_ptr->ctrl.frame_definer;
	dnt_ptr->frame.num = cm_ptr->frame_number;
	dnt_ptr->frame.time = _rev_u32(Get_Time_s());
	dnt_ptr->frame.crc16 = crc16_ccitt((uint8_t*)&dnt_ptr->frame, 62);
	cm_ptr->frame_number += 1;
	memcpy((uint8_t *)frame, (uint8_t *)(&dnt_ptr->frame), sizeof(typeDNTFrame));
	//
	Write_to_SubAddr(dnt_ptr->ctrl.sub_addr, frame);
	Save_Data_Frame((uint8_t*)frame, cm_ptr);
}

void DNT_Frame_Write_to_SA(typeDNTDevice *dnt_ptr, typeCMParameters* cm_ptr) 
{
    uint16_t frame[32] = {0};
	dnt_ptr->frame.label = 0x0FF1;
	dnt_ptr->frame.definer = dnt_ptr->ctrl.frame_definer;
	dnt_ptr->frame.num = cm_ptr->frame_number;
	dnt_ptr->frame.time = _rev_u32(Get_Time_s());
	dnt_ptr->frame.crc16 = crc16_ccitt((uint8_t*)&dnt_ptr->frame, 62);
	//cm_ptr->frame_number += 1;
	memcpy((uint8_t *)frame, (uint8_t *)(&dnt_ptr->frame), sizeof(typeDNTFrame));
	//
	Write_to_SubAddr(dnt_ptr->ctrl.sub_addr, frame);
}
/* функции для внутреннего использования */

/*** АДИИ ***/
/*Инициализация устойства*/
void ADII_Init(typeADIIDevice *adii_ptr, uint16_t frame_definer, uint8_t sub_addr, uint8_t dir_id, typeCMParameters* cm_ptr)
{
	// Инициализируем структуру урпавления
	adii_ptr->ctrl.frame_definer = frame_definer;
	adii_ptr->ctrl.sub_addr = sub_addr;
	adii_ptr->ctrl.dir_id = dir_id;
	// Иницилизация кадра
	ADII_Frame_Init(adii_ptr);
	//запускаем измерение
	ADII_Meas_Start(adii_ptr, cm_ptr);  //управление командой происходит переменной adii.ctrl.mode: 1 - режим тестирования, 0 - нормальный режим
	//
	Timers_Start(1, INIT_TIMEOUT_MS); 
    while (Timers_Status(1) == 0);
}

void ADII_Meas_Start(typeADIIDevice *adii_ptr, typeCMParameters* cm_ptr)  //управление командой происходит переменной adii.ctrl.mode: 1 - режим тестирования, 0 - нормальный режим
{
	typeADIIData adii_data;
	adii_ptr->ctrl.data.Run = 0x0001;  // кол-во байт для передачи
	adii_ptr->ctrl.data.RxLeng = 0x0000;
	memset((uint8_t*)&adii_ptr->ctrl.data.Data[0], 0x00, 64);
	// выбираем режим для запуска: если режим тестовый ("m"), то он сбрасывается на нормальный
	if(adii_ptr->ctrl.mode == 1){  //при режиме тестирования разово запускаем и меняем его на обычный режим
		adii_ptr->ctrl.mode = 0;  //переходим обратно на режим измерения
		adii_ptr->ctrl.data.Data[0] = 'm';
	}
	else if(adii_ptr->ctrl.mode == 0){  //при обычном режиме сохраняем его и читаем данные с помощью 's'
		adii_ptr->ctrl.data.Data[0] = 's';
	}
	else{  //при неопознаном режимеменяем его на обычный режим
		adii_ptr->ctrl.mode = 0; 
		adii_ptr->ctrl.data.Data[0] = 's';
	}
	adii_data = adii_ptr->ctrl.data;
	_adii_data_rev(&adii_data);
	F_Trans(cm_ptr, 16, adii_ptr->ctrl.dir_id, 0x0200, 3, (uint16_t*)&adii_data);
}

void ADII_Read_Data(typeADIIDevice *adii_ptr, typeCMParameters* cm_ptr)
{
	// Читаем данные из ДИР из структуры управления МКО
	F_Trans(cm_ptr, 3, adii_ptr->ctrl.dir_id, 0x0200, sizeof(typeADIIData)/2, (uint16_t*)&adii_ptr->ctrl.data);
	_adii_data_rev(&adii_ptr->ctrl.data);
	// данные в кадр
	memcpy((uint8_t*)adii_ptr->frame.cnt_val, (uint8_t*)&adii_ptr->ctrl.data.Data[1], 52);
	// проверка на неответ АДИИ
	if (adii_ptr->ctrl.data.RxLeng == 0){
		cm_ptr->adii_mode = 0x00;
		cm_ptr->adii_fk = 0x00;
		cm_ptr->bus_nans_cnt += 1;
		cm_ptr->bus_nans_status |= 1<<6;
	}
	else{
		// показания ФК и режима складываем в структуру параметров ЦМ
		if (_adii_crc_check((uint8_t*)&adii_ptr->ctrl.data.Data[0], 54) == adii_ptr->ctrl.data.Data[54]) cm_ptr->adii_mode = 0x80; // проверка контрольной суммы
		else	cm_ptr->adii_mode = 0;
		cm_ptr->adii_mode |= adii_ptr->ctrl.data.Data[53];
		cm_ptr->adii_fk = adii_ptr->ctrl.data.Data[0];
	}
	// Выкладываем на ПА и сохраняем в ЗУ
	if (adii_ptr->ctrl.data.Data[53] & 0x02){
		//режим диполяризации: не делаем ничего
	}
	else{
		ADII_Frame_Build(adii_ptr, cm_ptr);	
	}
}

/* формирование кадров */
void ADII_Frame_Init(typeADIIDevice *adii_ptr)
{
    uint16_t frame[32] = {0};
	memset(&adii_ptr->frame, 0xFEFE, sizeof(typeDNTFrame));
	adii_ptr->frame.label = 0x0FF1;
	adii_ptr->frame.definer = adii_ptr->ctrl.frame_definer;
	adii_ptr->frame.time = _rev_u32(Get_Time_s());
	adii_ptr->frame.crc16 = crc16_ccitt((uint8_t*)&adii_ptr->frame, 62);
	memcpy((uint8_t *)frame, (uint8_t *)(&adii_ptr->frame), sizeof(typeDNTFrame));
	//
	Write_to_SubAddr(adii_ptr->ctrl.sub_addr, frame);
}

void ADII_Frame_Build(typeADIIDevice *adii_ptr, typeCMParameters* cm_ptr) 
{
    uint16_t frame[32] = {0};
	adii_ptr->frame.label = 0x0FF1;
	adii_ptr->frame.definer = adii_ptr->ctrl.frame_definer;
	adii_ptr->frame.num = cm_ptr->frame_number;
	adii_ptr->frame.time = _rev_u32(Get_Time_s());
	adii_ptr->frame.crc16 = crc16_ccitt((uint8_t*)&adii_ptr->frame, 62);
	cm_ptr->frame_number += 1;
	memcpy((uint8_t *)frame, (uint8_t *)(&adii_ptr->frame), sizeof(typeDNTFrame));
	//
	Write_to_SubAddr(adii_ptr->ctrl.sub_addr, frame);
	Save_Data_Frame((uint8_t*)frame, cm_ptr);
}
/* функции для внутреннего использования */
uint8_t _adii_crc_check(uint8_t* buff, uint8_t leng) //проверка контрольной суммы для АДИИ !!! требуются тесты
{
	uint8_t crc=0, i;
	for (i=0; i<leng; i++){
		crc += buff[i];
	}
	return crc;
}

void _dir_mko_data_struct_rev(typeDIRMKOData* struct_ptr)
{  
    uint16_t data[64];
    uint8_t i = 0;
    memcpy((uint8_t *)data, (uint8_t *)struct_ptr, sizeof(typeDIRMKOData));
    for (i=0; i<(sizeof(typeDIRMKOData)/2); i++) {
        data[i] = __REV16(data[i]);
    }
    memcpy((uint8_t *)struct_ptr, (uint8_t *)data, sizeof(typeDIRMKOData));
}

void _adii_data_rev(typeADIIData* struct_ptr)
{  
    uint16_t data[64];
    uint8_t i = 0;
    memcpy((uint8_t *)data, (uint8_t *)struct_ptr, sizeof(typeADIIData));
    for (i=0; i<sizeof(typeADIIData)/2; i++) {
        data[i] = __REV16(data[i]);
    }
    memcpy((uint8_t *)struct_ptr, (uint8_t *)data, sizeof(typeADIIData));
}
