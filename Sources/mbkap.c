#include "mbkap.h"

// структуры для управления ЦМ
typeCMParameters cm;
typeSysFrames sys_frame;
typeSTMstruct stm;

//общие переменные
uint8_t in_buff[256];
uint8_t out_buff[256];

// функция для работы с памятью
int8_t Save_Data_Frame(uint8_t* frame, typeCMParameters* cm_ptr)  // сохранение кадра с данными в архивную память
{
	uint16_t frame_addr;
	uint16_t frame_max_num;
	frame_max_num = Get_Max_Data_Frame_Num();
	//
	cm_ptr->write_ptr += 1;
	if (cm_ptr->write_ptr >= frame_max_num) cm_ptr->write_ptr = 0;	
	//
	if (cm_ptr->write_ptr < (MEM1_FRAME_SIZE - 4)) frame_addr = cm_ptr->write_ptr + 2;
	else if (cm_ptr->write_ptr >= (MEM1_FRAME_SIZE - 4)) frame_addr = cm_ptr->write_ptr + 6;
	//
	Write_Frame(frame_addr, (uint8_t*)frame);
	return 0;
}

int8_t Load_Data_Frame(typeCMParameters* cm_ptr)  // загрузка кадра с данными из памяти с выкладыванием на подадрес
{
	uint16_t frame_addr;
	uint16_t frame_max_num;
	uint16_t frame[32];
	frame_max_num = Get_Max_Data_Frame_Num();
	//
	cm_ptr->read_ptr += 1;
	if (cm_ptr->read_ptr >= frame_max_num) cm_ptr->read_ptr = 0;	
	//
	if (cm_ptr->read_ptr < (MEM1_FRAME_SIZE - 4)) frame_addr = cm_ptr->read_ptr + 2;
	else if (cm_ptr->read_ptr >= (MEM1_FRAME_SIZE - 4)) frame_addr = cm_ptr->read_ptr + 6;
	//
	Read_Frame(frame_addr, (uint8_t*)frame);
	//
	Write_to_SubAddr(arch_frame, frame);
	return 0;
}

int8_t Write_Parameters(typeCMParameters* cm_ptr)   // загрузка параметров в структуру: параметры хранятся в начале и конце памяти
{
	int8_t state = 0, i;
	uint16_t param_addr_array[4] = {0, MEM1_FRAME_SIZE-2, MEM1_FRAME_SIZE, MEM1_FRAME_SIZE+MEM2_FRAME_SIZE-2};
	cm_ptr->crc16 = crc16_ccitt((uint8_t*)cm_ptr, 126);
	for(i=0; i<4; i++){
		if ((Write_Frame(param_addr_array[i], (uint8_t*)cm_ptr) < 0) || (Write_Frame(param_addr_array[i]+1, (uint8_t*)cm_ptr+64) < 0) ) state = i;
	}
	return state;
}

int8_t Read_Parameters(typeCMParameters* cm_ptr)  // сохранение параметров из структуры в память
{
	int8_t state = 0, i;
	uint16_t parameters_frame[128];
	uint16_t param_addr_array[4] = {0, MEM1_FRAME_SIZE-2, MEM1_FRAME_SIZE, MEM1_FRAME_SIZE+MEM2_FRAME_SIZE-2};
	for(i=0; i<4; i++){
		if (_read_cm_parameters_frame_with_crc16_check(param_addr_array[i], (uint8_t*)parameters_frame) < 0) state = -1;
		else {
				memcpy((uint8_t*)cm_ptr, (uint8_t*)parameters_frame, 128);
				state = i+1;
				break;
		}
	}
	return state;	
}

int8_t _read_cm_parameters_frame_with_crc16_check(uint16_t addr, uint8_t* frame) //чтение 128-байтового кадра с параметрами
{
	Read_Frame(addr, (uint8_t*)frame);
	Read_Frame(addr+1, (uint8_t*)frame+64);
	if (crc16_ccitt((uint8_t*)frame, 128) == 0){
		return 0;
	}
	else return -1;
}

// функции для работы со структурой управления ЦМ typeCMParameters
void CM_Parame_Full_Init(typeCMParameters* cm_ptr) //функция инициализации структуры, зануляет все кроме наработки
{
    uint32_t val;
    val = cm_ptr->operating_time;
    memset((uint8_t*)cm_ptr, 0x00, sizeof(typeCMParameters));
	_cm_params_set_default(cm_ptr);
	//
    cm_ptr->operating_time = val;
}

void _cm_params_set_default(typeCMParameters* cm_ptr)
{
	cm_ptr->label = 0x0FF1; 
	cm_ptr->time = 0x00000000;
	cm_ptr->bus_error_cnt = 0x00;
	cm_ptr->bus_nans_cnt = 0x00;
	cm_ptr->bus_nans_status = 0x00;
	cm_ptr->pwr_status = 0x00;
	cm_ptr->diff_time_s = 0x00;
	cm_ptr->diff_time_low = 0x00;
	cm_ptr->sync_num = 0x00;
	cm_ptr->speed_mode_state = 0; 
	cm_ptr->speed_mode_timeout = 0;
	cm_ptr->sync_time_s = 0;
	cm_ptr->sync_time_low = 0;
	cm_ptr->adii_mode = 0;
	cm_ptr->adii_fk = 0xFF;
	
	//
	cm_ptr->pwr_bounds[0] = 0; //МБКАП - не проверяем по току
	cm_ptr->pwr_bounds[1] = CM_BOUND;
	cm_ptr->pwr_bounds[2] = MPP27_BOUND;
	cm_ptr->pwr_bounds[3] = MPP100_BOUND;
	cm_ptr->pwr_bounds[4] = DIR_BOUND;
	cm_ptr->pwr_bounds[5] = DNT_BOUND;
	cm_ptr->pwr_bounds[6] = ADII_BOUND;
	cm_ptr->pwr_state = 0x3F; // все шесть модулей включены
	//
	cm_ptr->measure_interval = DEFAULT_MEAS_INTERVAL_S;
	cm_ptr->sys_interval = DEFAULT_SYS_INTERVAL_S;
	cm_ptr->adii_interval = DEFAULT_ADII_INTERVAL_S;
}

void CM_Parame_Start_Init(typeCMParameters* cm_ptr) //функция инициализации структуры, зануляет все, что нет необходимости хранить
{
	Read_Parameters(cm_ptr);
	//
    if (cm_ptr->label != 0x0FF1){ // если первое слово не 0x0FF1, то полностью инициализируем блок
        CM_Parame_Full_Init(cm_ptr);
    }
	else{
		_cm_params_set_default(cm_ptr);
		cm_ptr->rst_cnt += 1; // увеличиваем счетчик включений ЦМ БЭ Луна
	}
}

void CM_Parame_Command_Init(typeCMParameters* cm_ptr) //функция инициализации структуры по командному сообщению, зануляет все
{
	Read_Parameters(cm_ptr);
	CM_Parame_Full_Init(cm_ptr);
	_cm_params_set_default(cm_ptr);
}

void CM_Parame_Operating_Time_Init(uint32_t op_time, typeCMParameters* cm_ptr) //функция, которая устанавливает наработку
{
    cm_ptr->operating_time = op_time;
}

// общие функции для работы с кадрами
uint16_t _frame_definer(uint8_t frame_modification, uint16_t device_number,  uint16_t fabrication_num, uint8_t frame_type)
{
	switch(frame_modification){
		case 0: // для больших аппаратур: БДК2, БКАП, МБКАП
			return ((frame_modification&0x3)<<14) |   // модификатор кадра
						((device_number&0x03FF)<<4) |  // номер аппаратуры
						((frame_type&0xF)<<0); // тип кадра
		case 1: // для мелкосерийного производства
			return (uint16_t)((frame_modification&0x3) << 14) |  // модификатор кадра
                        (uint16_t)((device_number & 0x0F) << 10) |  // номер аппаратуры
                        (uint16_t)((fabrication_num & 0x7F) << 3) |  // заводской номер
                        (uint16_t)(frame_type & 0x07);  // тип кадра
		default:
			return 0;
	}
	
}

// формирование системного кадра
void Sys_Frame_Init(typeSysFrames *sys_frame) //инициализируются известные поля для системного кадра и выкладывается на подадрес
{
    uint16_t frame[32] = {0};
    uint8_t leng;
    leng = sizeof(typeSysFrames);
    memset((uint8_t*)sys_frame, 0xFE, leng);
    sys_frame->label = 0x0FF1;
    sys_frame->definer = _frame_definer(0, DEV_NUM, 0, SYS_FRAME_NUM);
    sys_frame->time = Get_Time_s();
    sys_frame->crc16 = crc16_ccitt((uint8_t*)sys_frame, 62);
    memcpy((uint8_t *)frame, (uint8_t *)sys_frame, sizeof(typeSysFrames));
    //
    Write_to_SubAddr(SYS_FRAME_NUM, frame);
}

void Sys_Frame_Build(typeSysFrames *sys_frame, typeCMParameters* cm_ptr)
{
	uint16_t frame[32] = {0};
	
	//обязательная часть кадра
	sys_frame->label = 0x0FF1;
	sys_frame->num = cm_ptr->frame_number;
	sys_frame->definer = _frame_definer(0, DEV_NUM, 0, SYS_FRAME_NUM);
    sys_frame->time =  Get_Time_s();
	//
    memcpy(sys_frame->currents, cm_ptr->currents, 7*sizeof(uint16_t));
    sys_frame->read_ptr = cm_ptr->read_ptr;
    sys_frame->write_ptr = cm_ptr->write_ptr;
	sys_frame->mko_error_cnt = cm_ptr->mko_error_cnt;
    sys_frame->mko_error = cm_ptr->mko_error;
    sys_frame->rst_cnt = cm_ptr->rst_cnt;
    sys_frame->diff_time_s = cm_ptr->diff_time_s;
    sys_frame->diff_time_low = cm_ptr->diff_time_low;
    sys_frame->sync_num = cm_ptr->sync_num;
	sys_frame->sync_time_s = cm_ptr->sync_time_s;
	sys_frame->stm_val = cm_ptr->stm_val;
	sys_frame->sync_time_low = cm_ptr->sync_time_low;
	sys_frame->bus_nans_cnt = cm_ptr->bus_nans_cnt;
    sys_frame->bus_nans_status = cm_ptr->bus_nans_status;
    sys_frame->bus_error_status = cm_ptr->bus_error_status;
    sys_frame->bus_error_cnt = cm_ptr->bus_error_cnt;
	sys_frame->bus_error_cnt = cm_ptr->bus_error_cnt;
	sys_frame->temp = 0x00; //todo: добавить температуру;
    sys_frame->operating_time = _rev_u32((uint32_t)cm_ptr->operating_time);
    sys_frame->measure_interval = cm_ptr->measure_interval;
    sys_frame->sys_interval = cm_ptr->sys_interval;
    sys_frame->pwr_status = cm_ptr->pwr_status;
	// adii 
    sys_frame->adii_mode = cm_ptr->adii_mode;
    sys_frame->adii_fk = cm_ptr->adii_fk;
    sys_frame->adii_interval = cm_ptr->adii_interval;
	
	sys_frame->crc16 = crc16_ccitt((uint8_t*)sys_frame, 62);
    memcpy((uint8_t *)frame, (uint8_t *)sys_frame, sizeof(typeSysFrames));
	//
	cm_ptr->frame_number ++;
    Write_to_SubAddr(SYS_FRAME_NUM, frame);
    Save_Data_Frame((uint8_t*)sys_frame, cm_ptr);
}

// работа с МКО
uint8_t get_mko_addr_from_gpio(void)
{
	uint8_t addr, addr_parity,  connector_parity;
	addr = GPIO_MKO_Id()  & 0x1F;
	addr_parity = ((((addr >> 4) & 0x01) + ((addr >> 3) & 0x01) + ((addr >> 2) & 0x01) + ((addr >> 1) & 0x01) + ((addr >> 0) & 0x01)) & 0x01) ^ 0x01;
	connector_parity = (GPIO_MKO_Id() >> 5)  & 0x1F;
	if (addr_parity == connector_parity) return addr;
	else return 0;
}

uint8_t get_mko_addr(uint8_t def_addr)
{
	uint8_t mko_addr = 0x00;
	mko_addr = get_mko_addr_from_gpio();
	if ((mko_addr == 0) && (def_addr)) {
		mko_addr = def_addr;
	}
	return mko_addr;
}

// управление питанием
void  Pwr_current_process(typeCMParameters* cm)
{
	uint8_t i, status_1 = 0, status_2 = 0;
	status_1 = Get_Modules_Current(cm->currents, cm->pwr_bounds); //токи [МБКАП, ЦМ, МПП100, МПП27, ДИР, ДНТ, АДИИ]
	if (status_1 & 0x3F) status_2 = Get_Modules_Current(cm->currents, cm->pwr_bounds); //в случае обнаружения превышения на всякий прочитаем еще раз, что бы исключаить разовый выброс от ВЧ-помехи
	cm->pwr_status |= (status_1 & status_2);
	if ((status_1 & status_2)  != 0){
		for (i=0; i<7; i++) { //отключаем таким способом все блоки кроме МБКАП и ЦМ
			if (((status_1 & status_2) & (1 << i)) != 0) {
				cm->pwr_state &= ~(0x01 << i); //если два раза измерения токов показали превышения для отдельного модуля, то подготовливаем state для его отключения
			} 
		}
	}
	Pwr_Ctrl_by_State(cm->pwr_state); //pwr_state: 0-CM, 1-MPP27, 2-MPP100, 3-DIR, 4-DNT, 5-ADII  6-7-NU
}

//отладочный интерфейс
int8_t Debug_Get_Packet (uint16_t* reg_addr, uint16_t* data, uint8_t* leng)
{
    int8_t status = 0;
    uint8_t data_leng = 0;
	uint8_t in_buff[128], out_buff[128];
    status = UART0_GetPacket(in_buff, &data_leng);
    if (status == -1)
    {
        return -1;
    }
    if (status == 0)
    {
        return 0;
    }
    else if(status == 1)
    {
        if (in_buff[0] == 1)
        {
            if (in_buff[1] == 0x10)
            {
                if ((in_buff[2] == 0x00) & (in_buff[3] == 0x00) & (in_buff[6] >= 4))
                {
                    memcpy(data, &in_buff[7], 4);
                    *leng = in_buff[5];
                    *reg_addr = (in_buff[2]<<8) + (in_buff[3]);
                    memcpy(out_buff, in_buff, 6);
                    UART0_SendPacket(out_buff, 6, 1);
                    return 1;
                }
            }
            else
            {
                out_buff[0] = in_buff[0];
                out_buff[1] = in_buff[0] | 0x80;
                out_buff[2] = 0x01;
                UART0_SendPacket(out_buff, 3, 1);
                return 0;
            }
        }
    }
    return -1;
}

// ВШ
int8_t F_Trans(uint8_t code, uint8_t dev_id, uint16_t start_addr, uint16_t cnt, uint16_t * data_arr) //функция, позволяющая отправлять стандартизованные ModBus запросы/ответы
{
	int8_t status = 0;
    uint8_t i, flag = 1, out_leng = 0, time_out = 0, time_bound = UART_TIMEOUT_MS;
    flag <<= (dev_id);
	UART0_GetPacket(in_buff, &out_leng); //clear input fifo
	// формирование запроса
	switch(code){
		case 3:
			out_buff[0] = dev_id;
			out_buff[1] = 0x03;
			out_buff[2] = start_addr >> 8;
			out_buff[3] = start_addr & 0xFF;
			out_buff[4] = cnt >> 8;
			out_buff[5] = cnt & 0xFF;
			out_leng = 6;
			break;
		case 6:
			out_buff[0] = dev_id;
			out_buff[1] = 0x06;
			out_buff[2] = start_addr >> 8;
			out_buff[3] = start_addr & 0xFF;
			out_buff[4] = data_arr[0] >> 8;
			out_buff[5] = data_arr[0] & 0xFF;
			out_leng = 6;
			break;
		case 16:
		    out_buff[0] = dev_id;
			out_buff[1] = 0x10;
			out_buff[2] = start_addr >> 8;
			out_buff[3] = start_addr & 0xFF;
			out_buff[4] = 0x00;
			out_buff[5] = cnt;
			out_buff[6] = cnt*2;
			memcpy(&out_buff[7], (uint8_t*)data_arr, cnt*2);    
			out_leng = cnt*2+7;
			break;
	}
	//
    for (i = 0; i<3; i++) {
		UART0_SendPacket(out_buff, out_leng, 1);  //отправляем запрос
		while (time_out <= time_bound) { //ждем начала приема
			Timers_Start(1, 1); // запускаем таймер на 1 мс
			while (Timers_Status(1) == 0) {};
			time_out += 1;
			if (UART0_PacketInWaitingOrReady()){
				time_bound += 1;
			}
			else{
				//
			}
			status = UART0_GetPacket(in_buff, &out_leng);
			if (status == -1) {
				cm.bus_error_cnt += 1;
				cm.bus_error_status |= flag;
				break;
			}
			else if (status == 0) {
				//
			}
			else if (status == 1) {
				i = 3;
				break;
			}
			else if (time_out >= 20){ //дополнительный таймаут на случай генерации в канале
				break;
			}
		}
		Timers_Start(1, 1); // дополнительный таймаут
		while (Timers_Status(1) == 0) {};
        Timers_Stop(1);
	}
	if (status == 0) {
		cm.bus_nans_cnt += 1;
		cm.bus_nans_status |= flag; // todo: сопоставить флаги с модулями
	}
	else { //
		if (code == 3){
			memcpy((uint8_t*)data_arr, &in_buff[6], in_buff[2]);
		}
	}
	return status;
}

//получение идентификационной строки для переферии
int8_t Pereph_On_and_Get_ID_Frame(uint8_t dev_num, typeDevStartInformation* dev_init_inf_ptr) //включаем переферии и получаем от нее идентификационный пакет
{
	uint8_t in_buff[256], leng=0, i=0, inf_leng=0;
	int8_t status;
	//включаем переферийное устройство
	GPIO_Pwr(dev_num, 1); //dev_num: 0-МБКАП, 1-CM, 2-MPP27, 3-MPP100, 4-DIR, 5-DNT, 6-ADII 
	Timers_Start(1, 500); 
    while (Timers_Status(1) == 0);
	status = UART0_GetPacket(in_buff, &leng);
	if (status == -1) return -1;
	else if (status == 0) return 0;
	else{
		dev_init_inf_ptr->id = in_buff[0];
		memcpy(dev_init_inf_ptr->fixed_field, in_buff+1, 6);
		dev_init_inf_ptr->inf_filed_number = (in_buff[7]) & 0x07; // пока ограничеваемся 8-ю информационными полями: сейчас используется 3, 5 в запасе
		for (i=0; i<(dev_init_inf_ptr->inf_filed_number); i++){
			dev_init_inf_ptr->inf_field_arr[i].number = in_buff[8 + 0 + inf_leng];
			dev_init_inf_ptr->inf_field_arr[i].leng = in_buff[8 + 1 + inf_leng] & 0x0F; //ограничиваем длину до 15-ти символов плюс 0-терминатор
			memcpy(dev_init_inf_ptr->inf_field_arr[i].string, &in_buff[8 + 2 + inf_leng], dev_init_inf_ptr->inf_field_arr[i].leng);
			inf_leng += dev_init_inf_ptr->inf_field_arr[i].leng + 2;
		}
		return dev_init_inf_ptr->inf_filed_number;
	}
}

// Внутрениие рабочие функции
uint32_t _rev_u32 (volatile uint32_t val) //перестановка по 16-ти битным словам
{
    return ((val & 0xFFFF0000) >> 16) + ((val & 0xFFFF) << 16);
}

void _buff_rev16(uint16_t *buff, uint8_t leng_16)
{
	uint16_t i = 0;
	for (i=0; i<leng_16; i++) {
		buff[i] = __REV16(buff[i]);
	}
}

uint8_t uint16_to_log2_uint8_t(uint16_t var)
{
	float man;
	uint8_t man_uint8, i;
	int exp;
	man = frexp(var, &exp);
	for (i=0; i<4;i++)
	{
		if (man == 0) break;
		man = man*2;
		exp -= 1;
		if (exp == 0) break;
	}
	man_uint8 = man;
	return ((exp & 0xF) << 4) + ((man_uint8 & 0xF) << 0);
}


