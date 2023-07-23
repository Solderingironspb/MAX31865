/**
 ******************************************************************************
 *  @file MAX31865.c
 *  @brief Библиотека для работы с MAX31865(Преобразователь температуры для термосопротивления PT100)
 *  @author Волков Олег
 *  @date 31.01.2023
 *
 ******************************************************************************
 * @attention
 *
 *  MAX31865 выполняет компенсацию подключения датчика в режимах 3 или 4 проводное.
 *  Поддерживает и 2 проводное подключение.
 *  Микросхема отлично держит температуру окружающей среды и стабильно измеряет сопротивление датчика,
 *  что очень хорошо для точных измерений.
 *
 *  SPI: Максимальная скорость SCK =  5 MHz
 *       CS(NSS) в активном состоянии подтянут к земле
 *       CPOL = 0 или 1
 *       CPHA = 1
 *       Работа с данными осуществляется в 8-битном формате.
 *
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb/Lessons-Stm32/blob/master/README.md Группа
 *  ВК: https://vk.com/solderingiron.stm32
 *  Документация: 
 *  
 *
 ******************************************************************************
 */

#include "MAX31865.h"

/*--------------Характеристики датчика типа PT100 и референсный резистор, подключенный к MAX31865------------*/
#define MAX31865_PT100_R0 (double)100.0 //Сопротивление датчика PT100, при 0 °С
#define MAX31865_R_REF (double)428.5 //Сопротивление референсного резистора, подключенного к MAX31865
/*--------------Характеристики датчика типа PT100 и референсный резистор, подключенный к MAX31865------------*/

/*-----------------------------------------Глобальные переменные---------------------------------------------*/
float MAX31865_PT100_R = 0.0; //Глобальная переменная, определяющая сопротивление датчика PT100
float MAX31865_PT100_T = 0.0; //Глобальная переменная, определяющая температуру датчика PT100
float MAX31865_Correction_additive = 0.0f; //Калибровка смещения
float MAX31865_Correction_multiplicative = 1.0f; //Калибровка наклона
bool MAX31865_Sensor_Error = 0; //Глобальная переменная, определяющая неисправность датчика PT100
/*-----------------------------------------Глобальные переменные---------------------------------------------*/

/*-------------------------------------------Для работы по spi-----------------------------------------------*/
#if defined (USE_HAL)
extern SPI_HandleTypeDef hspi1; //Шина SPI, которую будем подключать.
#endif
//P.S. Максимальная скорость spi 5 МГц.
//Также обратите внимание, что CLPOL = 1 или 0. CPHA = 1.
/*-------------------------------------------Для работы по spi-----------------------------------------------*/

/*
 **************************************************************************************************
 *  @breif Функция инициализация модуля MAX31865
 *  @attention Не вижу особого смысла выводить полную настройку модуля, поэтому сделаем
 *  небольшое упрощение для конечного пользователя. Все, что может настроить пользователь
 *  - это выбрать тип подключения: 2, 3 или 4 проводное
 *  @param  *SPI или *hspi - шина SPI
 *  @param  num_wires - тип подключения датчика 2,3 или 4 проводное
 *  @retval  Возвращает статус приема. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */

#if defined (USE_CMSIS)
void MAX31865_Init(SPI_TypeDef* SPI, uint8_t num_wires) {
#elif defined (USE_HAL)
void MAX31865_Init(SPI_HandleTypeDef *hspi, uint8_t num_wires) {
#endif

	MAX31865_Sensor_Error = 0;
	uint8_t MAX31865_Configuration_register_write[] = { 0x80, 0x00 };
	if (num_wires == 2 || num_wires == 4) {
		MAX31865_Configuration_register_write[1] = 0xC3;
	} else if (num_wires == 3) {
		MAX31865_Configuration_register_write[1] = 0xD3;
	}
	NSS_ON;
#if defined (USE_CMSIS)
	CMSIS_SPI_Data_Transmit_8BIT(SPI, MAX31865_Configuration_register_write, 2, 100);
#elif defined (USE_HAL)
	HAL_SPI_Transmit(hspi, MAX31865_Configuration_register_write, 2, 100);

#endif
	NSS_OFF
	;

}

/*
 **************************************************************************************************
 *  @breif Получить информацию о конфигурации модуля MAX31865 
 *  @param  *SPI или *hspi - шина SPI
 *  @retval  Возвращает значение конфигурации.
 *  @attention Не удивляйтесь, если отправите при инициализации 0xC3, а получите 0xC1
 *  (См. datasheet MAX31865 стр. 14 "The fault status clear bit D1, self-clears to 0.")
 **************************************************************************************************
 */

#if defined (USE_CMSIS)
uint8_t MAX31865_Configuration_info(SPI_TypeDef* SPI) {
#elif defined (USE_HAL)
uint8_t MAX31865_Configuration_info(SPI_HandleTypeDef *hspi) {
#endif

	uint8_t read_data = 0x00;
	uint8_t MAX31865_Configuration = 0x00;

	NSS_ON;
#if defined (USE_CMSIS)
	CMSIS_SPI_Data_Transmit_8BIT(SPI, &read_data, 1, 100);
	CMSIS_SPI_Data_Receive_8BIT(SPI, &MAX31865_Configuration, 1, 100);
#elif defined (USE_HAL)
	HAL_SPI_Transmit(hspi, &read_data, 1, 100);
	HAL_SPI_Receive(hspi, &MAX31865_Configuration, 1, 100);
#endif
	NSS_OFF
	;

	return MAX31865_Configuration;
}

/*
 **************************************************************************************************
 *  @breif Основная функция работы с модулем MAX31865
 *  @attention Просходит обращение к начальному адресу регистра памяти модуля и из него читаем 7 байт.
 *  В функцию также включена самодиагностика модуля, которая сообщит, если с датчиком будет что-то не так.
 *  @param  *SPI или *hspi - шина SPI
 *  @retval  Возвращает значение конфигурации.
 *  @attention Не удивляйтесь, если отправите при инициализации 0xC3, а получите 0xC1
 *  (См. datasheet MAX31865 стр. 14 "The fault status clear bit D1, self-clears to 0.")
 **************************************************************************************************
 */

#if defined (USE_CMSIS)
double MAX31865_Get_Resistance(SPI_TypeDef* SPI) {
#elif defined (USE_HAL)
double MAX31865_Get_Resistance(SPI_HandleTypeDef *hspi) {
#endif

	uint8_t MAX31865_rx_buffer[7]; //буфер, куда будем складывать приходящие данные
	double data; //переменная для вычислений

	struct rx_data_MAX31865 {
		uint16_t RTD_Resistance_Registers; //Регистры сопротивления
		uint16_t High_Fault_Threshold; //Верхний порог неисправности
		uint16_t Low_Fault_Threshold; //Нижний порог неисправности
		uint8_t Fault_Status; //Статус неисправности
	};

	struct rx_data_MAX31865 MAX31865_receieve_data;

	uint8_t MAX31865_start_address_of_the_poll = 0x01; //Адрес регистра, с которого начнем чтение данных

	NSS_ON;
#if defined (USE_CMSIS)
	CMSIS_SPI_Data_Transmit_8BIT(SPI, &MAX31865_start_address_of_the_poll, 1, 100);
	CMSIS_SPI_Data_Receive_8BIT(SPI,  MAX31865_rx_buffer, 7, 100);
#elif defined (USE_HAL)
	HAL_SPI_Transmit(hspi, &MAX31865_start_address_of_the_poll, 1, 100);
	HAL_SPI_Receive(hspi, MAX31865_rx_buffer, 7, 100);
#endif
	NSS_OFF
	;

	MAX31865_receieve_data.RTD_Resistance_Registers = ((MAX31865_rx_buffer[0] << 8) | MAX31865_rx_buffer[1]) >> 1; //Данные регистров сопротивления
	MAX31865_receieve_data.High_Fault_Threshold = ((MAX31865_rx_buffer[2] << 8) | MAX31865_rx_buffer[3]) >> 1; //Данные верхнего порого неисправности
	MAX31865_receieve_data.Low_Fault_Threshold = (MAX31865_rx_buffer[4] << 8) | MAX31865_rx_buffer[5]; //Данные нижнего порога неисправности
	MAX31865_receieve_data.Fault_Status = MAX31865_rx_buffer[6]; //Статус неисправности
	if (MAX31865_receieve_data.Fault_Status > 0x00) {

		/*--------------Здесь Ваши действия по реагированию на ошибку датчика---------------*/
		MAX31865_Sensor_Error = 1;

		/*----Автоматический сброс ошибки----*/
#if defined (USE_CMSIS)
		MAX31865_Init(SPI, 3);
#elif defined (USE_HAL)
		MAX31865_Init(hspi, 3);
#endif		
		MAX31865_Sensor_Error = 0;
		/*----Автоматический сброс ошибки----*/

		//Так можно сбросить ошибку, проинициализировав датчик заново.
		//Сброс ошибки, по желанию. Обычно ее не сбрасывают в автомате, а зовут оператора, чтоб квитировал ошибку.
		//До прихода оператора, установка находится в ошибке, все управляющие узлы должны отключаться.
	}

	data = ((double) MAX31865_receieve_data.RTD_Resistance_Registers * MAX31865_R_REF ) / (double) 32768.0;
	return data;
}

double MAX31865_Get_Temperature(double Resistance) {
	double Temperature = Get_Temperature_PT(Resistance, PT100_R0, 0);
	return Temperature;
}

