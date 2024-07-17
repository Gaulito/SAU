#include "NMEA.h"
#include "math.h"
#include "UartRingbuffer.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"



int GMT = +300;		// Настройка временной зоны



int inx = 0;
int hr=0,min=0,day=0,mon=0,yr=0;
int daychange = 0;

/* Декодирует данные GGA
   @GGAbuffer - это буфер, в котором хранятся данные GGA
   @GGASTRUCT - это указатель на структуру GGA (в структуре GPS)
   @Возвращает 0 в случае успешного выполнения
   @Возвращает 1, 2 в зависимости от того, где выполняется оператор return
*/
int decodeGGA (char *GGAbuffer, GGASTRUCT *gga)
{
	inx = 0;
	char buffer[12];
	int i = 0;
	while (GGAbuffer[inx] != ',') inx++;  // 1-ая часть ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // После времени ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // После широты ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // После NS ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // После долготы ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // После EW ','
	inx++;
	if ((GGAbuffer[inx] == '1') || (GGAbuffer[inx] == '2') || (GGAbuffer[inx] == '6'))   // 0 означает отсутствие FIX
	{
		gga->isfixValid = 1; // FIX доступен
		inx = 0;  			 // Сброс индекса. Мы начнем с inx=0 и теперь будем извлекать информацию
	}
	else
	{
		gga->isfixValid = 0; // В случае, если FIX не доступен
		return 1;  			 // Возврат ошибки
	}
	while (GGAbuffer[inx] != ',') inx++;  // Первый блок ','


/*********************** Получение ВРЕМЕНИ ***************************/

	inx++;   // Доходим до первого числа во времени
	memset(buffer, '\0', 12);
	i=0;
	while (GGAbuffer[inx] != ',')  // копируем пока не дойдем до конца блока ','
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}

	hr = (atoi(buffer)/10000) + GMT/100;   // получение часов из 6-тизначного числа

	min = ((atoi(buffer)/100)%100) + GMT%100;  // получение минут из 6-тизначного числа

	// донастройка времени (чтобы не было 60 минут или 24 часов)
	if (min > 59) 
	{
		min = min-60;
		hr++;
	}
	if (hr<0)
	{
		hr=24+hr;
		daychange--;
	}
	if (hr>=24)
	{
		hr=hr-24;
		daychange++;
	}

	// Храним время в структуре GGA
	gga->tim.hour = hr;
	gga->tim.min = min;
	gga->tim.sec = atoi(buffer)%100;

/***************** Получение ШИРОТЫ  **********************/
	inx++;   // Доходим до первого числа в широте
	memset(buffer, '\0', 12);
	i=0;
	while (GGAbuffer[inx] != ',')   // копируем пока не дойдем до конца блока широты ','
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	if (strlen(buffer) < 6) return 2;  // Если длина буфера неподходящая, возвращается ошибка
	int16_t num = (atoi(buffer));   // Переводит буфер в числовой формат. Конвертирует только до десятых
	int j = 0;
	while (buffer[j] != '.') j++;   // Подсчитывает, сколько цифр до десятых
	j++;
	int declen = (strlen(buffer))-j;  // Подсчитывает, сколько цифр после десятых
	int dec = atoi ((char *) buffer+j);  // Переводит десятичную часть в отдельное число
	float lat = (num/100) + (num%100 + dec/pow(10, (declen+2)))/0.6;  // 1234.56789 = 12.3456789
	gga->lcation.latitude = lat;  // сохраняет данные о широте в структуре
	inx++;  
	gga->lcation.NS = GGAbuffer[inx];  // сохраняет N/S в структуре


/***********************  Получение ДОЛГОТЫ **********************/
	inx++;  // ',' после знака NS
	inx++;  // При достижении первой цифры в долготе
	memset(buffer, '\0', 12);
	i=0;
	while (GGAbuffer[inx] != ',')  // копируем пока не дойдем до конца блока широты ','
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	num = (atoi(buffer));  // Переводит буфер в числовой формат. Конвертирует только до десятых
	j = 0;
	while (buffer[j] != '.') j++;  // Подсчитывает, сколько цифр до десятых
	j++;
	declen = (strlen(buffer))-j;  // Подсчитывает, сколько цифр после десятых
	dec = atoi ((char *) buffer+j);  // Переводит десятичную часть в отдельное число
	lat = (num/100) + (num%100 + dec/pow(10, (declen+2)))/0.6;  // 1234.56789 = 12.3456789

	gga->lcation.longitude = lat;  // сохраняет данные о долготе в структуре
	inx++;
	gga->lcation.EW = GGAbuffer[inx];  // сохраняет E/W в структуре

/**************************************************/
	// пропуск FIX-а позиции
	inx++;   // ',' после E/W
	inx++;   // FIX позиции
	inx++;   // ',' после FIX-а позиции;

	// число спутников
	inx++;  //  При достижении первой цифры в спутниках
	memset(buffer, '\0', 12);
	i=0;
	while (GGAbuffer[inx] != ',')  // Копируем до ',' после числа спутников
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	gga->numofsat = atoi(buffer);   // Преобразуем буфер в число и сохраняем в структуре


	/***************** Пропуск HDOP  *********************/
	inx++;
	while (GGAbuffer[inx] != ',') inx++;


	/*************** Вычисление высоты ********************/
	inx++;
	memset(buffer, '\0', 12);
	i=0;
	while (GGAbuffer[inx] != ',')
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	num = (atoi(buffer));
	j = 0;
	while (buffer[j] != '.') j++;
	j++;
	declen = (strlen(buffer))-j;
	dec = atoi ((char *) buffer+j);
	lat = (num) + (dec/pow(10, (declen)));
	gga->alt.altitude = lat;

	inx++;
	gga->alt.unit = GGAbuffer[inx];

	return 0;

}


int decodeRMC (char *RMCbuffer, RMCSTRUCT *rmc)
{
	inx = 0;
	char buffer[12];
	int i = 0;
	while (RMCbuffer[inx] != ',') inx++;  // 1-ый блок ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // После времени ,
	inx++;
	if (RMCbuffer[inx] == 'A')  // Здесь 'A' отображает что данные в порядке, а 'V' что не в порядке
	{
		rmc->isValid = 1;
	}
	else
	{
		rmc->isValid =0;
		return 1;
	}
	inx++;
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // после широты,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // после NS ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // после долготы ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // после EW ,

	// Получение скорости
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	if (strlen (buffer) > 0){          // проверка на то, что буффер скорости содержит какую-либо информацию
		int16_t num = (atoi(buffer));  // преобразование данных в число
		int j = 0;
		while (buffer[j] != '.') j++;   // аналогично вышеприведенному
		j++;
		int declen = (strlen(buffer))-j;
		int dec = atoi ((char *) buffer+j);
		float lat = num + (dec/pow(10, (declen)));
		rmc->speed = lat;
	}
	else rmc->speed = 0;

	// Получение курса
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	if (strlen (buffer) > 0){  // проверка на то, что буффер курса содержит какую-либо информацию
		int16_t num = (atoi(buffer));   // преобразование данных в число
		int j = 0;
		while (buffer[j] != '.') j++;   // аналогично вышеприведенному
		j++;
		int declen = (strlen(buffer))-j;
		int dec = atoi ((char *) buffer+j);
		float lat = num + (dec/pow(10, (declen)));
		rmc->course = lat;
	}
	else
		{
			rmc->course = 0;
		}

	// Получение даты
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	// Дата в формате 280222
	day = atoi(buffer)/10000;  // Выделяем 28
	mon = (atoi(buffer)/100)%100;  // Выделяем 02
	yr = atoi(buffer)%100;  // Выделяем 22

	day = day+daychange;   // Корректировка по временной зоне GMT

	// Сохраняем данные в структуру
	rmc->date.Day = day;
	rmc->date.Mon = mon;
	rmc->date.Yr = yr;

	return 0;
}






int getGPS (GPSSTRUCT *gpsData)
{
	char GGA[100];
	char RMC[100];

	int flagGGA = 0, flagRMC = 0;
	int VCCTimeout = 5000; // GGA или RMC не будут получены, если VCC недостаточен

	if (Wait_for("GGA") == 1)
	{

		VCCTimeout = 5000;  // Сброс тайм-аута VCC, указывающего на получение GGA

		Copy_upto("*", GGA);
		if (decodeGGA(GGA, &gpsData->ggastruct) == 0) flagGGA = 2;  // 2 показывает, что данные в порядке
		else flagGGA = 1;  // 1 показывает, что данные не в порядке
	}

	if (Wait_for("RMC") == 1)
	{
		VCCTimeout = 5000;  // Сброс тайм-аута VCC, указывающего на получение RMC

		Copy_upto("*", RMC);
		if (decodeRMC(RMC, &gpsData->rmcstruct) == 0) flagRMC = 2;  // 2 показывает, что данные в порядке
		else flagRMC = 1;  // 1 показывает, что данные не в порядке
	}

	if (VCCTimeout <= 0)
	{
		VCCTimeout = 5000;  // Сброс тайм-аута

		//Сброс флагов
		flagGGA =flagRMC =0;
	}

	if ((flagGGA == 2) | (flagRMC == 2))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

