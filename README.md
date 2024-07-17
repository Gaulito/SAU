# Readme файл с описанием файлов в репозитории

Для написания кода использован язык C, разработка ведется в среде разработки STM32CubeIDE.

Основные файлы располагаются в директории Core, внутри директорий Src и Inc соответственно.

Имеется написанная на языке Wolfram симуляция полета (использованы точки прошлого года), исходные файлы Wolfram Mathematica приложены в директории Wolfram.
Для запуска симуляции необходимо скачать и поместить все файлы в одну папку.

Файлы запускаются в таком порядке: 1) траектория полета.nb, 2) автопилот.nb 
## Список файлов для работы с периферийными устройствами:

bmp180.c, bmp180.h — Датчик атмосферного давления BMP180.

hmc5883.c, hmc5883.h — Трехосевой магнитометр HMC5883.

mpu6050.c, mpu6050.h — Трехосевой акселерометр-гироскоп MPU6050.

ms4525.c, ms4525.h — Датчик воздушной скорости Matek ASPD-4525

UartRingbuffer.c, UartRingbuffer.h — Получение строчных данных с GNSS датчиков.

NMEA.c, NMEA.h — Работа с протоколом NMEA0183 для расшифровки строчных данных с GNSS датчиков .

## Список файлов, содержащих основные алгоритмы программы:
main.c, main.h — Основной файл программы, содержащий управляющий алгоритм САУ, а также инициализацию подсистем микроконтроллера.

kalman.c, kalman.h — Фильтр Калмана.
