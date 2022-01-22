
Control of carbon dioxide in the air (stm32f411 + MQ135 + SI7021 + GC9A01)

#########################################################
#
#     Control of carbon dioxide in the air
# CO2_sensor - stm32f411 + MQ135 + SI7021/BME280 + GC9A01
#
#########################################################


## Состав рабочего оборудования:

```
* STM32F411 (New BlackPill board) - плата микроконтроллера
* GC9A01 - IPS дисплей 240x240 (интерфейы SPI + DMA + GPIO)
* SI7021 - датчик температуры и влажности воздуха(интерфейс I2C + DMA).
* MQ135 - датчик углекислого газа в атмосфере (интерфейс GPIO + ADC)
```


# Средства разработки:

```
* STM32CubeIDE - среда разработки под микроконтроллеры семейства STM32
  (https://www.st.com/en/development-tools/stm32cubeide.html).
* ST-LINK V2 - usb отладчик для микроконтроллеров семейства STM8/STM32.
* Saleae USB Logic 8ch - логический анализатор сигналов, 8 каналов , макс. частота дискретизации 24МГц
  (https://www.saleae.com/ru/downloads/)
```


# Функционал:
* Устройство измеряет уровень углекислого газа в атмосфере, при этом для корректировки используются данные
  температуры и влажности, полученные с датчика si7021(bme280). Результирующие значения уровня CO2, полученные с
  12-ти разрядного АЦП, пропускаются через цифровой фильтр ('скользящее' окно размерностью в 8 замеров).
  Полученное фильтрованное значение (ppm) преобразуется в проценты и выводится на дисплей вмести со
  значениеми температуры и влажности.
* ПО построено по модели BARE METAL (без использования ОС) с использованием буфера событий типа fifo.
  События обслуживаются в основном цикле программы. Формируются события в callBack-функциях
  по завершении прерываний от используемых модулей микроконтроллера.
* ПО реализовано средствами разработки STM32CubeIDE и состоит из 3-х частей.
    1. BOOTLOADER - загрузчик (занимает сектора 0,1 внутренней flash-памяти)
    2. RELEASE - собственно API (занимает сектора  внутренней flash-памяти со второго и далее)
    3. DEBUG - отладочная версия API (занимает сектора внутренней flash-памяти с 0 и далее )
  Для записи API средствами загрузчика написана утилита, позволяющая менять версию API через
  последовательный интерфейс UART1 микроконтроллера в режиме загрузчика.
* Устройство инициализирует некоторые интерфейсы микроконтроллера :
  - ADC1 : аналогово-цифровой преобразователь (измеряет напряжение на аналоговом выходе датчика MQ135).
  - GPIO : подключены два сетодиода : PC13 - секундный тик, PB10 - индикатор ошибки на устройстве,
           PB2 - цифровой выход датчика MQ135, PB12 - светодиод индикации записи/чтения внутренней
           flash-памяти в режиме загрузчика; PA0 - пользовательская кнопка.
  - I2C1 : режим мастера с частотой 400Кгц (шина ослуживает si7021).
  - USART1 : параметры порта 230400 8N1 - порт для логов и передачи команд устройству(в режиме API/DEBUG),
             а также для смены прошивки (версии API) устройства в режиме загрузчика.
  - USART2 : параметры порта 230400 8N1 - порт для выдачи логов в режиме загрузчика при записи/чтении flash-памяти.
  - TIM3 : таймер-счетчик временных интервалов в 10 мс., реализован в callback-функции.
  - SPI1 : обслуживает дисплей GC9A01.
  - DMA MemtoMem : этот DMA ресурс используется для чтения внутренней flash-памяти.
* Прием данных по последовательному порту (USART1) выполняется в callback-функции обработчика прерывания.

После подачи питания или нажатия на кнопку 'Reset' начинается выполнение загрузчика.
Загрузчик проверяет состояние 'пользовательской кнопки' и если она нажата, то переходит в режим
ожидания команд (30 сек.) от внешней утилиты по интерфейсу USART1, если же она не нажата - проверяется
есть ли во внутренней flash-памяти заголовок API и выполняется переключение на API по данным из заголовка.
При успешном запуске API в порту USART1 появятся следующие сообщения :


```
0.00:00:00 | ver.1.6.2 20.01.2022 API Start in fifo_event_loop mode
0.00:00:00 | Register callback function 'dmaTransferDone()' OK
0.00:00:01 | [main] MQ135 RZERO first calibration value : 327.50 kOhm
```

* Через USART1 можно отправлять команды на устройство, например :

```
ver
0.00:00:07 | [que:1] ver.1.6.2 20.01.2022 API


get
0.00:00:10 | [que:1] MQ135: adc=925 ppm=335, SI7021: temp=24.31 humi=23.03


hdr
0.00:00:12 | [que:1] Hdr(0x807FFF0): mark=0x5AA5F00F adr=0x8008000 len=0xF2C4 crc=0x77F9E9C9


sion
0.00:00:26 | humi: val=0x3C66 crc=0x1B/0x1B temp: val=0x67B8 crc=0xFD/0xFD
0.00:00:28 | humi: val=0x3BEE crc=0x76/0x76 temp: val=0x67B8 crc=0xFD/0xFD
0.00:00:30 | humi: val=0x3C96 crc=0x99/0x99 temp: val=0x67B4 crc=0x80/0x80
0.00:00:32 | humi: val=0x3C7A crc=0x25/0x25 temp: val=0x67B0 crc=0x44/0x44
0.00:00:33 | humi: val=0x3C22 crc=0xE2/0xE2 temp: val=0x67B0 crc=0x44/0x44
sioff


read:0x807ff00:256
0.00:00:45 | Read flash: adr=0x807FF00 len=256
0807FF00  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF
0807FF20  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF
0807FF40  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF
0807FF60  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF
0807FF80  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF
0807FFA0  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF
0807FFC0  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF
0807FFE0  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 0F F0 A5 5A 00 80 00 08 C4 F2 00 00 C9 E9 F9 77


restart
0.00:00:54 | [que:1] Restart...
```

Здесь же появляются периодические данные о текущих значения с датчика :

```
0.00:00:15 | [que:1] temp=24.33 humi=24.33 ppm=329
0.00:00:30 | [que:1] temp=24.33 humi=23.58 ppm=335
0.00:00:44 | [que:1] temp=24.32 humi=23.19 ppm=331
```

* Проект в процессе доработки

