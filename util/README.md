
# util - Утилита для записи прошивки API из двоичного файла

Утилита написана на языке С для ОС Linux с использование только средств libc.


## Запуск утилиты

При старте утилиты необходимо задать несколько параметров :

```
--dev=имя последовательного порта (например /dev/ttyUSB0)  - обязательный параметр
--speed=скорость (по умолчанию 230400) - не обязательный параметр
--mode=режим работы  - обязательный параметр
    возможны следующие режимы работы:
    - download - чтение прошивки API из устройства в бинарный файл
    - compare - сравнение прошивки API из устройства с бинарным файлом
    - prog - программирование прошивки API из бинарного файла в устройство
    - crc - подсчет контрольной суммы CRC32 бинарного файла
```

### Для чтения прошивки из устройства (предварительно переведя устройство в режим загрузчика) :

  ./util --dev=/dev/ttyUSB0 --mode=download (скрипт down.sh)

Прочитанные из устройства данные (версия API), будут записаны в двоичный файл с именем по такому формату :

api-<дата_время>.bin

например:
```
  api-20.01_19:37:28.bin
где
  20.01 - 20 января
  19:37:28 - время чтения прошивки из устройства
```

Пример:
```
20.01 19:37:22.007 | Start util ver.0.9 in mode 'download' port /dev/ttyUSB0:230400 ('')
.............................................................
20.01 19:37:28.234 | API from device saved to file 'api-20.01_19:37:28.bin', CRC32:0x77F9E9C9/0x77F9E9C9
20.01 19:37:28.234 | Stop util for /dev/ttyUSB0 (6 sec.)
```

### Для сравнения прошивки из устройства с файлом (предварительно переведя устройство в режим загрузчика) :

  ./util --dev=/dev/ttyUSB0 --mode=compare --file=new.bin (скрипт comp.sh)

Пример:
```
20.01 19:39:39.699 | Start util ver.0.9 in mode 'compare' port /dev/ttyUSB0:230400 ('new.bin')
.............................................................
20.01 19:39:45.925 | API in device and in file 'new.bin' COMPLETELY MATCH ! (SIZE:62148 CRC32:0x77F9E9C9)
20.01 19:39:45.925 | Stop util for /dev/ttyUSB0 (6 sec.)
```

### Для прошивки файла в устройство (предварительно переведя устройство в режим загрузчика) :

  ./util --dev=/dev/ttyUSB0 --mode=prog --file=new.bin (скрипт prog.sh)

Пример:
```
20.01 19:40:50.706 | Start util ver.0.9 in mode 'prog' port /dev/ttyUSB0:230400 ('new.bin')
.............................................................
20.01 19:40:57.846 | API from file 'new.bin' write to device done. Blocks:61 Bytes:62148 CRC32:0x77F9E9C9
.............................................................
20.01 19:41:05.053 | API in device and in file 'new.bin' COMPLETELY MATCH ! (SIZE:62148 CRC32:0x77F9E9C9)
20.01 19:41:05.053 | Stop util for /dev/ttyUSB0 (15 sec.)
```

### Для подсчета контрольной суммы файла (устройсво не принимает участия в этой процедуре) :

  ./util --mode=crc --file=new.bin

Пример:
```
20.01 19:41:59.371 | Start util ver.0.9 in mode 'crc' for file 'new.bin'
    mode:'crc' file:'new.bin' size:62148(0xF2C4) CRC32:0x77F9E9C9
20.01 19:41:59.372 | Stop util (0 сек.)
```