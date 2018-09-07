## xserial
С++ библиотека для работы с Com Port на Lunix и Windows

## Как подключить?
Подключите в своей проект всего лишь два файла: xserial.cpp и xserial.hpp

## Описание
Данная C++ библиотека позволят быстро начать использовать Com Port как под Windows, так и под Linux. 
Библиотека раcсчитана на использование компиляторов GCC и MinGW и определяет используемую операционную систему
по директивам определения компилятора и ОС (__MINGW32__ и __linux). В win32 для работы с портом используются средства WinAPI. 
В Linux системах для работы с устройствами используются специальные файлы.


## Пример использования библиотеки
```
// В файле main.cpp

#include <iostream>
#include "xserial.hpp"
using namespace std;

//...

// инициализируем доступный COM порт, без проверки бита четности, с 8-мью битами данных и одним стоп битом.
xserial::ComPort com(115200,com.COM_PORT_NOPARITY, 8, com.COM_PORT_ONESTOPBIT);

if (!com.getStateComPort()) { // Если порт не открылся
    cout << "Error: com port is not open!" << endl;
    return 0; // закрываем программу
}

// выводим список доступых портов
com.printListSerialPorts();

// получаем текст до символа \n
cout << "Test getLine()..." << endl;
com << "Test 1\n";
cout << com.getLine() << endl;

// проверяем функцию проверки количества принятых байт
cout << "Test bytesToRead()..." << endl;
com.print("Test 2\n");
int k = com.bytesToRead();
cout << "bytes to read = " << k << endl;
while(k < 6) {
    k = com.bytesToRead();
}
cout << "bytes to read = " << k << endl;

// проверяем функцию чтения
char data[512];
cout << "Test read()..." << endl;
com.read(data, 7);
cout << data << endl;

// проверяем функцию чтения слова
com.print("Bla Bla Bla\n");
cout << "Test getWord(), print Bla Bla Bla" << endl;
cout << "Word 1: " << com.getWord() << endl;
cout << "Word 2: " << com.getWord() << endl;
cout << "Word 3: " << com.getWord() << endl;

```
## Функции класса ComPort:
+ Открыть порт
```
xserial::ComPort com; // Инициализация порта с настройками по умолчанию

/* настройки по умолчанию
 скорость 9600
 1 стоп бит, 
 без проверки четности 
 длина даных 8 бит */

com.open(); // открыть порт с настройками по умолчанию (функция открывает первый из доступых портов)

/* ... */

const int numComPort = 1; // номер порта
com.open(numComPort); // открыть указанный порт
xserial::ComPort com1(numComPort);

/* ... */

const int baudRate = 9600; // скорость порта
com.open(numComPort, baudRate); // открыть указанный порт с настройкой скорости
xserial::ComPort com1(numComPort, baudRate);

/* ... */

const int dataBits = 8; // количество бит данных
// открыть порт с настройкой всех параметров
com.open(numComPort, baudRate, xserial::ComPort::COM_PORT_NOPARITY, dataBits, xserial::ComPort::COM_PORT_ONESTOPBIT);
xserial::ComPort com1(numComPort, baudRate, xserial::ComPort::COM_PORT_NOPARITY, dataBits, xserial::ComPort::COM_PORT_ONESTOPBIT);

/* ... */

// открыть первый доступный порт с настройкой всех параметров
com.open(baudRate, xserial::ComPort::COM_PORT_NOPARITY, dataBits, xserial::ComPort::COM_PORT_ONESTOPBIT);
xserial::ComPort com1(baudRate, xserial::ComPort::COM_PORT_NOPARITY, dataBits, xserial::ComPort::COM_PORT_ONESTOPBIT);

```
+ Записать в порт данные
```
xserial::ComPort com;

char data[32] = "test\n";
const int lenData = 5;

// Функция отправляет в порт массив данных типа char
if (com.write(data, lenData)) {
  printf("OK\n");
}
// Функция отправляет в порт массив данных типа char до символа '\0'
com.print(data);

std::string testText = "Hello!"
// Функция отправляет в порт массив данных типа std::string
com.print(testText);
// Второй вариант 
com << testText;

// Отправить число в порт
double num = 42;
com << num;
```
+ Читать из порта данные
```
xserial::ComPort com;

// Получить один байт
char byte = com.readByte();

char data[32];
const int dataLen = 32;
// Функция считывает из порта массив данных типа char
int len = com.read(data, dataLen);
// В len количество считанных байт

// Получить из порта строку до символа окончания строки
std::string text = com.getLine();

//  Получить из порта слово
std::string word = com.getWord();

// Еще вариант
std::string txtData;
com >> txtData;

```
+ Получить количество байт для чтения
```
// Получить количество байт для чтения
int lenRead = com.bytesToRead();
```
+ Получить номер открытого порта
```
// Получить номер открытого порта
int numCom = com.getNumComPort();
```
+ Получить состояние порта
```
if (com.getStateComPort()) {
  // если порт открыт
}

```
+ Показать список доступых портов
```
// Функция печатает в терминале список доступых портов.
com.printListSerialPorts();
//  Функция возарщает список доступых портов.
std::vector<std::string> listSerials;
com.getListSerialPorts(listSerials);
```
