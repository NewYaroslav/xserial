/*
* xserial - library for working with serial port.
*
* Copyright (c) 2018 Elektro Yar. Email: git.electroyar@gmail.com
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

/**@file
@brief Заголовочный файл c классом для работы с COM портом
*/

#ifndef COM_PORT_HPP_INCLUDED
#define COM_PORT_HPP_INCLUDED

#if defined(__MINGW32__) || defined(_WIN32)
#include <windows.h>
#endif

#include <string>
#include <vector>
#include <sstream>

//#define LINUX_SPECIFY_NAME_COM_PORT "ttyUSB"
//#define LINUX_SPECIFY_NAME_COM_PORT "ttyACM"

namespace xserial {

    /**@brief Класс для работы с COM портом.
        @version 1.0
        @date Февраль 2017 года
        @bug Под Windows могут возникнуть проблемы при записи в COM порт,
        если в качестве переходника USB-UART используется плата Nucleo STM32
        Данный класс позволяет работать с COM портом в синхронном режиме.
        @warning Данный класс не поддерживает асинхронную работу с портом.
        Пример использования
        @code
        // в Linux устройства могут называться по разному
        // если вам нужно по умолчанию подключаться не к ttyUSBх (где
        // x - номер), вы можете объявить имя устройства в .h .hpp файлах
        // например #define LINUX_SPECIFY_NAME_COM_PORT "ttyACM"


        xserial::ComPort com(0, 115200); // открыть порт COM0 со скоростью 115200
        com.print("Test Com Port\n"); // отправить в порт сообщение Test Com Port
        int k = com.bytesToRead(); // получить количество доступых для считывания байт
        cout << com.getLine() << endl; // считать из порта строку до символа переноса строки
        com << 0.001; // отправить в порт число 0.001
        com.close(); // закрыть порт
        com.open(); // открыть порт с настройками по умолчанию
        @endcode
    */
    class ComPort {
    public:
        /// Варианты проверки четности
        enum eParity {
            COM_PORT_EVENPARITY, ///< Проверка по четности
            COM_PORT_MARKPARITY, ///< Проверка четности по метке
            COM_PORT_NOPARITY, ///< Без проверки четности
            COM_PORT_ODDPARITY, ///< Проверка по нечетности
            COM_PORT_SPACEPARITY ///< Проверка четности по паузе
        };
        /// Настройки стоповых битов
        enum eStopBit {
            COM_PORT_ONESTOPBIT, ///< 1 стоповый бит
            COM_PORT_ONE5STOPBITS, ///< 1.5 стоповых бита
            COM_PORT_TWOSTOPBITS ///< 2 стоповых бита
        };
        /// Настройка режима работы с COM портом
        enum eMode {
            COM_SYNCHRONOUS, ///< синхронный режим
            COM_ASYNCHRONOUS, ///< асинхронный режим, не используется в текущей версии
        };

        const unsigned long defaultBaudRate = 9600; ///< Скорость UART по умолчанию
        const eParity defaultParity = COM_PORT_NOPARITY; ///< Настройка проверки четности по умолчанию
        const eStopBit defaultStopBit = COM_PORT_ONESTOPBIT; ///< Настройка стопового бита по умолчанию
        const char defaultDataBits = 8; ///< Количесвто бит данных по умолчанию
        const eMode defaultMode = COM_SYNCHRONOUS; ///< Настройка режима работы с портом по умолчанию
    private:
        #if defined(__MINGW32__) || defined(_WIN32)
        HANDLE hComPort = NULL;
        DCB	dcbComPort;
        COMMTIMEOUTS commTimeoutsComPort;
        COMMPROP commPropComPort;
        #endif
        #ifdef __linux
        int hComPort = 0;
            #ifndef LINUX_SPECIFY_NAME_COM_PORT
            std::string comPortName = "ttyUSB";
            #else
            std::string comPortName = LINUX_SPECIFY_NAME_COM_PORT;
            #endif
        #endif
        bool isOpenPort = false;
        unsigned char autoFoundComPort = 0;
        unsigned short numOpenComPort;
        bool openPort(unsigned short numComPort, unsigned long baudRate, eParity parity, char dataBits, eStopBit stopBits, eMode comPortMode);
        bool foundComPort(void);
    public:

        /**@brief Инициализация порта с настройками по умолчанию
        При инициализации класса будет октрыт первый из доступых портов
        со скоростью 9600, с 1-м стоп битом, без проверки четности и длиной
        данных 8 бит.
        */
        ComPort();

        /**@brief Инициализация порта с настройкой номера порта
        При инициализации класса будет октрыт указанный порт
        со скоростью 9600, с 1-м стоп битом, без проверки четности и длиной
        даных 8 бит.
        @param[in] numComPort номер порта
        */
        ComPort(unsigned short numComPort);

        /**@brief Инициализация порта с настройкой номера порта и его скорости
        При инициализации класса будет октрыт указанный порт с указанной скоростью,
        с 1-м стоп битом, без проверки четности и длиной данных 8 бит.
        @param[in] numComPort номер порта
        @param[in] baudRate скорость
        */
        ComPort(unsigned short numComPort, unsigned long baudRate);

        /**@brief Инициализация порта с настройкой всех параметров
        При инициализации класса будет октрыт указанный порт с настройкой всех параметров
        @param[in] numComPort номер порта
        @param[in] baudRate скорость
        @param[in] parity настройка проверки четности
        @param[in] dataBits количесвто бит данных
        @param[in] stopBits настройка количесвта стоп битов
        */
        ComPort(unsigned short numComPort, unsigned long baudRate, eParity parity, char dataBits, eStopBit stopBits);

        /**@brief Инициализация первого доступного порта с настройкой всех параметров
        При инициализации класса будет октрыт первый доступный порт с настройкой всех параметров
        @param[in] baudRate скорость
        @param[in] parity настройка проверки четности
        @param[in] dataBits количесвто бит данных
        @param[in] stopBits настройка количесвта стоп битов
        */
        ComPort(unsigned long baudRate, eParity parity, char dataBits, eStopBit stopBits);

        ~ComPort();

        /**@brief Открыть порт с настройками по умолчанию
        Функция открывает первый из доступых портов
        со скоростью 9600, с 1-м стоп битом, без проверки четности и длиной
        даных 8 бит.
        @return true в случе успешного выполнения
        */
        bool open(void);

        /**@brief Открыть указанный порт
        Функция откроет указанный порт
        со скоростью 9600, с 1-м стоп битом, без проверки четности и длиной
        даных 8 бит.
        @param[in] numComPort номер порта
        @return true в случе успешного выполнения и false в случае провала
        */
        bool open(unsigned short numComPort);

        /**@brief Открыть указанный порт с настройкой скорости
        Функция отктроет указанный порт с указанной скоростью,
        с 1-м стоп битом, без проверки четности и длиной данных 8 бит.
        @param[in] numComPort номер порта
        @param[in] baudRate скорость
        @return true в случе успешного выполнения и false в случае провала
        */
        bool open(unsigned short numComPort, unsigned long baudRate);

        /**@brief Открыть указанный порт с настройкой всех параметров
        Функция откроет указанный порт с настройкой всех параметров
        @param[in] numComPort номер порта
        @param[in] baudRate скорость
        @param[in] parity настройка проверки четности
        @param[in] dataBits количество бит данных
        @param[in] stopBits настройка количества стоп битов
        @return true в случе успешного выполнения и false в случае провала
        */
        bool open(unsigned short numComPort, unsigned long baudRate, eParity parity, char dataBits, eStopBit stopBits);

        /**@brief Открыть первый доступный порт с настройкой всех параметров
        Функция откроет первый доступынй порт с настройкой всех параметров
        @param[in] baudRate скорость
        @param[in] parity настройка проверки четности
        @param[in] dataBits количество бит данных
        @param[in] stopBits настройка количества стоп битов
        @return true в случе успешного выполнения и false в случае провала
        */
        bool open(unsigned long baudRate, eParity parity, char dataBits, eStopBit stopBits);

        /**@brief Записать в порт данные
        Функция отправляет в порт массив данных типа char
        @param[in] data указатель на массив
        @param[in] len длина массива
        @return true в случе успешного выполнения и false в случае провала
        */
        bool write(char* data, unsigned long len);

        /**@brief Читать из порта данные
        Функция считывает из порта массив данных типа char
        @param[out] data указатель на массив
        @param[in] maxNumBytesRead длина считываемого слова
        @return количество считанных байт
        */
        unsigned long read(char* data, unsigned long maxNumBytesRead);

        /**@brief Количество байт для чтения
        Функция возвращает количесвто байт в буфере для чтения
        @return количество байт для чтения
        */
        unsigned long bytesToRead(void);

        /**@brief Получить один байт
        Функция считывает из порта один байт
        @return считанный байт
        */
        char readByte(void);

        /**@brief Закрыть порт
        Функция закрывает порт
        */
        void close(void);

        /**@brief Получить из порта строку до символа окончания строки
        Функция считывает из порта массив данных типа string
        до символа окончания строки '\n'.
        @return считанная строка
        */
        std::string getLine(void);

        /**@brief Получить из порта слово
        Функция считывает из порта массив данных типа string
        до символа пробела или окончания строки '\n'.
        @return считанная строка
        */
        std::string getWord(void);

        /**@brief Записать в порт строку
        Функция отправляет в порт массив данных типа std::string
        @param[in] text указатель на std::string
        @return true в случе успешного выполнения и false в случае провала
        */
        bool print(std::string* text);

        /**@brief Записать в порт строку
        Функция отправляет в порт массив данных типа char до символа '\0'
        @param[in] text указатель на char
        @return true в случе успешного выполнения и false в случае провала
        */
        bool print(char* text);

        /**@brief Очистить буфер приема данных
        Функция очищает буфер
        */
        void flushRx(void);

        /**@brief Очистить буфер отправки данных
        Функция очищает буфер
        */
        void flushTx(void);

        /**@brief Очистить буфер для приема и отправки данных
        Функция очищает буфер
        */
        void flushRxAndTx(void);

        bool operator << (char data);
        bool operator >> (char& data);
        bool operator << (std::string data);
        bool operator >> (std::string& data);
        bool operator << (float data);
        bool operator << (double data);
        bool operator << (int data);
        bool operator << (std::stringstream& stream);

        /**@brief Получить номер открытого порта
        Функция возвращает номер открытого порта
        @return номер порта
        */
        unsigned short getNumComPort(void);

        /**@brief Получить состояние порта
        Функция возвращает true если порт открыт и false если иначе.
        @return состояние порта
        */
        bool getStateComPort(void);

        /**@brief Показать список доступых портов
        Функция печатает в терминале список доступых портов.
        */
        void printListSerialPorts(void);

        /**@brief Получить список доступых портов
        Функция возарщает список доступых портов.
        @param[out] serial ссылка на std::vector<std::string>
        */
        void getListSerialPorts(std::vector<std::string>& serial);

    };
}
#endif // COM_PORT_HPP_INCLUDED
