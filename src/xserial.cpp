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

#include "xserial.hpp"

#if defined(__MINGW32__) || defined(_WIN32)
#include <windows.h>
#include <iostream>
#endif
#ifdef __linux
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <list>
#include <stdlib.h>
#include <dirent.h>
#include <stdio.h>
#include <linux/serial.h>
#endif
#include <stdio.h>
#include <string>
#include <sstream>
#include <ctime>

// время ожидания получения символов до символа '\n'
#define COM_PORT_GETLINE_MAX_TIME 1000

#ifdef __linux
// получить список COM портов
std::list<std::string> getComList(void);
// вспомогательные функции
static void probe_serial8250_comports(std::list<std::string>& comList, std::list<std::string> comList8250);
static void register_comport(std::list<std::string>& comList, std::list<std::string>& comList8250, const std::string& dir);
static std::string get_driver(const std::string& tty);
#endif

namespace xserial {

    bool ComPort::openPort(unsigned short numComPort, unsigned long baudRate, eParity parity, char dataBits, eStopBit stopBits, eMode comPortMode) {
        if (isOpenPort) {
            close();
            isOpenPort = false;
        }
        numOpenComPort = numComPort;
        #if defined(__MINGW32__) || defined(_WIN32)
        std::string comPortName = "\\\\.\\COM";
        ZeroMemory(&dcbComPort,sizeof(DCB));
        //char dcbBuffer[256];
        //sprintf(dcbBuffer,"baud=%d parity=N data=8 stop=1", baudRate);
        if (numComPort == 0) {
            printf("Error opening port: numComPort != COM0\r\n");
            isOpenPort = false;
            return false;
        }
        if (dataBits < 5 || dataBits > 8) {
            printf("Error opening port: The number of data bits should be from 5 to 8 bits.\r\n");
            isOpenPort = false;
            return false;
        }
        comPortName += std::to_string(numComPort);
        //printf("num = %d\n",numComPort);
        hComPort = CreateFile(comPortName.c_str(),GENERIC_READ | GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);
        if (hComPort == INVALID_HANDLE_VALUE) {
            printf("Error opening port\r\n");
            isOpenPort = false;
            return false;
        }
        if(!GetCommState(hComPort, &dcbComPort)) {
            printf("Error opening port: GetCommState\r\n");
            isOpenPort = false;
            return false;
        };
        //  устанавливает параметры структуры DCB, которую потом можно передать в порт
        //if(!BuildCommDCB((char*)&dcbBuffer,&dcbComPort)) {
         //   printf("Error opening port: BuildCommDCB\r\n");
        //    isOpenPort = false;
        //	return false;
        //};
        // memset(&dcbComPort,0,sizeof(dcbComPort));
        dcbComPort.DCBlength = sizeof(DCB);
        GetCommState(hComPort, &dcbComPort);
        dcbComPort.fBinary = true;
        dcbComPort.BaudRate = DWORD(baudRate);
        dcbComPort.ByteSize = dataBits;
        switch (parity) {
            case COM_PORT_EVENPARITY:
                dcbComPort.Parity = EVENPARITY;
            break;
            case COM_PORT_MARKPARITY:
                dcbComPort.Parity = MARKPARITY;
            break;
            case COM_PORT_NOPARITY:
                dcbComPort.Parity = NOPARITY;
            break;
            case COM_PORT_ODDPARITY:
                dcbComPort.Parity = ODDPARITY;
            break;
            case COM_PORT_SPACEPARITY:
                dcbComPort.Parity = SPACEPARITY;
            break;
        }
        switch (stopBits) {
            case COM_PORT_ONESTOPBIT:
                dcbComPort.StopBits = ONESTOPBIT;
            break;
            case COM_PORT_ONE5STOPBITS:
                if (dataBits >= 6) {
                    printf("Error opening port: Using five data bits with 2 stop bits - invalid combination,\r\n");
                    printf("as well as 6, 7 or 8 data bits with 1.5 stop bits.\r\n");
                    isOpenPort = false;
                    return false;
                }
                dcbComPort.StopBits = ONE5STOPBITS;
            break;
            case COM_PORT_TWOSTOPBITS:
                if (dataBits == 5) {
                    printf("Error opening port: The number of data bits should be from 5 to 8 bits.\r\n");
                    isOpenPort = false;
                    return false;
                }
                dcbComPort.StopBits = TWOSTOPBITS;
            break;
        }

        //dcbComPort.Parity = NOPARITY;
        //dcbComPort.StopBits = ONESTOPBIT;
        dcbComPort.fAbortOnError = FALSE; //fAbortOnError
        dcbComPort.wReserved = 0;
        dcbComPort.fDtrControl = DTR_CONTROL_DISABLE;
        dcbComPort.fRtsControl = RTS_CONTROL_DISABLE;
        dcbComPort.fBinary = TRUE;
        dcbComPort.fParity = FALSE;
        dcbComPort.fInX = FALSE;
        dcbComPort.fOutX = FALSE;
        dcbComPort.XonChar = 0x11;
        dcbComPort.XoffChar = (unsigned char)0x13;
        dcbComPort.ErrorChar = 0;
        dcbComPort.fErrorChar = FALSE;
        dcbComPort.fNull = FALSE;
        dcbComPort.fOutxCtsFlow = FALSE;
        dcbComPort.fOutxDsrFlow = FALSE;
        dcbComPort.XonLim = 128;
        dcbComPort.XoffLim = 128;

        if(!SetCommState(hComPort, &dcbComPort)) {
            printf("Error opening port: SetCommState\r\n");
            isOpenPort = false;
            return false;
        };
        if(!GetCommTimeouts(hComPort, &commTimeoutsComPort)) {
            printf("Error opening port: GetCommTimeouts\r\n");
            isOpenPort = false;
            return false;
        };
        commTimeoutsComPort.ReadIntervalTimeout          = 100; // milliseconds
        commTimeoutsComPort.ReadTotalTimeoutMultiplier   = 0;  //
        commTimeoutsComPort.ReadTotalTimeoutConstant     = 0;  //
        commTimeoutsComPort.WriteTotalTimeoutMultiplier  = 0;  //
        commTimeoutsComPort.WriteTotalTimeoutConstant    = 0;  //
        if(!SetCommTimeouts(hComPort, &commTimeoutsComPort)) {
            printf("Error opening port: SetCommTimeouts\r\n");
            isOpenPort = false;
            return false;
        };
        isOpenPort = true;
        return true;
        #endif

        #ifdef __linux
        std::string _comPortName("/dev/");
        _comPortName += comPortName;
        _comPortName += std::to_string(numComPort);
        //::printf(_comPortName.c_str());
        if (dataBits < 5 || dataBits > 8) {
            printf("Error opening port: The number of data bits should be from 5 to 8 bits.\r\n");
            isOpenPort = false;
            return false;
        }
        hComPort = ::open(const_cast<char*>(_comPortName.c_str()), O_RDWR | O_NOCTTY );
        if (hComPort <0) {
            printf("Error opening port\n");
            isOpenPort = false;
            return false;
        } else {
            isOpenPort = true;
        }
        // Configure Port
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if ( tcgetattr (hComPort, &tty) != 0 ) {
            printf("Error opening port\n");
            isOpenPort = false;
            return false;
        }
        // Set Baud Rate
        switch (baudRate) {
            case 0:
                cfsetospeed (&tty, B0);
                cfsetispeed (&tty, B0);
            break;
            case 50:
                cfsetospeed (&tty, B50);
                cfsetispeed (&tty, B50);
            break;
            case 110:
                cfsetospeed (&tty, B110);
                cfsetispeed (&tty, B110);
            break;
            case 134:
                cfsetospeed (&tty, B134);
                cfsetispeed (&tty, B134);
            break;
            case 150:
                cfsetospeed (&tty, B150);
                cfsetispeed (&tty, B150);
            break;
            case 200:
                cfsetospeed (&tty, B200);
                cfsetispeed (&tty, B200);
            break;
            case 300:
                cfsetospeed (&tty, B300);
                cfsetispeed (&tty, B300);
            break;
            case 600:
                cfsetospeed (&tty, B600);
                cfsetispeed (&tty, B600);
            break;
            case 1200:
                cfsetospeed (&tty, B1200);
                cfsetispeed (&tty, B1200);
            break;
            case 1800:
                cfsetospeed (&tty, B1800);
                cfsetispeed (&tty, B1800);
            break;
            case 2400:
                cfsetospeed (&tty, B2400);
                cfsetispeed (&tty, B2400);
            break;
            case 4800:
                cfsetospeed (&tty, B4800);
                cfsetispeed (&tty, B4800);
            break;
            case 9600:
                cfsetospeed (&tty, B9600);
                cfsetispeed (&tty, B9600);
            break;
            case 19200:
                cfsetospeed (&tty, B19200);
                cfsetispeed (&tty, B19200);
            break;
            case 38400:
                cfsetospeed (&tty, B38400);
                cfsetispeed (&tty, B38400);
            break;
            case 57600:
                cfsetospeed (&tty, B57600);
                cfsetispeed (&tty, B57600);
            break;
            case 115200:
                cfsetospeed (&tty, B115200);
                cfsetispeed (&tty, B115200);
            break;
            case 230400:
                cfsetospeed (&tty, B230400);
                cfsetispeed (&tty, B230400);
            break;
            case  460800:
                cfsetospeed (&tty, B460800);
                cfsetispeed (&tty, B460800);
            break;
            case  500000:
                cfsetospeed (&tty, B500000);
                cfsetispeed (&tty, B500000);
            break;
            case  576000:
                cfsetospeed (&tty, B576000);
                cfsetispeed (&tty, B576000);
            break;
            case  921600:
                cfsetospeed (&tty, B921600);
                cfsetispeed (&tty, B921600);
            break;
            case 1000000:
                cfsetospeed (&tty, B1000000);
                cfsetispeed (&tty, B1000000);
            break;
            case 1152000:
                cfsetospeed (&tty, B1152000);
                cfsetispeed (&tty, B1152000);
            break;
            case 1500000:
                cfsetospeed (&tty, B1500000);
                cfsetispeed (&tty, B1500000);
            break;
            case 2000000:
                cfsetospeed (&tty, B2000000);
                cfsetispeed (&tty, B2000000);
            break;
            case 2500000:
                cfsetospeed (&tty, B2500000);
                cfsetispeed (&tty, B2500000);
            break;
            case 3000000:
                cfsetospeed (&tty, B3000000);
                cfsetispeed (&tty, B3000000);
            break;
            case 3500000:
                cfsetospeed (&tty, B3500000);
                cfsetispeed (&tty, B3500000);
            break;
            case 4000000:
                cfsetospeed (&tty, B4000000);
                cfsetispeed (&tty, B4000000);
            break;
            default:
                printf("Error opening port: invalid baudrate! non-standard value baudrate.\n");
                isOpenPort = false;
                return false;
            break;
        }
        cfsetospeed (&tty, baudRate);
        cfsetispeed (&tty, baudRate);
        // Setting other Port Stuff
        switch (parity) {
            case COM_PORT_EVENPARITY:
                tty.c_cflag |= PARENB;
            break;
            case COM_PORT_MARKPARITY:
                printf("Error opening port: parity != COM_PORT_MARKPARITY.\r\n");
                isOpenPort = false;
                return false;
            break;
            case COM_PORT_NOPARITY:
                tty.c_cflag &= ~PARENB;
            break;
            case COM_PORT_ODDPARITY:
                tty.c_cflag |= PARODD;
                tty.c_cflag &= ~PARENB;
            break;
            case COM_PORT_SPACEPARITY:
                printf("Error opening port: parity != COM_PORT_SPACEPARITY.\r\n");
                isOpenPort = false;
                return false;
            break;
        }
        switch (stopBits) {
            case COM_PORT_ONESTOPBIT:
                tty.c_cflag &= ~CSTOPB;
            break;
            case COM_PORT_ONE5STOPBITS:
                if (dataBits >= 6) {
                    printf("Error opening port: stopBits != COM_PORT_ONE5STOPBITS.\r\n");
                    isOpenPort = false;
                    return 0;
                }
            break;
            case COM_PORT_TWOSTOPBITS:
                tty.c_cflag |= CSTOPB;
            break;
        }
        tty.c_cflag     &=  ~CSTOPB;

        tty.c_cflag     &=  ~CSIZE;
        switch (dataBits) {
            case 5:
                tty.c_cflag     |=  CS5;
            break;
            case 6:
                tty.c_cflag     |=  CS6;
            break;
            case 7:
                tty.c_cflag     |=  CS7;
            break;
            case 8:
                tty.c_cflag     |=  CS8;
            break;
            default:
                printf("Error opening port: invalid number of data-bits '%d'\n", dataBits);
            return false;
            break;
        }
        tty.c_cflag     &=  ~CRTSCTS;       // no flow control
        tty.c_lflag     =   0;          // no signaling chars, no echo, no canonical processing
        tty.c_oflag     =   0;                  // no remapping, no delays
        tty.c_cc[VMIN]      =   0;                  // read doesn't block
        tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout

        tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
        tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
        tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
        tty.c_oflag     &=  ~OPOST;              // make raw
        // Flush Port, then applies attributes
        tcflush(hComPort, TCIFLUSH);

        if (tcsetattr (hComPort, TCSANOW, &tty) != 0) {
            printf("Error opening port\n");
            isOpenPort = false;
            return false;
        }
        isOpenPort = true;
        return true;
        #endif
        return false;
    }

    bool ComPort::foundComPort(void) {
        bool isFound = 0;
        #if defined(__MINGW32__) || defined(_WIN32)
        char physical[65536]; // имена устройств
        // получим список устройств
        QueryDosDevice(NULL, physical, sizeof(physical));
        unsigned long i = 0;
        while(1) {
            char* nameDevice = &physical[i]; // текущее имя устройства
            int nLen = strlen(nameDevice); // длина имени устройства
            char* istr = strstr (nameDevice,"COM"); // ищем COM
            if (nLen > 3 && istr != NULL) {
                isFound = true;
                //printf(nameDevice);
                //printf("\n");
                if (autoFoundComPort == 0) {;
                    nameDevice+=3;
                    autoFoundComPort = atoi (nameDevice);
                    return isFound;
                }
            }
            // Переходим к следующему символу терминатору
            while(physical[i] != '\0')
                i++;
            // Перескакиваем на следующую строку
            i++;
            // Список завершается двойным симмволом терминатором, так что если символ
            // NULL, мы дошли до конца
            if(physical[i] == '\0') {
                break;
            }
        }
        return isFound;
        #endif

        #ifdef __linux
        std::list<std::string> l = getComList();
        std::list<std::string>::iterator it = l.begin();
        while (it != l.end()) {
            std::string name = *it;
            #ifndef LINUX_SPECIFY_NAME_COM_PORT
                int pos = name.find_first_of("0123456789");
                int pos0 = name.find_first_of("t");
                isFound = true;
                comPortName = name.substr(pos0, pos - pos0);
                std::string num = name.substr(pos);
                autoFoundComPort = atoi(const_cast<char*>(num.c_str()));
                break;
            #else
            int posName = name.find(LINUX_SPECIFY_NAME_COM_PORT);
            if (posName != std::string::npos) {
                int pos = name.find_first_of("0123456789");
                isFound = true;
                comPortName = LINUX_SPECIFY_NAME_COM_PORT;
                std::string num = name.substr(pos);
                autoFoundComPort = atoi(num.c_str());
                break;
            }
            #endif
            it++;
        }
        return isFound;
        #endif
    }

    ComPort::ComPort() {
        // найдем COM порт (первый в списке)
        if (foundComPort()) {
            openPort(autoFoundComPort, defaultBaudRate, defaultParity, defaultDataBits, defaultStopBit, defaultMode);
        } else {
            printf("Error: port is not found!\r\n");
        }
    }

    bool ComPort::open(void) {
        close();
        // найдем COM порт (первый в списке)
        if (foundComPort()) {
            return openPort(autoFoundComPort, defaultBaudRate, defaultParity, defaultDataBits, defaultStopBit, defaultMode);
        } else {
            printf("Error: port is not found!\r\n");
            return false;
        }
    }

    ComPort::ComPort(unsigned short numComPort) {
        openPort(numComPort, defaultBaudRate, defaultParity, defaultDataBits, defaultStopBit, defaultMode);
    }

    bool ComPort::open(unsigned short numComPort) {
        close();
        return openPort(numComPort, defaultBaudRate, defaultParity, defaultDataBits, defaultStopBit, defaultMode);
    }

    ComPort::ComPort(unsigned short numComPort, unsigned long baudRate) {
        openPort(numComPort, baudRate, defaultParity, defaultDataBits, defaultStopBit, defaultMode);
    }

    bool ComPort::open(unsigned short numComPort, unsigned long baudRate) {
        close();
        return openPort(numComPort, baudRate, defaultParity, defaultDataBits, defaultStopBit, defaultMode);
    }

    ComPort::ComPort(unsigned short numComPort, unsigned long baudRate, eParity parity, char dataBits, eStopBit stopBits) {
        openPort(numComPort, baudRate, parity, dataBits, stopBits, defaultMode);
    }

    bool ComPort::open(unsigned short numComPort, unsigned long baudRate, eParity parity, char dataBits, eStopBit stopBits) {
        close();
        return openPort(numComPort, baudRate, parity, dataBits, stopBits, defaultMode);
    }

    ComPort::ComPort(unsigned long baudRate, eParity parity, char dataBits, eStopBit stopBits) {
        if (foundComPort()) {
            openPort(autoFoundComPort, baudRate, parity, dataBits, stopBits, defaultMode);
        } else {
            printf("Error: port is not found!\r\n");
        }
    }

    bool ComPort::open(unsigned long baudRate, eParity parity, char dataBits, eStopBit stopBits) {
        close();
        if (foundComPort()) {
            return openPort(autoFoundComPort, baudRate, parity, dataBits, stopBits, defaultMode);
        } else {
            printf("Error: port is not found!\r\n");
            return false;
        }
        return false;
    }

    ComPort::~ComPort() {
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            CloseHandle(hComPort);
            #endif
            #ifdef __linux
            ::close(hComPort);
            ::flock(hComPort, LOCK_UN);  // free the port so that others can use it.
            #endif
        }
    }

    bool ComPort::write(char* data, unsigned long len) {
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            DWORD dwBytesWrite = len; // кол-во записанных байтов
            if(!WriteFile(hComPort, data, len, &dwBytesWrite, NULL)){
                printf("write error\r\n");
                return false;
            }
            if(!FlushFileBuffers(hComPort)) {
                printf("write error: FlushFileBuffers\r\n");
                return false;
            };
            return true;
            #endif
            #ifdef __linux
            int iOut = ::write(hComPort, data, len);
            if (iOut < 0){
                printf("write error\n");
                return false;
            } else
                return true;
            #endif
        } else {
            #if defined(__MINGW32__) || defined(_WIN32)
            printf("com port is not open!\r\n");
            return false;
            #endif
            #ifdef __linux
            printf("com port is not open!\n");
            return false;
            #endif
        }
    }

    unsigned long ComPort::read(char* data, unsigned long maxNumBytesRead) {
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            DWORD dwBytesRead = 0;
            DWORD numRedByte, temp;
            COMSTAT comstat;
            ClearCommError(hComPort, &temp, &comstat); // заполнить структуру COMSTAT
            numRedByte = comstat.cbInQue; //получить количество принятых байтов
            if (numRedByte > 0) {
                if(!ReadFile(hComPort, data, maxNumBytesRead, &dwBytesRead, NULL)){
                    printf("read error\r\n");
                    return 0;
                } else {
                    return dwBytesRead;
                }
            } else {
                return 0;
            }
            #endif
            #ifdef __linux
            int iOut = ::read(hComPort, data, maxNumBytesRead);
            if (iOut < 0){
                printf("read error\n");
                return 0;
            } else {
                return iOut;
            }
            #endif
        } else {
            #if defined(__MINGW32__) || defined(_WIN32)
            printf("com port is not open!\r\n");
            return 0;
            #endif
            #ifdef __linux
            printf("com port is not open!\n");
            return 0;
            #endif
        }
    }

    unsigned long ComPort::bytesToRead(void) {
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            unsigned long numRedByte, temp; // temp - заглушка
            COMSTAT comstat; // структура для получения притяных байтов
            ClearCommError(hComPort, &temp, &comstat); // заполнить структуру COMSTAT
            numRedByte = comstat.cbInQue; //получить количество принятых байтов
            return numRedByte;
            #endif
            #ifdef __linux
            // количество байтов в буфере приема
            int bytes;
            ioctl(hComPort, FIONREAD, &bytes);
            return bytes;
            #endif
        } else {
            #if defined(__MINGW32__) || defined(_WIN32)
            printf("com port is not open!\r\n");
            return 0;
            #endif
            #ifdef __linux
            printf("com port is not open!\n");
            return 0;
            #endif
        }
    }

    char ComPort::readByte(void) {
        char data = 0;
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            DWORD dwBytesRead; // считанные байты
            DWORD numRedByte, temp; // temp - заглушка
            COMSTAT comstat; // структура для получения притяных байтов
            ClearCommError(hComPort, &temp, &comstat); // заполнить структуру COMSTAT
            numRedByte = comstat.cbInQue; //получить количество принятых байтов
            // будем проверять наличие принятого байта, пока он не появится
            while(numRedByte == 0) {
                ClearCommError(hComPort, &temp, &comstat); // заполнить структуру COMSTAT
                numRedByte = comstat.cbInQue; //получить количество принятых байтов
            }
            if(!ReadFile(hComPort, &data, 1, &dwBytesRead, NULL)){
                printf("read error\r\n");
                return '\0';
            }
            if (dwBytesRead > 0) {
                return data;
            } else
                return '\0';
            #endif
            #ifdef __linux

            // количество байтов в буфере приема
            int bytesAvaiable;
            ioctl(hComPort, FIONREAD, &bytesAvaiable);
            // будем проверять наличие принятого байта, пока он не появится
            while(bytesAvaiable == 0) {
                ioctl(hComPort, FIONREAD, &bytesAvaiable);
            }
            int iOut = ::read(hComPort, &data, bytesAvaiable);
            if (iOut < 0){
                printf("read error\n");
                return 0;
            }
            if (iOut > 0) {
                return data;
            } else
                return '\0';
            #endif
        } else {
            #if defined(__MINGW32__) || defined(_WIN32)
            printf("com port is not open!\r\n");
            return 0;
            #endif
            #ifdef __linux
            printf("com port is not open!\n");
            return 0;
            #endif
        }
        return 0;
    }

    void ComPort::close(void) {
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            CloseHandle(hComPort);
            #endif
            #ifdef __linux
            ::close(hComPort);
            ::flock(hComPort, LOCK_UN);  // free the port so that others can use it.
            #endif
            isOpenPort = false;
        }
    }

    std::string ComPort::getLine(void) {
        char data;
        std::string strLine = "";
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            DWORD dwBytesRead; // считанные байты
            DWORD numRedByte, temp; // temp - заглушка
            COMSTAT comstat; // структура для получения притяных байтов
            while(1) {
                ClearCommError(hComPort, &temp, &comstat); // заполнить структуру COMSTAT
                numRedByte = comstat.cbInQue; //получить количество принятых байтов
                // будем проверять наличие принятого байта, пока он не появится
                while(numRedByte == 0) {
                    ClearCommError(hComPort, &temp, &comstat); // заполнить структуру COMSTAT
                    numRedByte = comstat.cbInQue; //получить количество принятых байтов
                }
                // считаем 1 байт
                if(!ReadFile(hComPort, &data, 1, &dwBytesRead, NULL)){
                    printf("read error\r\n");
                    return strLine;
                }
                if (dwBytesRead > 0) {
                    // если был получен символ завершения строки
                    if (data == '\n')
                        break;
                    // иначе увеличим строку
                    strLine = strLine + data;
                }
            }
            return strLine;
            #endif
            #ifdef __linux
            while(1) {
                // количество байтов в буфере приема
                int bytesAvaiable;
                ioctl(hComPort, FIONREAD, &bytesAvaiable);
                // будем проверять наличие принятого байта, пока он не появится
                while(bytesAvaiable == 0) {
                    ioctl(hComPort, FIONREAD, &bytesAvaiable);
                }
                // считаем 1 байт
                int iOut = ::read(hComPort, &data, 1);
                if (iOut < 0){
                    printf("read error\n");
                    return strLine;
                }
                if (iOut > 0) {
                    // если был получен символ завершения строки
                    if (data == '\n')
                        break;
                    // иначе увеличим строку
                    strLine = strLine + data;
                }
            }
            return strLine;
            #endif
        } else {
            #if defined(__MINGW32__) || defined(_WIN32)
            printf("com port is not open!\r\n");
            return strLine;
            #endif
            #ifdef __linux
            printf("com port is not open!\n");
            return strLine;
            #endif
        }
        return strLine;
    }

    std::string ComPort::getWord(void) {
        char data;
        bool isStart = false;
        std::string strLine = "";
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            DWORD dwBytesRead; // считанные байты
            DWORD numRedByte, temp; // temp - заглушка
            COMSTAT comstat; // структура для получения притяных байтов
            while(1) {
                ClearCommError(hComPort, &temp, &comstat); // заполнить структуру COMSTAT
                numRedByte = comstat.cbInQue; //получить количество принятых байтов
                // будем проверять наличие принятого байта, пока он не появится
                while(numRedByte == 0) {
                    ClearCommError(hComPort, &temp, &comstat); // заполнить структуру COMSTAT
                    numRedByte = comstat.cbInQue; //получить количество принятых байтов
                }
                // считаем 1 байт
                if(!ReadFile(hComPort, &data, 1, &dwBytesRead, NULL)){
                    printf("read error\r\n");
                    return strLine;
                }
                if (dwBytesRead > 0) {
                    if (isStart == false && data != ' ') {
                        isStart = true; // установим флаг, что слово началось
                    }
                    // если был получен символ пробела
                    if (isStart && (data == ' ' || data == '\0' || data == '\n'))
                        break;
                    if (isStart) {
                        // иначе увеличим строку
                        strLine = strLine + data;
                    }
                }
            }
            return strLine;
            #endif
            #ifdef __linux
            while(1) {
                // количество байтов в буфере приема
                int bytesAvaiable;
                ioctl(hComPort, FIONREAD, &bytesAvaiable);
                // будем проверять наличие принятого байта, пока он не появится
                while(bytesAvaiable == 0) {
                    ioctl(hComPort, FIONREAD, &bytesAvaiable);
                }
                // считаем 1 байт
                int iOut = ::read(hComPort, &data, 1);
                if (iOut < 0){
                    printf("read error\n");
                    return strLine;
                }
                if (iOut > 0) {
                    if (isStart == false && data != ' ') {
                        isStart = true; // установим флаг, что слово началось
                    }
                    // если был получен символ пробела
                    if (isStart && (data == ' ' || data == '\0' || data == '\n'))
                        break;
                    if (isStart) {
                        // иначе увеличим строку
                        strLine = strLine + data;
                    }
                }
            }
            return strLine;
            #endif
        } else {
            #if defined(__MINGW32__) || defined(_WIN32)
            printf("com port is not open!\r\n");
            return "";
            #endif
            #ifdef __linux
            printf("com port is not open!\n");
            return "";
            #endif
        }
    }

    bool ComPort::print(std::string* text) {
        return write(const_cast<char*>(text->c_str()), text->size());
    }

    bool ComPort::print(char* text) {
        return write(text, strlen(text));
    }

    void ComPort::flushRx(void) {
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            PurgeComm(hComPort, PURGE_RXCLEAR | PURGE_RXABORT);
            #endif
        }
    }

    void ComPort::flushTx(void) {
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            PurgeComm(hComPort, PURGE_TXCLEAR | PURGE_TXABORT);
            #endif
        }
    }

    void ComPort::flushRxAndTx(void) {
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            PurgeComm(hComPort, PURGE_RXCLEAR | PURGE_RXABORT);
            PurgeComm(hComPort, PURGE_TXCLEAR | PURGE_TXABORT);
            #endif
            #ifdef __linux

            #endif
        }
    }

    bool ComPort::operator << (char data) {
        if (isOpenPort) {
            #if defined(__MINGW32__) || defined(_WIN32)
            DWORD dwBytesWrite = 0; // кол-во записанных байтов
            if(!WriteFile(hComPort, &data, 1, &dwBytesWrite, NULL)){
                printf("write error\r\n");
                return false;
            }
            if(!FlushFileBuffers(hComPort)) {
                printf("write error: FlushFileBuffers\r\n");
                return false;
            };
            return true;
            #endif
            #ifdef __linux
            int iOut = ::write(hComPort, &data, 1);
            if (iOut < 0){
                printf("write error\n");
                return false;
            } else
                return true;
            #endif
        } else {
            #if defined(__MINGW32__) || defined(_WIN32)
            printf("com port is not open!\r\n");
            return false;
            #endif
            #ifdef __linux
            printf("com port is not open!\n");
            return false;
            #endif
        }
        return true;
    }

    bool ComPort::operator >> (char& data) {
        data = readByte();
        return true;
    }

    bool ComPort::operator << (std::string data) {
        return write(const_cast<char*>(data.c_str()),data.size());
    }

    bool ComPort::operator >> (std::string& data) {
        data = getLine();
        return true;
    }

    bool ComPort::operator << (float data) {
        std::string numStr = std::to_string(data);
        return write(const_cast<char*>(numStr.c_str()),numStr.size());
    }

    bool ComPort::operator << (double data) {
        std::string numStr = std::to_string(data);
        return write(const_cast<char*>(numStr.c_str()),numStr.size());
    }

    bool ComPort::operator << (int data) {
        std::string numStr = std::to_string(data);
        return write(const_cast<char*>(numStr.c_str()),numStr.size());
    }

    bool ComPort::operator << (std::stringstream& stream) {
        std::string str(stream.str());
        //std::getline(stream, str);
        return write(const_cast<char*>(str.c_str()),str.size());
    }

     unsigned short ComPort::getNumComPort(void) {
        return numOpenComPort;
     }

     bool ComPort::getStateComPort(void) {
        return isOpenPort;
     }

     void ComPort::printListSerialPorts(void) {
        #ifdef __linux
        std::list<std::string> l = getComList();
        std::list<std::string>::iterator it = l.begin();
        while (it != l.end()) {
            //std::cout << *it << std::endl;
            std::string text = *it;
            text += "\n";
            ::printf(text.c_str());
            it++;
        }
        #endif
        #if defined(__MINGW32__) || defined(_WIN32)
        char physical[65536]; // имена устройств
        // получим список устройств
        QueryDosDevice(NULL, physical, sizeof(physical));
        unsigned long i = 0;
        while(1) {
            char* nameDevice = &physical[i]; // текущее имя устройства
            int nLen = strlen(nameDevice); // длина имени устройства
            char* istr = strstr (nameDevice,"COM"); // ищем COM
            if (nLen > 3 && istr != NULL) {
                ::printf(nameDevice);
                ::printf("\n");
            }
            // Переходим к следующему символу терминатору
            while(physical[i] != '\0')
                i++;
            // Перескакиваем на следующую строку
            i++;
            // Список завершается двойным симмволом терминатором, так что если символ
            // NULL, мы дошли до конца
            if(physical[i] == '\0') {
                break;
            }
        }
        #endif
     }

     void ComPort::getListSerialPorts(std::vector<std::string>& serial) {
        #ifdef __linux
        std::list<std::string> l = ::getComList();
        std::list<std::string>::iterator it = l.begin();
        serial.resize(0);
        serial.clear();
        while (it != l.end()) {
            //std::cout << *it << std::endl;
            std::string text = *it;
            serial.push_back(text);
            it++;
        }
        #endif
        #if defined(__MINGW32__) || defined(_WIN32)
        char physical[65536]; // имена устройств
        // получим список устройств
        QueryDosDevice(NULL, physical, sizeof(physical));
        unsigned long i = 0;
        serial.resize(0);
        serial.clear();
        while(1) {
            char* nameDevice = &physical[i]; // текущее имя устройства
            int nLen = strlen(nameDevice); // длина имени устройства
            char* istr = strstr (nameDevice,"COM"); // ищем COM
            if (nLen > 3 && istr != NULL) {
                serial.push_back(nameDevice);
            }
            // Переходим к следующему символу терминатору
            while(physical[i] != '\0')
                i++;
            // Перескакиваем на следующую строку
            i++;
            // Список завершается двойным симмволом терминатором, так что если символ
            // NULL, мы дошли до конца
            if(physical[i] == '\0') {
                break;
            }
        }
        #endif
     }

} // end namespace xserial


#ifdef __linux
    static std::string get_driver(const std::string& tty) {
        struct stat st;
        std::string devicedir = tty;

        // Append '/device' to the tty-path
        devicedir += "/device";

        // Stat the devicedir and handle it if it is a symlink
        if (lstat(devicedir.c_str(), &st)==0 && S_ISLNK(st.st_mode)) {
            char buffer[1024];
            memset(buffer, 0, sizeof(buffer));

            // Append '/driver' and return basename of the target
            devicedir += "/driver";

            if (readlink(devicedir.c_str(), buffer, sizeof(buffer)) > 0)
                return basename(buffer);
        }
        return "";
    }

    static void register_comport(std::list<std::string>& comList, std::list<std::string>& comList8250, const std::string& dir) {
        // Get the driver the device is using
        std::string driver = get_driver(dir);

        // Skip devices without a driver
        if (driver.size() > 0) {
            std::string devfile = std::string("/dev/") + basename(dir.c_str());

            // Put serial8250-devices in a seperate list
            if (driver == "serial8250") {
                comList8250.push_back(devfile);
            } else
                comList.push_back(devfile);
        }
    }

    static void probe_serial8250_comports(std::list<std::string>& comList, std::list<std::string> comList8250) {
        struct serial_struct serinfo;
        std::list<std::string>::iterator it = comList8250.begin();

        // Iterate over all serial8250-devices
        while (it != comList8250.end()) {

            // Try to open the device
            int fd = open((*it).c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);

            if (fd >= 0) {
                // Get serial_info
                if (ioctl(fd, TIOCGSERIAL, &serinfo)==0) {
                    // If device type is no PORT_UNKNOWN we accept the port
                    if (serinfo.type != PORT_UNKNOWN)
                        comList.push_back(*it);
                }
                close(fd);
            }
            it ++;
        }
    }

    std::list<std::string> getComList(void) {
        int n;
        struct dirent **namelist;
        std::list<std::string> comList;
        std::list<std::string> comList8250;
        const char* sysdir = "/sys/class/tty/";
        // Scan through /sys/class/tty - it contains all tty-devices in the system
        n = scandir(sysdir, &namelist, NULL, NULL);
        if (n < 0)
            perror("scandir");
        else {
            while (n--) {
                if (strcmp(namelist[n]->d_name,"..") && strcmp(namelist[n]->d_name,".")) {

                    // Construct full absolute file path
                    std::string devicedir = sysdir;
                    devicedir += namelist[n]->d_name;

                    // Register the device
                    register_comport(comList, comList8250, devicedir);
                }
                free(namelist[n]);
            }
            free(namelist);
        }
        // Only non-serial8250 has been added to comList without any further testing
        // serial8250-devices must be probe to check for validity
        probe_serial8250_comports(comList, comList8250);
        // Return the lsit of detected comports
        return comList;
    }

#endif
