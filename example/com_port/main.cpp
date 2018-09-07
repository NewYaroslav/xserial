#include <iostream>
#include "xserial.hpp"
using namespace std;

int main() {
    // инициализируем доступный COM порт, без проверки бита четности, с 8-мью битами данных и одним стоп битом.
    const int baudRate = 115200; // скорость порта
    const int dataBits = 8; // длина данных
    xserial::ComPort serial(baudRate, xserial::ComPort::COM_PORT_NOPARITY, dataBits, xserial::ComPort::COM_PORT_ONESTOPBIT);

    if (!serial.getStateComPort()) { // Если порт не открылся
        cout << "Error: com port is not open!" << endl;
        return 0;
    }

    // выводим список доступных портов
    serial.printListSerialPorts();

    // получаем текст до символа \n
    cout << "Test getLine()..." << endl;
    serial << "Test 1\n";
    cout << serial.getLine() << endl;

    // проверяем функцию проверки количества принятых байт
    cout << "Test bytesToRead()..." << endl;
    serial.print("Test 2\n");
    int k = serial.bytesToRead();
    cout << "bytes to read = " << k << endl;
    while(k < 6) {
        k = serial.bytesToRead();
    }
    cout << "bytes to read = " << k << endl;

    // проверяем функцию чтения
    char data[512];
    cout << "Test read()..." << endl;
    serial.read(data, 7);
    cout << data << endl;

    // проверяем функцию чтения слова
    serial.print("Bla Bla Bla\n");
    cout << "Test getWord(), print Bla Bla Bla" << endl;
    cout << "Word 1: " << serial.getWord() << endl;
    cout << "Word 2: " << serial.getWord() << endl;
    cout << "Word 3: " << serial.getWord() << endl;

    return 0;
}
