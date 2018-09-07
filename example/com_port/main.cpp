#include <iostream>
#include "xserial.hpp"
using namespace std;

int main() {
    cout << "Hello world!" << endl;
    // инициализируем доступный COM порт, без проверки бита четности, с 8-мью битами данных и одним стоп битом.
    xserial::ComPort com(115200,com.COM_PORT_NOPARITY, 8, com.COM_PORT_ONESTOPBIT);

    if (!com.getStateComPort()) { // Если порт не открылся
        cout << "Error: com port is not open!" << endl;
        return 0;
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

    return 0;
}
