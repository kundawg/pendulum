#include "../include/SerialCommunicator.h"

#include <iostream>

//#include <cstdio>
//#include <windows.h>
//#define _USE_MATH_DEFINES // for C++
//#include <cmath>

#include <thread>

#define M_PI 3.14159265358979323846f

#define LOOPRATE_MS 2
#define POWER_OFF 32767
#define BACKWARD_MAX 0
#define FORWARD_MAX USHRT_MAX
#define MESSAGE_MAX 65535.0f
#define RAD_S_TO_RPM 9.549296596f
#define RPM_TO_RAD_S 0.104719755f
#define RADS_MAX RPM_MAX*RPM_TO_RAD_S

//using std::string;
//using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

/* Constructor */
SerialCommunicator::SerialCommunicator():
        port("/dev/ttyACM0"),
        baud(115200),
        my_serial(port, baud, serial::Timeout::simpleTimeout(1))
{
    //my_serial.setTimeout(serial::Timeout::max(), 1, 0, 1, 0);

    my_serial.flush(); //flush the serial line

    //	Sleep(100);
    cout << "Is the serial port open?";
    if (my_serial.isOpen())
        cout << " Yes." << endl;
    else
        cout << " No." << endl;

    // setup the message by setting last byte to be endmessage
    bytesToBeSent[MESSAGESIZE_WRITE - 1] = endMessage;

    sentNumber.unsignedShort = 0;
    //sentNumberLast = sentNumber;
    writeCount = 0;
    //int writeCountOld = writeCount;
    bytes_read = 0;
    readCount = 0;
    //durationReadThreadSleepUS = 500;
}

/* Destructor */
SerialCommunicator::~SerialCommunicator()
{

}


void SerialCommunicator::sendMotorTorque(float torque)
{
    uint16_t message = this->convertTorqueToMessage(torque);
    this->sendMessage(message);
}

void SerialCommunicator::sendMessage(uint16_t message)
{
    sentNumber.unsignedShort = message;


    std::memcpy(bytesToBeSent, sentNumber.binary, SHORTSIZE);
    bytes_wrote = my_serial.write(bytesToBeSent, MESSAGESIZE_WRITE);
    //printf("bytes_wrote = %f\n", bytes_wrote*1.0f);
    printf("Sent Number = %d\n", sentNumber.unsignedShort);

    writeCount += 1;
    /*
    if (writeCount % LOOPCOUNTS_INT == 0)
    {
        // cout << "Writ Iter: " << writeCount << ", Len: " << bytes_wrote << ", Val: " << sentNumber.floatingPoint << " BIN: " << (int)sentNumber.binary[0] << " " << (int)sentNumber.binary[1] << " " << (int)sentNumber.binary[2] << " " << (int)sentNumber.binary[3] << endl;
        printf("Delta: %d, lastSent: %d, received: %d \n", sentNumberLast.unsignedShort - receivedNumber.unsignedShort, sentNumberLast.unsignedShort, receivedNumber.unsignedShort);
        // writeCountOld = writeCount;
    }

    if (writeCount % LOOPCOUNTS_INT == 1)
    {
        // cout << "Read Iter: " << readCount << ", Len: " << bytes_read << ", Val: " << receivedNumber.floatingPoint << " BIN: " << (int)receivedNumber.binary[0] << " " << (int)receivedNumber.binary[1] << " " << (int)receivedNumber.binary[2] << " " << (int)receivedNumber.binary[3] << endl;
        // cout << "-----------------------------------------------------------------------------" << endl;
        //	writeCountOld = writeCount;
    }
    */
    sentNumberLast = sentNumber;

}

float SerialCommunicator::readEncoderRad()
{
    this->getNewMessage();
    return this->convertMessageToRad(receivedNumber.unsignedShort);
}

void SerialCommunicator::getNewMessage()
{
    whatIsAvailable = my_serial.available();
    //cout << "Bytes Available: " << whatIsAvailable << endl;
    if (whatIsAvailable < MESSAGESIZE)
        printf("too short message - bytes Available: %d\n", whatIsAvailable);

    if (whatIsAvailable > MESSAGESIZE - 1)
    {
        bytes_read = my_serial.read(incomingData, whatIsAvailable);
        //cout << "Bytes read: " << length << endl;
        //if (bytes_read == MESSAGESIZE && incomingData[MESSAGESIZE - 1] == endMessage)
        if (incomingData[bytes_read - 1] == endMessage)
        {
            std::memcpy(receivedNumber.binary, &(incomingData[bytes_read - 1 - SHORTSIZE]), SHORTSIZE);
            //std::memcpy(receivedNumber.binary, incomingData), SHORTSIZE);
            //std::memcpy(otherNumber.binary, &(incomingData[FLOATSIZE]), FLOATSIZE);
            readCount++;
            //if (readCount % LOOPCOUNTS_INT == 0)
            //	cout << "Read Iter: " << readCount << ", Len: " << bytes_read << ", Val: " << receivedNumber.unsignedShort << endl;
            if (whatIsAvailable > MESSAGESIZE)
                printf("long message (multiple) - bytes Available: %d\n", whatIsAvailable);
        }
        else {
            printf("long message (wrong length) - bytes Available: %d\n", whatIsAvailable);
        }
    }
}

uint16_t SerialCommunicator::convertTorqueToMessage(float torque)
{
    double desiredCurrent = torque/KTAU;

    if (desiredCurrent < -CURRENT_LIMIT)
        desiredCurrent = -CURRENT_LIMIT;
    if (desiredCurrent > CURRENT_LIMIT)
        desiredCurrent = CURRENT_LIMIT;

    printf("Tau = %4.3f\n", torque);

    return (USHRT_MAX/2.0/CURRENT_LIMIT * desiredCurrent + USHRT_MAX/2);
}

float SerialCommunicator::convertMessageToRad(uint16_t message)
{
    return MAX_REVOLUTIONS * 2 * M_PI / USHRT_MAX * message - MAX_REVOLUTIONS / 2 * 2 * M_PI;
}

void SerialCommunicator::enumerate_ports()
{
    vector<serial::PortInfo> devices_found = serial::list_ports();

    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end())
    {
        serial::PortInfo device = *iter++;

        printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
               device.hardware_id.c_str());
    }
}

void SerialCommunicator::print_usage()
{
    cerr << "Usage: test_serial {-e|<serial port address>} ";
    cerr << "<baudrate> [test string]" << endl;
}