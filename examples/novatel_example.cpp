#include <string>
#include <iostream>
#include <sstream>

#include "novatel/novatel.h"
using namespace novatel;
using namespace std;


//void ProcessData(const ImuData& data) {
//    cout << "Received data. ax: " << data.ax << " ay: " <<
//        data.ay << " az: " << data.az << std::endl;
//
//};

int main(int argc, char **argv)
{
    if(argc < 3) {
        std::cerr << "Usage: novatel_example <serial port address> <baud rate>" << std::endl;
        return 0;
    }
    std::string port(argv[1]);
    int baudrate=115200;
    istringstream(argv[2]) >> baudrate;


    Novatel my_gps;
    bool result = my_gps.Connect(port,baudrate);

    if (result) {
        cout << "Successfully connected." << endl;
    }
    else {
        cout << "Failed to connect." << endl;
        return 0;
    }

    my_gps.ConfigureLogs("BESTPOSB ONTIME 1.0; GPSEPHEMB ONCE");



    while(1);

//    my_gps.Disconnect();

    return 0;
}
