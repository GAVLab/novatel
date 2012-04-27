#include <string>
#include <iostream>
#include <sstream>

#include "xbow440/xbow440.h"
using namespace xbow440;
using namespace std;


void ProcessData(const ImuData& data) {
    cout << "Received data. ax: " << data.ax << " ay: " << 
        data.ay << " az: " << data.az << std::endl;

};

int main(int argc, char **argv)
{
    if(argc < 3) {
        std::cerr << "Usage: xbow440_example <serial port address> <baud rate>" << std::endl;
        return 0;
    }
    std::string port(argv[1]);
    int baudrate=38400;
    istringstream(argv[2]) >> baudrate;


    XBOW440 my_xbow;
    bool result = my_xbow.Connect(port,baudrate);
    //my_xbow.set_data_handler(&ProcessData);

    if (result) {
        cout << "Successfully connected." << endl;
    }
    else {
        cout << "Failed to connect." << endl;
        return -1;
    }

    while(1);

    my_xbow.Disconnect();

    return 0;
}
