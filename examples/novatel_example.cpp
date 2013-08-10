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

    my_gps.SetSvElevationAngleCutoff(5);
//    my_gps.PDPFilterDisable();
    my_gps.PDPFilterEnable();
//    my_gps.PDPFilterReset();
//    my_gps.PDPModeConfigure(NORMAL,AUTO);
    my_gps.SetCarrierSmoothing(9,9);

    //    my_gps.SendCommand("PASSTOPASSMODE ENABLE ON OFF");
    /*
    my_gps.HardwareReset(0);
    my_gps.HotStartReset();
    my_gps.WarmStartReset();
    my_gps.ColdStartReset();

    my_gps.SaveConfiguration();
    my_gps.Unlog("BESTUTMB");
    my_gps.UnlogAll();
    */

//    my_gps.SetInitialPosition();
//    my_gps.SetInitialTime();







    //while(1);

    my_gps.Disconnect();
    std::cout << "Disconnected." << endl;

    return 0;
}
