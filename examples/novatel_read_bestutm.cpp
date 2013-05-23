#include <string>
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
#include <signal.h>

#include "novatel/novatel.h"
using namespace novatel;
using namespace std;




void BestUtmHandler(UtmPosition &pos, double &timestamp) {
    std::cout << "[" << setprecision(2) << std::fixed << timestamp << 
            setprecision(std::cout.precision()) <<  "] BestUtm: " << 
            pos.header.gps_week << ":" << pos.header.gps_millisecs << 
            "   Type: " << pos.position_type << " Pos: ("<<pos.easting<<
            ","<<pos.northing<<")" << std::endl;
}

void BestVelHandler(Velocity &vel, double &timestamp) {
    std::cout << "[" << setprecision(2) << std::fixed << timestamp << 
        setprecision(std::cout.precision()) <<  "] BestVel: " << 
        vel.header.gps_week << ":" << vel.header.gps_millisecs << 
        "   Horiz: " << vel.horizontal_speed << " Course: "<<
        vel.track_over_ground<<std::endl;
}

void sighandler(int sig) {
    std::cout << "Exiting." << std::endl;
    exit(EXIT_SUCCESS);    
}

int main(int argc, char **argv)
{
    // catch ctrl-c
    signal(SIGINT, &sighandler);

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

    my_gps.set_best_utm_position_callback(&BestUtmHandler);
    my_gps.set_best_velocity_callback(&BestVelHandler);
    my_gps.ConfigureLogs("BESTUTMB ONTIME 0.1;BESTVELB ONTIME 0.1");

    while(1) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
    }

    my_gps.Disconnect();

    return 0;
}
