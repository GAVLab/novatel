

#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>

#include "novatel/novatel.h"
#include "novatel/novatel_structures.h"


void RangeMeasurementsHandler(novatel::RangeMeasurements &range,
                              double &timestamp)
{
  std::cout << "[" << std::setprecision(2) << std::fixed << timestamp
            << std::setprecision(std::cout.precision())
            <<  "] RANGE: "
            << range.header.gps_week << ":" << range.header.gps_millisecs
            << "\tNumber of observations: "
            << range.number_of_observations
            << std::endl;

  for (size_t kk = 0; kk < range.number_of_observations; ++kk)
  {
/*    std::cout << "\tPRN: " << range.range_data[kk].satellite_prn <<
        "\tC/N0: " << range.range_data[kk].carrier_to_noise <<
        "\tPSR: " << range.range_data[kk].pseudorange <<
        "\tDop: " << range.range_data[kk].doppler << std::endl;*/

/*    std::cout << "\tPRN: " << range.range_data[kk].satellite_prn <<
        "\tC/N0: " << range.range_data[kk].carrier_to_noise <<
        "\tADR " << range.range_data[kk].accumulated_doppler << std::endl;*/

    std::cout << "PRN: " << range.range_data[kk].satellite_prn
              << "\tSat: " << range.range_data[kk].channel_status.satellite_sys
              << "\tSig: " << range.range_data[kk].channel_status.signal_type
              << std::endl;
  }

};

void CompressedRangeMeasurementsHandler(
    novatel::CompressedRangeMeasurements &range,
    double &timestamp)
{

  std::cout << "[" << std::setprecision(2) << std::fixed << timestamp
            << std::setprecision(std::cout.precision())
            <<  "] RANGECMP: "
            << range.header.gps_week << ":" << range.header.gps_millisecs
            << "\tNumber of observations: "
            << range.number_of_observations
            << std::endl;

  for (size_t kk = 0; kk < range.number_of_observations; ++kk)
  {
    std::cout << "\tPRN: " << range.range_data[kk].range_record.satellite_prn
              << "\tC/N0: "
              << range.range_data[kk].range_record.carrier_to_noise
              << "\tPSR: " << range.range_data[kk].range_record.pseudorange
              << std::endl;
  }

};

void PositionEcefHandler(novatel::PositionEcef &pos, double &timestamp)
{
  std::cout << "[" << std::setprecision(2) << std::fixed << timestamp
            << std::setprecision(std::cout.precision())
            << "] BESTXYZ: "
            << pos.header.gps_week << ":" << pos.header.gps_millisecs
            << "   Type: " << pos.position_type
            << std::endl;

  std::cout << "\tPos: (" << pos.x_position << ","
                          << pos.y_position << ","
                          << pos.z_position << ")" << std::endl;
};


int main(int argc, char* argv[])
{

  if (argc != 2)
  {
    std::cout << "Incorrect number of input arguments" << std::endl;
    return -1;
	}

  // Load the input filename
  std::ifstream file1(argv[1], std::ios::binary);

  if (!file1.is_open())
  {
    std::cout << "File " << argv[1] << " did not open." << std::endl;
    return 1;
  }

  // Find file size
  file1.seekg(0, file1.end);
  size_t size = file1.tellg();

  // Put data into buffer
  unsigned char buff[size];

  file1.seekg (0, file1.beg);
  file1.read ((char*)buff, size);

  // Finished with the file, close it
  file1.close();

  // Create Novatel class
  novatel::Novatel rx1;

  // Set callbacks
  rx1.set_range_measurements_callback(&RangeMeasurementsHandler);
  //rx1.set_compressed_range_measurements_callback(&CompressedRangeMeasurementsHandler);
  //rx1.set_best_position_ecef_callback(&PositionEcefHandler);

  // Pass the binary data to the parser
  rx1.ReadFromFile(buff, sizeof(buff));

  // Alternatively, pass in single bytes
  //for (unsigned int kk=0; kk<sizeof(buff); ++kk)
  //{
  //  rx1.ReadFromFile((&buff[kk]), 1);
  //}

}
