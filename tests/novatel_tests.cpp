#include <iostream>
#include <fstream>
// #include <ifstream>
#include "gtest/gtest.h"
#include "novatel/novatel_enums.h"
#include "novatel/novatel_structures.h"
// #include <string_utils/string_utils.h>

// OMG this is so nasty...
#define private public
#define protected public

#include "novatel/novatel.h"
using namespace novatel;

extern void Tokenize(const std::string&, std::vector<std::string>&, const std::string&);

// // stolen from: http://oopweb.com/CPP/Documents/CPPHOWTO/Volume/C++Programming-HOWTO-7.html
// void Tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " ") {
//     // Skip delimiters at beginning.
//     std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
//     // Find first "non-delimiter".
//     std::string::size_type pos     = str.find_first_of(delimiters, lastPos);

//     while (std::string::npos != pos || std::string::npos != lastPos)
//     {
//         // Found a token, add it to the vector.
//         tokens.push_back(str.substr(lastPos, pos - lastPos));
//         // Skip delimiters.  Note the "not_of"
//         lastPos = str.find_first_not_of(delimiters, pos);
//         // Find next "non-delimiter"
//         pos = str.find_first_of(delimiters, lastPos);
//     }
// }

TEST(StructureSizeTest, Headers) {
	ASSERT_EQ(HEADER_SIZE, sizeof(Oem4BinaryHeader));
	ASSERT_EQ(SHORT_HEADER_SIZE, sizeof(OEM4ShortBinaryHeader));
}

TEST(StructureSizeTest, IMUMessageStructures) {
	ASSERT_EQ(HEADER_SIZE+92, sizeof(InsPositionVelocityAttitude));
	ASSERT_EQ(SHORT_HEADER_SIZE+92, sizeof(InsPositionVelocityAttitudeShort));
	ASSERT_EQ(HEADER_SIZE+52, sizeof(VehicleBodyRotation));
	ASSERT_EQ(HEADER_SIZE+44, sizeof(InsSpeed));
	ASSERT_EQ(HEADER_SIZE+44, sizeof(RawImu));
	ASSERT_EQ(SHORT_HEADER_SIZE+44, sizeof(RawImuShort));
	ASSERT_EQ(HEADER_SIZE+56, sizeof(BestLeverArm));
	ASSERT_EQ(HEADER_SIZE+232, sizeof(InsCovariance));
	ASSERT_EQ(SHORT_HEADER_SIZE+232, sizeof(InsCovarianceShort));
}

TEST(StructureSizeTest, GPSMessageStructures) {
    ASSERT_EQ(HEADER_SIZE+76, sizeof(Position));
    ASSERT_EQ(HEADER_SIZE+116, sizeof(PositionEcef));
    ASSERT_EQ(HEADER_SIZE+48, sizeof(Velocity));
    ASSERT_EQ(HEADER_SIZE+32+4*MAX_CHAN, sizeof(Dop));
    ASSERT_EQ(HEADER_SIZE+84, sizeof(UtmPosition));
    ASSERT_EQ(HEADER_SIZE+60, sizeof(BaselineEcef));
    ASSERT_EQ(HEADER_SIZE+112, sizeof(IonosphericModel));
    ASSERT_EQ(44, sizeof(RangeData));
    ASSERT_EQ(HEADER_SIZE+8+44*MAX_CHAN, sizeof(RangeMeasurements));
    ASSERT_EQ(24, sizeof(CompressedRangeData));
    ASSERT_EQ(HEADER_SIZE+8+24*MAX_CHAN, sizeof(CompressedRangeMeasurements));
    ASSERT_EQ(HEADER_SIZE+228, sizeof(GpsEphemeris));
    ASSERT_EQ(68, sizeof(SatellitePositionData));
    ASSERT_EQ(HEADER_SIZE+16+68*MAX_CHAN, sizeof(SatellitePositions));
    ASSERT_EQ(HEADER_SIZE+48, sizeof(TimeOffset));
    ASSERT_EQ(HEADER_SIZE+44, sizeof(ReceiverHardwareStatus));

}

TEST(DataParsing, Oem4SpanVersion) {
	// load data file and pass through parse methods
	std::ifstream test_datafile;
	test_datafile.open("./"
			"test_data/ascii_version_test_data_oem4_span.log");

	if (test_datafile.is_open()) {
		// read data from the file and pass to the novatel parse methods
		std::string file_contents((std::istreambuf_iterator<char>(test_datafile)),
				std::istreambuf_iterator<char>());

		Novatel my_gps;

		std::vector<std::string> packets;

		Tokenize(file_contents, packets, "\n");

		// loop through all packets in file and check for version messages
		// stop when the first is found or all packets are read
		for (size_t ii=0; ii<packets.size(); ii++) {
			if (my_gps.ParseVersion(packets[ii]))
				break;
		}

		// check values
		ASSERT_TRUE(my_gps.span_capable_);
		ASSERT_TRUE(my_gps.rtk_capable_);
		ASSERT_FALSE(my_gps.glonass_capable_);
		ASSERT_TRUE(my_gps.l2_capable_);
		ASSERT_TRUE(my_gps.raw_capable_);

	} else {
		// fail the test if the file can't be opened
		std::cout << "Test file could not be opened." << std::endl;
		ASSERT_TRUE(false);
	}
}

//TEST(DataParsing, Oem5Dlv3Version) {
//	// load data file and pass through parse methods
//	std::ifstream test_datafile;
//	test_datafile.open("/home/hododav/Development/sensors/novatel/tests/"
//			"test_data/ascii_version_test_data_oem5_dlv3.log");

//	if (test_datafile.is_open()) {
//		// read data from the file and pass to the novatel parse methods
//		std::string file_contents((std::istreambuf_iterator<char>(test_datafile)),
//				std::istreambuf_iterator<char>());

//		Novatel my_gps;

//		std::vector<std::string> packets;

//		string_utils::Tokenize(file_contents, packets, "\n");

//		// loop through all packets in file and check for version messages
//		// stop when the first is found or all packets are read
//		for (size_t ii=0; ii<packets.size(); ii++) {
//			if (my_gps.ParseVersion(packets[ii]))
//				break;
//		}

//		// check values
//		ASSERT_FALSE(my_gps.span_capable_);
//		ASSERT_TRUE(my_gps.rtk_capable_);
//		ASSERT_FALSE(my_gps.glonass_capable_);
//		ASSERT_TRUE(my_gps.l2_capable_);
//		ASSERT_TRUE(my_gps.raw_capable_);

//	} else {
//		// fail the test if the file can't be opened
//		std::cout << "Test file could not be opened." << std::endl;
//		ASSERT_TRUE(false);
//	}
//}

TEST(DataParsing, Oem5PropakGlonassVersion) {
    // load data file and pass through parse methods
    std::ifstream test_datafile;
    test_datafile.open("./"
            "test_data/ascii_version_test_data_oem5_propak3_glonass.log");

    if (test_datafile.is_open()) {
        // read data from the file and pass to the novatel parse methods
        std::string file_contents((std::istreambuf_iterator<char>(test_datafile)),
                std::istreambuf_iterator<char>());

        Novatel my_gps;

        std::vector<std::string> packets;

        Tokenize(file_contents, packets, "\n");

        // loop through all packets in file and check for version messages
        // stop when the first is found or all packets are read
        for (size_t ii=0; ii<packets.size(); ii++) {
            if (my_gps.ParseVersion(packets[ii]))
                break;
        }

        // check values
        ASSERT_FALSE(my_gps.span_capable_);
        ASSERT_TRUE(my_gps.rtk_capable_);
        ASSERT_TRUE(my_gps.glonass_capable_);
        ASSERT_TRUE(my_gps.l2_capable_);
        ASSERT_TRUE(my_gps.raw_capable_);

    } else {
        // fail the test if the file can't be opened
        std::cout << "Test file could not be opened." << std::endl;
        ASSERT_TRUE(false);
    }
}

TEST(DataParsing, BinaryDataSet1) {
    // load data file and pass through parse methods
    std::ifstream test_datafile;
    test_datafile.open("./"
            "test_data/OneEach.GPS",std::ios::in|std::ios::binary);

    if (test_datafile.is_open()) {
        // read data from the file and pass to the novatel parse methods
        Novatel my_gps;
        char *file_data = new char[1000];
        while (!test_datafile.eof())
        {
            test_datafile.read(file_data, sizeof(file_data));
            my_gps.BufferIncomingData((unsigned char*)file_data,test_datafile.gcount());
        }

    } else {
        // fail the test if the file can't be opened
        std::cout << "Test file could not be opened." << std::endl;
        ASSERT_TRUE(false);
    }
}


int main(int argc, char **argv) {
  try {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
  return 1;
}
