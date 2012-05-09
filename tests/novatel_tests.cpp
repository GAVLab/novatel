#include <iostream>
#include <fstream>
#include "gtest/gtest.h"
#include "novatel/novatel_enums.h"
#include "novatel/novatel_structures.h"

// OMG this is so nasty...
#define private public
#define protected public

#include "novatel/novatel.h"
using namespace novatel;

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
	ASSERT_EQ(HEADER_SIZE+76, sizeof(BestGpsPosition));
}

TEST(StructureSizeTest, GPSMessageStructures) {
	ASSERT_EQ(HEADER_SIZE+76, sizeof(BestPosition));
	ASSERT_EQ(HEADER_SIZE+84, sizeof(BestUtmPosition));
	ASSERT_EQ(HEADER_SIZE+48, sizeof(BestVelocity));
	ASSERT_EQ(HEADER_SIZE+116, sizeof(PseudorangePositionECEF));
	ASSERT_EQ(HEADER_SIZE+60, sizeof(BaselineECEF));
	ASSERT_EQ(HEADER_SIZE+116, sizeof(BestPositionECEF));
}

TEST(StructureSizeTest, StatusStructures) {
	ASSERT_EQ(HEADER_SIZE+112, sizeof(Version));
}

TEST(DataParsing, AsciiData) {
	// load data file and pass through parse methods
	std::ifstream test_datafile;
	test_datafile.open("/home/hododav/Development/sensors/novatel/tests/ascii_version_test_data.log");

	if (test_datafile.is_open()) {
		// read data from the file and pass to the novatel parse methods
		std::string file_contents((std::istreambuf_iterator<char>(test_datafile)),
				std::istreambuf_iterator<char>());

		Novatel my_gps;
		std::string packet, pre, post;
		bool result = my_gps.TokenizeAscii(file_contents, packet, pre, post);
		file_contents=post;
		// loop through all packets in file and check for version messages
		// stop when the first is found or all packets are read
		bool version_found=false;
		while ((!version_found) && (post!="")) {
			version_found = my_gps.ParseVersion(packet);
			result = my_gps.TokenizeAscii(file_contents, packet, pre, post);

			file_contents=post;
		}

		if (version_found) {
			// check values
			ASSERT_TRUE(my_gps.span_capable_);
			ASSERT_TRUE(my_gps.rtk_capable_);
			ASSERT_FALSE(my_gps.glonass_capable_);
			ASSERT_TRUE(my_gps.l2_capable_);
			ASSERT_TRUE(my_gps.raw_capable_);
		} else {
			ASSERT_TRUE(false);
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
