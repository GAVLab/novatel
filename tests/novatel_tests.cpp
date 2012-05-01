#include <iostream>
#include "gtest/gtest.h"
#include "novatel/novatel_enums.h"
#include "novatel/novatel_structures.h"

// OMG this is so nasty...
#define private public
#define protected public


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


int main(int argc, char **argv) {
  try {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
  return 1;
}
