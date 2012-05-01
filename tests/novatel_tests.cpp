#include <iostream>
#include "gtest/gtest.h"
#include "novatel/novatel_enums.h"
#include "novatel/novatel_structures.h"

// OMG this is so nasty...
#define private public
#define protected public


TEST(StructureSizeTest, Headers) {
	ASSERT_EQ(HEADER_SIZE, sizeof(Oem4BinaryHeader));
	ASSERT_EQ(12, sizeof(OEM4ShortBinaryHeader));
	//ASSERT_EQ(sizeof(OEM4_Binary_Header), 28);
}


TEST(StructureSizeTest, SubStructures) {
	//ASSERT_EQ(sizeof(OEM4_Binary_Header), 28);
	//ASSERT_EQ(sizeof(OEM4_Binary_Header_Short), 12);
	//ASSERT_EQ(sizeof(OEM4_Binary_Header), 28);
}

TEST(StructureSizeTest, MessageStructures) {
	ASSERT_EQ(HEADER_SIZE+76, sizeof(BestPosition));
	ASSERT_EQ(HEADER_SIZE+84, sizeof(BestUtmPosition));

}

//TEST(StructureSizeTest, Enums) {
//	std::cout << sizeof(TIME_STATUS) << std::endl;
//	ASSERT_EQ(1,sizeof(TIME_STATUS));
//}


int main(int argc, char **argv) {
  try {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
  return 1;
}
