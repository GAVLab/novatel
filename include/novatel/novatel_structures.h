/*!
 * \file novatel/novatel_structures.h
 * \author David Hodo <david.hodo@is4s.com>
 * Portions based on previous code by William Travis and Scott Martin
 * \version 1.1
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 David Hodo - Integrated Solutions for Systems
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides structure definitions for messages output by a Novatel GPS.
 *
 */


#ifndef NOVATELSTRUCTURES_H
#define NOVATELSTRUCTURES_H

#include "novatel_enums.h"

#define MAX_NOUT_SIZE      (8192)   // Maximum size of a NovAtel log buffer (ALMANACA logs are big!)
#define EPH_CHAN 33
#define NUMSAT 14
#define MAXCHAN		28  // Maximum number of signal channels
#define MAX_NUM_SAT 28	// Maximum number of satellites with information in the RTKDATA log
#define HEADER_SIZE 28 // Binary header size for OEM 4, V, and 6 receivers


// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
	#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
	#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#include <stdint.h>  // use fixed size integer types, rather than standard c++ types


//*******************************************************************************
// HEADER STRUCTURES
//*******************************************************************************

//! Header prepended to OEM4 binary messages
PACK(
struct Oem4BinaryHeader
{
   uint8_t          sync1;          //!< start of packet first byte (0xAA)
   uint8_t          sync2;          //!< start of packet second byte (0x44)
   uint8_t          sync3;          //!< start of packet third  byte (0x12)
   uint8_t          header_length; 	//!< Length of the header in bytes ( From start of packet )
   uint16_t         message_id;    	//!< Message ID number
   uint8_t          message_type;  	//!< Message type - binary, ascii, nmea, etc...
   uint8_t          port_address;  	//!< Address of the data port the log was received on
   uint16_t         message_length;	//!< Message length (Not including header or CRC)
   uint16_t         sequence;      	//!< Counts down from N-1 to 0 for multiple related logs
   uint8_t          idle;          	//!< Time the processor was idle in last sec between logs with same ID
   uint8_t          time_status;    //!< Indicates the quality of the GPS time
   uint16_t         gps_week;      	//!< GPS Week number
   uint32_t         millisecs;     	//!< Milliseconds into week
   uint32_t         status;        	//!< Receiver status word
   uint16_t         Reserved;      	//!< Reserved for internal use
   uint16_t         version;       	//!< Receiver software build number (0-65535)
});

//! Header prepended to OEM4 binary messages
PACK(
struct OEM4ShortBinaryHeader
{
   uint8_t          sync1;          //!< start of packet first byte (0xAA)
   uint8_t          sync2;          //!< start of packet second byte (0x44)
   uint8_t          sync3;          //!< start of packet third  byte (0x12)
   uint8_t          message_length; //!< Message length (Not including header or CRC)
   uint16_t         message_id;     //!< Message ID number
   uint16_t         gps_week;       //!< GPS Week number
   uint32_t         millisecs;      //!< Milliseconds into week
});

//! IMU status structure that is included in the RAWIMU message
PACK(
struct ImuStatus
{
	unsigned counter : 4;						//!< 4 byte counter
	unsigned IMUTest : 1;						//!< IMU test: Passed=0, Failed=1
	unsigned z_axis_path_length_control : 1;	//!< Z-axis path length control: Good=0, Reset=1
	unsigned y_axis_path_length_control : 1;	//!< Y-axis path length control: Good=0, Reset=1
	unsigned x_axis_path_length_control : 1;	//!< X-axis path length control: Good=0, Reset=1
	unsigned accelerometer_temperature : 8;		//!< Accelerometer temperature
	unsigned software_version : 8;				//!< IMU software version number
	unsigned reserved : 3;						//!< Reserved
	unsigned gyro_test : 1;						//!< Gyro tests: Passed=0, Failed=1
	unsigned accel_test : 1;						//!< Accelerometer tests: Passed=0, Failed=1
	unsigned other_tests : 1;					//!< Other tests: Passed=0, Failed=1
	unsigned memory_tests : 1;					//!< Memory tests: Passed=0, Failed=1
	unsigned processor_tests : 1;				//!< Processor tests: Passed=0, Failed=1
});



//*******************************************************************************
// MESSAGE STRUCTURES
//*******************************************************************************

/*!
 * BESTPOS Message Structure
 * This log contains the best available combined GPS and
 * inertial navigation system (INS - if available)
 * position (in meters) computed by the receiver. With the
 * system operating in an RTK mode, this log reflects the
 * latest low-latency solution for up to 60 seconds after
 * reception of the last base station observation. After this
 * 60 second period, the position reverts to the best solution
 * available; the degradation in accuracy is reflected in the
 * standard deviation fields.
 */
PACK(
struct BestPosition
{
	Oem4BinaryHeader header;			//!< Message header
	SolutionStatus solution_status;		//!< Solution status
	PositionType position_type;			//!< Position type
	double latitude;					//!< latitude (deg)
	double longitutude;					//!< longitude (deg)
	double height;						//!< height above mean sea level (m)
	float undulation;					//!< Undulation - the relationship between the geoid and the ellipsoid (m)
	DatumID datum_id;					//!< datum id number
	float latitude_standard_deviation;	//!< latitude standard deviation (m)
	float longitude_standard_deviation;	//!< longitude standard deviation (m)
	float height_standard_deviation;	//!< height standard deviation (m)
	int8_t base_station_id[4];			//!< base station id
	float differential_age;				//!< differential position age (sec)
	float solution_age;					//!< solution age (sec)
	uint8_t number_of_satellites;		//!< number of satellites tracked
	uint8_t number_of_satellites_in_solution;	//!< number of satellites used in solution
	uint8_t num_gps_plus_glonass_l1;	//!< number of GPS plus GLONASS L1 satellites used in solution
	uint8_t num_gps_plus_glonass_l2;	//!< number of GPS plus GLONASS L2 satellites used in solution
	uint8_t reserved;					//!< reserved
	uint8_t extended_solution_status;	//!< extended solution status - OEMV and greater only
	uint8_t reserved2; 					//!< reserved
	uint8_t signals_used_mask;			//!< signals used mask - OEMV and greater only
	uint8_t crc[4];						//!< 32-bit cyclic redundancy check (CRC)
});


/*!
 * BESTUTM Message Structure
 * his log contains the best available position
 * computed by the receiver in UTM coordinates.
 *
 * The latitude limits of the UTM System are 80°S to
 * 84°N. If your position is outside this range, the
 * BESTUTM log outputs a northing, easting and height
 * of 0.0, along with a zone letter of ‘*’and a zone
 * number of 0, so that it is obvious that the data
 * in the log is unusable.
 */
PACK(
struct BestUtmPosition
{
	Oem4BinaryHeader header;			//!< Message header
	SolutionStatus solution_status;		//!< Solution status
	PositionType position_type;			//!< Position type
	uint32_t longitude_zone_number; 	//!< longitude utm zone number
	uint32_t latitude_zone_letter; 		//!< latitude utm zone letter
	double northing;					//!< northing (m)
	double easting;						//!< easting (m)
	double height;						//!< height above mean sea level (m)
	float undulation;					//!< Undulation - the relationship between the geoid and the ellipsoid (m)
	DatumID datum_id;					//!< datum id number
	float northing_standard_deviation;	//!< northing standard deviation (m)
	float easting_standard_deviation;	//!< easting standard deviation (m)
	float height_standard_deviation;	//!< height standard deviation (m)
	int8_t base_station_id[4];			//!< base station id
	float differential_age;				//!< differential position age (sec)
	float solution_age;					//!< solution age (sec)
	uint8_t number_of_satellites;		//!< number of satellites tracked
	uint8_t number_of_satellites_in_solution;	//!< number of satellites used in solution
	uint8_t num_gps_plus_glonass_l1;	//!< number of GPS plus GLONASS L1 satellites used in solution
	uint8_t num_gps_plus_glonass_l2;	//!< number of GPS plus GLONASS L2 satellites used in solution
	uint8_t reserved;					//!< reserved
	uint8_t extended_solution_status;	//!< extended solution status - OEMV and greater only
	uint8_t reserved2; 					//!< reserved
	uint8_t signals_used_mask;			//!< signals used mask - OEMV and greater only
	uint8_t crc[4];						//!< 32-bit cyclic redundancy check (CRC)
});

struct INSPVA
{
	Oem4BinaryHeader header;
	uint32_t gpsWeek;
	double gpsSeconds;
	double latitude;
	double longitude;
	double height;
	double northVel;
	double eastVel;
	double upVel;
	double roll;
	double pitch;
	double azimuth;
	INSStatus status;
	int8_t crc[4];
};

struct INSPVAS
{
	OEM4ShortBinaryHeader header;
	uint32_t gpsWeek;
	double gpsSeconds;
	double latitude;
	double longitude;
	double height;
	double northVel;	//
	double eastVel;
	double upVel;
	double roll;		// degrees
	double pitch;		// degrees
	double azimuth;		// degrees
	INSStatus status;
	int8_t crc[4];
};


struct INSUTM
{
	Oem4BinaryHeader header;
	uint32_t gpsWeek;
	double gpsSeconds;
	uint32_t longZoneNumber;
	uint32_t latZoneLetter;
	double northing;
	double easting;
	double height;
	INSStatus status;
	int8_t crc[4];
};

struct VehicleBodyRotation
{
	double xAngle;
	double yAngle;
	double zAngle;
	double xUncertainty;
	double yUncertainty;
	double zUncertainty;
	int8_t crc[4];
};

struct BestVelocity
{
	Oem4BinaryHeader header;
	SolutionStatus solutionStatus;
	PositionType velocityType;
	float latency;		//!< measure of the latency of the velocity time tag in seconds
	float age;			//!< differential age in seconds
	double horizSpeed;	//!< horizontal speed in m/s
	double trackOverGround;	//!< course in degrees
	double verticalSpeed; //!< vertical speed in m/s
	float reserved;
	int8_t crc[4];
};

struct PsrXYZ
{
	Oem4BinaryHeader header;
	SolutionStatus solutionStatus;
	PositionType velocityType;
    double            psrX;                //x coordinate (m)
    double            psrY;                //y coordinate (m)
    double            psrZ;                //z coordinate (m)
    float            sig_psrX;            //Standard deviation of x coordinate (m)
    float            sig_psrY;            //Standard deviation of y coordinate (m)
    float            sig_psrZ;            //Standard deviation of z coordinate (m)
    SolutionStatus    velStatus;        //Velocity solution status
    PositionType    velType;        //Velocity type
    double            velX;                //Velcoity in x (m/s)
    double            velY;                //Velcoity in y (m/s)
    double            velZ;                //Velcoity in z (m/s)
    float            sig_velX;            //Standard deviation of velcoity in x (m/s)
    float            sig_velY;            //Standard deviation of velcoity in y (m/s)
    float            sig_velZ;            //Standard deviation of velcoity in z (m/s)
    int8_t            baseStationID[4];    //Base station ID
    float            latency;            //Latency in time tag (s)
    float            diff_age;            //Differential age (s)
    float            sol_age;            //Solution age (s)
    uint8_t            num_obs;            //Number of observations tracked
    uint8_t            numL1used;            //Number of GPS L1 observations used in computation
    uint8_t            reserved[6];        //Reserved
	uint8_t crc[4];
};


struct INSSPD
{
	Oem4BinaryHeader header;
	uint32_t gpsWeek;
	double gpsSeconds;
	double trackOverGround;	//!< actual direction of motion over ground (degrees)
    double horizSpeed;		//!< horizontal speed in m/s
	double verticalSpeed;	//!< vertical speed in m/s
	INSStatus status;
	int8_t crc[4];
};

struct RawIMU
{
	Oem4BinaryHeader header;
	uint32_t gpsWeek;
	double gpsSeconds;
	ImuStatus imuStatus;
	int32_t accZ;	//!< acceleration along z axis in m/s
	int32_t accNegY; //!< -*acceleration along y axis) in m/s
	int32_t accX;	//!< acceleration along x axis in m/s
	int32_t gyroZ;	//!< change in angle around z axis in radians
	int32_t gyroNegY; //!< -(change in angle around y axis) in radians
	int32_t gyroX; //!< change in angle around x axis in radians
	int8_t crc[4];
};
// scale factor for change in angle (1.0/((double)8589934592.0)) for the AG11, AG58, AG17, and AG62
// scale factor for change in velocity (acceleration)
//(0.3048/((double)134217728.0)) for the AG11 and AG58
//(0.3048/((double)67108864.0)) for the AG17 and AG62

struct RawIMUS
{
	OEM4ShortBinaryHeader header;
	uint32_t gpsWeek;
	double gpsSeconds;
	ImuStatus imuStatus;
	int32_t accZ;	//!< acceleration along z axis in m/s
	int32_t accNegY; //!< -*acceleration along y axis) in m/s
	int32_t accX;	//!< acceleration along x axis in m/s
	int32_t gyroZ;	//!< change in angle around z axis in radians
	int32_t gyroNegY; //!< -(change in angle around y axis) in radians
	int32_t gyroX; //!< change in angle around x axis in radians
	int8_t crc[4];
};

//BSLNXYZB
struct bslnxyz   //Structure for BSLNXYZ message
{
	Oem4BinaryHeader header;
	SolutionStatus solutionStatus;
	PositionType velocityType;
	double    bslnX;                //Baseline x coordinate (m)
    double    bslnY;                //Baseline y coordinate (m)
    double    bslnZ;                //Baseline z coordinate (m)
    float    sigbslnX;            //Standard deviation of baseline x coordinate (m)
    float    sigbslnY;            //Standard deviation of baseline y coordinate (m)
    float    sigbslnZ;            //Standard deviation of baseline z coordinate (m)
    int8_t    baseStationID[4];    //Base station ID
    uint8_t    num_obs;            //Number of observations tracked
    uint8_t     numL1used;            //Number of GPS L1 observations used in computation
    uint8_t     numL1;                //Number of GPS L1 observations above the RTK mask angle
    uint8_t     numL2;                //Number of GPS L2 observations above the RTK mask angle
    uint8_t     reserved[4];        //Reserved
    uint8_t     crc[4];                //32 bit CRC
};


struct ReceiverError
{
	 int32_t DRAMStatus :1;
	int32_t invalidFirmware : 1;
	int32_t ROMStatus : 1;
	int32_t reserved1 : 1;
	int32_t ESNaccessStatus : 1;
	int32_t authorizationCodeStatus : 1;
	int32_t slowADCStatus : 1;
	int32_t supplyVoltageStatus : 1;
	int32_t thermometerStatus : 1;
	int32_t temperatusStatus : 1;
	int32_t MINOS4Status : 1;
	int32_t PLLRf1Status : 1;
	int32_t PLLRf2Status : 1;
	int32_t RF1Status : 1;
	int32_t RF2Status : 1;
	int32_t NVMStatus : 1;
	int32_t softwareResourceLimit : 1;
	int32_t reserved2 : 3;
	int32_t remoteLoadingBegun : 1;
	int32_t exportRestriction : 1;
	int32_t reserved3 : 9;
	int32_t componentHardwareFailure : 1;

};

struct ReceiverStatus
{
	int32_t errorFlag : 1;
	int32_t temperatureStatus : 1;
	int32_t voltageSupplyStatus : 1;
	int32_t antennaPowerStatus : 1;
	int32_t reserved1 : 1;
	int32_t antennaOpenFlag : 1;
	int32_t antennaShortedFlag : 1;
	int32_t CPUoverloadFlag : 1;
	int32_t COM1overrunFlag : 1;
	int32_t COM2overrunFlag : 1;
	int32_t COM3overrunFlag : 1;
	int32_t USBoverrun : 1;
	int32_t reserved2 : 3;
	int32_t RF1AGCStatus : 1;
	int32_t reserved3 : 1;
	int32_t RF2AGCStatus : 1;
	int32_t almanacFlag : 1;
	int32_t positionSolutionFlag : 1;
	int32_t positionFixedFlag : 1;
	int32_t clockSteeringStatus : 1;
	int32_t clockModelFlag : 1;
	int32_t extOscillatorFlag : 1;
	int32_t softwarerResource : 1;
	int32_t reserved4 : 4;
	int32_t AUX3statusEventFlag : 1;
	int32_t AUX2statusEventFlag : 1;
	int32_t AUX1statusEventFlag : 1;
};


struct RXStatus
{
	Oem4BinaryHeader header;
	ReceiverError error;	//!< receiver error field
	uint32_t numStats;		//!< number of status messages
	ReceiverStatus rxStat;	//!< receiver status word
	uint32_t rxStatPri;
	uint32_t rxStatSet;
	uint32_t rxStatClear;
	uint32_t aux1Stat;		//!< auxiliary 1 status field
	uint32_t aux1Pri;
	uint32_t aux1Set;
	uint32_t aux1Clear;
	uint32_t aux2Stat;
	uint32_t aux2Pri;
	uint32_t aux2Set;
	uint32_t aux2Clear;
	uint32_t aux3Stat;
	uint32_t aux3Pri;
	uint32_t aux3Set;
	uint32_t aux3Clear;
	int8_t crc[4];

};

struct RXStatusEvent
{
	Oem4BinaryHeader header;
	StatusWord	status;		// the status word that generated the event message
	uint32_t bitPosition;		// location of the bit in the status word (Table 81, pg 303
	EventType	type;		// event type (Table 86, pg 306)
	int8_t description[32];	// text description of the event or error
};

struct RXHwLevels
{
	Oem4BinaryHeader header;
	float boardTemp;		//!< board temperature in degrees celcius
	float antCurrent;		//!< antenna current (A)
	float coreVoltage;		//!< CPU core voltage (V)
	float supplyVoltage;	//!< supply voltage(V)
	float rfVoltage;		//!< 5V RF supply voltage(V)
	float lnaVoltage;		//!< internal LNA voltage (V)
	float GPAI;				//!< general purpose analog input
	float reserved1;
	float reserved2;
	float lnaGPSCardVoltage;	//!< LNA voltage (V) at GPSCard output
	int8_t crc[4];			//!< 32-bit crc
};

struct BestGPSPositionTest
{
	Oem4BinaryHeader header;
	uint8_t solutionStatus[4];
	uint8_t positionType[4];
	uint8_t latitude[8];
	uint8_t longitutude[8];
	uint8_t height[8];
	uint8_t undulation[4];
	uint8_t datumID[4];
	uint8_t latStdDev[4];
	uint8_t lonStdDev[4];
	uint8_t heightStdDev[4];
	int8_t baseStationID[4];
	uint8_t diff_age[4];
	uint8_t sol_age[4];
	uint8_t numObs;
	uint8_t numGPSL1;
	uint8_t numGPSL1aboveRTK;
	uint8_t numGPSL2aboveRTK;
	uint8_t reserved[4];
	uint8_t crc[4];
};

//*******************************************************************************
// MESSAGE STRUCTURES

//********************
//BESTPOSB

struct bestposb_data {
    double latitude;
    double longitutude;
    double height;
    float undulation;
    DatumID datumID;
    float latStdDev;
    float lonStdDev;
    float heightStdDev;
    int8_t baseStationID[4];
    float diff_age;
    float sol_age;
    uint8_t numObs;
    uint8_t numGPSL1;
    uint8_t numGPSL1aboveRTK;
    uint8_t numGPSL2aboveRTK;
    uint8_t reserved[4];
    uint8_t crc[4];
} __attribute__((packed));

struct bestposb_log {
    Oem4BinaryHeader header;
    SolutionStatus solutionStatus;
    PositionType positionType;
    bestposb_data data;
} __attribute__((packed));
//********************


//********************
//BESTUTMB

struct bestutmb_data {
    uint32_t longZoneNumber;
    uint32_t latZoneLetter;
    double northing;
    double easting;
    double hgt;
    float undulation;
    DatumID datumID;
    float northingStdDev;
    float eastingStdDev;
    float heightStdDev;
    int8_t baseStationID[4];
    float diff_age;
    float sol_age;
    uint8_t numObs;
    uint8_t numGPSL1;
    uint8_t numGPSL1aboveRTK;
    uint8_t numGPSL2aboveRTK;
    uint8_t reserved[4];
    uint8_t crc[4];
} __attribute__((packed));

struct bestutmb_log {
    Oem4BinaryHeader header;
    SolutionStatus solutionStatus;
    PositionType positionType;
    bestutmb_data data;
} __attribute__((packed));
//********************

//********************
//BESTVELB

struct bestvelb_data {
    float latency;
    float age;
    double hor_spd;
    double course;
    double vert_spd;
    float reserved;
    uint8_t crc[4];
} __attribute__((packed));

struct bestvelb_log {
    Oem4BinaryHeader header;
    SolutionStatus solutionStatus;
    PositionType velocityType;
    bestvelb_data data;
} __attribute__((packed));
//********************

//********************
//BESTXYZB

struct bestxyzb_data {
    double X; //x coordinate (m)
    double Y; //y coordinate (m)
    double Z; //z coordinate (m)
    float sig_X; //Standard deviation of x coordinate (m)
    float sig_Y; //Standard deviation of y coordinate (m)
    float sig_Z; //Standard deviation of z coordinate (m)
    SolutionStatus velocityStatus; //Velocity solution status
    PositionType velocityType; //Velocity type
    double velX; //Velcoity in x (m/s)
    double velY; //Velcoity in y (m/s)
    double velZ; //Velcoity in z (m/s)
    float sig_velX; //Standard deviation of velcoity in x (m/s)
    float sig_velY; //Standard deviation of velcoity in y (m/s)
    float sig_velZ; //Standard deviation of velcoity in z (m/s)
    int8_t baseStationID[4]; //Base station ID
    float latency; //Latency in time tag (s)
    float diff_age; //Differential age (s)
    float sol_age; //Solution age (s)
    uint8_t num_obs; //Number of observations tracked
    uint8_t numL1used; //Number of GPS L1 observations used in computation
    uint8_t numL1overmask[1]; //Number of L1 ranges over mask angle
    uint8_t numL2overmask[1]; //Number of L1 ranges over mask angle
    uint8_t reserved[4]; //Reserved
    uint8_t crc[4]; //32bit CRC
} __attribute__((packed));

struct bestxyzb_log {
    Oem4BinaryHeader header; //Log header
    SolutionStatus solutionStatus; //Position solution status
    PositionType positionType; //Position type
    bestxyzb_data data;
} __attribute__((packed));

//********************

//********************
//BSLNXYZB

struct bslnxyzb_data //Structure for BSLNXYZ message
{
    double bslnX; //Baseline x coordinate (m)
    double bslnY; //Baseline y coordinate (m)
    double bslnZ; //Baseline z coordinate (m)
    float sigbslnX; //Standard deviation of baseline x coordinate (m)
    float sigbslnY; //Standard deviation of baseline y coordinate (m)
    float sigbslnZ; //Standard deviation of baseline z coordinate (m)
    int8_t baseStationID[4]; //Base station ID
    uint8_t num_obs; //Number of observations tracked
    uint8_t numL1used; //Number of GPS L1 observations used in computation
    uint8_t numL1; //Number of GPS L1 observations above the RTK mask angle
    uint8_t numL2; //Number of GPS L2 observations above the RTK mask angle
    uint8_t reserved[4]; //Reserved
    uint8_t crc[4]; //32 bit CRC
} __attribute__((packed));

struct bslnxyzb_log {
    Oem4BinaryHeader header; //Log header
    SolutionStatus solutionStatus; //Solution status
    PositionType positionType; //Baseline type
    bslnxyzb_data data;
} __attribute__((packed));
//********************


//********************
//GPSEPHEMB

struct gpsephemb_data {
    uint32_t prn; //PRN number
    double tow; //time stamp of subframe 0 (s)
    uint32_t health; //health status, defined in ICD-GPS-200
    uint32_t iode1; //issue of ephemeris data 1
    uint32_t iode2; //issue of ephemeris data 2
    uint32_t week; //GPS week number
    uint32_t zweek; //z count week number
    double toe; //reference time for ephemeris (s)
    double majaxis; //semi major axis (m)
    double dN; //Mean motion difference (rad/s)
    double anrtime; //mean anomoly reference time (rad)
    double ecc; //eccentricity
    double omega; //arguement of perigee (rad)
    double cuc; //arugument of latitude - cos (rad)
    double cus; //argument of latitude - sine (rad)
    double crc; //orbit radius - cos (rad)
    double crs; //orbit radius - sine (rad)
    double cic; //inclination - cos (rad)
    double cis; //inclination - sine (rad)
    double ia; //inclination angle (rad)
    double dia; //rate of inclination angle (rad/s)
    double wo; //right ascension (rad)
    double dwo; //rate of right ascension (rad/s)
    uint32_t iodc; //issue of data clock
    double toc; //SV clock correction term (s)
    double tgd; //estimated group delay difference
    double af0; //clock aiging parameter 0
    double af1; //clock aiging parameter 1
    double af2; //clock aiging parameter 2
    yes_no spoof; //anti spoofing on
    double cmot; //corrected mean motion
    double ura; //user range accuracy variance
} __attribute__((packed));

struct gpsephemb_log {
    Oem4BinaryHeader header; //Log header
    gpsephemb_data data;
} __attribute__((packed));
//********************


//********************
//IONUTCB

struct ionutcb_data {
    double a0; //alpha parameter constant term
    double a1; //alpha parameter 1st order term
    double a2; //alpha parameter 2nd order term
    double a3; //alpha parameter 3rd order term
    double b0; //beta parameter constant term
    double b1; //beta parameter 1st order term
    double b2; //beta parameter 2nd order term
    double b3; //beta parameter 3rd order term
    uint32_t num_wk; //UTC reference week number
    uint32_t tot; //reference time of UTC parameters
    double A0; //UTC constant term
    double A1; //UTC 1st order term
    uint32_t fut_wk; //future week number
    uint32_t num_day; //day number
    int32_t dells; //delta time due to leap seconds
    int32_t fut_dells; //future delta time due to leap seconds
    uint32_t delutc; //time difference
} __attribute__((packed));

struct ionutcb_log {
    Oem4BinaryHeader header; //Log header
    ionutcb_data data;
} __attribute__((packed));
//********************


//********************
//PSRDOPB

struct psrdopb_data { //Structure for PSRDOP message
    float gdop; //Geometric DOP
    float pdop; //Position DOP
    float hdop; //Horizontal DOP
    float htdop; //Horizontal position and time DOP
    float tdop; //Time DOP
    float cutoff; //Elevation cutoff angle
    int32_t num_prn; //Number of PRNs to follow
} __attribute__((packed));

struct psrdopb_log {
    Oem4BinaryHeader header; //Log header
    psrdopb_data data;
    uint32_t prn[12];
} __attribute__((packed));
//********************


//********************
//PSRPOSB

struct psrposb_data { //Structure for PSRPOS message
    double lat; //Latitude
    double lon; //Longitude
    double hgt; //Height above mean sea level
    float undulation; //Undulation
    DatumID datumID; //Datum ID number
    float sig_lat; //Standard deviation of latitude
    float sig_lon; //Standard deviation of longitude
    float sig_hgt; //Standard deviation of height
    int8_t baseStationID[4]; //Base station ID
    float diff_age; //Differential age (s)
    float sol_age; //Solution age (s)
    uint8_t num_obs; //Number of observations tracked
    uint8_t numL1used; //Number of GPS L1 observations used in computation
    uint8_t reserved[6]; //Reserved
    int8_t crc[4]; //32 bit CRC
} __attribute__((packed));

struct psrposb_log {
    Oem4BinaryHeader header; //Log header
    SolutionStatus solutionStatus; //Solution status
    PositionType positionType; //Position type
    psrposb_data data;
} __attribute__((packed));
//********************


//********************
//PSRVELB

struct psrvelb_data //Structure for PSRVEL message
{
    float latency; //Latency in time tag (s)
    float age; //Differential age (s)
    double hor_spd; //Horizontal speed over ground (m/s)
    double trk_gnd; //Direction of motion over ground (deg from N)
    double vert_spd; //Vertical speed (m/s)
    uint8_t reserved[4]; //Reserved
    uint8_t crc; //32bit CRC
} __attribute__((packed));

struct psrvelb_log {
    Oem4BinaryHeader header; //Log header
    SolutionStatus solutionStatus; //Solution status
    PositionType velocityType; //Velocity type
    psrvelb_data data;
} __attribute__((packed));

//********************


//********************
//PSRXYZB

struct psrxyzb_data //Structure for PSRXYZ message
{
    double psrX; //x coordinate (m)
    double psrY; //y coordinate (m)
    double psrZ; //z coordinate (m)
    float sig_psrX; //Standard deviation of x coordinate (m)
    float sig_psrY; //Standard deviation of y coordinate (m)
    float sig_psrZ; //Standard deviation of z coordinate (m)
    SolutionStatus velocityStatus; //Velocity solution status
    PositionType velocityType; //Velocity type
    double velX; //Velcoity in x (m/s)
    double velY; //Velcoity in y (m/s)
    double velZ; //Velcoity in z (m/s)
    float sig_velX; //Standard deviation of velcoity in x (m/s)
    float sig_velY; //Standard deviation of velcoity in y (m/s)
    float sig_velZ; //Standard deviation of velcoity in z (m/s)
    int8_t baseStationID[4]; //Base station ID
    float latency; //Latency in time tag (s)
    float diff_age; //Differential age (s)
    float sol_age; //Solution age (s)
    uint8_t num_obs; //Number of observations tracked
    uint8_t numL1used; //Number of GPS L1 observations used in computation
    uint8_t reserved[6]; //Reserved
    uint8_t crc; //32bit CRC
} __attribute__((packed));

struct psrxyzb_log {
    Oem4BinaryHeader header; //Log header
    SolutionStatus solutionStatus; //Position solution status
    PositionType positionType; //Position type
    psrxyzb_data data;
} __attribute__((packed));

//********************


//********************
//RANGEB
//********************

struct channel_status {
	unsigned tracking_state : 5;
	unsigned sv_chan_num : 5;
	unsigned phase_lock_flag : 1;
	unsigned parity_known_flag : 1;
	unsigned code_locked_flag : 1;
	unsigned correlator_type : 3;
	unsigned satellite_sys : 3;
	unsigned reserved1 : 1;
	unsigned grouping : 1;
	unsigned signal_type : 5;
	unsigned forward_err_correction : 1;
	unsigned primary_L1_chan : 1;
	unsigned carrier_phase_meas : 1;
	unsigned reserved2 : 1;
	unsigned prn_lock_flag : 1;
	unsigned channel_assignment : 1;
} __attribute__((packed));

struct rangeb_data {
    uint16_t svprn; // PRN
    uint16_t frequency; // Frequency number of GLONASS SV (0 for GPS)
    double psr; // pseudo range
    float psrstd; // pseudorange standard deviation
    double adr; // accumulated doppler
    float adrstd; // accumulated doppler standard deviation
    float dop; // Doppler
    float sno; // Signal/Noise
    float locktime; // time locked
    channel_status ch_status; // channel status
} __attribute__((packed));

struct rangeb_log {
    Oem4BinaryHeader hdr;
    uint32_t iObs; // Number of observations
    rangeb_data data[MAXCHAN];
} __attribute__((packed));
//********************


//********************
//RANGECMPB

struct compressed_data {
    signed dop : 28;
    signed psr : 36;
    signed adr : 32;
    unsigned psr_std : 4;
    unsigned adr_std : 4;
    unsigned svprn : 8;
    unsigned locktime : 21;
    unsigned cno : 5;
    unsigned reserved : 22;
} __attribute__((packed));

struct rangecmpb_data {
    channel_status ch_status; // channel status (defined in rangeb section)
    compressed_data range_record;

} __attribute__((packed));

struct rangecmpb_log {
    Oem4BinaryHeader hdr;
    uint32_t iObs; // Number of observations
    rangecmpb_data data[MAXCHAN]; // packed binary range data
    int8_t crc[4];
} __attribute__((packed));
//********************


//********************
//RAWEPHEMB
//struct rawephem_data
//{
//   uint32_t  prn;              // Satellite PRN number
//   uint32_t  ereferweek;       // Ephemeris Reference Week
//   uint32_t  erefertime;       // Ephemeris Reference Time
//   int8_t           subframe1[30];    // Ephemeris subframe 1
//   int8_t           subframe2[30];    // Ephemeris subframe 2
//   int8_t           subframe3[30];    // Ephemeris subframe 3
//   uint16_t filler1;
//   uint16_t filler2;
//} __attribute__ ((packed));
//
//struct rawephem_log
//{
//   Oem4BinaryHeader      hdr;
//   rawephem_data           data;
//} __attribute__ ((packed));
//********************


//********************
//RTCADATA1B

struct rtcadata1b_header {
    double zcount; //Week number from subframe one of the ephemeris
    uint8_t aeb; //Acceleration error bound
    uint32_t num_prn; //Number of satellite corrections with info to follow
} __attribute__((packed));

struct rtcadata1b_data {
    uint32_t prn; //PRN number of range measurement
    double range; //pseudorange correction (m)
    uint8_t iode; //Issue of ephemeris data
    double rrate; //pseudorange rate correction (m/s)
    float udre; //user differential range error
} __attribute__((packed));

struct rtcadata1b_log {
    Oem4BinaryHeader header; //Log header
    rtcadata1b_header info;
    rtcadata1b_data data[MAX_NUM_SAT];
} __attribute__((packed));

//********************


//********************
//RTCADATAEPHEMB

struct rtcadataephemb_data {
    uint8_t des; //Novatel designator
    uint8_t subtype; //RTCA message subtype
    uint32_t week; //GPS week number
    uint32_t sec; //Seconds into week
    uint32_t prn; //PRN number
    int8_t reserved[4];
    int8_t ephem[92]; //Raw ephemeris data
} __attribute__((packed));

struct rtcadataephemb_log {
    Oem4BinaryHeader header; //Log header
    rtcadataephemb_data data;
} __attribute__((packed));
//********************


//********************
//RTCADATAOBSB - CHECK

struct rtcadataobsb_header {
    uint8_t des; //Novatel designator
    uint8_t subtype; //RTCA message subtype
    double min_psr; //minimum pseudorange
    float sec; //seconds into GPS week
    int8_t reserved[4];
    uint32_t num_ids; //Number of transmitter ids with info to follow
} __attribute__((packed));

struct rtcadataobsb_data //Structure for RTCADATAEPHEM message
{
    uint8_t transID; //Transmitter ID
    uint8_t L1lock; //L1 lock flag
    uint8_t L2lock; //L2 lock flag
    double L1psr; //L1 pseudorange offset
    double L2psr; //L2 pseudorange offset
    float L1adr; //L1 carrier phase offset, accumulated doppler range
    float L2adr; //L2 carrier phase offset, accumulated doppler range
    yes_no L2encrypt; //If L2 is encrypted
    int8_t reserved[4];
} __attribute__((packed));

struct rtcadataobsb_log {
    Oem4BinaryHeader header; //Log header
    rtcadataobsb_header info;
    rtcadataobsb_data data[MAX_NUM_SAT]; //WT:  This is probably too many... need to verify how many id's can be sent.
} __attribute__((packed));
//********************


//********************
//RTCADATAREFB

struct rtcadatarefb_data {
    uint8_t des; //Novatel designator
    uint8_t subtype; //RTCA message subtype
    double posX; //base station X coordinate position (mm)
    double posY; //base station Y coordinate position (mm)
    double posZ; //base station Z coordinate position (mm)
    int8_t reserved[4];
} __attribute__((packed));

struct rtcadatarefb_log {
    Oem4BinaryHeader header; //Log header
    rtcadatarefb_data data;
} __attribute__((packed));
//********************


//********************
//RTKDATAB

struct rtkdatab_header {
    uint32_t rtkinfo; //RTK information
    uint8_t num_obs; //Number of observations tracked
    uint8_t gps_l1_ranges; // Number of GPS L1 ranges used
    uint8_t gps_l1_mask_ranges; // Number of GPS L1 ranges above the RTK mask angle used
    uint8_t gps_l2_mask_ranges; // Number of GPS L2 ranges above the RTK mask angle used
    uint8_t reserved[4];
    SEARCHER_TYPE searcher_type; // Searcher type
    uint32_t num_lane_combs; // Number of possible lane combinations
    float Cxx; // ECEF position covariance matrix
    float Cxy;
    float Cxz;
    float Cyx;
    float Cyy;
    float Cyz;
    float Czx;
    float Czy;
    float Czz;
    double delta_x; // Float solution baseline in ECEF - x
    double delta_y; // Float solution baseline in ECEF - y
    double delta_z; // Float solution baseline in ECEF - z
    float sd_x; // Standard deviation of float solution baseline in ECEF - x
    float sd_y; // Standard deviation of float solution baseline in ECEF - y
    float sd_z; // Standard deviation of float solution baseline in ECEF - z
    uint32_t ref_prn; // Reference PRN
    int32_t num_svs; // The number of SVs in data portion
} __attribute__((packed));

struct rtkdatab_data {
    uint32_t prn; // GPS satellite PRN
    AMBIGUITY_TYPE ambiguity_type; // Type of ambiguity
    float residual; // Satellite health
} __attribute__((packed));

struct rtkdatab_log {
    Oem4BinaryHeader hdr;
    SolutionStatus solutionStatus; //Solution status
    PositionType positionType; //Position type
    rtkdatab_header header;
    rtkdatab_data data[MAX_NUM_SAT];
} __attribute__((packed));
//********************


//********************
//RTKPOSB

struct rtkposb_data //Structure for RTKPOS message
{
    double lat; //Latitude
    double lon; //Longitude
    double hgt; //Height above mean sea level
    float undulation; //Undulation
    DatumID datumID; //Datum ID number
    float sig_lat; //Standard deviation of latitude
    float sig_lon; //Standard deviation of longitude
    float sig_hgt; //Standard deviation of height
    int8_t baseStationID[4]; //Base station ID
    float diff_age; //Differential age (s)
    float sol_age; //Solution age (s)
    uint8_t num_obs; //Number of observations tracked
    uint8_t numL1used; //Number of GPS L1 ranges used in computation
    uint8_t numL1mask; //Number of GPS L1 ranges above RTK mask
    uint8_t numL2mask; //Number of GPS L2 ranges above RTK mask
    uint8_t reserved[4]; //Reserved
    uint8_t crc[4]; //32 bit CRC
} __attribute__((packed));

struct rtkposb_log {
    Oem4BinaryHeader header; //Log header
    SolutionStatus solutionStatus; //Solution status
    PositionType positionType; //Position type
    rtkposb_data data;
} __attribute__((packed));
//********************


//********************
//RTKVELB

struct rtkvelb_data //Structure for PSRVEL message
{
    float latency; //Latency in time tag (s)
    float age; //Differential age (s)
    double hor_spd; //Horizontal speed over ground (m/s)
    double trk_gnd; //Direction of motion over ground (deg from N)
    double vert_spd; //Vertical speed (m/s)
    uint8_t reserved[4]; //Reserved
    uint8_t crc; //32bit CRC
} __attribute__((packed));

struct rtkvelb_log {
    Oem4BinaryHeader header; //Log header
    SolutionStatus solutionStatus; //Solution status
    PositionType velocityType; //Velocity type
    rtkvelb_data data;
} __attribute__((packed));
//********************


//********************
//RTKXYZB

struct rtkxyzb_data //Structure for PSRXYZ message
{
    double posX; //x coordinate (m)
    double posY; //y coordinate (m)
    double posZ; //z coordinate (m)
    float sig_posX; //Standard deviation of x coordinate (m)
    float sig_posY; //Standard deviation of y coordinate (m)
    float sig_posZ; //Standard deviation of z coordinate (m)
    SolutionStatus velocityStatus; //Velocity solution status
    PositionType velocityType; //Velocity type
    double velX; //Velcoity in x (m/s)
    double velY; //Velcoity in y (m/s)
    double velZ; //Velcoity in z (m/s)
    float sig_velX; //Standard deviation of velcoity in x (m/s)
    float sig_velY; //Standard deviation of velcoity in y (m/s)
    float sig_velZ; //Standard deviation of velcoity in z (m/s)
    int8_t baseStationID[4]; //Base station ID
    float latency; //Latency in time tag (s)
    float diff_age; //Differential age (s)
    float sol_age; //Solution age (s)
    uint8_t num_obs; //Number of observations tracked
    uint8_t numL1used; //Number of GPS L1 ranges used in computation
    uint8_t numL1mask; //Number of GPS L1 ranges above RTK mask
    uint8_t numL2mask; //Number of GPS L2 ranges above RTK mask
    uint8_t reserved[4]; //Reserved
    uint8_t crc[4]; //32 bit CRC
} __attribute__((packed));

struct rtkxyzb_log {
    Oem4BinaryHeader header; //Log header
    SolutionStatus solutionStatus; //Position solution status
    PositionType positionType; //Position type
    rtkxyzb_data data;
} __attribute__((packed));
//********************


//********************
//SATXYZB

struct satxyzb_header {
    double dReserved1;
    uint32_t ulNumSats; // number of satellites
} __attribute__((packed));

struct satxyzb_data {
    uint32_t ulPRN; // SV PRN
    double dX; // SV X coordinate (metres)
    double dY; // SV Y coordinate (metres)
    double dZ; // SV Z coordinate (metres)
    double dClkCorr; // SV clock correction
    double dIonCorr; // ionospheric correction
    double dTropCorr; // tropospheric correction
    double dReserved2; // reserved
    double dReserved3; // reserved
} __attribute__((packed));

struct satxyzb_log {
    Oem4BinaryHeader oem4hdr;
    satxyzb_header header;
    satxyzb_data data[MAXCHAN];
} __attribute__((packed));
//********************


//********************
//TIMEB

struct time_data {
    uint32_t ulClockModel; // ClockModelStatus
    double dGPSOffset; // Receiver Offset in seconds from GPS time
    double dOffsetStd; // Instantaneous Standard Deviation of Receiver Clock Offset
    double dUtcOffset; // Offset in seconds of GPS time from UTC time
    int32_t lUtcYear; // UTC Year
    uint8_t ucUtcMonth; // UTC Month
    uint8_t ucUtcDay; // UTC Day
    uint8_t ucUtcHour; // UTC Hour
    uint8_t ucUtcMin; // UTC Minutes
    int32_t lUtcMillisec; // UTC Milliseconds
    int32_t bUtcStatus; // UTC Status
} __attribute__((packed));

struct timeb_log {
    Oem4BinaryHeader hdr;
    time_data body;
} __attribute__((packed));
//********************


//********************
//VISIONSOLB

struct visionsolb_data {
    uint16_t channel; //Channel tracking number
    uint16_t prn; //Prn number of range measurement
    yes_no multipath; //Is multipath detected?
    float sigdelay; //Direct signal delay
    float sigphase; //Direct signal phase angle
    float sigamp; //Direct signal amplitude
    float mpdelay; //Multipath delay
    float mpphase; //Multipath phase angle
    float mpamp; //Multipath amplityde
    float sqres; //Sum of the squares residual
} __attribute__((packed));

struct visionsolb_header {
    uint32_t num_vis; //number of vision solutions with info to follow
} __attribute__((packed));

struct visionsolb_log {
    Oem4BinaryHeader hdr;
    visionsolb_header info;
    visionsolb_data data[MAX_NUM_SAT];
} __attribute__((packed));
//********************


#endif
