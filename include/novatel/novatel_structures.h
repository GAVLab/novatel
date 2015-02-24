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
#include <stdint.h>  // use fixed size integer types, rather than standard c++ types
#include <stdint.h>  // use fixed size integer types, rather than standard c++ types

namespace novatel {

#define MAX_NOUT_SIZE 8192 // Maximum size of a NovAtel log buffer (ALMANACA logs are big!)
#define EPH_CHAN 33
#define NUMSAT 14
#define MAX_CHAN	54  // Maximum number of signal channels
#define MAX_NUM_SAT 28	// Maximum number of satellites with information in the RTKDATA log
#define HEADER_SIZE 28 // Binary header size for OEM 4, V, and 6 receivers
#define SHORT_HEADER_SIZE 12 // short binary header size
#define CHECKSUM_SIZE 4  // size of the message CRC


#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x44
#define SYNC_BYTE_3 0x12

#define SYNC_1_IDX 0 	// first sync byte location
#define SYNC_2_IDX 1 	// second sync byte location
#define SYNC_3_IDX 2 	// third sync byte location
#define HEADER_LEN_IDX 3 // header length location
#define MSG_ID_END_IDX 5	// Message ID location
#define MSG_LENGTH_END_IDX 9 // message length index

#define NOVATEL_SYNC_BYTE_1 0xAA
#define NOVATEL_SYNC_BYTE_2 0x44
#define NOVATEL_SYNC_BYTE_3 0x12
#define NOVATEL_ACK_BYTE_1 '<'
#define NOVATEL_ACK_BYTE_2 'O'
#define NOVATEL_ACK_BYTE_3 'K'
#define NOVATEL_RESET_BYTE_1 0X5B
#define NOVATEL_RESET_BYTE_2 'C'
#define NOVATEL_RESET_BYTE_3 'O'
#define NOVATEL_RESET_BYTE_4 'M'
#define NOVATEL_RESET_BYTE_6 0X5D

// IMU Constants
// scale factor between integer counts and change in velocity in m/s for AG11 and AG58
#define VELOCITY_CHANGE_SCALE_FACTOR_11_58 (0.3048/((double)134217728.0))
// scale factor between integer counts and change in velocity in m/s for AG17 and AG62
#define VELOCITY_CHANGE_SCALE_FACTOR_17_62 (0.3048/((double)67108864.0))
// scale factor between integer counts and change in angle in rad for AG11, AG17, AG58, and AG62
#define ANGULAR_CHANGE_SCALE_FACTOR (1.0/((double)8589934592.0))

// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
	#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
	#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif




//*******************************************************************************
// HEADER STRUCTURES
//*******************************************************************************
/*!
 * Message Type
 */
PACK(
struct MessageType {
    unsigned reserved:5;
    MessageFormat format:2;
    ResponseBit response:1;
});

//! Header prepended to OEM4 binary messages
PACK(
struct Oem4BinaryHeader
{
   uint8_t          sync1;          //!< start of packet first byte (0xAA)
   uint8_t          sync2;          //!< start of packet second byte (0x44)
   uint8_t          sync3;          //!< start of packet third  byte (0x12)
   uint8_t          header_length; 	//!< Length of the header in bytes ( From start of packet )
   uint16_t         message_id;    	//!< Message ID number
   MessageType      message_type;  	//!< Message type - binary, ascii, nmea, etc...
   uint8_t          port_address;  	//!< Address of the data port the log was received on
   uint16_t         message_length;	//!< Message length (Not including header or CRC)
   uint16_t         sequence;      	//!< Counts down from N-1 to 0 for multiple related logs
   uint8_t          idle;          	//!< Time the processor was idle in last sec between logs with same ID
   uint8_t          time_status;    //!< Indicates the quality of the GPS time
   uint16_t         gps_week;      	//!< GPS Week number
   uint32_t         gps_millisecs; 	//!< Milliseconds into week
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


//*******************************************************************************
// INS STRUCTURES
//*******************************************************************************

//! IMU status structure that is included in the RAWIMU message
PACK(
struct ImuStatus
{
   unsigned counter : 4;						//!< 4 byte counter
   unsigned imu_test : 1;						//!< IMU test: Passed=0, Failed=1
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


/*!
 * INSPVA Message Structure
 * This log allows INS position, velocity and
 * attitude to be collected in one log, instead
 * of using three separate logs.
 */
PACK(
struct InsPositionVelocityAttitude
{
	Oem4BinaryHeader header;	//!< Message header
	uint32_t gps_week;			//!< GPS week number
	double gps_millisecs;		//!< Milliseconds into GPS week
	double latitude;			//!< latitude - WGS84 (deg)
	double longitude;			//!< longitude - WGS84 (deg)
	double height;				//!< Ellipsoidal height - WGS84 (m)
	double north_velocity;		//!< velocity in a northerly direction (m/s)
	double east_velocity;		//!< velocity in an easterly direction (m/s)
	double up_velocity;			//!< velocity in an up direction
	double roll;				//!< right handed rotation around y-axis (degrees)
	double pitch;				//!< right handed rotation aruond x-axis (degrees)
	double azimuth;				//!< right handed rotation around z-axis (degrees)
	InsStatus status;			//!< status of the INS system
	int8_t crc[4];
});

/*!
 * INSPVAS Message Structure
 * This log allows INS position, velocity and
 * attitude to be collected in one log, instead
 * of using three separate logs. Short header version
 */
PACK(
struct InsPositionVelocityAttitudeShort
{
	OEM4ShortBinaryHeader header;
	uint32_t gps_week;			//!< GPS week number
	double gps_millisecs;		//!< Milliseconds into GPS week
	double latitude;			//!< latitude - WGS84 (deg)
	double longitude;			//!< longitude - WGS84 (deg)
	double height;				//!< Ellipsoidal height - WGS84 (m)
	double north_velocity;		//!< velocity in a northerly direction (m/s)
	double east_velocity;		//!< velocity in an easterly direction (m/s)
	double up_velocity;			//!< velocity in an up direction
	double roll;				//!< right handed rotation around y-axis (degrees)
	double pitch;				//!< right handed rotation aruond x-axis (degrees)
	double azimuth;				//!< right handed rotation around z-axis (degrees)
	InsStatus status;			//!< status of the INS system
	int8_t crc[4];
});

/*!
 * INSCOV Message Structure
 * The position, attitude, and velocity matrices
 * in this log each contain 9 covariance values,
 * with respect to the local level frame. For the
 * attitude angles, they are given in the SPAN computation
 * frame. These values are computed once per second and
 * are only available after alignment.
 */
PACK(
struct InsCovariance
{
	Oem4BinaryHeader header;	//!< Message header
	uint32_t gps_week;				//!< GPS week number
	double gps_millisecs;			//!< Milliseconds into GPS week
	double position_covariance[9];  //!< Position covariance matrix [m^2] (xx,xy,xz,yz,yy,...)
	double attitude_covariance[9];  //!< Attitude covariance matrix [deg^2] (xx,xy,xz,yz,yy,...)
	double velocity_covariance[9];  //!< Velocity covariance matrix [(m/s)^2] (xx,xy,xz,yz,yy,...)
	int8_t crc[4];
});

/*!
 * INSCOVS Message Structure
 * The position, attitude, and velocity matrices
 * in this log each contain 9 covariance values,
 * with respect to the local level frame. For the
 * attitude angles, they are given in the SPAN computation
 * frame. These values are computed once per second and
 * are only available after alignment. This log is a short
 * header version of INSCOV
 */
PACK(
struct InsCovarianceShort
{
	OEM4ShortBinaryHeader header;	//!< Message header
	uint32_t gps_week;				//!< GPS week number
	double gps_millisecs;			//!< Milliseconds into GPS week
	double position_covariance[9];  //!< Position covariance matrix [m^2] (xx,xy,xz,yz,yy,...)
	double attitude_covariance[9];  //!< Attitude covariance matrix [deg^2] (xx,xy,xz,yz,yy,...)
	double velocity_covariance[9];  //!< Velocity covariance matrix [(m/s)^2] (xx,xy,xz,yz,yy,...)
	int8_t crc[4];
});



/*!
 * INSSPD Message Structure
 * This log contains the most recent speed
 * measurements in the horizontal and vertical
 * directions, and includes an INS status indicator.
 */
PACK(
struct InsSpeed
{
	Oem4BinaryHeader header;	//!< Message header
	uint32_t gps_week;			//!< GPS week number
	double gps_millisecs;		//!< Milliseconds into GPS week
	double track_over_ground;	//!< actual direction of motion over ground (degrees)
    double horizontal_speed;	//!< horizontal speed in m/s
	double vertical_speed;		//!< vertical speed in m/s
	InsStatus status;			//!< status of the INS system
	int8_t crc[4];
});


/*!
 * RAWIMU Message Structure
 * This log contains an IMU status indicator
 * and the measurements from the accelerometers
 * and gyros with respect to the IMU enclosure
 * frame. If logging this data, consider the RAWIMUS
 * log to reduce the amount of data,
 */
PACK(
struct RawImu
{
	Oem4BinaryHeader header;	//!< Message header
	uint32_t gps_week;			//!< GPS week number
	double gps_millisecs;		//!< Milliseconds into GPS week
	ImuStatus imuStatus;		//!< Status of the IMU
	int32_t z_acceleration;		//!< change in velocity along z axis in scaled m/s
	int32_t y_acceleration_neg; //!< -change in velocity along y axis in scaled m/s
	int32_t x_acceleration;		//!< change in velocity along x axis in scaled m/s
	int32_t z_gyro_rate;		//!< change in angle around z axis in radians
	int32_t y_gyro_rate_neg; 	//!< -(change in angle around y axis) in radians
	int32_t x_gyro_rate;		//!< change in angle around x axis in radians
	int8_t crc[4];
});
// scale factor for change in angle (1.0/((double)8589934592.0)) for the AG11, AG58, AG17, and AG62
// scale factor for change in velocity (acceleration)
//(0.3048/((double)134217728.0)) for the AG11 and AG58
//(0.3048/((double)67108864.0)) for the AG17 and AG62

/*!
 * RAWIMUS Message Structure
 * This log contains an IMU status indicator
 * and the measurements from the accelerometers
 * and gyros with respect to the IMU enclosure
 * frame. It is a short header version of RAWIMU
 */
PACK(
struct RawImuShort
{
	OEM4ShortBinaryHeader header;	//!< Message header
	uint32_t gps_week;				//!< GPS week number
	double gps_millisecs;			//!< Milliseconds into GPS week
	ImuStatus imuStatus;			//!< Status of the IMU
	int32_t z_acceleration;			//!< change in velocity along z axis in scaled m/s
	int32_t y_acceleration_neg; 	//!< -change in velocity along y axis in scaled m/s
	int32_t x_acceleration;			//!< change in velocity along x axis in scaled m/s
	int32_t z_gyro_rate;			//!< change in angle around z axis in radians
	int32_t y_gyro_rate_neg; 		//!< -(change in angle around y axis) in radians
	int32_t x_gyro_rate;			//!< change in angle around x axis in radians
	int8_t crc[4];
});


/*!
 * VEHICLEBODYROTATION Message Structure
 * The VEHICLEBODYROTATION log reports the angular
 *  offset from the vehicle frame to the SPAN frame.
 *  The SPAN frame is defined by the transformed IMU
 *  enclosure axis with Z pointing up, see the
 *  SETIMUORIENTATION command on page 126. If your
 *  IMU is mounted with the Z axis (as marked on the IMU
 *  enclosure) pointing up, the IMU enclosure frame is
 *  the same as the SPAN frame.
 */
PACK(
struct VehicleBodyRotation
{
	Oem4BinaryHeader header;//!< Message header
	double x_angle;			//!< rotation about vehicle frame x axis (deg)
	double y_angle;			//!< rotation about vehicle frame y axis (deg)
	double z_angle;			//!< rotation about vehicle frame z axis (deg)
	double x_uncertainty;	//!< uncertainty of x axis rotation (deg)
	double y_uncertainty;	//!< uncertainty of y axis rotation (deg)
	double z_uncertainty;	//!< uncertainty of z axis rotation (deg)
	int8_t crc[4];
});


/*!
 * BESTLEVERARM Message Structure
 * This log contains the distance between the
 * IMU’s centre of navigation and the GPS phase
 * centre in the IMU enclosure frame and its associated
 * uncertainties.
 */
PACK(
struct BestLeverArm
{
	Oem4BinaryHeader header;//!< Message header
	double x_offset;		//!< x offset in IMU enclosure frame (m)
	double y_offset;		//!< y offset in IMU enclosure frame (m)
	double z_offset;		//!< z offset in IMU enclosure frame (m)
	double x_uncertainty;	//!< uncertainty of x offset (m)
	double y_uncertainty;	//!< uncertainty of x offset (m)
	double z_uncertainty;	//!< uncertainty of x offset (m)
	int32_t mapping;		//!< IMU axis mapping setting
	int8_t crc[4];
});


//*******************************************************************************
// GENERIC GPS STRUCTURES
//*******************************************************************************

/*!
 * Position Message Structure
 * This log contains the a position received from the receiver
 * in latitude and longitude as well as status information such as
 * the solution type and the number of satellites used.
 *
 * This structure represents the format of the following messages:
 *  - BESTPOS
 *  - RTKPOS
 *  - PSRPOS
 */
PACK(
struct Position
{
	Oem4BinaryHeader header;			//!< Message header
	SolutionStatus solution_status;		//!< Solution status
	PositionType position_type;			//!< Position type
	double latitude;					//!< latitude (deg)
    double longitude;					//!< longitude (deg)
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
 * ECEF Position Message Structure
 * This log contains the receiver’s best
 * available position and velocity in ECEF
 * coordinates. The position and velocity status
 * fields indicate whether or not the corresponding
 * data is valid.
 *
 * This structure represents the format of the following messages:
 *  - BESTXYZ
 *  - RTKXYZ
 *  - PSRXYZ
 */
PACK(
struct PositionEcef
{
    Oem4BinaryHeader header;				//!< Message header
    SolutionStatus 	 solution_status;		//!< Solution status
    PositionType 	 position_type;			//!< Position type
    double       	 x_position;        	//!< x coordinate in ECEF (m)
    double           y_position;        	//!< x coordinate in ECEF (m)
    double           z_position;        	//!< x coordinate in ECEF (m)
    float            x_standard_deviation;  //!< Standard deviation of x coordinate (m)
    float            y_standard_deviation;  //!< Standard deviation of y coordinate (m)
    float            z_standard_deviation;  //!< Standard deviation of z coordinate (m)
    SolutionStatus   velocity_status;       //!< Velocity solution status
    PositionType     velocity_type;         //!< Velocity solution type
    double           x_velocity;            //Velocity in x (m/s)
    double           y_velocity;            //Velocity in y (m/s)
    double           z_velocity;            //Velocity in z (m/s)
    float            x_velocity_standard_deviation;  //!< Standard deviation of velcoity in x (m/s)
    float            y_velocity_standard_deviation;  //!< Standard deviation of velcoity in y (m/s)
    float            z_velocity_standard_deviation;  //!< Standard deviation of velcoity in z (m/s)
    int8_t           base_station_id[4];    //!< Base station ID
    float            velocity_latency;      //!< Latency in velocity time tag (s)
    float 		 	 differential_age;		//!< differential position age (sec)
    float 			 solution_age;			//!< solution age (sec)
    uint8_t 		 number_of_satellites;	//!< number of satellites tracked
    uint8_t 		 number_of_satellites_in_solution;	//!< number of satellites used in solution
    uint8_t          reserved[3];         	//!< Reserved
    uint8_t 		 extended_solution_status;	//!< extended solution status - OEMV and greater only
    uint8_t 		 reserved2; 			//!< reserved
    uint8_t 		 signals_used_mask;		//!< signals used mask - OEMV and greater only
    uint8_t crc[4];
});



/*!
 * Velocity Message Structure
 * This log contains the best available velocity
 * information computed by the receiver. In addition,
 * it reports a velocity status indicator, which is
 * useful in indicating whether or not the corresponding
 * data is valid. The velocity measurements sometimes
 * have a latency associated with them. The time of validity
 * is the time tag in the log minus the latency value.
 *
 * This structure represents the format of the following messages:
 *  - BESTVEL
 *  - RTKVEL
 *  - PSRVEL
 */
PACK(
struct Velocity
{
	Oem4BinaryHeader header;			//!< Message header
	SolutionStatus solution_status;		//!< Solution status
	PositionType position_type;			//!< Position type
	float latency;						//!< measure of the latency of the velocity time tag in seconds
	float age;							//!< differential age in seconds
	double horizontal_speed;			//!< horizontal speed in m/s
	double track_over_ground;			//!< direction of travel in degrees
	double vertical_speed; 				//!< vertical speed in m/s
	float reserved;
	int8_t crc[4];
});


/*!
 * DOP Message Structure
 * The dilution of precision data is calculated
 * using the geometry of only those satellites that are
 * currently being tracked and used in the position
 * solution by the receiver. This log is updated once
 * every 60 seconds or whenever a change in the satellite
 * constellation occurs. Therefore, the total number of
 * data fields output by the log is variable and depends
 * on the number of SVs that are being tracked.
 *
 * This structure represents the format of the following messages:
 *  - PSRDOP
 *  - RTKDOP
 */
PACK(
struct Dop {
    Oem4BinaryHeader header;            //!< Message header
    float geometric_dop;                //!< Geometric DOP
    float position_dop;                 //!< Position DOP
    float horizontal_dop;               //!< Horizontal DOP
    float horizontal_position_time_dop; //!< Horizontal position and time DOP
    float time_dop;                     //!< Time DOP
    float elevation_cutoff_angle;       //!< Elevation cutoff angle
    int32_t number_of_prns;             //!< Number of PRNs to follow
    uint32_t prn[MAX_CHAN];                   //!< PRNof each satellite used
    uint8_t 	crc[4];                 //!< 32-bit cyclic redundancy check (CRC)
});

//*******************************************************************************
// MESSAGE SPECIFIC GPS STRUCTURES
//*******************************************************************************

/*!
 * BSLNXYZ Message Structure
 * This log contains the receiver’s RTK baseline
 * in ECEF coordinates. The position status field
 * indicates whether or not the corresponding data
 * is valid.
 */
PACK(
struct BaselineEcef
{
  Oem4BinaryHeader header;				//!< Message header
  SolutionStatus 	 solution_status;		//!< Solution status
  PositionType 	 position_type;			//!< Position type
  double		x_baseline;                	//!< Baseline x coordinate (m)
  double    	y_baseline;                	//!< Baseline y coordinate (m)
  double    	z_baseline;                	//!< Baseline z coordinate (m)
  float     	x_baseline_standard_deviation;  //!< Standard deviation of baseline x coordinate (m)
  float    	y_baseline_standard_deviation;  //!< Standard deviation of baseline y coordinate (m)
  float    	z_baseline_standard_deviation;  //!< Standard deviation of baseline z coordinate (m)
  int8_t      base_station_id[4];         //!< Base station ID
  uint8_t 	number_of_satellites;		//!< number of satellites tracked
  uint8_t 	number_of_satellites_in_solution;	//!< number of satellites used in solution
  uint8_t 	num_gps_plus_glonass_l1;	//!< number of GPS plus GLONASS L1 satellites used in solution
  uint8_t 	num_gps_plus_glonass_l2;	//!< number of GPS plus GLONASS L2 satellites used in solution
  uint8_t 	reserved;					//!< reserved
  uint8_t 	extended_solution_status;	//!< extended solution status - OEMV and greater only
  uint8_t 	reserved2; 					//!< reserved
  uint8_t 	signals_used_mask;			//!< signals used mask - OEMV and greater only
  uint8_t 	crc[4];						//!< 32-bit cyclic redundancy check (CRC)
});


/*!
 * UTM Position Message Structure
 * This log contains the best available position
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
struct UtmPosition
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

/*!
 * IONUTC Message Structure
 * This log contains The Ionospheric Model
 * parametres (ION) and the Universal Time
 * Coordinated parametres (UTC)
 */
PACK(
struct IonosphericModel {
    Oem4BinaryHeader header;				//!< Message header
    double a0; //!< alpha parameter constant term
    double a1; //!< alpha parameter 1st order term
    double a2; //!< alpha parameter 2nd order term
    double a3; //!< alpha parameter 3rd order term
    double b0; //!< beta parameter constant term
    double b1; //!< beta parameter 1st order term
    double b2; //!< beta parameter 2nd order term
    double b3; //!< beta parameter 3rd order term
    uint32_t num_wk; //!< UTC reference week number
    uint32_t tot; //!< reference time of UTC parameters
    double A0; //!< UTC constant term
    double A1; //!< UTC 1st order term
    uint32_t fut_wk; //!< future week number
    uint32_t num_day; //!< day number
    int32_t dells; //!< delta time due to leap seconds
    int32_t fut_dells; //!< future delta time due to leap seconds
    uint32_t delutc; //!< time difference
    uint8_t 	crc[4];	//!< 32-bit cyclic redundancy check (CRC)
});

/*!
 * Channel Tracking Status
 * Used in logs RANGE and TRACKSTAT
 */
PACK(
struct ChannelStatus {
	unsigned int tracking_state : 5;
	unsigned int sv_chan_num : 5;
	unsigned int phase_lock_flag : 1;
	unsigned int parity_known_flag : 1;
	unsigned int code_locked_flag : 1;
	unsigned int correlator_type : 3;
	unsigned int satellite_sys : 3;
	unsigned int reserved1 : 1;
	unsigned int grouping : 1;
	unsigned int signal_type : 5;
	unsigned int forward_err_correction : 1;
	unsigned int primary_L1_chan : 1;
	unsigned int carrier_phase_meas : 1;
	unsigned int reserved2 : 1;
	unsigned int prn_lock_flag : 1;
	unsigned int channel_assignment : 1;
});

/*!
 * Pseudorange Message Structure
 * This log contains the pseudorange information for a
 * single channel. Used in the RangeMeasurements structure.
 */
PACK(
struct RangeData {
    uint16_t satellite_prn;                  //!< SV PRN number
    uint16_t glonass_frequency;              //!< Frequency number of GLONASS SV (0 for GPS)
    double pseudorange;                      //!<  pseudorange [m]
    float pseudorange_standard_deviation;    //!< pseudorange standard deviation [m]
    double accumulated_doppler;             //!< accumulated doppler [cycles]
    float accumulated_doppler_std_deviation; //!< accumulated doppler standard deviation [cycles]
    float doppler;                           //!< Doppler frequency [Hz]
    float carrier_to_noise;                  //!< Signal/Noise [dB-Hz]
    float locktime;                          //!< Number of seconds of continuous tracking [sec]
    ChannelStatus channel_status;            //!< channel tracking status
});


/*!
 * RANGE Message Structure
 * This log contains the channel measurements for the
 * currently tracked satellites. When using this log,
 * please keep in mind the constraints noted along with
 * the description.
 */
PACK(
struct RangeMeasurements {
    Oem4BinaryHeader header;			//!< Message header
    int32_t number_of_observations;     //!< Number of ranges observations in the following message
    RangeData range_data[MAX_CHAN];      //!< Range data for each available channel
    uint8_t 	crc[4];						//!< 32-bit cyclic redundancy check (CRC)
});


/*!
 * Compressed Pseudorange Message Structure
 * This log contains the pseudorange information for a
 * single channel. Used in the RangeMeasurements structure.
 */
PACK( 
struct CompressedRangeRecord {
    int64_t doppler:28;                             //!< Doppler frequency [Hz]; SF = 1/256
    uint64_t pseudorange:36;                         //!<  pseudorange [m]; SF = 1/128
    int32_t accumulated_doppler:32;                //!< accumulated doppler [cycles]; SF = 1/256
    uint16_t pseudorange_standard_deviation:4;      //!< pseudorange standard deviation [m]
    uint16_t accumulated_doppler_std_deviation:4;   //!< accumulated doppler standard deviation [cycles]
    uint16_t satellite_prn:8;                       //!< SV PRN number
    uint32_t locktime:21;                           //!< Number of seconds of continuous tracking [sec]
    uint32_t carrier_to_noise:5;                    //!< Signal/Noise [dB-Hz]
    uint32_t reserved:6;
    uint16_t reservedb:16;
}//;
);

PACK(
struct CompressedRangeData {
    ChannelStatus channel_status;                   //!< channel tracking status
    CompressedRangeRecord range_record;
}//;
);

/*!
 * RANGECMP Message Structure
 * This log contains the compressed version of the RANGE log.
 */
PACK(
struct CompressedRangeMeasurements {
    Oem4BinaryHeader header;                        //!< Message header
    int32_t number_of_observations;                 //!< Number of ranges observations in the following message
    CompressedRangeData range_data[MAX_CHAN];       //!< Range data for each available channel
    uint8_t 	crc[4];                             //!< 32-bit cyclic redundancy check (CRC)
});

//*******************************************************************************
// SATELLITE INFORMATION GPS STRUCTURES
//*******************************************************************************


/*!
 * GPSEPHEM Message Structure
 * This log contains a single set of
 * GPS ephemeris parametres.
 */
PACK(
struct GpsEphemeris
{
    Oem4BinaryHeader header;		//!< Message header
    uint32_t prn;                   //!< PRN number
    double time_of_week;            //!< time stamp of subframe 0 (s)
    uint32_t health;                //!< health status, defined in ICD-GPS-200
    uint32_t issue_of_ephemeris_1;  //!< issue of ephemeris data 1
    uint32_t issue_of_ephemeris_2;  //!< issue of ephemeris data 2
    uint32_t gps_week;              //!< GPS week number
    uint32_t z_count_week;          //!< z count week number
    double time_of_ephemeris;       //!< reference time for ephemeris (s)
    double semi_major_axis;         //!< semi major axis (m)
    double mean_motion_difference;  //!< Mean motion difference (rad/s)
    double anomoly_reference_time;  //!< mean anomoly reference time (rad)
    double eccentricity;            //!< eccentricity
    double omega;                   //!< arguement of perigee (rad)
    double latitude_cosine;         //!< arugument of latitude - cos (rad)
    double latitude_sine;           //!< argument of latitude - sine (rad)
    double orbit_radius_cosine;     //!< orbit radius - cos (rad)
    double orbit_radius_sine;       //!< orbit radius - sine (rad)
    double inclination_cosine;      //!< inclination - cos (rad)
    double inclination_sine;        //!< inclination - sine (rad)
    double inclination_angle;       //!< inclination angle (rad)
    double inclination_angle_rate;  //!< rate of inclination angle (rad/s)
    double right_ascension;         //!< right ascension (rad)
    double right_ascension_rate;    //!< rate of right ascension (rad/s)
    uint32_t issue_of_data_clock;   //!< issue of data clock
    double sv_clock_correction;     //!< SV clock correction term (s)
    double group_delay_difference;  //!< estimated group delay difference
    double clock_aligning_param_0;  //!< clock aging parameter 0
    double clock_aligning_param_1;  //!< clock aging parameter 1
    double clock_aligning_param_2;  //!< clock aging parameter 2
    yes_no anti_spoofing;           //!< anti spoofing on
    double corrected_mean_motion;   //!< corrected mean motion
    double range_accuracy_variance; //!< user range accuracy variance
    uint8_t crc[4];                 //!< 32-bit cyclic redundancy check (CRC)
});

/*!
 * RAWEPHEM Message Structure
 * contains the raw binary information for subframes one, two
 * and three from the satellite with the parity information removed.
 * Ephemeris older than 6 hours is not output
 */
PACK(
struct RawEphemeris {
    Oem4BinaryHeader header;
    uint32_t prn;                       //!< Satellite PRN number
    uint32_t ephem_reference_week_num;  //!< Ephemeris reference week number
    uint32_t ephem_reference_seconds;   //!< Ephemeris reference time [sec]
    uint8_t subframe1[30];              //!< Subframe 1 data
    uint8_t subframe2[30];              //!< Subframe 2 data
    uint8_t subframe3[30];              //!< Subframe 3 data
    uint8_t crc[4];                     //!< 32-bit cyclic redundancy check (CRC)
});
PACK(
struct RawEphemerides {
    RawEphemeris ephemeris[MAX_NUM_SAT];
});

/*!
 * RAWALM Message Structure
 * Contains the undecoded almanac subframes as received from the satellite
 */
PACK(
struct RawAlmanacData
{
	uint16_t svid;
    uint8_t subframe[30];			// 30 bytes of subframe page data
});
PACK(
struct RawAlmanac
{
	Oem4BinaryHeader header;
	uint32_t ref_week;
	uint32_t ref_time;			// [sec]
	uint32_t num_of_subframes;	// numbers of subframes to follow
	RawAlmanacData subframe_data;
	uint8_t crc[4];

});

/*!
 * ALMANAC
 * Contains decoded almanac parameters from Subframes 4 and 5 with parity 
 * info removed.
 */
PACK(
struct AlmanacData {
	uint32_t prn;
	uint32_t ref_week;
	double ref_time;					//!< [sec]
	double eccentricity;			
	double right_ascension_rate;		//!< [rad/sec]
	double right_ascension;				//!< [rad]
	double perigee;						//!< [rad]
	double mean_anomoly_of_ref_time;	//!< [rad]
	double clock_aging_param_0;			//!< [sec]
	double clock_aging_param_1;			//!< [sec/sec]
	double corrected_mean_motion;		//!< [rad/sec]
	double semi_major_axis;				//!< [m]
	double inclination_angle;			//!< [rad] Angle of inclination relative to .3*pi
	uint32_t sv_configuration;			//!< 
	uint32_t sv_health;					//!< (6 bits) From Page 25 of subframe 4 or 5
	uint32_t sv_health_from_almanac;	//!< (8 bits) 
	true_false anti_spoofing;			//!< 
});
PACK(
struct Almanac {
	Oem4BinaryHeader header;
	int32_t number_of_prns;
	AlmanacData data[MAX_NUM_SAT];
	uint8_t crc[4];

});

/*!
 * Satellite Position Structure
 * Contains the position of one satellite in ECEF coordinates.
 * Also contains satellite correction information.  Used to make
 * up the SatellitePositions structure
 */
PACK(
struct SatellitePositionData {
    uint32_t satellite_prn;             //!< SV PRN number
    double x_position;                  //!< SV X coordinate [m]
    double y_position;                  //!< SV Y coordinate [m]
    double z_position;                  //!< SV Z coordinate [m]
    double clock_correction;            //!< SV clock correction [m]
    double ionospheric_correction;      //!< ionospheric correction [m]
    double tropospheric_correction;     //!< tropospheric correction [m]
    double dReserved1;                  //!< reserved
    double dReserved2;                  //!< reserved
});

/*!
 * SATXYZ Message Structure
 * When combined with a RANGE log, this data set contains
 * the decoded satellite information necessary to compute the
 * solution: satellite coordinates (ECEF WGS84), satellite clock
 * correction, ionospheric corrections and tropospheric corrections.
 */
PACK(
struct SatellitePositions {
    Oem4BinaryHeader header;				//!< Message header
    double dReserved1;                      //!< Reserved
    uint32_t number_of_satellites;          //!< Number of satellites in following message
    SatellitePositionData data[MAX_CHAN];   //!< Position data for each satellite
    uint8_t 	crc[4];                     //!< 32-bit cyclic redundancy check (CRC)
});

/*!
 * SATVIS Message Structure
 * The SATVIS log is meant to provide a brief overview. The satellite positions
 * and velocities used in the computation of this log are based on Almanac
 * orbital parameters, not the higher precision Ephemeris parameters
 */
PACK(
struct SatelliteVisibilityData {
    int16_t satellite_prn;              //!< SV PRN number
        //!< GPS 1-32
        //!< SBAS 120-138
        //!< GLONASS
    int16_t glonass_frequency;          //!< GLONASS frequency +7
    uint32_t health;                    //!< Satellite Health
    double elevation;                   //!< SV elevation [deg]
    double azimuth;                     //!< SV azimuth [deg]
    double theoretical_doppler;         //!< Theoretical Doppler frequency of SV [Hz]
    double apparent_doppler;            //!< Theoretical Doppler with clock drift correction added [Hz]
});

PACK(
struct SatelliteVisibility {
    Oem4BinaryHeader header;				//!< Message header
    true_false sat_vis;                     //!< Reserved
    true_false complete_almanac_used;       //!< Was Complete almanac used
    uint32_t number_of_satellites;          //!< Number of satellites in following message
    SatelliteVisibilityData data[MAX_CHAN];   //!< Position data for each satellite
    uint8_t 	crc[4];                     //!< 32-bit cyclic redundancy check (CRC)
});


/*!
 * TIME Message Structure
 * This log provides several time related pieces of information
 * including receiver clock offset and UTC time and offset. It
 * can also be used to determine any offset in the PPS signal
 * relative to GPS time. To find any offset in the PPS signal,
 * log the TIME log 'ontime' at the same rate as the PPS output.
 *
 * GPS time = receiver time - offset
 * UTC time = GPS time + offset + UTC offset
 */
PACK(
struct TimeOffset {
    Oem4BinaryHeader header;			//!< Message header
    uint32_t clock_model_status;        //!< ClockModelStatus
    double offset;                      //!< Receiver Offset in seconds from GPS time
    double offset_standard_deviation;   //!< Instantaneous Standard Deviation of Receiver Clock Offset
    double gps_to_utc_offset;           //!< Offset in seconds of GPS time from UTC time
    int32_t utc_year;                   //!< UTC Year
    uint8_t utc_month;                  //!< UTC Month
    uint8_t utc_day;                    //!< UTC Day
    uint8_t utc_hour;                   //!< UTC Hour
    uint8_t utc_minute;                 //!< UTC Minutes
    int32_t utc_millisecond;            //!< UTC Milliseconds
    int32_t utc_status;                 //!< UTC Status
    uint8_t crc[4];                     //!< 32-bit cyclic redundancy check (CRC)
});

/*!
 * TRACKSTAT Message Structure
 * This log provides the Tracking Status information for each
 * receiver channel
 */
struct TrackStatusData {
    uint16_t prn;                       //!< SV prn
    int16_t glonass_frequency;          //!< GLONASS frequency +7
    ChannelStatus channel_track_status; //!< Channel tracking status
    double pseudorange;                 //!< Pseudorange
    float doppler_frequency;            //!< Doppler frequency [Hz]
    float cno_ratio;                    //!< Carrier to noise density ratio [dB-Hz]
    float lock_time;                    //!< Number of seconds of continuous tracking (no cycle slips)
    float pseudorange_residual;         //!< Pseudorange residual from pseudorange filter [m]
    RangeRejectCode range_reject_code;  //!< Range reject code from pseudorange filter
    float pseudorange_weight;           //!< Pseudorange filter weighting
};
struct TrackStatus {
    Oem4BinaryHeader header;            //!< Message header
    SolutionStatus solution_status;     //!< Solution status
    PositionType position_type;         //!< Position type
    float elevation_cutoff_angle;       //!< Tracking elevation cutoff angle
    int32_t number_of_channels;         //!< Number of channels with information following
    TrackStatusData data[MAX_CHAN];     //!< Tracking Status data repeated per channel
    uint8_t crc[4];
};

//*******************************************************************************
// RTK GPS STRUCTURES
//*******************************************************************************

//TODO: CONVERT AND CLEANUP THESE STRCUTURES

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


//*******************************************************************************
// STATUS STRUCTURES
//*******************************************************************************


/*!
 * VERSION Message Structure
 * This log contains the version information for
 * all components of a system. When using a standard
 * receiver, there is only one component in the log. A
 * component may be hardware (for example, a receiver
 * or data collector) or firmware in the form of
 * applications or data (for example, data blocks for
 * height models or user applications).
 *
 */
PACK(
struct Version
{
	Oem4BinaryHeader header;		//!< Message header
	int32_t number_of_components;	//!< Number of components (cards, etc..)
	int32_t component_type;			//!< Component type
	char model[16];  				//!< Base model name
	char serial_number[16];			//!< Product serial number
	char hardware_version[16];  	//!< Hardware version number
	char software_version[16];		//!< firmware version number
	char boost_version[16];  		//!< boot code version
	char compile_date[12];			//!< Firmware compile date
	char compile_time[12];			//!< Firmware compile time
	int8_t crc[4];
});


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



/*!
 * RXSTATUS Message Structure
 * This log conveys various status parameters
 * of the GNSS receiver system. These include
 * the Receiver Status and Error words which contain
 * several flags specifying status and error conditions.
 * If an error occurs (shown in the Receiver Error word)
 * the receiver idles all channels, turns off the antenna,
 * anddisables the RF hardware as these conditions are
 * considered to be fatal errors. The log contains a variable
 * number of status words to allow for maximum flexibility and
 * future expansion.
 */
PACK(
struct RXStatus
{
	Oem4BinaryHeader header;	//!<
	ReceiverError error;		//!< receiver error field
	uint32_t numStats;			//!< number of status messages
	ReceiverStatus rxStat;		//!< receiver status word
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
});


struct RXStatusEvent
{
	Oem4BinaryHeader header;
	StatusWord	status;		// the status word that generated the event message
	uint32_t bitPosition;		// location of the bit in the status word (Table 81, pg 303
	EventType	type;		// event type (Table 86, pg 306)
	int8_t description[32];	// text description of the event or error
};

/*!
 * RXHWLEVELS Message Structure
 * This log contains the receiver environmental and voltage parametres.
 */
struct ReceiverHardwareStatus
{
	Oem4BinaryHeader header;
    float board_temperature;	//!< board temperature in degrees celcius
    float antenna_current;		//!< antenna current (A)
    float core_voltage;         //!< CPU core voltage (V)
    float supply_voltage;       //!< supply voltage(V)
    float rf_voltage;           //!< 5V RF supply voltage(V)
    float lna_voltage;          //!< internal LNA voltage (V)
    float GPAI;                 //!< general purpose analog input
    float reserved1;            //!< Reserved
    float reserved2;            //!< Reserved
    float lna_card_voltage;     //!< LNA voltage (V) at GPSCard output
    int8_t crc[4];              //!< 32-bit crc
};

}


#endif
