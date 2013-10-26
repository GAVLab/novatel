// Novatel OEM4 enumerations
#ifndef NOVATELENUMS_H
#define NOVATELENUMS_H

#include <stdint.h>  // use fixed size integer types, rather than standard c++ types

namespace novatel {

#define THISPORT 0xc0
//*******************************************************************************
// USER-DEFINED ENUMS
//*******************************************************************************

enum true_false {
    FALSE = 0,
    TRUE = 1
};

enum return_type {
    success,
    fail
};

enum base_type {
    stationary,
    dynamic
};

enum yes_no {
    no,
    yes
};

enum rec_type {
    stand_alone,
    rover_with_static_base,
    static_base_station,
    rover_with_dynamic_base,
    dynamic_base_station
};


//*******************************************************************************
// NOVATEL ENUMS
//*******************************************************************************

enum RangeRejectCode { //!< Used in TRACKSTAT
    GOOD = 0,               //!< Observation is good
    BADHEALTH = 1,          //!< Bad SV health indicated by ephemeris
    OLDEPHEMERIS = 2,       //!< Ephemeris not updated during the las 3 hours
    ECCENTRICANOMALY = 3,   //!< Eccentric anomaly error during computation of SV position
    TRUEANOMALY = 4,        //!< True anomaly error during computation of SV position
    SATCOORDINATEERROR = 5, //!< SV coordinate error during computation of SV position
    ELEVATIONERROR = 6,     //!< Elevation error due to SV below the cut-off angle
    MISCLOSURE = 7,         //!< Misclosure too large due to excessive gap between estimated and actual positions
    NODIFFCORR = 8,         //!< No compatible differential correction is available for this particular satellite
    NOEPHEMERIS = 9,        //!< Ephemeris data not yet received for this SV
    INVALIDIODE = 10,       //!< Invalide IODE (Issue of Data Ephemeris)
    LOCKEDOUT = 11,         //!< SV is excluded by the user (LOCKOUT command)
    LOWPOWER = 12,          //!< SV rejected due to low C/No ratio
    NOIONOCORR = 16,        //!< No ionospheric correction available for this SV
    NOTUSED = 17,           //!< Observation ignored and not used in solution
    NA = 99,                //!< No obseration
    BAD_INTEGRITY = 100,    //!< Integrity of the pseudorange is bad
};

enum AMBIGUITY_TYPE {
    UNDEFINED = 0,
    FLOAT_L1 = 1,
    FLOAT_IONOFREE = 2,
    FLOAT_NARROW = 3,
    NLF_FROM_WL1 = 4,
    INT_L1 = 5,
    INT_WIDE = 6,
    INT_NARROW = 7,
    IONOFREE_DISCRETE = 8
};

enum SEARCHER_TYPE {
    NONE_REQUESTED = 0,
    BUFFERING_MEASUREMENTS = 1,
    SEARCHING = 2,
    COMPLETE = 3,
    HANDOFF_COMPLETE = 4
};


enum TIME_STATUS  {
   GPSTIME_UNKNOWN = 20,
   GPSTIME_APPROXIMATE =60 ,
   GPSTIME_COARSEADJUSTING=80,
   GPSTIME_COARSE=100,
   GPSTIME_COARSESTEERING=120,
   GPSTIME_FREEWHEELING=130,
   GPSTIME_FINEADJUSTING=140,
   GPSTIME_FINE=160,
   GPSTIME_FINESTEERING=180,
   GPSTIME_SATTIME=200,
};


enum LogMode
{
	ONNEW,			//!< does not output current message, but outputs when message is updated
	ONCHANGED,		//!< outputs the current message and then continues to output when the message is changed
	ONTIME,			//!< output on a time interval
	ONNEXT,			//!< output only the next message
	ONCE,			//!< output only the current message
	ONMARK,			//!< output when a pulse is detected on the mark1 input, MK1I
	STOPPED			//!< unlog message
};


enum SolutionStatus
{
	SOL_COMPUTED,		//!< solution computed
	INSUFFICIENT_OBS,	//!< insufficient observations
	NO_CONVERGENCE,		//!< noconvergence
	SINGULARITY,		//!< singularity at parameters matrix
	COV_TRACE,			//!< covariance trace exceeds maximum (trace>1000m)
	TEST_DIST,			//!< test distance exceeded (max of 3 rejections if distance > 10km)
	COLD_START,			//!< not yet converged from cold start
	V_H_LIMIT,			//!< height or velocity limits exceeded 
	VARIANCE,			//!< variance exceeds limits
	RESIDUALS,			//!< residuals are too large
	DELTA_POS,			//!< delta position is too large
	NEGATIVE_VAR,		//!< negative variance
	INTEGRITY_WARNING=13,	//!< large residuals make position unreliable
	INS_INACTIVE,		//!< ins has not started yet
	INS_ALIGNING,		//!< ins doing its coarse alignment
	INS_BAD,			//!< ins position is bad
	IMU_UNPLUGGED,		//!< no imu detected
	PENDING,			//!< when a fix position command is entered, the receiver computes its own position and determines if the fixed position is valid
	INVALID_FIX,		//!< the fixed position entered using the fix position command is not valid
	UNAUTHORIZED
};

enum MessageFormat //!< Bits 5-6 of MessageType struct
{
    BINARY = 0b00,
    ASCII = 0b01,
    ABREVIATED_ASCII = 0b10,
    NMEA = 0b11,
};

enum ResponseBit //!< Last bit (7) of MessageType struct
{
    ORIGINAL_MESSAGE = 0b0,
    RESPONSE_MESSAGE = 0b1,
};

enum PositionType
{
    NONE = 0,
    FIXEDPOS = 1,
    FIXEDHEIGHT = 2,
    Reserved = 3,
    FLOATCONV = 4,
    WIDELANE = 5,
    NARROWLANE = 6,
    DOPPLER_VELOCITY = 8,
    SINGLE = 16,
    PSRDIFF = 17,
    WAAS = 18,
    PROPOGATED = 19,
    OMNISTAR = 20,
    L1_FLOAT = 32,
    IONOFREE_FLOAT = 33,
    NARROW_FLOAT = 34,
    L1_INT = 48,
    WIDE_INT = 49,
    NARROW_INT = 50,
    RTK_DIRECT_INS = 51,
    INS = 52,
    INS_PSRSP = 53,
    INS_PSRDIFF = 54,
    INS_RTKFLOAT = 55,
    INS_RTKFIXED = 56,
    OMNISTAR_HP = 64,
	OMNISTAR_XP = 65,
	CDGPS = 66,
};

enum DatumID
{
	ADIND=1,
	ARC50,
	ARC60,
	AGD66,
	AGD84,
	BUKIT,
	ASTRO,
	CHATM,
	CARTH,
	CAPE,
	DJAKA,
	EGYPT,
	ED50,
	ED79,
	GUNSG,
	GEO49,
	GRB36,
	GUAM,
	HAWAII,
	KAUAI,
	MAUI,
	OAHU,
	HERAT,
	HJORS,
	HONGK,
	HUTZU,
	INDIA,
	IRE65,
	KERTA,
	KANDA,
	LIBER,
	LUZON,
	MINDA,
	MERCH,
	NAHR,
	NAD83,
	CANADA,
	ALASKA,
	NAD27,
	CARIBB,
	MEXICO,
	CAMER,
	MINNA,
	OMAN,
	PUERTO,
	QORNO,
	ROME,
	CHUA,
	SAM56,
	SAM69,
	CAMPO,
	SACOR,
	YACAR,
	TANAN,
	TIMBA,
	TOKYO,
	TRIST,
	VITI,
	WAK60,
	WGS72,
	WGS84,
	ZANDE,
	USER,
	CSRS,
	ADIM,
	ARSM,
	ENW,
	HTN,
	INDB,
	INDI,
	IRL,
	LUZA,
	LUZB,
	NAHC,
	NASP,
	OGBM,
	OHAA,
	OHAB,
	OHAC,
	OHAD,
	OHIA,
	OHIB,
	OHIC,
	OHID,
	TIL,
	TOYM
};


enum InsStatus
{
	INS_STATUS_INACTIVE,
	INS_STATUS_ALIGNING,
	INS_SOLUTION_NOT_GOOD,
	INS_SOLUTION_GOOD,
	INS_TEST_ALIGNING,
	INS_TEST_SOLUTION_GOOD,
	INS_BAD_GPS_AGREEMENT,
	INS_ALIGNMENT_COMPLETE
};

enum StatusWord
{
	RCV_ERROR,		//!< Receiver error word
	RCV_STATUS,		//!< receiver status word
	AUX1,		//!< auxillary 1 status word
	AUX2,		//!< auxillary 2 status word
	AUX3		//!< auxillary 3 status word
};

enum EventType
{
	CLEAR=0,	//!< bit was cleared
	SET=1		//!< bit was set
};

enum PDPSwitch //!< Used in PDPFILTER Command
{
    DISABLE = 0,
    ENABLE = 1,
    RESET = 2,
};

enum PDPMode
{
    NORMAL = 0,
    RELATIVE = 1,
};

enum PDPDynamics
{
    AUTO = 0,       //!< Autodetect dynamics mode
    STATIC = 1,     //!< Static Mode
    DYNAMIC = 2,    //!< Dynamic Mode
};

enum FRESET_TARGET
{
    STANDARD = 0,           //!< [DEFAULT] Clears commands, ephemeris, and almanac
    COMMAND = 1,            //!< Clears saved configuration
    GPSALMANAC = 2,         //!< Clears stored GPS almanac
    GPSEPHEM = 3,           //!< Clears stored GPS ephemeris
    GLOEPHEM = 4,           //!< Clears stored GLONASS ephemeris
    MODEL = 5,              //!< Clears the currently selected model
    CLKCALIBRATION = 11,    //!< Clears parameters entered using CLOCKCALIBRATE command
    SBASALMANAC = 20,       //!< Clears stored SBAS almanac
    LAST_POSITION = 21,     //!< Resets the position using the last stored position
    GLOALMANAC = 31,        //!< Clears the stored GLONASS almanac
    LBAND_TCXO_OFFSET = 38, //!< Removes the TCXO offset information from NVM (not in OEMStar firmware)
};

enum BINARY_LOG_TYPE
{
  // OEM4 logs
  GPSEPHEMB_LOG_TYPE = 7,
  IONUTCB_LOG_TYPE = 8 ,
  CLOCKMODELB_LOG_TYPE = 16,
  VERSIONB_LOG_TYPE = 37,
  RAWEPHEMB_LOG_TYPE = 41,
  BESTPOSB_LOG_TYPE = 42,
  BESTUTMB_LOG_TYPE = 726,
  BESTXYZB_LOG_TYPE = 241,
  RANGEB_LOG_TYPE = 43,
  PSRPOSB_LOG_TYPE = 47,
  SATVISB_LOG_TYPE = 48,
  ALMANACB_LOG_TYPE = 73,
  RAWALMB_LOG_TYPE = 74,
  TRACKSTATB_LOG_TYPE = 83,
  SATSTATB_LOG_TYPE = 84,
  SATXYZB_LOG_TYPE = 270,
  RXSTATUSB_LOG_TYPE = 93,
  RXSTATUSEVENTB_LOG_TYPE = 94,
  RXHWLEVELSB_LOG_TYPE = 195,
  MATCHEDPOSB_LOG_TYPE = 96,
  BESTVELB_LOG_TYPE = 99,
  PSRVELB_LOG_TYPE = 100,
  TIMEB_LOG_TYPE = 101,
  RANGEPNB_LOG_TYPE = 126,
  RXCONFIGB_LOG_TYPE = 128,
  RANGECMPB_LOG_TYPE = 140,
  RTKPOSB_LOG_TYPE = 141,
  RTKDOPB_LOG_TYPE = 952,
  NAVIGATEB_LOG_TYPE = 161,
  AVEPOSB_LOG_TYPE = 172,
  REFSTATIONB_LOG_TYPE = 175,
  PASSCOM1B_LOG_TYPE = 233,
  PASSCOM2B_LOG_TYPE = 234,
  PASSCOM3B_LOG_TYPE = 235,
  BSLNXYZ_LOG_TYPE= 686,
  PSRXYZ_LOG_TYPE = 243,
  PSRDOPB_LOG_TYPE = 174,


  //SPAN - INS specific logs
  BESTGPSPOS_LOG_TYPE = 423,
  BESTGPSVEL_LOG_TYPE = 506,
  BESTLEVERARM_LOG_TYPE = 674, 
  INSATT_LOG_TYPE = 263,		//INS ATTITUDE
  INSCOV_LOG_TYPE = 264,
  INSCOVS_LOG_TYPE = 320,
  INSPOS_LOG_TYPE = 265,
  INSPOSSYNC_LOG_TYPE = 322,
  INSPVA_LOG_TYPE = 507,	// INS POSITION, VELOCITY, AND ATTITUDE
  INSPVAS_LOG_TYPE = 508,	// INS POSITION, VELOCITY, AND ATTITUDE short header
  INSSPD_LOG_TYPE = 266,
  INSUTM_LOG_TYPE = 756,
  INSUPDATE_LOG_TYPE = 757,
  INSVEL_LOG_TYPE = 267,
  RAWIMU_LOG_TYPE = 268,
  RAWIMUS_LOG_TYPE = 325,
  VEHICLEBODYROTATION_LOG_TYPE = 642
};
typedef enum BINARY_LOG_TYPE BINARY_LOG_TYPE;

}

#endif
