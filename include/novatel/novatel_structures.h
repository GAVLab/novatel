// Novatel OEM4 Data Structures
#ifndef NOVATELSTRUCTURES_H
#define NOVATELSTRUCTURES_H

#pragma pack (push, 1)

enum PORT_NAME 
{
   NO_PORTS = 0,
   COM1_ALL = 1,
   COM2_ALL = 2,
   COM3_ALL = 3,
   USB_ALL = 4,
   THISPORT_ALL = 6,
   ALLPORTS = 8,
   COM1 = 32,
   COM1_1,
   COM1_2,
   COM1_3,
   COM1_4,
   COM1_5,
   COM1_6,
   COM1_7,
   COM1_8,
   COM1_9,
   COM1_10,
   COM1_11,
   COM1_12,
   COM1_13,
   COM1_14,
   COM1_15,
   COM1_16,
   COM1_17,
   COM1_18,
   COM1_19,
   COM1_20,
   COM1_21,
   COM1_22,
   COM1_23,
   COM1_24,
   COM1_25,
   COM1_26,
   COM1_27,
   COM1_28,
   COM1_29,
   COM1_30,
   COM1_31,
   COM2 = 64,
   COM2_1,
   COM2_2,
   COM2_3,
   COM2_4,
   COM2_5,
   COM2_6,
   COM2_7,
   COM2_8,
   COM2_9,
   COM2_10,
   COM2_11,
   COM2_12,
   COM2_13,
   COM2_14,
   COM2_15,
   COM2_16,
   COM2_17,
   COM2_18,
   COM2_19,
   COM2_20,
   COM2_21,
   COM2_22,
   COM2_23,
   COM2_24,
   COM2_25,
   COM2_26,
   COM2_27,
   COM2_28,
   COM2_29,
   COM2_30,
   COM2_31,
   COM3 = 96,
   COM3_1,
   COM3_2,
   COM3_3,
   COM3_4,
   COM3_5,
   COM3_6,
   COM3_7,
   COM3_8,
   COM3_9,
   COM3_10,
   COM3_11,
   COM3_12,
   COM3_13,
   COM3_14,
   COM3_15,
   COM3_16,
   COM3_17,
   COM3_18,
   COM3_19,
   COM3_20,
   COM3_21,
   COM3_22,
   COM3_23,
   COM3_24,
   COM3_25,
   COM3_26,
   COM3_27,
   COM3_28,
   COM3_29,
   COM3_30,
   COM3_31,
   USB = 128,
   USB_1,
   USB_2,
   USB_3,
   USB_4,
   USB_5,
   USB_6,
   USB_7,
   USB_8,
   USB_9,
   USB_10,
   USB_11,
   USB_12,
   USB_13,
   USB_14,
   USB_15,
   USB_16,
   USB_17,
   USB_18,
   USB_19,
   USB_20,
   USB_21,
   USB_22,
   USB_23,
   USB_24,
   USB_25,
   USB_26,
   USB_27,
   USB_28,
   USB_29,
   USB_30,
   USB_31,
   THISPORT = 192,
   THISPORT_1, 
   THISPORT_2, 
   THISPORT_3, 
   THISPORT_4, 
   THISPORT_5, 
   THISPORT_6, 
   THISPORT_7, 
   THISPORT_8, 
   THISPORT_9, 
   THISPORT_10,
   THISPORT_11,
   THISPORT_12,
   THISPORT_13,
   THISPORT_14,
   THISPORT_15,
   THISPORT_16,
   THISPORT_17,
   THISPORT_18,
   THISPORT_19,
   THISPORT_20,
   THISPORT_21,
   THISPORT_22,
   THISPORT_23,
   THISPORT_24,
   THISPORT_25,
   THISPORT_26,
   THISPORT_27,
   THISPORT_28,
   THISPORT_29,
   THISPORT_30,
   THISPORT_31,
   MAX_PORT
};

struct OEM4_BINARY_HEADER         // Standard binary header
{
   unsigned char          sop1;          // start of packet first byte
   unsigned char          sop2;          // start of packet second byte
   unsigned char          sop3;          // start of packet third  byte
   unsigned char          header_length; // Length of the header ( From start of packet )
   unsigned short         number;        // Message number
   unsigned char          type;          // Message type
   unsigned char          port_address;  // Address of the data port the log was received on
   unsigned short         length;        // Message length (Not including header or CRC)
   unsigned short         sequence;      // Sequence #
   unsigned char          idle;          // Idle time
   unsigned char          gps_stat;      // GPS Time Status 
   unsigned short         gps_week;      // GPS Week number
   unsigned long          millisecs;     // Milliseconds into week
   unsigned long          status;        // Receiver status word
   unsigned short         Reserved;      // 
   unsigned short         version;       // Receiver software version
};

struct OEM4_BINARY_HEADER_SHORT         // short version of binary header
{
   unsigned char          sop1;          // start of packet first byte
   unsigned char          sop2;          // start of packet second byte
   unsigned char          sop3;          // start of packet third  byte
   unsigned char          message_length; // Length of the 
   unsigned short         number;        // Message number
   unsigned short         gps_week;      // GPS Week number
   unsigned long          millisecs;     // Milliseconds into week
};

enum TIME_STATUS
{
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

//*******************************************************************************
// NOVATEL ENUMS
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

enum PositionType
{
	NONE,
	FIXEDPOS,
	FIXEDHEIGHT,
	Reserved,
	FLOATCONV,
	WIDELANE,
	NARROWLANE,
	DOPPLER_VELOCITY=8,
	SINGLE=16,
	PSRDIFF,
	WAAS,
	PROPOGATED,
	OMNISTAR,
	L1_FLOAT=32,
	IONOFREE_FLOAT,
	NARROW_FLOAT,
	L1_INT=48,
	WIDE_INT,
	NARROW_INT,
	RTK_DIRECT_INS,
	OMNISTAR_HP=64,
	OMNISTAR_XP,
	CDGPS,
	RTK_FIXED_INS = 56
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


enum INSStatus
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

//*******************************************************************************
// MESSAGE STRUCTURES
struct IMUStatus
{
	unsigned counter : 4;
	unsigned IMUTest : 1;
	unsigned ZaxisPathLngCtrl : 1;
	unsigned YaxisPathLngCtrl : 1;
	unsigned XaxisPathLngCtrl : 1;
	unsigned accelTemp : 8;
	unsigned softVerNum : 8;
	unsigned reserved : 3;
	unsigned gyroTest : 1;
	unsigned accelTest : 1;
	unsigned otherTests : 1;
	unsigned memoryTests : 1;
	unsigned processorTests : 1;
};

struct BestPosition
{
	OEM4_BINARY_HEADER header;
	SolutionStatus solutionStatus;
	PositionType positionType;
	double latitude;
	double longitutude;
	double height;
	float undulation;
	DatumID datumID;
	float latStdDev;
	float lonStdDev;
	float heightStdDev;
	char baseStationID[4];
	float diff_age;
	float sol_age;
	unsigned char numObs;
	unsigned char numGPSL1;
	unsigned char numGPSL1aboveRTK;
	unsigned char numGPSL2aboveRTK;
	unsigned char reserved[4];
	unsigned char crc[4];
};

struct BestUTMPosition
{
	OEM4_BINARY_HEADER header;
	SolutionStatus solutionStatus;
	PositionType positionType;
	unsigned long longZoneNumber;
	unsigned long latZoneLetter;
	double northing;
	double easting;
	double hgt;
	float undulation;
	DatumID datumID;
	float northingStdDev;
	float eastingStdDev;
	float heightStdDev;
	char baseStationID[4];
	float diff_age;
	float sol_age;	
	unsigned char numObs;
	unsigned char numGPSL1;
	unsigned char numGPSL1aboveRTK;
	unsigned char numGPSL2aboveRTK;
	unsigned char reserved[4];
	unsigned char crc[4];
};

struct INSPVA
{
	OEM4_BINARY_HEADER header;
	unsigned long gpsWeek;
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
	char crc[4];
};

struct INSPVAS
{
	OEM4_BINARY_HEADER_SHORT header;
	unsigned long gpsWeek;
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
	char crc[4];
};


struct INSUTM
{
	OEM4_BINARY_HEADER header;
	unsigned long gpsWeek;
	double gpsSeconds;
	unsigned long longZoneNumber;
	unsigned long latZoneLetter;
	double northing;
	double easting;
	double height;
	INSStatus status;
	char crc[4];
};

struct VehicleBodyRotation
{
	double xAngle;
	double yAngle;
	double zAngle;
	double xUncertainty;
	double yUncertainty;
	double zUncertainty;
	char crc[4];
};

struct BestVelocity
{
	OEM4_BINARY_HEADER header;
	SolutionStatus solutionStatus;
	PositionType velocityType;
	float latency;		//!< measure of the latency of the velocity time tag in seconds
	float age;			//!< differential age in seconds
	double horizSpeed;	//!< horizontal speed in m/s
	double trackOverGround;	//!< course in degrees
	double verticalSpeed; //!< vertical speed in m/s
	float reserved;
	char crc[4];
};

struct PsrXYZ
{
	OEM4_BINARY_HEADER header;
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
    char            baseStationID[4];    //Base station ID
    float            latency;            //Latency in time tag (s)
    float            diff_age;            //Differential age (s)
    float            sol_age;            //Solution age (s)
    unsigned char            num_obs;            //Number of observations tracked
    unsigned char            numL1used;            //Number of GPS L1 observations used in computation
    unsigned char            reserved[6];        //Reserved
	unsigned char crc[4];
};


struct INSSPD
{
	OEM4_BINARY_HEADER header;
	unsigned long gpsWeek;
	double gpsSeconds;
	double trackOverGround;	//!< actual direction of motion over ground (degrees)
    double horizSpeed;		//!< horizontal speed in m/s
	double verticalSpeed;	//!< vertical speed in m/s
	INSStatus status;
	char crc[4];
};

struct RawIMU
{
	OEM4_BINARY_HEADER header;
	unsigned long gpsWeek;
	double gpsSeconds;
	IMUStatus imuStatus;
	long accZ;	//!< acceleration along z axis in m/s
	long accNegY; //!< -*acceleration along y axis) in m/s
	long accX;	//!< acceleration along x axis in m/s
	long gyroZ;	//!< change in angle around z axis in radians
	long gyroNegY; //!< -(change in angle around y axis) in radians
	long gyroX; //!< change in angle around x axis in radians
	char crc[4];
};
// scale factor for change in angle (1.0/((double)8589934592.0)) for the AG11, AG58, AG17, and AG62
// scale factor for change in velocity (acceleration)
//(0.3048/((double)134217728.0)) for the AG11 and AG58
//(0.3048/((double)67108864.0)) for the AG17 and AG62

struct RawIMUS
{
	OEM4_BINARY_HEADER_SHORT header;
	unsigned long gpsWeek;
	double gpsSeconds;
	IMUStatus imuStatus;
	long accZ;	//!< acceleration along z axis in m/s
	long accNegY; //!< -*acceleration along y axis) in m/s
	long accX;	//!< acceleration along x axis in m/s
	long gyroZ;	//!< change in angle around z axis in radians
	long gyroNegY; //!< -(change in angle around y axis) in radians
	long gyroX; //!< change in angle around x axis in radians
	char crc[4];
};

//BSLNXYZB
struct bslnxyz   //Structure for BSLNXYZ message
{
	OEM4_BINARY_HEADER header;
	SolutionStatus solutionStatus;
	PositionType velocityType;
	double    bslnX;                //Baseline x coordinate (m)
    double    bslnY;                //Baseline y coordinate (m)
    double    bslnZ;                //Baseline z coordinate (m)
    float    sigbslnX;            //Standard deviation of baseline x coordinate (m)
    float    sigbslnY;            //Standard deviation of baseline y coordinate (m)
    float    sigbslnZ;            //Standard deviation of baseline z coordinate (m)
    char    baseStationID[4];    //Base station ID
    unsigned char    num_obs;            //Number of observations tracked
    unsigned char     numL1used;            //Number of GPS L1 observations used in computation
    unsigned char     numL1;                //Number of GPS L1 observations above the RTK mask angle
    unsigned char     numL2;                //Number of GPS L2 observations above the RTK mask angle
    unsigned char     reserved[4];        //Reserved
    unsigned char     crc[4];                //32 bit CRC
};


struct ReceiverError
{
	unsigned int DRAMStatus :1;
	unsigned int invalidFirmware : 1;
	unsigned int ROMStatus : 1;
	unsigned int reserved1 : 1;
	unsigned int ESNaccessStatus : 1;
	unsigned int authorizationCodeStatus : 1;
	unsigned int slowADCStatus : 1;
	unsigned int supplyVoltageStatus : 1;
	unsigned int thermometerStatus : 1;
	unsigned int temperatusStatus : 1;
	unsigned int MINOS4Status : 1;
	unsigned int PLLRf1Status : 1;
	unsigned int PLLRf2Status : 1;
	unsigned int RF1Status : 1;
	unsigned int RF2Status : 1;
	unsigned int NVMStatus : 1;
	unsigned int softwareResourceLimit : 1;
	unsigned int reserved2 : 3;
	unsigned int remoteLoadingBegun : 1;
	unsigned int exportRestriction : 1;
	unsigned int reserved3 : 9;
	unsigned int componentHardwareFailure : 1;

};

struct ReceiverStatus
{
	unsigned int errorFlag : 1;
	unsigned int temperatureStatus : 1;
	unsigned int voltageSupplyStatus : 1;
	unsigned int antennaPowerStatus : 1;
	unsigned int reserved1 : 1;
	unsigned int antennaOpenFlag : 1;
	unsigned int antennaShortedFlag : 1;
	unsigned int CPUoverloadFlag : 1;
	unsigned int COM1overrunFlag : 1;
	unsigned int COM2overrunFlag : 1;
	unsigned int COM3overrunFlag : 1;
	unsigned int USBoverrun : 1;
	unsigned int reserved2 : 3;
	unsigned int RF1AGCStatus : 1;
	unsigned int reserved3 : 1;
	unsigned int RF2AGCStatus : 1;
	unsigned int almanacFlag : 1;
	unsigned int positionSolutionFlag : 1;
	unsigned int positionFixedFlag : 1;
	unsigned int clockSteeringStatus : 1;
	unsigned int clockModelFlag : 1;
	unsigned int extOscillatorFlag : 1;
	unsigned int softwarerResource : 1;
	unsigned int reserved4 : 4;
	unsigned int AUX3statusEventFlag : 1;
	unsigned int AUX2statusEventFlag : 1;
	unsigned int AUX1statusEventFlag : 1;
};


struct RXStatus
{
	OEM4_BINARY_HEADER header;
	ReceiverError error;	//!< receiver error field
	unsigned long numStats;		//!< number of status messages
	ReceiverStatus rxStat;	//!< receiver status word
	unsigned long rxStatPri;
	unsigned long rxStatSet;
	unsigned long rxStatClear;
	unsigned long aux1Stat;		//!< auxiliary 1 status field
	unsigned long aux1Pri;
	unsigned long aux1Set;
	unsigned long aux1Clear;
	unsigned long aux2Stat;
	unsigned long aux2Pri;
	unsigned long aux2Set;
	unsigned long aux2Clear;
	unsigned long aux3Stat;
	unsigned long aux3Pri;
	unsigned long aux3Set;
	unsigned long aux3Clear;
	char crc[4];

};

struct RXStatusEvent
{
	OEM4_BINARY_HEADER header;
	StatusWord	status;		// the status word that generated the event message
	unsigned long bitPosition;		// location of the bit in the status word (Table 81, pg 303
	EventType	type;		// event type (Table 86, pg 306)
	char description[32];	// text description of the event or error
};

struct RXHwLevels
{
	OEM4_BINARY_HEADER header;
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
	char crc[4];			//!< 32-bit crc
};

struct BestGPSPositionTest
{
	OEM4_BINARY_HEADER header;
	unsigned char solutionStatus[4];
	unsigned char positionType[4];
	unsigned char latitude[8];
	unsigned char longitutude[8];
	unsigned char height[8];
	unsigned char undulation[4];
	unsigned char datumID[4];
	unsigned char latStdDev[4];
	unsigned char lonStdDev[4];
	unsigned char heightStdDev[4];
	char baseStationID[4];
	unsigned char diff_age[4];
	unsigned char sol_age[4];
	unsigned char numObs;
	unsigned char numGPSL1;
	unsigned char numGPSL1aboveRTK;
	unsigned char numGPSL2aboveRTK;
	unsigned char reserved[4];
	unsigned char crc[4];
};

enum LOG_TYPE                       // Used by iType
{
   UNKNOWN_LOG_TYPE,                // Unknown logs type
   NOVATEL_ASCII,                   // Novatel ASCII logs
   NOVATEL_BINARY,                  // Novatel Binary logs
   NOVATEL_OEM4_ASCII,              // Novatel ASCII logs
   NOVATEL_OEM4_BINARY,             // Novatel Binary logs
   NOVATEL_OEM4_ABBREVIATED_ASCII,  // Novatel Abbreviated ASCII logs
   NMEA,                            // National Marine Electronics Association logs
   RINEX,                           // Rinex ASCII log
   CARD_RESPONSE,                   // GPSCard response statement
   ERROR_PROMPT,                    // Prompt for an error message
   ACK_PROMPT,                      // Prompt for acknowledge messages (eg. <OK)
   STANDARD_PROMPT                  // Standard Novatel prompt
};

#define MAX_NOUT_SIZE      (8192)   // Maximum size of a NovAtel log buffer (ALMANACA logs are big!)

// !!!!!  Make sure that when you add a new log, it is between the LOGS_START and LOGS_END entries of the appropriate derived parsing class
// !!!!!  When changing this table make sure changes are also made to g_astLogMap in NoutUnknown.CPP
enum NOUT_ID
{
   UNKNOWN,

   // Start of log types
   LOGS_START,
   ACPA,
   AGCA,
   ALMA,
   ATTA,
   BATA,
   BSLA,
   CALA,
   CDSA,
   CLKA,
   CLMA,
   COM1A,
   COM2A,
   CONSOLEA,
   CORA,
   CTSA,
   DCSA,
   DIRA,
   DOPA,
   ETSA,
   FRMA,
   FRWA,
   GALA,
   GCLA,
   GEPA,
   GROUPA,
   GRPA,
   HDRA,
   IONA,
   ISMRA,
   KPHA,
   LPSTATUSA,
   META,
   MKPA,
   MKTA,
   MPMA,
   MSGA,
   NAVA,
   OPTA,
   P20A,
   PAVA,
   PDCA,
   PDCDBG1A,
   PDCVERA,
   PNTA,
   POSA,
   PROJECTA,
   PRTKA,
   PSNA,
   PXYA,
   PVAA,
   RALA,
   RASA,
   RBTA,
   RCCA,
   RCSA,
   REPA,
   RGEA,
   RNGA,
   RPSA,
   RT20A,
   RTCA,
   RTCMA,
   RTCM16T,
   RTKA,
   RTKOA,
   RVSA,
   SATA,
   SBLA,
   SBTA,
   SCHA,
   SFDA,
   SITELOGA,
   SNOA,
   SPHA,
   STATUSA,
   SVDA,
   TM1A,
   UTCA,
   VERA,
   VLHA,
   WALA,
   WUTCA,
   WBRA,
   WRCA,

   ACPB, 
   AGCB, 
   ALMB,
   ATTB,
   BATB,
   BSLB,
   CALB,
   CDSB,
   CLKB,
   CLMB,
   COM1B,
   COM2B,
   CONSOLEB,
   CORB,
   CTSB,
   DCSB,
   DIRB,
   DLLB,
   DOPB,
   ETSB,
   FRMB,
   FRWB,
   GALB,
   GCLB,
   GEPB,
   GROUPB,
   GRPB,
   HDRB,
   IONB,
   ISMRB,
   KPHB,
   LPSTATUSB,
   METB,
   MKPB,
   MKTB,
   MPMB,
   MSGB,
   NAVB,
   OPTB,
   P20B,
   PAVB,
   PDCB,
   PDCDBG1B,
   PDCVERB,
   POSB,
   PROJECTB,
   PRTKB,
   PSNB,
   PVAB,
   PXYB,
   RALB,
   RASB,
   RBTB,
   RCSB,
   REPB,
   RGEB,
   RGEC,
   RGED,
   RPSB,
   RT20B,
   RTCAB,
   RTCMB,
   RTKB,
   RTKOB,
   RVSB,
   SATB,
   SBLB,
   SBTB,
   SCHB,
   SFDB,
   SITELOGB,
   SNOB,
   SPHB,
   STATUSB,
   SVDB,
   TM1B,
   UTCB,
   VERB,
   VLHB,
   WALB,
   WUTCB,
   WBRB,
   WRCB,
   SSOBSL1L2,
   SSOBSL1,
   SSOBSGISMO,
   TAGB,
   DICB,

   GPALM,
   GPGGA,
   GPGLL,
   GPGNS,
   GPGRS,
   GPGSA,
   GPGST,
   GPGSV,
   GPRMB,
   GPRMC,
   GPVTG,
   GPZDA,
   GPZTG,
   GLALM,
   GLGGA,
   GLGLL,
   GLGNS,
   GLGRS,
   GLGSA,
   GLGST,
   GLGSV,
   GLRMB,
   GLRMC,
   GLVTG,
   GLZDA,
   GLZTG,
   GNALM,
   GNGGA,
   GNGLL,
   GNGNS,
   GNGRS,
   GNGSA,
   GNGST,
   GNGSV,
   GNRMB,
   GNRMC,
   GNVTG,
   GNZDA,
   GNZTG,

   PTNL_GGK,

   XOBS,
   XOHD,
   XNHD,
   XNAV,

   ZMESB,
   ZPOSB,
   ZEPHB,
   ZSTNB,
   ZCFGB,
   ZTAGB,

   TESTBED_TIMEA,
   TESTBED_TESTIDA,
   TESTBED_REPORTA,
   TESTBED_MIRRORA,
   TESTBED_TIMEB,
   TESTBED_TESTIDB,
   TESTBED_REPORTB,
   TESTBED_MIRRORB,

      // OEM4
   CLOCKMODELA,
   CLOCKMODELB,
   VERSIONA,
   VERSIONB,
   AVEPOSA,
   AVEPOSB,
   BESTPOSA,
   BESTPOSB,
   BESTUTMA,
   BESTUTMB,
   BESTVELA,
   BESTVELB,
   BSLNXYZA,
   BSLNXYZB,
   MARKPOSA,
   MARKPOSB,
   MATCHEDPOSA,
   MATCHEDPOSB,
   NAVIGATEA,
   NAVIGATEB,
   PASSCOM1A,
   PASSCOM1B,
   PASSCOM2A,
   PASSCOM2B,
   PASSCOM3A,
   PASSCOM3B,
   PSRPOSA,
   PSRPOSB,
   PSRVELA,
   PSRVELB,
   PSRXYZA,
   PSRXYZB,
   PROPAGATEDCLOCKMODELA,
   PROPAGATEDCLOCKMODELB,
   RTKPOSA,
   RTKPOSB,
   RANGEA,
   RANGEB,
   RANGECMPA,
   RANGECMPB,
   RAWEPHEMA,
   RAWEPHEMB,
   RAWGPSSUBFRAMEA,
   RAWGPSSUBFRAMEB,
   REFSTATIONA,
   REFSTATIONB,
   RXHWLEVELSA,
   RXHWLEVELSB,
   RXCONFIGA,
   RXCONFIGB,
   RXSTATUSA,
   RXSTATUSB,
   RXSTATUSEVENTA,
   RXSTATUSEVENTB,
   SATSTATA,
   SATSTATB,
   TIMEA,
   TIMEB,
   TRACKSTATA,
   TRACKSTATB,
   ALMANACA,
   ALMANACB,
   IONUTCA,
   IONUTCB,
   CHANDEBUGA,
   CHANDEBUGB,   
   UNKNOWNOEM4A,                 // Unknown but valid OEM4 ASCII log
   UNKNOWNOEM4B,                 // Unknown but valid OEM4 binary log

   POINTOBS,                     // Point format NCObservation
   POINTEPH,                     // Point format NCOrbit

   PASHR_POS,                    // Ashtech
   // start of INS specific logs
   INSPVAA,
   INSPVAB,
   INSPVASA,
   INSPVASB,
   RAWIMUA,
   RAWIMUB,
   RAWIMUSA,
   RAWIMUSB,
   VEHICLEBODYROTATIONA,
   VEHICLEBODYROTATIONB,
   INSSPDA,
   INSSPDB,
   INSUTMA,
   INSUTMB,
   // end of ins specific logs


   LOGS_END,
   // End of log types


   // Start of Card responses
   FSU_TUNED,
   FSU_DETUNED,
   TRANSPARENTMODE_ON,
   TRANSPARENTMODE_OFF,

   COM1_PROMPT,
   COM2_PROMPT,
   COM3_PROMPT,
   COM1_PROMPT_LF,
   COM2_PROMPT_LF,
   COM3_PROMPT_LF,
   CONSOLE_PROMPT,
   CRLF_PROMPT,

   INVALID_COMMAND_NAME,
   INVALID_NUMBER_OF_ARGUMENTS,
   INVALID_DATA_LOGGER_TYPE,
   INVALID_COMMAND_OPTION,
   INVALID_IN_DATA,
   INVALID_CHANNEL_NUMBER,
   INVALID_SATELLITE_NUMBER,
   INVALID_DOPPLER,
   INVALID_DOPPLER_WINDOW,
   NVM_ERROR,
   OK_ACK,
   // End of Card responses

   // All identities flag  (IMPORTANT:  These should always be last in the enum)
   MAX_NOUT_ID,
   ALL_NOUT_ID
};

#pragma pack (pop) 

enum BINARY_LOG_TYPE
{
  POSB_LOG_TYPE = 1,
  CLKB_LOG_TYPE = 2,
  TM1B_LOG_TYPE = 3,
  MKTB_LOG_TYPE = 4,
  MKPB_LOG_TYPE = 5,
  SPHB_LOG_TYPE = 6,
  DOPB_LOG_TYPE = 7,
  NAVB_LOG_TYPE = 8,
  DCSB_LOG_TYPE = 9,
  RTCM_LOG_TYPE = 10,
  RNGB_LOG_TYPE = 11,      /* obsolete */
  SATB_LOG_TYPE = 12,
  RCSB_LOG_TYPE = 13,
  REPB_LOG_TYPE = 14,
  RALB_LOG_TYPE = 15,
  IONB_LOG_TYPE = 16,
  UTCB_LOG_TYPE = 17,
  ALMB_LOG_TYPE = 18,
  CTSB_LOG_TYPE = 19,
  EPHB_LOG_TYPE = 20,
  SVPB_LOG_TYPE = 21,
  KPHB_LOG_TYPE = 22,
  RQGB_LOG_TYPE = 24,      /* obsolete */
  CMSB_LOG_TYPE = 25,      /* obsolete */
  PXYB_LOG_TYPE = 26,
  GGAB_LOG_TYPE = 27,
  SVCB_LOG_TYPE = 28,
  CONSOLEDATA_LOG_TYPE = 29,  /* bin logs to support      */
  COM1DATA_LOG_TYPE = 30,  /* pass through data logging  */
  COM2DATA_LOG_TYPE = 31,  /* by P. Fenton Mar 94      */
  RGEB_LOG_TYPE = 32,      /* new rngb that has new cstatus and is smaller */
  RGEC_LOG_TYPE = 33,      /* new compressed rngb that has new cstatus and is smaller */
  VLHB_LOG_TYPE = 34,      /* velocity, latency, heading */
  RT20B_LOG_TYPE = 35,     /* matched obs rt20 (posb) */
  SVDB_LOG_TYPE = 36,      /* another sat ECEF and other related range data */
  P20B_LOG_TYPE = 37,
  RTCAB_LOG_TYPE = 38,
  CDSB_LOG_TYPE = 39,      /* new version of CMSB with RTCA */
  MONB_LOG_TYPE = 40,
  RTCM3_LOG_TYPE = 41,
  RTCM9_LOG_TYPE = 42,
  RTCM16_LOG_TYPE = 43,
  RTCM59_LOG_TYPE = 44,
  PVLB_LOG_TYPE = 45,
  DDSB_LOG_TYPE = 46,
  VXYB_LOG_TYPE = 47,
  ETSB_LOG_TYPE = 48,
  PVAB_LOG_TYPE = 49,
  PAVB_LOG_TYPE = 50,
  CLMB_LOG_TYPE = 51,
  RBTB_LOG_TYPE = 52,      /* Raw navigation data */
  SBTB_LOG_TYPE = 53,      /* strange numbers for compatibility with GSV */
  FRMB_LOG_TYPE = 54,
  WBRB_LOG_TYPE = 55,
  RVSB_LOG_TYPE = 56,
  DLLB_LOG_TYPE = 57,
  VERB_LOG_TYPE = 58,

  BSLB_LOG_TYPE = 59,
  RPSB_LOG_TYPE = 60,
  RTKB_LOG_TYPE = 61,
  RTKOB_LOG_TYPE = 62,
  PRTKB_LOG_TYPE = 63,
  OPTB_LOG_TYPE = 64,
  RGED_LOG_TYPE = 65,      // Same as RGEC with a coded pseudorange SD
  RASB_LOG_TYPE = 66,

  WRCB_LOG_TYPE = 67,
  SNOB_LOG_TYPE = 68,

  RTCM1819_LOG_TYPE = 69,
  RTCM2021_LOG_TYPE = 70,
  RTCM22_LOG_TYPE = 71,
  ATTB_LOG_TYPE = 72,
  SBLB_LOG_TYPE = 73,
  AGCB_LOG_TYPE = 74,      // hidden
  ACPB_LOG_TYPE = 75,      // hidden

  FRWB_LOG_TYPE = 79,

  // LogPak PDC specific logs
  PDCB_LOG_TYPE = 76,
  PDCDBG1B_LOG_TYPE = 1999,

  // WAAS specific
  WALB_LOG_TYPE = 81,
  WUTCB_LOG_TYPE = 82,
  CORB_LOG_TYPE = 83,
  MPMB_LOG_TYPE = 95,
  CRLB_LOG_TYPE = 96,

  SFDB_LOG_TYPE = 98,

  // ISM Specific
  ISMRB_LOG_TYPE = 124,

  MSGB_LOG_TYPE = 1024,
  HDRB_LOG_TYPE = 1025,
  GRPB_LOG_TYPE = 1026,
  DIRB_LOG_TYPE = 1027,
  SCHB_LOG_TYPE = 1028,
  LPSTATUSB_LOG_TYPE = 1029,
  SITELOGB_LOG_TYPE = 1030,
  METB_LOG_TYPE = 1031,
  BATB_LOG_TYPE = 1032,
  PSNB_LOG_TYPE = 1033,
  PDCVERB_LOG_TYPE = 1034,
  STATUSB_LOG_TYPE = 1035,
  PROJECTB_LOG_TYPE = 1036,
  GROUPB_LOG_TYPE = 1037,

  DICB_LOG_TYPE = 15400,
  TAGB_LOG_TYPE = 15401,

  // Add new logs here synchronizing numbers with GPSCARD (MINOS1&2) tree


  // leaving the type number for the old RBT, SBT, and FRM binary logs the same.
  IRBTB_LOG_TYPE = 102,    /* Raw navigation data */
  ISBTB_LOG_TYPE = 103,    /* strange numbers for compatibility with GSV */
  IFRMB_LOG_TYPE = 104,

  // Softsurv format observation blocks
  SSOBS_L1L2_LOG_TYPE = 20001,
  SSOBS_L1_LOG_TYPE = 10001,
  SSOBS_GISMO_LOG_TYPE = 10000,  // now obsolete

  // GLONASS logs
  GEPB_LOG_TYPE = 77,
  GALB_LOG_TYPE = 78,
  CALB_LOG_TYPE = 87,
  GCLB_LOG_TYPE = 88,

  // OEM4 logs
  IONUTCB_LOG_TYPE = 8 ,
  CLOCKMODELB_LOG_TYPE = 16,
  RAWGPSSUBFRAMEB_LOG_TYPE = 25,
  CHANDEBUGB_LOG_TYPE = 32,
  VERSIONB_LOG_TYPE = 37,
  RAWEPHEMB_LOG_TYPE = 41,
  BESTPOSB_LOG_TYPE = 42,
  BESTUTMB_LOG_TYPE = 726,
  RANGEB_LOG_TYPE = 43,
  PSRPOSB_LOG_TYPE = 47,
  SATVISB_LOG_TYPE = 48,
  PROPAGATEDCLOCKMODELB_LOG_TYPE = 71,
  ALMANACB_LOG_TYPE = 73,
  RAWALMB_LOG_TYPE = 74,
  TRACKSTATB_LOG_TYPE = 83,
  SATSTATB_LOG_TYPE = 84,
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
  NAVIGATEB_LOG_TYPE = 161,
  AVEPOSB_LOG_TYPE = 172,
  REFSTATIONB_LOG_TYPE = 175,
  PASSCOM1B_LOG_TYPE = 233,
  PASSCOM2B_LOG_TYPE = 234,
  PASSCOM3B_LOG_TYPE = 235,
  BSLNXYZ_LOG_TYPE= 686,
  PSRXYZ_LOG_TYPE = 243,


  //SPAN - INS specific logs
  BESTGPSPOS_LOG_TYPE = 423,
  BESTGPSVEL_LOG_TYPE = 506,
  BESTLEVERARM_LOG_TYPE = 674, 
  INSATT_LOG_TYPE = 263,		//INS ATTITUDE
  INSCOV_LOG_TYPE = 264,
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
  VEHICLEBODYROTATION_LOG_TYPE = 642,



  // OEM4 commands
  FIX_CMD_TYPE = 44,

  // Zeiss logs supported
  ZMESB_LOG_TYPE = 201,
  ZPOSB_LOG_TYPE = 202,
  ZSTNB_LOG_TYPE = 203,
  ZCFGB_LOG_TYPE = 204,
  ZEPHB_LOG_TYPE = 205,
  ZTAGB_LOG_TYPE = 206,

};
typedef enum BINARY_LOG_TYPE BINARY_LOG_TYPE;

#endif