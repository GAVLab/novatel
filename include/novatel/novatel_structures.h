// Novatel OEM4 Data Structures
#ifndef NOVATELSTRUCTURES_H
#define NOVATELSTRUCTURES_H

#define MAX_NOUT_SIZE      (8192)   // Maximum size of a NovAtel log buffer (ALMANACA logs are big!)

#pragma pack (push, 1)

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
    char baseStationID[4];
    float diff_age;
    float sol_age;
    unsigned char numObs;
    unsigned char numGPSL1;
    unsigned char numGPSL1aboveRTK;
    unsigned char numGPSL2aboveRTK;
    unsigned char reserved[4];
    unsigned char crc[4];
} __attribute__((packed));

struct bestposb_log {
    OEM4_BINARY_HEADER header;
    SolutionStatus solutionStatus;
    PositionType positionType;
    bestposb_data data;
} __attribute__((packed));
//********************


//********************
//BESTUTMB

struct bestutmb_data {
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
} __attribute__((packed));

struct bestutmb_log {
    OEM4_BINARY_HEADER header;
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
    unsigned char crc[4];
} __attribute__((packed));

struct bestvelb_log {
    OEM4_BINARY_HEADER header;
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
    char baseStationID[4]; //Base station ID
    float latency; //Latency in time tag (s)
    float diff_age; //Differential age (s)
    float sol_age; //Solution age (s)
    unsigned char num_obs; //Number of observations tracked
    unsigned char numL1used; //Number of GPS L1 observations used in computation
    unsigned char numL1overmask[1]; //Number of L1 ranges over mask angle
    unsigned char numL2overmask[1]; //Number of L1 ranges over mask angle
    unsigned char reserved[4]; //Reserved
    unsigned char crc[4]; //32bit CRC
} __attribute__((packed));

struct bestxyzb_log {
    OEM4_BINARY_HEADER header; //Log header
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
    char baseStationID[4]; //Base station ID
    unsigned char num_obs; //Number of observations tracked
    unsigned char numL1used; //Number of GPS L1 observations used in computation
    unsigned char numL1; //Number of GPS L1 observations above the RTK mask angle
    unsigned char numL2; //Number of GPS L2 observations above the RTK mask angle
    unsigned char reserved[4]; //Reserved
    unsigned char crc[4]; //32 bit CRC
} __attribute__((packed));

struct bslnxyzb_log {
    OEM4_BINARY_HEADER header; //Log header
    SolutionStatus solutionStatus; //Solution status
    PositionType positionType; //Baseline type
    bslnxyzb_data data;
} __attribute__((packed));
//********************


//********************
//GPSEPHEMB

struct gpsephemb_data {
    unsigned long prn; //PRN number
    double tow; //time stamp of subframe 0 (s)
    unsigned long health; //health status, defined in ICD-GPS-200
    unsigned long iode1; //issue of ephemeris data 1
    unsigned long iode2; //issue of ephemeris data 2
    unsigned long week; //GPS week number
    unsigned long zweek; //z count week number
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
    unsigned long iodc; //issue of data clock
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
    OEM4_BINARY_HEADER header; //Log header
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
    unsigned long num_wk; //UTC reference week number
    unsigned long tot; //reference time of UTC parameters
    double A0; //UTC constant term
    double A1; //UTC 1st order term
    unsigned long fut_wk; //future week number
    unsigned long num_day; //day number
    long dells; //delta time due to leap seconds
    long fut_dells; //future delta time due to leap seconds
    unsigned long delutc; //time difference
} __attribute__((packed));

struct ionutcb_log {
    OEM4_BINARY_HEADER header; //Log header
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
    long num_prn; //Number of PRNs to follow
} __attribute__((packed));

struct psrdopb_log {
    OEM4_BINARY_HEADER header; //Log header
    psrdopb_data data;
    unsigned long prn[12];
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
    char baseStationID[4]; //Base station ID
    float diff_age; //Differential age (s)
    float sol_age; //Solution age (s)
    unsigned char num_obs; //Number of observations tracked
    unsigned char numL1used; //Number of GPS L1 observations used in computation
    unsigned char reserved[6]; //Reserved
    char crc[4]; //32 bit CRC
} __attribute__((packed));

struct psrposb_log {
    OEM4_BINARY_HEADER header; //Log header
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
    unsigned char reserved[4]; //Reserved
    unsigned char crc; //32bit CRC
} __attribute__((packed));

struct psrvelb_log {
    OEM4_BINARY_HEADER header; //Log header
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
    char baseStationID[4]; //Base station ID
    float latency; //Latency in time tag (s)
    float diff_age; //Differential age (s)
    float sol_age; //Solution age (s)
    unsigned char num_obs; //Number of observations tracked
    unsigned char numL1used; //Number of GPS L1 observations used in computation
    unsigned char reserved[6]; //Reserved
    unsigned char crc; //32bit CRC
} __attribute__((packed));

struct psrxyzb_log {
    OEM4_BINARY_HEADER header; //Log header
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
    unsigned short svprn; // PRN
    unsigned short frequency; // Frequency number of GLONASS SV (0 for GPS)
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
    OEM4_BINARY_HEADER hdr;
    unsigned long iObs; // Number of observations
    rangeb_data data[MAXCHAN];
} __attribute__((packed));
//********************


//********************
//RANGECMPB

struct compressed_data {
    signed dop : 28;
    unsigned long long psr : 36;
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
    OEM4_BINARY_HEADER hdr;
    unsigned long iObs; // Number of observations
    rangecmpb_data data[MAXCHAN]; // packed binary range data
    char crc[4];
} __attribute__((packed));
//********************


//********************
//RAWEPHEMB
//struct rawephem_data
//{
//   unsigned long  prn;              // Satellite PRN number
//   unsigned long  ereferweek;       // Ephemeris Reference Week
//   unsigned long  erefertime;       // Ephemeris Reference Time
//   char           subframe1[30];    // Ephemeris subframe 1
//   char           subframe2[30];    // Ephemeris subframe 2
//   char           subframe3[30];    // Ephemeris subframe 3
//   unsigned short filler1;
//   unsigned short filler2;
//} __attribute__ ((packed));
//
//struct rawephem_log
//{
//   OEM4_BINARY_HEADER      hdr;
//   rawephem_data           data;
//} __attribute__ ((packed));
//********************


//********************
//RTCADATA1B

struct rtcadata1b_header {
    double zcount; //Week number from subframe one of the ephemeris
    unsigned char aeb; //Acceleration error bound
    unsigned long num_prn; //Number of satellite corrections with info to follow
} __attribute__((packed));

struct rtcadata1b_data {
    unsigned long prn; //PRN number of range measurement
    double range; //pseudorange correction (m)
    unsigned char iode; //Issue of ephemeris data
    double rrate; //pseudorange rate correction (m/s)
    float udre; //user differential range error
} __attribute__((packed));

struct rtcadata1b_log {
    OEM4_BINARY_HEADER header; //Log header
    rtcadata1b_header info;
    rtcadata1b_data data[MAX_NUM_SAT];
} __attribute__((packed));

//********************


//********************
//RTCADATAEPHEMB

struct rtcadataephemb_data {
    unsigned char des; //Novatel designator
    unsigned char subtype; //RTCA message subtype
    unsigned long week; //GPS week number
    unsigned long sec; //Seconds into week
    unsigned long prn; //PRN number
    char reserved[4];
    char ephem[92]; //Raw ephemeris data
} __attribute__((packed));

struct rtcadataephemb_log {
    OEM4_BINARY_HEADER header; //Log header
    rtcadataephemb_data data;
} __attribute__((packed));
//********************


//********************
//RTCADATAOBSB - CHECK

struct rtcadataobsb_header {
    unsigned char des; //Novatel designator
    unsigned char subtype; //RTCA message subtype
    double min_psr; //minimum pseudorange
    float sec; //seconds into GPS week
    char reserved[4];
    unsigned long num_ids; //Number of transmitter ids with info to follow
} __attribute__((packed));

struct rtcadataobsb_data //Structure for RTCADATAEPHEM message
{
    unsigned char transID; //Transmitter ID
    unsigned char L1lock; //L1 lock flag
    unsigned char L2lock; //L2 lock flag
    double L1psr; //L1 pseudorange offset
    double L2psr; //L2 pseudorange offset
    float L1adr; //L1 carrier phase offset, accumulated doppler range
    float L2adr; //L2 carrier phase offset, accumulated doppler range
    yes_no L2encrypt; //If L2 is encrypted
    char reserved[4];
} __attribute__((packed));

struct rtcadataobsb_log {
    OEM4_BINARY_HEADER header; //Log header
    rtcadataobsb_header info;
    rtcadataobsb_data data[MAX_NUM_SAT]; //WT:  This is probably too many... need to verify how many id's can be sent.
} __attribute__((packed));
//********************


//********************
//RTCADATAREFB

struct rtcadatarefb_data {
    unsigned char des; //Novatel designator
    unsigned char subtype; //RTCA message subtype
    double posX; //base station X coordinate position (mm)
    double posY; //base station Y coordinate position (mm)
    double posZ; //base station Z coordinate position (mm)
    char reserved[4];
} __attribute__((packed));

struct rtcadatarefb_log {
    OEM4_BINARY_HEADER header; //Log header
    rtcadatarefb_data data;
} __attribute__((packed));
//********************


//********************
//RTKDATAB

struct rtkdatab_header {
    unsigned long rtkinfo; //RTK information
    unsigned char num_obs; //Number of observations tracked
    unsigned char gps_l1_ranges; // Number of GPS L1 ranges used
    unsigned char gps_l1_mask_ranges; // Number of GPS L1 ranges above the RTK mask angle used
    unsigned char gps_l2_mask_ranges; // Number of GPS L2 ranges above the RTK mask angle used
    unsigned char reserved[4];
    SEARCHER_TYPE searcher_type; // Searcher type
    unsigned long num_lane_combs; // Number of possible lane combinations
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
    unsigned long ref_prn; // Reference PRN
    long num_svs; // The number of SVs in data portion
} __attribute__((packed));

struct rtkdatab_data {
    unsigned long prn; // GPS satellite PRN
    AMBIGUITY_TYPE ambiguity_type; // Type of ambiguity
    float residual; // Satellite health
} __attribute__((packed));

struct rtkdatab_log {
    OEM4_BINARY_HEADER hdr;
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
    char baseStationID[4]; //Base station ID
    float diff_age; //Differential age (s)
    float sol_age; //Solution age (s)
    unsigned char num_obs; //Number of observations tracked
    unsigned char numL1used; //Number of GPS L1 ranges used in computation
    unsigned char numL1mask; //Number of GPS L1 ranges above RTK mask
    unsigned char numL2mask; //Number of GPS L2 ranges above RTK mask
    unsigned char reserved[4]; //Reserved
    unsigned char crc[4]; //32 bit CRC
} __attribute__((packed));

struct rtkposb_log {
    OEM4_BINARY_HEADER header; //Log header
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
    unsigned char reserved[4]; //Reserved
    unsigned char crc; //32bit CRC
} __attribute__((packed));

struct rtkvelb_log {
    OEM4_BINARY_HEADER header; //Log header
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
    char baseStationID[4]; //Base station ID
    float latency; //Latency in time tag (s)
    float diff_age; //Differential age (s)
    float sol_age; //Solution age (s)
    unsigned char num_obs; //Number of observations tracked
    unsigned char numL1used; //Number of GPS L1 ranges used in computation
    unsigned char numL1mask; //Number of GPS L1 ranges above RTK mask
    unsigned char numL2mask; //Number of GPS L2 ranges above RTK mask
    unsigned char reserved[4]; //Reserved
    unsigned char crc[4]; //32 bit CRC
} __attribute__((packed));

struct rtkxyzb_log {
    OEM4_BINARY_HEADER header; //Log header
    SolutionStatus solutionStatus; //Position solution status
    PositionType positionType; //Position type
    rtkxyzb_data data;
} __attribute__((packed));
//********************


//********************
//SATXYZB

struct satxyzb_header {
    double dReserved1;
    unsigned long ulNumSats; // number of satellites
} __attribute__((packed));

struct satxyzb_data {
    unsigned long ulPRN; // SV PRN
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
    OEM4_BINARY_HEADER oem4hdr;
    satxyzb_header header;
    satxyzb_data data[MAXCHAN];
} __attribute__((packed));
//********************


//********************
//TIMEB

struct time_data {
    unsigned long ulClockModel; // ClockModelStatus
    double dGPSOffset; // Receiver Offset in seconds from GPS time
    double dOffsetStd; // Instantaneous Standard Deviation of Receiver Clock Offset
    double dUtcOffset; // Offset in seconds of GPS time from UTC time
    long lUtcYear; // UTC Year
    unsigned char ucUtcMonth; // UTC Month
    unsigned char ucUtcDay; // UTC Day
    unsigned char ucUtcHour; // UTC Hour
    unsigned char ucUtcMin; // UTC Minutes
    long lUtcMillisec; // UTC Milliseconds
    int bUtcStatus; // UTC Status
} __attribute__((packed));

struct timeb_log {
    OEM4_BINARY_HEADER hdr;
    time_data body;
} __attribute__((packed));
//********************


//********************
//VISIONSOLB

struct visionsolb_data {
    unsigned short channel; //Channel tracking number
    unsigned short prn; //Prn number of range measurement
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
    unsigned long num_vis; //number of vision solutions with info to follow
} __attribute__((packed));

struct visionsolb_log {
    OEM4_BINARY_HEADER hdr;
    visionsolb_header info;
    visionsolb_data data[MAX_NUM_SAT];
} __attribute__((packed));
//********************

#pragma pack (pop)

#endif
