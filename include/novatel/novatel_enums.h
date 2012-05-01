// Novatel OEM4 enumerations
#ifndef NOVATELENUMS_H
#define NOVATELENUMS_H

#include <stdint.h>  // use fixed size integer types, rather than standard c++ types


//*******************************************************************************
// USER-DEFINED ENUMS
//*******************************************************************************


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
