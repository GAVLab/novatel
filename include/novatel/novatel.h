#ifndef NOVATEL_H
#define NOVATEL_H

#define BOOST_ASIO_NO_WIN32_LEAN_AND_MEAN
#include "novatel_structures.h"

// define message ids
#define NEWINSPVA 0
#define NEWRXHWLEVELS 1
#define NEWBESTUTM 2
#define NEWRAWIMU 3
#define NEWBESTPOS 4
#define NEWBESTVELOCITY	5
#define NEWINSSPD 6
#define NEWINSUTM 7
#define NEWRXSTATUS 8
#define NEWRXSTATUSEVENT 9
#define	NEWVEHICLEBODYROTATION 10
#define NEWRAWIMUS 11
#define NEWBSLNXYZ 12
#define NEWPSRXYZ 13
#define CMDACK 14


class Novatel
{
public:
	Novatel();
	~Novatel();

	//************************************************************************
	//! array of event handles to signal when new messages have arrived
	//HANDLE hNovatelEvents[15];

	//! Initializes com port
	/*! Creates instance of com port object
	 *  \param comPort com port number to listen on
	 *	\param baudRate
	 *  \return true if initialized successfully, false otherwise */
	//bool Initialize(int comPortNumber, int baudRate);
	//bool Initialize(int comPortNumber, int baudRate, int byteSize,int parity,int stopSits);
	//! Starts listening for incoming data
	//bool StartReading();
	//! stops listening for incoming data
	//bool StopReading();

	//! shutsdown device and closes serial port
	//bool Shutdown();

	//! Indicates whether object has been initialized successfully or not
	/*! Returns true if successfully initialized, false otherwise */
	//bool isActive(){return bActive;};
	//! Indicates status of read thread
	/*! Returns true if read thread is running, false otherwise */
	//bool isReading(){return bReading;};

	//! Sends the specified command to the SPAN and waits for an acknowledgement
	//bool SendMessage(char* msg, unsigned int length);

	//bool SendString(char* msg);


	//************************************************************************
	// PUBLIC METHODS TO GET LAST READ LOGS
	//! Method to get structure containing last BESTPOS log
	BestPosition getLastBestPOS();
	//! Method to get structure containing last BestUTM log
	BestUTMPosition getLastBestUTM();
	//! Method to get structure containing last INSPVA log
	INSPVA getLastINSPosVelAtt();
	//! Method to get structure containing last PsrXYZ log
	PsrXYZ getLastPsrXYZ();
	//! Method to get structure containing last INSPVA log
	INSPVAS getLastINSPosVelAttShort();
	//! Method to get last baseline message
	bslnxyz getLastBslnXYZ();
	//! Method to get structure containing last RAWIMU log
	RawIMU getLastRawIMU();
	//! Method to get structure containing last RAWIMUS log
	RawIMUS getLastRawIMUS();
	//! Method to get structure containing last VEHICLEBODYROTATION log
	VehicleBodyRotation getLastVehicleBodyRotation();
	//! Method to get structure containing last VEHICLEBODYROTATION log
	BestVelocity getLastBestVelocity();
	//! Method to get structure containing last INSSPD log
	INSSPD getLastINSSpeed();
	//! Method to get structure containing last INSUTM log
	INSUTM getLastINSUTM();
	//! Method to get structure containing last RXStatus log
	RXStatus getLastRXStatus();
	//! Method to get structure containing last RXStatusEvent log
	RXStatusEvent getLastRXStatusEvent();
	//! Method to get structure containing last RXHWLevels log
	RXHwLevels getLastRXHwLevels();

	//************************************************************************
	// SPAN COMMANDS
	//! Send command to SPAN to stop sending all logs
	bool UnlogAll();

	NOUT_ID DecodeBinaryID(unsigned int msgID);  // Member function to identify the log

	void InitializeEvents();

private:

	void bufferIncomingData(unsigned char* msg, unsigned int length);
	//! Function to parse logs into a usable structure
	void ParseLog(unsigned char* log, NOUT_ID logID);
	unsigned char dataBuf[MAX_NOUT_SIZE];
	unsigned char* dataRead;
	unsigned int bytesRemaining;
	unsigned int bufIndex;

	bool readingACK;
 
	void opened();
	void closed();
	bool startedReading();
	bool stoppedReading();

	//*******************************************************************
	//DATA STRUCTURES TO HOLD LAST READ LOGS
	//! Last received BESTPOS log
	BestPosition curBestPosition;
	//! Mutex to control access to bestgpspos structure
//	HANDLE hBestPOSMutex;
//
//	//! Last received INSPVA log
//	INSPVA curINSPosVelAtt;
//	//! Mutex to control access to INSPVA structure
//	HANDLE hINSPVAMutex;
//
//	//! Last received PsrXYZ log
//	PsrXYZ curPsrXYZ;
//	//! Mutex to control access to PsrXYZ structure
//	HANDLE hPsrXYZMutex;
//
//	//! Last received INSPVAS log
//	INSPVAS curINSPosVelAttShort;
//	//! Mutex to control access to INSPVAS structure
//	HANDLE hINSPVASMutex;
//
//	//! Last received BslnXYZ log
//	bslnxyz curBslnXYZ;
//	//! Mutex to control access to BslnXYZ structure
//	HANDLE hBslnXYZMutex;
//
//	//! Last received RAWIMU log
//	RawIMU curRawIMU;
//	//! Mutex to control access to RAWIMU structure
//	HANDLE hRawIMUMutex;
//
//	//! Last received RAWIMUS log
//	RawIMUS curRawIMUS;
//	//! Mutex to control access to RAWIMUS structure
//	HANDLE hRawIMUSMutex;
//
//
//	//! Last received BESTUTM log
//	BestUTMPosition curBestUTMPosition;
//	//! Mutex to control access to bestgpspos structure
//	HANDLE hBestUTMMutex;
//
//	//! Last received VehicleBodyRotation log
//	VehicleBodyRotation curVehicleBodyRotation;
//	//! Mutex to control access to VehicleBodyRotation structure
//	HANDLE hVehicleBodyRotationMutex;
//
//	//! Last received BestVelocity log
//	BestVelocity curBestVelocity;
//	//! Mutex to control access to BestVelocity structure
//	HANDLE hBestVelocityMutex;
//
//	//! Last received INSSPD log
//	INSSPD curINSSPD;
//	//! Mutex to control access to INSSPD structure
//	HANDLE hINSSPDMutex;
//
//	//! Last received INSUTM log
//	INSUTM curINSUTM;
//	//! Mutex to control access to INSUTM structure
//	HANDLE hINSUTMMutex;
//
//	//! Last received RXStatus log
//	RXStatus curRXStatus;
//	//! Mutex to control access to RXStatus structure
//	HANDLE hRXStatusMutex;
//
//	//! Last received RXStatusEvent log
//	RXStatusEvent curRXStatusEvent;
//	//! Mutex to control access to RXStatusEvent structure
//	HANDLE hRXStatusEventMutex;
//
//	//! Last received RXHwLevels log
//	RXHwLevels curRXHwLevels;
//	//! Mutex to control access to RXHwLevels structure
//	HANDLE hRXHwLevelsMutex;

	int hdrLength;
	unsigned int msgID;

	//! Defines an object to provide low level interaction with the serial port
	//com_port_event_driven comPort;

	//int portNumber;
	//int baudRate;

	//! Function to start thread to listen for incoming com data
	/*! static function used to call thread to handle new span measurements
	 *  these are necessary because the callback function used to create a thread
	 *	cannot call a member function, because member functions are passed a hidden
	 *	argument (this), workaround using a static method and pointer to current object */
	//static DWORD WINAPI ComPortListenThread(LPVOID Parameter)
	//{
	//	// cast pointer and call actual thread method(which is a member function)
	//	((Novatel*)Parameter)->ComListen();
	//	return 1;
	//};

	////! thread function to read incoming com messages and parse them into a message structure
	//void ComListen();

	////! handle to read thread
	//HANDLE hReadThread;
	////! handle to event to signal that read thread has started
	//HANDLE hReadStarted;

	////! indicates whether com port has been initialized or not
	//bool bActive;
	////! indicates whether read thread is running or not
	//bool bReading;

};

#endif
