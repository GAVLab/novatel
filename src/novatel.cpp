#include "novatel.h"
#include <iostream>
#include <fstream>
using namespace std;

Novatel::Novatel()
{
	// create mutexes to control access to most recent read data
	hBestPOSMutex=CreateMutex(0,false,"BestPOSMutex");
	hBestUTMMutex=CreateMutex(0,false,"BestUTMMutex");
	hINSPVAMutex=CreateMutex(0,false,"INSPVA Mutex");
	hINSPVASMutex=CreateMutex(0,false,"INSPVAS Mutex");
	hRawIMUMutex=CreateMutex(0,false,"Raw IMU Mutex");
	hRawIMUSMutex=CreateMutex(0,false,"Raw IMU Short Mutex");
	hVehicleBodyRotationMutex=CreateMutex(0,false,"Vehicle Body Rotation Mutex");
	hBestVelocityMutex=CreateMutex(0,false,"Best Velocity Mutex");
	hINSSPDMutex=CreateMutex(0,false,"INS Speed Mutex");
	hINSUTMMutex=CreateMutex(0,false,"INS UTM Mutex");
	hRXStatusMutex=CreateMutex(0,false,"RX Status Mutex");
	hRXStatusEventMutex=CreateMutex(0,false,"RX Status Event Mutex");
    hRXHwLevelsMutex=CreateMutex(0,false,"RX Hardware Levels");
	hBslnXYZMutex=CreateMutex(0,false,"Baseline Measurment");
	hPsrXYZMutex=CreateMutex(0,false,"ECEF Position");

	InitializeEvents();

	bufIndex=0;
	readingACK=false;
	dataRead = NULL;

	/*bActive=false;
	bReading=false;
	hReadStarted=CreateEvent(0, false, false, "Read Started");*/

}

Novatel::~Novatel()
{
	//comPort.close();

	int sz = sizeof(hNovatelEvents);

	// close event handles
	for (int ii=0; ii<sizeof(hNovatelEvents)/sizeof(HANDLE); ii++)
		CloseHandle(hNovatelEvents[ii]);

	if(dataRead)
		delete[] dataRead;
	//CloseHandle(hReadStarted);

	// close mutex handles
	CloseHandle(hBestPOSMutex);
	CloseHandle(hBestUTMMutex);
	CloseHandle(hINSPVAMutex);
	CloseHandle(hINSPVASMutex);
	CloseHandle(hRawIMUMutex);
	CloseHandle(hRawIMUSMutex);
	CloseHandle(hVehicleBodyRotationMutex);
	CloseHandle(hBestVelocityMutex);
	CloseHandle(hINSSPDMutex);
	CloseHandle(hINSUTMMutex);
	CloseHandle(hRXStatusMutex);
	CloseHandle(hRXStatusEventMutex);
    CloseHandle(hRXHwLevelsMutex);
	CloseHandle(hBslnXYZMutex);
	CloseHandle(hPsrXYZMutex);
}

void Novatel::InitializeEvents()
{
	//// create events to signal when new logs have arrived
	hNovatelEvents[NEWBESTPOS] = CreateEvent(0,false,false, NULL);
	hNovatelEvents[NEWBESTUTM] = CreateEvent(0,false,false, NULL);
	hNovatelEvents[NEWINSPVA] = CreateEvent(0,false,false, NULL);
	//hNovatelEvents[NEWINSPVA] = CreateEvent(0,false,false, NULL);
	hNovatelEvents[NEWRAWIMU] = CreateEvent(0,false,false,NULL);
	hNovatelEvents[NEWBSLNXYZ] = CreateEvent(0,false,false,NULL);
	hNovatelEvents[NEWRAWIMUS] = CreateEvent(0,false,false,NULL);
	hNovatelEvents[NEWVEHICLEBODYROTATION] = CreateEvent(0,false,false,NULL);
	hNovatelEvents[NEWBESTVELOCITY] = CreateEvent(0,false,false,NULL);
	hNovatelEvents[NEWINSSPD] = CreateEvent(0,false,false,NULL);
	hNovatelEvents[NEWINSUTM] = CreateEvent(0,false,false,NULL);
	hNovatelEvents[CMDACK] = CreateEvent(0,false,false,NULL);
	hNovatelEvents[NEWRXSTATUS] = CreateEvent(0,false,false,NULL);
	hNovatelEvents[NEWRXSTATUSEVENT] = CreateEvent(0,false,false,NULL);
	hNovatelEvents[NEWRXHWLEVELS] = CreateEvent(0,false,false,NULL);
	hNovatelEvents[NEWPSRXYZ] = CreateEvent(0,false,false,NULL);
}

bool Novatel::startedReading()
{
	//InitializeEvents();
	return true;
}

void Novatel::opened()
{
	// turn off all logs from GPS
	//UnlogAll();
}

void Novatel::bufferIncomingData(unsigned char *msg, unsigned int length)
{

	//cout << "Received data: " << dec <<length << endl;
	//cout << msg << endl;

	// add incoming data to buffer
	for (unsigned int i=0; i<length; i++)
	{
		//cout << hex << (int)msg[i] << endl;
		// make sure bufIndex is not larger than buffer
		if (bufIndex>=MAX_NOUT_SIZE)
		{
			bufIndex=0;
			MessageDisplay::Instance().DisplayMessage("Novatel: Overflowed receive buffer. Reset",1);
		}

		if (bufIndex==0)
		{	// looking for beginning of message
			if (msg[i]==0xAA)
			{	// beginning of msg found - add to buffer
				dataBuf[bufIndex++]=msg[i];
				bytesRemaining=0;
			}	// end if (msg[i]
			else if (msg[i]=='<')
			{
				// received beginning of acknowledgement
				readingACK=true;
				bufIndex=1;
			}
			else
			{
				//MessageDisplay::Instance().DisplayMessage("Novatel: Received unknown data.\r\n", 1);
			}
		} // end if (bufIndex==0)
		else if (bufIndex==1)
		{	// verify 2nd character of header
			if (msg[i]==0x44)
			{	// 2nd byte ok - add to buffer
				dataBuf[bufIndex++]=msg[i];
			}
			else if ((msg[i]=='O')&&readingACK)
			{
				// 2nd byte of acknowledgement
				bufIndex=2;
			}
			else
			{
				// start looking for new message again
				bufIndex=0;
				bytesRemaining=0;
				readingACK=false;
			} // end if (msg[i]==0x44)
		}	// end else if (bufIndex==1)
		else if (bufIndex==2)
		{	// verify 3rd character of header
			if (msg[i]==0x12)
			{	// 2nd byte ok - add to buffer
				dataBuf[bufIndex++]=msg[i];
			}
			else if ((msg[i]=='K')&&(readingACK))
			{
				// final byte of acknowledgement received
				bufIndex=0;
				readingACK=false;
				// ACK received
				SetEvent(hNovatelEvents[CMDACK]);
				//cout << "ACK Received" << endl;
			}
			else
			{
				// start looking for new message again
				bufIndex=0;
				bytesRemaining=0;
				readingACK=false;
			} // end if (msg[i]==0x12)
		}	// end else if (bufIndex==2)
		else if (bufIndex==3)
		{	// number of bytes in header - not including sync
			dataBuf[bufIndex++]=msg[i];
			// length of header is in byte 4
			hdrLength=msg[i];
		}
		else if (bufIndex==5)
		{	// message id
			// add byte to buffer
			dataBuf[bufIndex++]=msg[i];
			bytesRemaining--;

			msgID=((dataBuf[bufIndex-1])<<8)+dataBuf[bufIndex-2];
		}
		else if (bufIndex==8)
		{	// set number of bytes
			dataBuf[bufIndex++]=msg[i];
			// length of message is in byte 8
			// bytes remaining = remainder of header  + 4 byte checksum + length of body
			// TODO: added a -2 to make things work right, figure out why i need this
			bytesRemaining=msg[i]+4+(hdrLength-7)-2;
		}
		else if (bytesRemaining==1)
		{	// add last byte and parse
			dataBuf[bufIndex++]=msg[i];
			ParseLog(dataBuf,DecodeBinaryID(msgID));
			// reset counters
			bufIndex=0;
			bytesRemaining=0;
		}  // end else if (bytesRemaining==1)
		else
		{	// add data to buffer
			dataBuf[bufIndex++]=msg[i];
			bytesRemaining--;
		}
	}	// end for
}

void Novatel::ParseLog(unsigned char *log, NOUT_ID logID)
{

	 //cout << "Parsing Log: " << logID << endl;
	//cout << "Parsing Log: " << logID << " => " << parser.GetIDStr(logID) << endl;

	switch (logID)
	{
		case BESTPOSB:
			// wait for mutex
			WaitForSingleObject(hBestPOSMutex,INFINITE);
			// copy data read in into best position structure
			memcpy(&curBestPosition, log, sizeof(curBestPosition));
			// release mutex
			ReleaseMutex(hBestPOSMutex);
			// raise event to indicate that new log is available
			SetEvent(hNovatelEvents[NEWBESTPOS]);
			break;
		case BESTUTMB:
			// wait for mutex
			WaitForSingleObject(hBestUTMMutex,INFINITE);
			// copy data read in into best UTM position structure
			memcpy(&curBestUTMPosition, log, sizeof(curBestUTMPosition));
			// release mutex
			ReleaseMutex(hBestUTMMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWBESTUTM]);
			break;
		case BESTVELB:
			// wait for mutex
			WaitForSingleObject(hBestVelocityMutex,INFINITE);
			// copy data read in into best UTM position structure
			memcpy(&curBestVelocity, log, sizeof(curBestVelocity));
			// release mutex
			ReleaseMutex(hBestVelocityMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWBESTVELOCITY]);
			break;
		case PSRXYZB:
			// wait for mutex
			WaitForSingleObject(hPsrXYZMutex,INFINITE);
			// copy data read in into structure
			memcpy(&curPsrXYZ, log, sizeof(curPsrXYZ));
			// release mutex
			ReleaseMutex(hPsrXYZMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWPSRXYZ]);
			break;
		case INSPVAB:
			// wait for mutex
			WaitForSingleObject(hINSPVAMutex,INFINITE);
			// copy data read in into structure
			memcpy(&curINSPosVelAtt, log, sizeof(curINSPosVelAtt));
			// release mutex
			ReleaseMutex(hINSPVAMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWINSPVA]);
			//SetEvent(hNewINSPosVelAtt);
			break;
		case INSPVASB:
			// wait for mutex
			WaitForSingleObject(hINSPVASMutex,INFINITE);
			// copy data read in into structure
			memcpy(&curINSPosVelAttShort, log, sizeof(curINSPosVelAttShort));
			// release mutex
			ReleaseMutex(hINSPVASMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWINSPVA]);
			break;
		case RAWIMUB:
			// wait for mutex
			WaitForSingleObject(hRawIMUMutex,INFINITE);
			// copy data read in into structure
			memcpy(&curRawIMU, log, sizeof(curRawIMU));
			// release mutex
			ReleaseMutex(hRawIMUMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWRAWIMU]);
			break;
		case BSLNXYZB:
			// wait for mutex
			WaitForSingleObject(hBslnXYZMutex,INFINITE);
			// copy data read in into structure
			memcpy(&curBslnXYZ, log, sizeof(curBslnXYZ));
			// release mutex
			ReleaseMutex(hBslnXYZMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWBSLNXYZ]);
			break;
		case RAWIMUSB:
			// wait for mutex
			WaitForSingleObject(hRawIMUSMutex,INFINITE);
			// copy data read in into structure
			memcpy(&curRawIMUS, log, sizeof(curRawIMUS));
			// release mutex
			ReleaseMutex(hRawIMUSMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWRAWIMUS]);
			break;
		case VEHICLEBODYROTATIONB:
			// wait for mutex
			WaitForSingleObject(hVehicleBodyRotationMutex,INFINITE);
			// copy data read in into structure
			memcpy(&curVehicleBodyRotation, log, sizeof(curVehicleBodyRotation));
			// release mutex
			ReleaseMutex(hVehicleBodyRotationMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWVEHICLEBODYROTATION]);
			break;
		case INSSPDB:
			// wait for mutex
			WaitForSingleObject(hINSSPDMutex,INFINITE);
			// copy data read in into structure
			memcpy(&curINSSPD, log, sizeof(curINSSPD));
			// release mutex
			ReleaseMutex(hINSSPDMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWINSSPD]);
			break;
		case INSUTMB:
			// wait for mutex
			WaitForSingleObject(hINSUTMMutex,INFINITE);
			// copy data read in into best UTM position structure
			memcpy(&curINSUTM, log, sizeof(curINSUTM));
			// release mutex
			ReleaseMutex(hINSUTMMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWINSUTM]);
			break;
		case RXSTATUSB:
			// wait for mutex
			WaitForSingleObject(hRXStatusMutex,INFINITE);
			// copy data read in into rxstatus structure
			memcpy(&curRXStatus, log, sizeof(curRXStatus));
			// release mutex
			ReleaseMutex(hRXStatusMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWRXSTATUS]);
			break;
		case RXSTATUSEVENTB:
			// wait for mutex
			WaitForSingleObject(hRXStatusEventMutex,INFINITE);
			// copy data read in into rx status event structure
			memcpy(&curRXStatusEvent, log, sizeof(curRXStatusEvent));
			// release mutex
			ReleaseMutex(hRXStatusEventMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWRXSTATUSEVENT]);
			break;
		case RXHWLEVELSB:
			// wait for mutex
			WaitForSingleObject(hRXHwLevelsMutex,INFINITE);
			// copy data read in into rx hw levels structure
			memcpy(&curRXHwLevels, log, sizeof(curRXHwLevels));
			// release mutex
			ReleaseMutex(hRXHwLevelsMutex);
			// set event to indicate new log has arrived
			SetEvent(hNovatelEvents[NEWRXHWLEVELS]);
			break;
		case INVALID_IN_DATA:
			MessageDisplay::Instance().DisplayMessage("SPAN.cpp: Invalid command received",1);
			break;
		default:
			MessageDisplay::Instance().DisplayMessage("Unrecognized GPS message received.",1);
			break;

	}
}

BestPosition Novatel::getLastBestPOS()
{
	BestPosition temp;

	// get exclusive access to structure
	WaitForSingleObject(hBestPOSMutex,INFINITE);
	// get copy of structure
	temp = curBestPosition;
	// release mutex
	ReleaseMutex(hBestPOSMutex);
	// return structure
	return temp;

}

BestUTMPosition Novatel::getLastBestUTM()
{
	BestUTMPosition temp;

	// get exclusive access to structure
	WaitForSingleObject(hBestUTMMutex,INFINITE);
	// get copy of structure
	temp = curBestUTMPosition;
	// release mutex
	ReleaseMutex(hBestUTMMutex);
	// return structure
	return temp;

}

BestVelocity Novatel::getLastBestVelocity()
{
	BestVelocity temp;

	// get exclusive access to structure
	WaitForSingleObject(hBestVelocityMutex,INFINITE);
	// get copy of structure
	temp = curBestVelocity;
	// release mutex
	ReleaseMutex(hBestVelocityMutex);
	// return structure
	return temp;
}


INSPVA Novatel::getLastINSPosVelAtt()
{
	INSPVA temp;

	// get exclusive access to structure
	WaitForSingleObject(hINSPVAMutex,INFINITE);
	// get copy of structure
	temp = curINSPosVelAtt;
	// release mutex
	ReleaseMutex(hINSPVAMutex);
	// return structure
	return temp;
}

PsrXYZ Novatel::getLastPsrXYZ()
{
	PsrXYZ temp;

	// get exclusive access to structure
	WaitForSingleObject(hPsrXYZMutex,INFINITE);
	// get copy of structure
	temp = curPsrXYZ;
	// release mutex
	ReleaseMutex(hPsrXYZMutex);
	// return structure
	return temp;
}

INSPVAS Novatel::getLastINSPosVelAttShort()
{
	INSPVAS temp;

	// get exclusive access to structure
	WaitForSingleObject(hINSPVASMutex,INFINITE);
	// get copy of structure
	temp = curINSPosVelAttShort;
	// release mutex
	ReleaseMutex(hINSPVASMutex);
	// return structure
	return temp;
}

RawIMU Novatel::getLastRawIMU()
{
	RawIMU temp;

	// get exclusive access to structure
	WaitForSingleObject(hRawIMUMutex,INFINITE);
	// get copy of structure
	temp = curRawIMU;
	// release mutex
	ReleaseMutex(hRawIMUMutex);
	// return structure
	return temp;
}

bslnxyz Novatel::getLastBslnXYZ()
{
	bslnxyz temp;

	// get exclusive access to structure
	WaitForSingleObject(hBslnXYZMutex,INFINITE);
	// get copy of structure
	temp = curBslnXYZ;
	// release mutex
	ReleaseMutex(hBslnXYZMutex);
	// return structure
	return temp;
}

RawIMUS Novatel::getLastRawIMUS()
{
	RawIMUS temp;

	// get exclusive access to structure
	WaitForSingleObject(hRawIMUSMutex,INFINITE);
	// get copy of structure
	temp = curRawIMUS;
	// release mutex
	ReleaseMutex(hRawIMUSMutex);
	// return structure
	return temp;
}

INSSPD Novatel::getLastINSSpeed()
{
	INSSPD temp;

	// get exclusive access to structure
	WaitForSingleObject(hINSSPDMutex,INFINITE);
	// get copy of structure
	temp = curINSSPD;
	// release mutex
	ReleaseMutex(hINSSPDMutex);
	// return structure
	return temp;
}

INSUTM Novatel::getLastINSUTM()
{
	INSUTM temp;

	// get exclusive access to structure
	WaitForSingleObject(hINSUTMMutex,INFINITE);
	// get copy of structure
	temp = curINSUTM;
	// release mutex
	ReleaseMutex(hINSUTMMutex);
	// return structure
	return temp;
}

VehicleBodyRotation Novatel::getLastVehicleBodyRotation()
{
	VehicleBodyRotation temp;

	// get exclusive access to structure
	WaitForSingleObject(hVehicleBodyRotationMutex,INFINITE);
	// get copy of structure
	temp = curVehicleBodyRotation;
	// release mutex
	ReleaseMutex(hVehicleBodyRotationMutex);
	// return structure
	return temp;
}

RXStatus Novatel::getLastRXStatus()
{
	RXStatus temp;

	// get exclusive access to structure
	WaitForSingleObject(hRXStatusMutex,INFINITE);
	// get copy of structure
	temp = curRXStatus;
	// release mutex
	ReleaseMutex(hRXStatusMutex);
	// return structure
	return temp;
}

RXStatusEvent Novatel::getLastRXStatusEvent()
{
	RXStatusEvent temp;

	// get exclusive access to structure
	WaitForSingleObject(hRXStatusEventMutex,INFINITE);
	// get copy of structure
	temp = curRXStatusEvent;
	// release mutex
	ReleaseMutex(hRXStatusEventMutex);
	// return structure
	return temp;
}

RXHwLevels Novatel::getLastRXHwLevels()
{
	RXHwLevels temp;

	// get exclusive access to structure
	WaitForSingleObject(hRXHwLevelsMutex,INFINITE);
	// get copy of structure
	temp = curRXHwLevels;
	// release mutex
	ReleaseMutex(hRXHwLevelsMutex);
	// return structure
	return temp;
}

bool Novatel::UnlogAll()
{
	MessageDisplay::Instance().DisplayMessage("SPAN: Stopping all GPS messages",0);
	return sendString("UNLOGALL\r\n");
	//return true;
}

NOUT_ID Novatel::DecodeBinaryID(unsigned int msgID)  // Member function to identify the log
{
	NOUT_ID  iId = UNKNOWN;

   // Map the log id into the tlog id
   switch(msgID)
   {
      case  ACPB_LOG_TYPE:
      {
         iId = ACPB;
         break;
      }

      case  AGCB_LOG_TYPE:
      {
         iId = AGCB;
         break;
      }

      case  ALMB_LOG_TYPE:
      {
         iId = ALMB;
         break;
      }

      case  ATTB_LOG_TYPE:
      {
         iId = ATTB;
         break;
      }

      case  BATB_LOG_TYPE:
      {
         iId = BATB;
         break;
      }

      case BSLB_LOG_TYPE:
      {
         iId = BSLB;
         break;
      }

      case  CALB_LOG_TYPE:
      {
         iId = CALB;
         break;
      }

      case  CDSB_LOG_TYPE:
      {
         iId = CDSB;
         break;
      }

      case  CLKB_LOG_TYPE:
      {
         iId = CLKB;
         break;
      }

      case  CLMB_LOG_TYPE:
      {
         iId = CLMB;
         break;
      }

      case  COM1DATA_LOG_TYPE:
      {
         iId = COM1B;
         break;
      }

      case  COM2DATA_LOG_TYPE:
      {
         iId = COM2B;
         break;
      }

      case  CONSOLEDATA_LOG_TYPE:
      {
         iId = CONSOLEB;
         break;
      }

      case  CORB_LOG_TYPE:
      {
         iId = CORB;
         break;
      }

      case  CTSB_LOG_TYPE:
      {
         iId = CTSB;
         break;
      }

      case  DCSB_LOG_TYPE:
      {
         iId = DCSB;
         break;
      }

      case  DIRB_LOG_TYPE:
      {
         iId = DIRB;
         break;
      }

      case  DLLB_LOG_TYPE:
      {
         iId = DLLB;
         break;
      }

      case  DOPB_LOG_TYPE:
      {
         iId = DOPB;
         break;
      }

      case  ETSB_LOG_TYPE:
      {
         iId = ETSB;
         break;
      }

      case  FRMB_LOG_TYPE:
      {
         iId = FRMB;
         break;
      }

      case  FRWB_LOG_TYPE:
      {
         iId = FRWB;
         break;
      }

      case GALB_LOG_TYPE:
      {
         iId = GALB;
         break;
      }

      case GCLB_LOG_TYPE:
      {
         iId = GCLB;
         break;
      }

      case GEPB_LOG_TYPE:
      {
         iId = GEPB;
         break;
      }

      case  GROUPB_LOG_TYPE:
      {
         iId = GROUPB;
         break;
      }

      case  GRPB_LOG_TYPE:
      {
         iId = GRPB;
         break;
      }

      case  HDRB_LOG_TYPE:
      {
         iId = HDRB;
         break;
      }

      case  IONB_LOG_TYPE:
      {
         iId = IONB;
         break;
      }

      case  ISMRB_LOG_TYPE:
      {
         iId = ISMRB;
         break;
      }

      case  KPHB_LOG_TYPE:
      {
         iId = KPHB;
         break;
      }

      case  LPSTATUSB_LOG_TYPE:
      {
         iId = LPSTATUSB;
         break;
      }

      case  METB_LOG_TYPE:
      {
         iId = METB;
         break;
      }

      case  MKPB_LOG_TYPE:
      {
         iId = MKPB;
         break;
      }

      case  MKTB_LOG_TYPE:
      {
         iId = MKTB;
         break;
      }

      case  MPMB_LOG_TYPE:
      {
         iId = MPMB;
         break;
      }

      case  MSGB_LOG_TYPE:
      {
         iId = MSGB;
         break;
      }

      case  NAVB_LOG_TYPE:
      {
         iId = NAVB;
         break;
      }

      case  OPTB_LOG_TYPE:
      {
         iId = OPTB;
         break;
      }

      case  P20B_LOG_TYPE:
      {
         iId = P20B;
         break;
      }

      case  PAVB_LOG_TYPE:
      {
         iId = PAVB;
         break;
      }

      case  PDCB_LOG_TYPE:
      {
         iId = PDCB;
         break;
      }

      case  PDCDBG1B_LOG_TYPE:
      {
         iId = PDCDBG1B;
         break;
      }

      case  PDCVERB_LOG_TYPE:
      {
         iId = PDCVERB;
         break;
      }

      case  POSB_LOG_TYPE:
      {
         iId = POSB;
         break;
      }

      case  PROJECTB_LOG_TYPE:
      {
         iId = PROJECTB;
         break;
      }

      case PRTKB_LOG_TYPE:
      {
         iId = PRTKB;
         break;
      }

      case PSNB_LOG_TYPE:
      {
         iId = PSNB;
         break;
      }

      case  PVAB_LOG_TYPE:
      {
         iId = PVAB;
         break;
      }

      case  PXYB_LOG_TYPE:
      {
         iId = PXYB;
         break;
      }

      case  RALB_LOG_TYPE:
      {
         iId = RALB;
         break;
      }

      case  RASB_LOG_TYPE:
      {
         iId = RASB;
         break;
      }

      case  RBTB_LOG_TYPE:
      {
         iId = RBTB;
         break;
      }

      case  RCSB_LOG_TYPE:
      {
         iId = RCSB;
         break;
      }

      case  REPB_LOG_TYPE:
      {
         iId = REPB;
         break;
      }

      case  RGEB_LOG_TYPE:
      {
         iId = RGEB;
         break;
      }

      case  RGEC_LOG_TYPE:
      {
         iId = RGEC;
         break;
      }

      case  RGED_LOG_TYPE:
      {
         iId = RGED;
         break;
      }

      case RPSB_LOG_TYPE:
      {
         iId = RPSB;
         break;
      }

      case RT20B_LOG_TYPE:
      {
         iId = RT20B;
         break;
      }

      case RTCAB_LOG_TYPE:
      {
         iId = RTCAB;
         break;
      }

      case RTCM_LOG_TYPE:
      {
         iId = RTCMB;
         break;
      }

      case RTKB_LOG_TYPE:
      {
         iId = RTKB;
         break;
      }

      case RTKOB_LOG_TYPE:
      {
         iId = RTKOB;
         break;
      }

      case  RVSB_LOG_TYPE:
      {
         iId = RVSB;
         break;
      }

      case  SATB_LOG_TYPE:
      {
         iId = SATB;
         break;
      }

      case  SBLB_LOG_TYPE:
      {
         iId = SBLB;
         break;
      }

      case  SBTB_LOG_TYPE:
      {
         iId = SBTB;
         break;
      }

      case  SCHB_LOG_TYPE:
      {
         iId = SCHB;
         break;
      }

      case  SFDB_LOG_TYPE:
      {
         iId = SFDB;
         break;
      }

      case  SITELOGB_LOG_TYPE:
      {
         iId = SITELOGB;
         break;
      }

      case  SNOB_LOG_TYPE:
      {
         iId = SNOB;
         break;
      }

      case  SPHB_LOG_TYPE:
      {
         iId = SPHB;
         break;
      }

      case  STATUSB_LOG_TYPE:
      {
         iId = STATUSB;
         break;
      }

      case  SVDB_LOG_TYPE:
      {
         iId = SVDB;
         break;
      }

      case  TM1B_LOG_TYPE:
      {
         iId = TM1B;
         break;
      }

      case UTCB_LOG_TYPE:
      {
         iId = UTCB;
         break;
      }

      case VERB_LOG_TYPE:
      {
         iId = VERB;
         break;
      }

      case  VLHB_LOG_TYPE:
      {
         iId = VLHB;
         break;
      }

      case  WALB_LOG_TYPE:
      {
         iId = WALB;
         break;
      }

      case  WUTCB_LOG_TYPE:
      {
         iId = WUTCB;
         break;
      }

      case  WBRB_LOG_TYPE:
      {
         iId = WBRB;
         break;
      }

      case  WRCB_LOG_TYPE:
      {
         iId = WRCB;
         break;
      }

      case  SSOBS_L1L2_LOG_TYPE:
      {
         iId = SSOBSL1L2;
         break;
      }

      case  SSOBS_L1_LOG_TYPE:
      {
         iId = SSOBSL1;
         break;
      }

      case SSOBS_GISMO_LOG_TYPE:
      {
         iId = SSOBSGISMO;
         break;
      }

      case TAGB_LOG_TYPE:
      {
         iId = TAGB;
         break;
      }

      case DICB_LOG_TYPE:
      {
         iId = DICB;
         break;
      }

      case ZMESB_LOG_TYPE:
      {
         iId = ZMESB;
         break;
      }

      case ZPOSB_LOG_TYPE:
      {
         iId = ZPOSB;
         break;
      }

      case ZEPHB_LOG_TYPE:
      {
         iId = ZEPHB;
         break;
      }

      case ZSTNB_LOG_TYPE:
      {
         iId = ZSTNB;
         break;
      }

      case ZCFGB_LOG_TYPE:
      {
         iId = ZCFGB;
         break;
      }

      case ZTAGB_LOG_TYPE:
      {
         iId = ZTAGB;
         break;
      }

      //case  ALMANACB_LOG_TYPE:
      //{
      //   iId = ALMANACB;
      //   break;
      //}
      case  AVEPOSB_LOG_TYPE:
      {
         iId = AVEPOSB;
         break;
      }
      case  BESTPOSB_LOG_TYPE:
      {
         iId = BESTPOSB;
         break;
      }
      case  BESTVELB_LOG_TYPE:
      {
         iId = BESTVELB;
         break;
      }
      //case  CLOCKMODELB_LOG_TYPE:
      //{
      //   iId = CLOCKMODELB;
      //   break;
      //}
      //case  CHANDEBUGB_LOG_TYPE:
      //{
      //   iId = CHANDEBUGB;
      //   break;
      //}
      //case IONUTCB_LOG_TYPE:
      //{
      //   iId = IONUTCB;
      //   break;
      //}
      case  MATCHEDPOSB_LOG_TYPE:
      {
         iId = MATCHEDPOSB;
         break;
      }
      case NAVIGATEB_LOG_TYPE:
      {
         iId = NAVIGATEB;
         break;
      }
      case PASSCOM1B_LOG_TYPE:
      {
         iId = PASSCOM1B;
         break;
      }
      case PASSCOM2B_LOG_TYPE:
      {
         iId = PASSCOM2B;
         break;
      }
      case PASSCOM3B_LOG_TYPE:
      {
         iId = PASSCOM3B;
         break;
      }
      case PSRPOSB_LOG_TYPE:
      {
         iId = PSRPOSB;
         break;
      }
      case PSRVELB_LOG_TYPE:
      {
         iId = PSRVELB;
         break;
      }
      case PROPAGATEDCLOCKMODELB_LOG_TYPE:
      {
         iId = PROPAGATEDCLOCKMODELB;
         break;
      }
      case  RANGEB_LOG_TYPE:
      {
         iId = RANGEB;
         break;
      }
      case  RANGECMPB_LOG_TYPE:
      {
         iId = RANGECMPB;
         break;
      }
      case  RAWEPHEMB_LOG_TYPE:
      {
         iId = RAWEPHEMB;
         break;
      }
      case RAWGPSSUBFRAMEB_LOG_TYPE:
      {
         iId = RAWGPSSUBFRAMEB;
         break;
      }
      case REFSTATIONB_LOG_TYPE:
      {
         iId = REFSTATIONB;
         break;
      }
      case  RTKPOSB_LOG_TYPE:
      {
         iId = RTKPOSB;
         break;
      }
      case RXCONFIGB_LOG_TYPE:
      {
         iId = RXCONFIGB;
         break;
      }
      case RXSTATUSB_LOG_TYPE:
      {
         iId = RXSTATUSB;
         break;
      }
	  case RXSTATUSEVENTB_LOG_TYPE:
	  {
		iId = RXSTATUSEVENTB;
		break;
	  }
	  case RXHWLEVELSB_LOG_TYPE:
	  {
		iId = RXHWLEVELSB;
		break;
	  }
      case SATSTATB_LOG_TYPE:
      {
         iId = SATSTATB;
         break;
      }
      case TIMEB_LOG_TYPE:
      {
         iId = TIMEB;
         break;
      }
      //case TRACKSTATB_LOG_TYPE:
      //{
      //   iId = TRACKSTATB;
      //   break;
      //}
      //case  VERSIONB_LOG_TYPE:
      //{
      //   iId = VERSIONB;
      //   break;
      //}
      case  BESTUTMB_LOG_TYPE:
      {
         iId = BESTUTMB;
         break;
      }
      case  INSPVA_LOG_TYPE:
      {
         iId = INSPVAB;
         break;
      }
	  case PSRXYZ_LOG_TYPE:
	  {
		iId = PSRXYZB;
		break;
	  }
      case  INSPVAS_LOG_TYPE:
      {
         iId = INSPVASB;
         break;
      }
      case  INSUTM_LOG_TYPE:
      {
         iId = INSUTMB;
         break;
      }
      case  INSSPD_LOG_TYPE:
      {
         iId = INSSPDB;
         break;
      }
	  case  RAWIMU_LOG_TYPE:
      {
         iId = RAWIMUB;
         break;
      }
	  case BSLNXYZ_LOG_TYPE:
	  {
		iId = BSLNXYZB;
		break;
	  }
	  case  RAWIMUS_LOG_TYPE:
      {
         iId = RAWIMUSB;
         break;
      }
      case  VEHICLEBODYROTATION_LOG_TYPE:
      {
         iId = VEHICLEBODYROTATIONB;
         break;
      }

	  default:
      {
         iId = UNKNOWN;
         break;
      }
   }

   return iId;
}

//bool Novatel::Initialize(int comPortNumber,int baudRate)
//{
//	return Initialize(comPortNumber, baudRate,8,NOPARITY,ONESTOPBIT);
//}

//bool Novatel::Initialize(int comPortNumber,int baudRate, int byteSize,int parity,int stopSits)
//{
//	// attempt to initialze com port
//	if (!comPort.init(comPortNumber,baudRate,byteSize,parity,stopSits))
//	{	// com port init'd successfully
//		bActive = true;
//		evtInitialized();
//	}
//	else
//	{	// com port failed to init
//		bActive=false;
//	}
//
//	return bActive;
//
//}

//bool Novatel::StartReading()
//{
//	if (bReading)
//		return true;
//
//	// make sure com port is active before starting read thread
//	if (bActive)
//	{
//		//allocate buffer for the thread.
//		dataRead = new unsigned char[2001];
//		// start thread to listen for incoming com messages
//		hReadThread = CreateThread(0,0,ComPortListenThread,this,0,0);
//		// wait for read thread to start
//		WaitForSingleObject(hReadStarted,INFINITE);
//		bReading = true;
//		evtStartedReading();
//	}
//	else
//		bReading=false;
//
//	return bReading;
//}

//bool Novatel::StopReading()
//{
//
//	if (bReading)
//	{
//		bReading=false;
//		// stop read thread - exit normally (return code 0)
//		TerminateThread(hReadThread,0);
//		if(dataRead)
//			delete[] dataRead;
//		dataRead = NULL;
//		evtStoppedReading();
//	}
//	return !bReading;
//}

//void Novatel::ComListen()
//{
//	// number of bytes read in on last read
//	int bytesRead;
//	// character array to hold data read from com port
//	//dataRead = new unsigned char[2000];
//
//	//cout << "Read thread started." << endl;
//
//	SetEvent(hReadStarted);
//
//	for (;;)
//	{
//		// wait for new data to arrive on the com port
//		//bytesRead=comPort.read_next_string(2000,(char*)dataRead);
//		bytesRead=comPort.read_next_string(2000,(char*)dataRead);
//
//		cout << dataRead << endl;
//
//		if (bytesRead>0)
//			BufferIncomingData(dataRead, bytesRead);
//		else
//			StopReading();
//	}
//
//}
//
//bool Novatel::SendMessage(char* msg, unsigned int length)
//{
//	unsigned int bytesWritten;
//
//	// make sure com port is active before attempting to write to it
//	if (!bActive)
//		return false;
//
//	// write message
//	bytesWritten=comPort.write_bytes(msg,length);
//
//	// make sure all bytes were written
//	return (bytesWritten==length);
//}

//bool Novatel::SendString(char *msg)
//{
//	int bytesWritten;
//
//	// make sure com port is active before attempting to write to it
//	if (!bActive)
//		return false;
//
//	// write message
//	bytesWritten=comPort.write_string(msg);
//
//	// make sure all bytes were written
//	return (bytesWritten>0);
//}


//bool Novatel::Shutdown()
//{
//	comPort.close();
//
//	return true;
//}

void Novatel::closed()
{
	MessageDisplay::Instance().DisplayMessage("Device Shutdown.",0);
}
bool Novatel::stoppedReading()
{
	MessageDisplay::Instance().DisplayMessage("Stopped Reading Device.",0);
	return true;
}
