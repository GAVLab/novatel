#include "novatel/novatel.h"
#include <iostream>
#include <fstream>
#include <string_utils/string_utils.h>
using namespace std;
using namespace novatel;



/////////////////////////////////////////////////////
// includes for default time callback
#define WIN32_LEAN_AND_MEAN
#include "boost/date_time/posix_time/posix_time.hpp"
////////////////////////////////////////////////////


/*!
 * Default callback method for timestamping data.  Used if a
 * user callback is not set.  Returns the current time from the
 * CPU clock as the number of seconds from Jan 1, 1970
 */
double DefaultGetTime() {
	boost::posix_time::ptime present_time(boost::posix_time::microsec_clock::universal_time());
	boost::posix_time::time_duration duration(present_time.time_of_day());
	return duration.total_seconds();
}

void DefaultAcknowledgementHandler() {
    std::cout << "Acknowledgement received." << std::endl;
}

void DefaultBestPositionCallback(Position best_position){
    std:: cout << "BESTPOS: \nGPS Week: " << best_position.header.gps_week <<
                  "  GPS milliseconds: " << best_position.header.gps_millisecs << std::endl <<
                  "Latitude: " << best_position.latitude << std::endl <<
                  "Longitude: " << best_position.longitude << std::endl <<
                  "Height: " << best_position.height << std::endl << std::endl;
}

Novatel::Novatel() {
	serial_port_=NULL;
	reading_status_=false;
	time_handler_ = DefaultGetTime;
    handle_acknowledgement_=DefaultAcknowledgementHandler;
    best_position_callback_=DefaultBestPositionCallback;
	reading_acknowledgement_=false;
    buffer_index_=0;
    read_timestamp_=0;
    parse_timestamp_=0;
}

Novatel::~Novatel() {


}

bool Novatel::Connect(std::string port, int baudrate) {
	//serial_port_ = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(1000));
	serial::Timeout my_timeout(1000,50,0,50,0);
	serial_port_ = new serial::Serial(port,baudrate,my_timeout);

	if (!serial_port_->isOpen()){
		std::cout << "Serial port: " << port << " failed to open." << std::endl;
		delete serial_port_;
		serial_port_ = NULL;
		return false;
	} else {
		std::cout << "Serial port: " << port << " opened successfully." << std::endl;
	}


	// stop any incoming data and flush buffers
	serial_port_->write("UNLOGALL\r\n");
	// wait for data to stop cominig in
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	// clear serial port buffers
	serial_port_->flush();

	// look for GPS by sending ping and waiting for response
	if (!Ping()){
		std::cout << "Novatel GPS not found on port: " << port << std::endl;
		delete serial_port_;
		serial_port_ = NULL;
		return false;
	}

	// start reading
	StartReading();
	return true;

}

void Novatel::Disconnect() {
	StopReading();
	serial_port_->close();
	delete serial_port_;
	serial_port_=NULL;
}

bool Novatel::Ping(int num_attempts) {

	while ((num_attempts--)>0) {
		std::cout << "Searching for Novatel receiver..." << std::endl;
		if (UpdateVersion()) {
			std::cout << "Found Novatel receiver." << std::endl;
			std::cout << "\tModel: " << model_ << std::endl;
			std::cout << "\tSerial Number: " << serial_number_ << std::endl;
			std::cout << "\tHardware version: " << hardware_version_ << std::endl;
			std::cout << "\tSoftware version: " << software_version_ << std::endl << std::endl;;
			std::cout << "Receiver capabilities:" << std::endl;
			std::cout << "\tL2: ";
			if (l2_capable_)
				std::cout << "+" << std::endl;
			else
				std::cout << "-" << std::endl;
			std::cout << "\tRaw measurements: ";
			if (raw_capable_)
				std::cout << "+" << std::endl;
			else
				std::cout << "-" << std::endl;
			std::cout << "\tRTK: ";
			if (rtk_capable_)
				std::cout << "+" << std::endl;
			else
				std::cout << "-" << std::endl;
			std::cout << "\tSPAN: ";
			if (span_capable_)
				std::cout << "+" << std::endl;
			else
				std::cout << "-" << std::endl;
			std::cout << "\tGLONASS: ";
			if (glonass_capable_)
				std::cout << "+" << std::endl;
			else
				std::cout << "-" << std::endl;
			return true;
		}
	}

	// no response found
	return false;

}


bool Novatel::UpdateVersion()
{
	// request the receiver version and wait for a response
	// example response:
	//#VERSIONA,COM1,0,71.5,FINESTEERING,1362,340308.478,00000008,3681,2291;
	//    1,GPSCARD,"L12RV","DZZ06040010","OEMV2G-2.00-2T","3.000A19","3.000A9",
	//    "2006/Feb/ 9","17:14:33"*5e8df6e0

	// clear port
	//serial_port_->flush();
	// send request for version
	serial_port_->write("log versiona once\r\n");
	boost::this_thread::sleep(boost::posix_time::milliseconds(250));
	// read from the serial port until a new line character is seen
	std::string gps_response = serial_port_->read(5000);

	std::vector<std::string> packets;

	string_utils::Tokenize(gps_response, packets, "\n");

	// loop through all packets in file and check for version messages
	// stop when the first is found or all packets are read
	for (size_t ii=0; ii<packets.size(); ii++) {
		if (ParseVersion(packets[ii])) {
			return true;
		}
	}

	return false;

}

bool Novatel::ParseVersion(std::string packet) {
	// parse the results - message should start with "#VERSIONA"
        size_t found_version=packet.find("VERSIONA");
		if (found_version==string::npos)
			return false;

		// parse version information
		// remove header
		size_t pos=packet.find(";");
		if (pos==string::npos) {
			std::cout << "Error parsing received version."
					" End of message was not found" << std::endl;
			return false;
		}

		// remove header from message
		std::string message=packet.substr(pos+1, packet.length()-pos-2);
		// parse message body by tokening on ","
		typedef boost::tokenizer<boost::char_separator<char> >
			tokenizer;
		boost::char_separator<char> sep(",");
		tokenizer tokens(message, sep);
		// set up iterator to go through token list
		tokenizer::iterator current_token=tokens.begin();
		string num_comps_string=*(current_token);
		int number_components=atoi(num_comps_string.c_str());
		// make sure the correct number of tokens were found
		int token_count=0;
		for(current_token=tokens.begin(); current_token!=tokens.end();++current_token)
			token_count++;



		// should be 9 tokens, if not something is wrong
		if (token_count!=(8*number_components+1)) {
			std::cout << "Error parsing received version. "
					"Incorrect number of tokens found." << std::endl;
			return false;
		}

		current_token=tokens.begin();
		// device type is 2nd token
		string device_type=*(++current_token);
		// model is 3rd token
		model_=*(++current_token);
		// serial number is 4th token
		serial_number_=*(++current_token);
		// model is 5rd token
		hardware_version_=*(++current_token);
		// model is 6rd token
		software_version_=*(++current_token);

		// parse the version:
		if (hardware_version_.length()>3)
            protocol_version_=hardware_version_.substr(1,4);
		else
			protocol_version_="UNKNOWN";

		// parse model number:
		// is the receiver capable of raw measurements?
        if (model_.find("L")!=string::npos)
			raw_capable_=true;
		else
			raw_capable_=false;

		// can the receiver receive L2?
		if (model_.find("12")!=string::npos)
			l2_capable_=true;
		else
			l2_capable_=false;

		// can the receiver receive GLONASS?
		if (model_.find("G")!=string::npos)
			glonass_capable_=true;
		else
			glonass_capable_=false;

		// Is this a SPAN unit?
		if ((model_.find("I")!=string::npos)||(model_.find("J")!=string::npos))
			span_capable_=true;
		else
			span_capable_=false;

		// Can the receiver process RTK?
		if (model_.find("R")!=string::npos)
			rtk_capable_=true;
		else
			rtk_capable_=false;


        // fix for oem4 span receivers - do not use l12 notation
        // i think all oem4 spans are l1 l2 capable and raw capable
        if ((protocol_version_=="OEM4")&&(span_capable_)) {
            l2_capable_=true;
            raw_capable_=true;
        }

		return true;

}


void Novatel::StartReading() {
	// create thread to read from sensor
	reading_status_=true;
	read_thread_ptr_ = boost::shared_ptr<boost::thread >
		(new boost::thread(boost::bind(&Novatel::ReadSerialPort, this)));
}

void Novatel::StopReading() {
	reading_status_=false;
}


void Novatel::ReadSerialPort() {
	unsigned char buffer[MAX_NOUT_SIZE];
	size_t len;
	double time_stamp;

	// continuously read data from serial port
	while (reading_status_) {
		// read data
		len = serial_port_->read(buffer, MAX_NOUT_SIZE);
		// timestamp the read
		read_timestamp_ = time_handler_();
		// add data to the buffer to be parsed
		BufferIncomingData(buffer, len);
	}

}


void Novatel::BufferIncomingData(unsigned char *message, unsigned int length)
{

	//cout << "Received data: " << dec <<length << endl;
	//cout << msg << endl;

	// add incoming data to buffer
	for (unsigned int ii=0; ii<length; ii++)
	{
		//cout << hex << (int)msg[i] << endl;
		// make sure bufIndex is not larger than buffer
		if (buffer_index_>=MAX_NOUT_SIZE)
		{
			buffer_index_=0;
			std::cout << "Novatel: Overflowed receive buffer. Reset" << std::endl;
		}

		if (buffer_index_==0)
		{	// looking for beginning of message
			if (message[ii]==0xAA)
			{	// beginning of msg found - add to buffer
				data_buffer_[buffer_index_++]=message[ii];
				bytes_remaining_=0;
			}	// end if (msg[ii]
			else if (message[ii]=='<')
			{
				// received beginning of acknowledgement
				reading_acknowledgement_=true;
				buffer_index_=1;
			}
			else
			{
				//MessageDisplay::Instance().DisplayMessage("Novatel: Received unknown data.\r\n", 1);
			}
		} // end if (bufIndex==0)
		else if (buffer_index_==1)
		{	// verify 2nd character of header
			if (message[ii]==0x44)
			{	// 2nd byte ok - add to buffer
				data_buffer_[buffer_index_++]=message[ii];
			}
			else if ((message[ii]=='O')&&reading_acknowledgement_)
			{
				// 2nd byte of acknowledgement
				buffer_index_=2;
			}
			else
			{
				// start looking for new message again
				buffer_index_=0;
				bytes_remaining_=0;
				reading_acknowledgement_=false;
			} // end if (msg[i]==0x44)
		}	// end else if (bufIndex==1)
		else if (buffer_index_==2)
		{	// verify 3rd character of header
			if (message[ii]==0x12)
			{	// 2nd byte ok - add to buffer
				data_buffer_[buffer_index_++]=message[ii];
			}
			else if ((message[ii]=='K')&&(reading_acknowledgement_))
			{
				// final byte of acknowledgement received
				buffer_index_=0;
				reading_acknowledgement_=false;
				// ACK received
				handle_acknowledgement_();
			}
			else
			{
				// start looking for new message again
				buffer_index_=0;
				bytes_remaining_=0;
				reading_acknowledgement_=false;
			} // end if (msg[i]==0x12)
		}	// end else if (bufIndex==2)
		else if (buffer_index_==3)
		{	// number of bytes in header - not including sync
			data_buffer_[buffer_index_++]=message[ii];
			// length of header is in byte 4
			header_length_=message[ii];
		}
		else if (buffer_index_==8)
		{	// set number of bytes
			data_buffer_[buffer_index_++]=message[ii];
			// length of message is in byte 8
			// bytes remaining = remainder of header  + 4 byte checksum + length of body
			// TODO: added a -2 to make things work right, figure out why i need this
			bytes_remaining_=message[ii]+4+(header_length_-7)-2;
		}
		else if (bytes_remaining_==1)
		{	// add last byte and parse
			data_buffer_[buffer_index_++]=message[ii];
			BINARY_LOG_TYPE message_id=(BINARY_LOG_TYPE)(((data_buffer_[5])<<8)+data_buffer_[4]);
			ParseBinary(data_buffer_,message_id);
			// reset counters
			buffer_index_=0;
			bytes_remaining_=0;
		}  // end else if (bytesRemaining==1)
		else
		{	// add data to buffer
			data_buffer_[buffer_index_++]=message[ii];
			bytes_remaining_--;
		}
	}	// end for
}

void Novatel::ParseBinary(unsigned char *message, BINARY_LOG_TYPE message_id)
{
	cout << "Parsing Log: " << message_id << endl;
	//cout << "Parsing Log: " << logID << " => " << parser.GetIDStr(logID) << endl;

    switch (message_id) {
        case BESTPOSB_LOG_TYPE:
            Position best_position;
            memcpy(&best_position, message, sizeof(best_position));
            best_position_callback_(best_position);
            break;

        default:
            break;
    }


}


//
//NOUT_ID Novatel::DecodeBinaryID(unsigned int msgID)  // Member function to identify the log
//{
//	NOUT_ID  iId = UNKNOWN;
//
//   // Map the log id into the tlog id
//   switch(msgID)
//   {
//      case  ACPB_LOG_TYPE:
//      {
//         iId = ACPB;
//         break;
//      }
//
//      case  AGCB_LOG_TYPE:
//      {
//         iId = AGCB;
//         break;
//      }
//
//      case  ALMB_LOG_TYPE:
//      {
//         iId = ALMB;
//         break;
//      }
//
//      case  ATTB_LOG_TYPE:
//      {
//         iId = ATTB;
//         break;
//      }
//
//      case  BATB_LOG_TYPE:
//      {
//         iId = BATB;
//         break;
//      }
//
//      case BSLB_LOG_TYPE:
//      {
//         iId = BSLB;
//         break;
//      }
//
//      case  CALB_LOG_TYPE:
//      {
//         iId = CALB;
//         break;
//      }
//
//      case  CDSB_LOG_TYPE:
//      {
//         iId = CDSB;
//         break;
//      }
//
//      case  CLKB_LOG_TYPE:
//      {
//         iId = CLKB;
//         break;
//      }
//
//      case  CLMB_LOG_TYPE:
//      {
//         iId = CLMB;
//         break;
//      }
//
//      case  COM1DATA_LOG_TYPE:
//      {
//         iId = COM1B;
//         break;
//      }
//
//      case  COM2DATA_LOG_TYPE:
//      {
//         iId = COM2B;
//         break;
//      }
//
//      case  CONSOLEDATA_LOG_TYPE:
//      {
//         iId = CONSOLEB;
//         break;
//      }
//
//      case  CORB_LOG_TYPE:
//      {
//         iId = CORB;
//         break;
//      }
//
//      case  CTSB_LOG_TYPE:
//      {
//         iId = CTSB;
//         break;
//      }
//
//      case  DCSB_LOG_TYPE:
//      {
//         iId = DCSB;
//         break;
//      }
//
//      case  DIRB_LOG_TYPE:
//      {
//         iId = DIRB;
//         break;
//      }
//
//      case  DLLB_LOG_TYPE:
//      {
//         iId = DLLB;
//         break;
//      }
//
//      case  DOPB_LOG_TYPE:
//      {
//         iId = DOPB;
//         break;
//      }
//
//      case  ETSB_LOG_TYPE:
//      {
//         iId = ETSB;
//         break;
//      }
//
//      case  FRMB_LOG_TYPE:
//      {
//         iId = FRMB;
//         break;
//      }
//
//      case  FRWB_LOG_TYPE:
//      {
//         iId = FRWB;
//         break;
//      }
//
//      case GALB_LOG_TYPE:
//      {
//         iId = GALB;
//         break;
//      }
//
//      case GCLB_LOG_TYPE:
//      {
//         iId = GCLB;
//         break;
//      }
//
//      case GEPB_LOG_TYPE:
//      {
//         iId = GEPB;
//         break;
//      }
//
//      case  GROUPB_LOG_TYPE:
//      {
//         iId = GROUPB;
//         break;
//      }
//
//      case  GRPB_LOG_TYPE:
//      {
//         iId = GRPB;
//         break;
//      }
//
//      case  HDRB_LOG_TYPE:
//      {
//         iId = HDRB;
//         break;
//      }
//
//      case  IONB_LOG_TYPE:
//      {
//         iId = IONB;
//         break;
//      }
//
//      case  ISMRB_LOG_TYPE:
//      {
//         iId = ISMRB;
//         break;
//      }
//
//      case  KPHB_LOG_TYPE:
//      {
//         iId = KPHB;
//         break;
//      }
//
//      case  LPSTATUSB_LOG_TYPE:
//      {
//         iId = LPSTATUSB;
//         break;
//      }
//
//      case  METB_LOG_TYPE:
//      {
//         iId = METB;
//         break;
//      }
//
//      case  MKPB_LOG_TYPE:
//      {
//         iId = MKPB;
//         break;
//      }
//
//      case  MKTB_LOG_TYPE:
//      {
//         iId = MKTB;
//         break;
//      }
//
//      case  MPMB_LOG_TYPE:
//      {
//         iId = MPMB;
//         break;
//      }
//
//      case  MSGB_LOG_TYPE:
//      {
//         iId = MSGB;
//         break;
//      }
//
//      case  NAVB_LOG_TYPE:
//      {
//         iId = NAVB;
//         break;
//      }
//
//      case  OPTB_LOG_TYPE:
//      {
//         iId = OPTB;
//         break;
//      }
//
//      case  P20B_LOG_TYPE:
//      {
//         iId = P20B;
//         break;
//      }
//
//      case  PAVB_LOG_TYPE:
//      {
//         iId = PAVB;
//         break;
//      }
//
//      case  PDCB_LOG_TYPE:
//      {
//         iId = PDCB;
//         break;
//      }
//
//      case  PDCDBG1B_LOG_TYPE:
//      {
//         iId = PDCDBG1B;
//         break;
//      }
//
//      case  PDCVERB_LOG_TYPE:
//      {
//         iId = PDCVERB;
//         break;
//      }
//
//      case  POSB_LOG_TYPE:
//      {
//         iId = POSB;
//         break;
//      }
//
//      case  PROJECTB_LOG_TYPE:
//      {
//         iId = PROJECTB;
//         break;
//      }
//
//      case PRTKB_LOG_TYPE:
//      {
//         iId = PRTKB;
//         break;
//      }
//
//      case PSNB_LOG_TYPE:
//      {
//         iId = PSNB;
//         break;
//      }
//
//      case  PVAB_LOG_TYPE:
//      {
//         iId = PVAB;
//         break;
//      }
//
//      case  PXYB_LOG_TYPE:
//      {
//         iId = PXYB;
//         break;
//      }
//
//      case  RALB_LOG_TYPE:
//      {
//         iId = RALB;
//         break;
//      }
//
//      case  RASB_LOG_TYPE:
//      {
//         iId = RASB;
//         break;
//      }
//
//      case  RBTB_LOG_TYPE:
//      {
//         iId = RBTB;
//         break;
//      }
//
//      case  RCSB_LOG_TYPE:
//      {
//         iId = RCSB;
//         break;
//      }
//
//      case  REPB_LOG_TYPE:
//      {
//         iId = REPB;
//         break;
//      }
//
//      case  RGEB_LOG_TYPE:
//      {
//         iId = RGEB;
//         break;
//      }
//
//      case  RGEC_LOG_TYPE:
//      {
//         iId = RGEC;
//         break;
//      }
//
//      case  RGED_LOG_TYPE:
//      {
//         iId = RGED;
//         break;
//      }
//
//      case RPSB_LOG_TYPE:
//      {
//         iId = RPSB;
//         break;
//      }
//
//      case RT20B_LOG_TYPE:
//      {
//         iId = RT20B;
//         break;
//      }
//
//      case RTCAB_LOG_TYPE:
//      {
//         iId = RTCAB;
//         break;
//      }
//
//      case RTCM_LOG_TYPE:
//      {
//         iId = RTCMB;
//         break;
//      }
//
//      case RTKB_LOG_TYPE:
//      {
//         iId = RTKB;
//         break;
//      }
//
//      case RTKOB_LOG_TYPE:
//      {
//         iId = RTKOB;
//         break;
//      }
//
//      case  RVSB_LOG_TYPE:
//      {
//         iId = RVSB;
//         break;
//      }
//
//      case  SATB_LOG_TYPE:
//      {
//         iId = SATB;
//         break;
//      }
//
//      case  SBLB_LOG_TYPE:
//      {
//         iId = SBLB;
//         break;
//      }
//
//      case  SBTB_LOG_TYPE:
//      {
//         iId = SBTB;
//         break;
//      }
//
//      case  SCHB_LOG_TYPE:
//      {
//         iId = SCHB;
//         break;
//      }
//
//      case  SFDB_LOG_TYPE:
//      {
//         iId = SFDB;
//         break;
//      }
//
//      case  SITELOGB_LOG_TYPE:
//      {
//         iId = SITELOGB;
//         break;
//      }
//
//      case  SNOB_LOG_TYPE:
//      {
//         iId = SNOB;
//         break;
//      }
//
//      case  SPHB_LOG_TYPE:
//      {
//         iId = SPHB;
//         break;
//      }
//
//      case  STATUSB_LOG_TYPE:
//      {
//         iId = STATUSB;
//         break;
//      }
//
//      case  SVDB_LOG_TYPE:
//      {
//         iId = SVDB;
//         break;
//      }
//
//      case  TM1B_LOG_TYPE:
//      {
//         iId = TM1B;
//         break;
//      }
//
//      case UTCB_LOG_TYPE:
//      {
//         iId = UTCB;
//         break;
//      }
//
//      case VERB_LOG_TYPE:
//      {
//         iId = VERB;
//         break;
//      }
//
//      case  VLHB_LOG_TYPE:
//      {
//         iId = VLHB;
//         break;
//      }
//
//      case  WALB_LOG_TYPE:
//      {
//         iId = WALB;
//         break;
//      }
//
//      case  WUTCB_LOG_TYPE:
//      {
//         iId = WUTCB;
//         break;
//      }
//
//      case  WBRB_LOG_TYPE:
//      {
//         iId = WBRB;
//         break;
//      }
//
//      case  WRCB_LOG_TYPE:
//      {
//         iId = WRCB;
//         break;
//      }
//
//      case  SSOBS_L1L2_LOG_TYPE:
//      {
//         iId = SSOBSL1L2;
//         break;
//      }
//
//      case  SSOBS_L1_LOG_TYPE:
//      {
//         iId = SSOBSL1;
//         break;
//      }
//
//      case SSOBS_GISMO_LOG_TYPE:
//      {
//         iId = SSOBSGISMO;
//         break;
//      }
//
//      case TAGB_LOG_TYPE:
//      {
//         iId = TAGB;
//         break;
//      }
//
//      case DICB_LOG_TYPE:
//      {
//         iId = DICB;
//         break;
//      }
//
//      case ZMESB_LOG_TYPE:
//      {
//         iId = ZMESB;
//         break;
//      }
//
//      case ZPOSB_LOG_TYPE:
//      {
//         iId = ZPOSB;
//         break;
//      }
//
//      case ZEPHB_LOG_TYPE:
//      {
//         iId = ZEPHB;
//         break;
//      }
//
//      case ZSTNB_LOG_TYPE:
//      {
//         iId = ZSTNB;
//         break;
//      }
//
//      case ZCFGB_LOG_TYPE:
//      {
//         iId = ZCFGB;
//         break;
//      }
//
//      case ZTAGB_LOG_TYPE:
//      {
//         iId = ZTAGB;
//         break;
//      }
//
//      //case  ALMANACB_LOG_TYPE:
//      //{
//      //   iId = ALMANACB;
//      //   break;
//      //}
//      case  AVEPOSB_LOG_TYPE:
//      {
//         iId = AVEPOSB;
//         break;
//      }
//      case  BESTPOSB_LOG_TYPE:
//      {
//         iId = BESTPOSB;
//         break;
//      }
//      case  BESTVELB_LOG_TYPE:
//      {
//         iId = BESTVELB;
//         break;
//      }
//      //case  CLOCKMODELB_LOG_TYPE:
//      //{
//      //   iId = CLOCKMODELB;
//      //   break;
//      //}
//      //case  CHANDEBUGB_LOG_TYPE:
//      //{
//      //   iId = CHANDEBUGB;
//      //   break;
//      //}
//      //case IONUTCB_LOG_TYPE:
//      //{
//      //   iId = IONUTCB;
//      //   break;
//      //}
//      case  MATCHEDPOSB_LOG_TYPE:
//      {
//         iId = MATCHEDPOSB;
//         break;
//      }
//      case NAVIGATEB_LOG_TYPE:
//      {
//         iId = NAVIGATEB;
//         break;
//      }
//      case PASSCOM1B_LOG_TYPE:
//      {
//         iId = PASSCOM1B;
//         break;
//      }
//      case PASSCOM2B_LOG_TYPE:
//      {
//         iId = PASSCOM2B;
//         break;
//      }
//      case PASSCOM3B_LOG_TYPE:
//      {
//         iId = PASSCOM3B;
//         break;
//      }
//      case PSRPOSB_LOG_TYPE:
//      {
//         iId = PSRPOSB;
//         break;
//      }
//      case PSRVELB_LOG_TYPE:
//      {
//         iId = PSRVELB;
//         break;
//      }
//      case PROPAGATEDCLOCKMODELB_LOG_TYPE:
//      {
//         iId = PROPAGATEDCLOCKMODELB;
//         break;
//      }
//      case  RANGEB_LOG_TYPE:
//      {
//         iId = RANGEB;
//         break;
//      }
//      case  RANGECMPB_LOG_TYPE:
//      {
//         iId = RANGECMPB;
//         break;
//      }
//      case  RAWEPHEMB_LOG_TYPE:
//      {
//         iId = RAWEPHEMB;
//         break;
//      }
//      case RAWGPSSUBFRAMEB_LOG_TYPE:
//      {
//         iId = RAWGPSSUBFRAMEB;
//         break;
//      }
//      case REFSTATIONB_LOG_TYPE:
//      {
//         iId = REFSTATIONB;
//         break;
//      }
//      case  RTKPOSB_LOG_TYPE:
//      {
//         iId = RTKPOSB;
//         break;
//      }
//      case RXCONFIGB_LOG_TYPE:
//      {
//         iId = RXCONFIGB;
//         break;
//      }
//      case RXSTATUSB_LOG_TYPE:
//      {
//         iId = RXSTATUSB;
//         break;
//      }
//	  case RXSTATUSEVENTB_LOG_TYPE:
//	  {
//		iId = RXSTATUSEVENTB;
//		break;
//	  }
//	  case RXHWLEVELSB_LOG_TYPE:
//	  {
//		iId = RXHWLEVELSB;
//		break;
//	  }
//      case SATSTATB_LOG_TYPE:
//      {
//         iId = SATSTATB;
//         break;
//      }
//      case TIMEB_LOG_TYPE:
//      {
//         iId = TIMEB;
//         break;
//      }
//      //case TRACKSTATB_LOG_TYPE:
//      //{
//      //   iId = TRACKSTATB;
//      //   break;
//      //}
//      //case  VERSIONB_LOG_TYPE:
//      //{
//      //   iId = VERSIONB;
//      //   break;
//      //}
//      case  BESTUTMB_LOG_TYPE:
//      {
//         iId = BESTUTMB;
//         break;
//      }
//      case  INSPVA_LOG_TYPE:
//      {
//         iId = INSPVAB;
//         break;
//      }
//	  case PSRXYZ_LOG_TYPE:
//	  {
//		iId = PSRXYZB;
//		break;
//	  }
//      case  INSPVAS_LOG_TYPE:
//      {
//         iId = INSPVASB;
//         break;
//      }
//      case  INSUTM_LOG_TYPE:
//      {
//         iId = INSUTMB;
//         break;
//      }
//      case  INSSPD_LOG_TYPE:
//      {
//         iId = INSSPDB;
//         break;
//      }
//	  case  RAWIMU_LOG_TYPE:
//      {
//         iId = RAWIMUB;
//         break;
//      }
//	  case BSLNXYZ_LOG_TYPE:
//	  {
//		iId = BSLNXYZB;
//		break;
//	  }
//	  case  RAWIMUS_LOG_TYPE:
//      {
//         iId = RAWIMUSB;
//         break;
//      }
//      case  VEHICLEBODYROTATION_LOG_TYPE:
//      {
//         iId = VEHICLEBODYROTATIONB;
//         break;
//      }
//
//	  default:
//      {
//         iId = UNKNOWN;
//         break;
//      }
//   }
//
//   return iId;
//}


