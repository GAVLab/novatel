/*!
 * \file novatel/novatel.h
 * \author David Hodo <david.hodo@gmail.com>
 * \version 1.0
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 David Hodo - Integrated Solutions for Systems (IS4S)
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
 * This provides an interface for OEM 4, V, and 6 series of Novatel GPS receivers
 *
 * This library depends on CMake-2.4.6 or later: http://www.cmake.org/
 * This library depends on Serial: https://github.com/wjwwood/serial
 *
 */

#ifndef NOVATEL_H
#define NOVATEL_H

#include <string>
#include <cstring> // for size_t

// Structure definition headers
#include "novatel/novatel_enums.h"
#include "novatel/novatel_structures.h"
// Boost Headers
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
// Serial Headers
#include "serial/serial.h"

namespace novatel{


typedef boost::function<double()> GetTimeCallback;
typedef boost::function<void()> HandleAcknowledgementCallback;

// INS Specific Callbacks
typedef boost::function<void(InsPositionVelocityAttitude&)> InsPositionVelocityAttitudeCallback;
typedef boost::function<void(InsPositionVelocityAttitudeShort&)> InsPositionVelocityAttitudeShortCallback;
typedef boost::function<void(VehicleBodyRotation&)> VehicleBodyRotationCallback;
typedef boost::function<void(InsSpeed&)> InsSpeedCallback;
typedef boost::function<void(RawImu&)> RawImuCallback;
typedef boost::function<void(RawImuShort&)> RawImuShortCallback;
typedef boost::function<void(Position&)> BestGpsPositionCallback;
typedef boost::function<void(BestLeverArm&)> BestLeverArmCallback;
typedef boost::function<void(InsCovariance&)> InsCovarianceCallback;
typedef boost::function<void(InsCovarianceShort&)> InsCovarianceShortCallback;

// GPS Callbacks
typedef boost::function<void(UtmPosition&)> BestUtmPositionCallback;
typedef boost::function<void(Velocity&)> BestVelocityCallback;
typedef boost::function<void(PositionEcef&)> BestPositionEcefCallback;
typedef boost::function<void(Dop&)> PseudorangeDopCallback;
typedef boost::function<void(Dop&)> RtkDopCallback;
typedef boost::function<void(BaselineEcef&)> BaselineEcefCallback;
typedef boost::function<void(IonosphericModel&)> IonosphericModelCallback;
typedef boost::function<void(RangeMeasurements&)> RangeMeasurementsCallback;
typedef boost::function<void(CompressedRangeMeasurements&)> CompressedRangeMeasurementsCallback;
typedef boost::function<void(GpsEphemeris&)> GpsEphemerisCallback;
typedef boost::function<void(SatellitePositions&)> SatellitePositionsCallback;
typedef boost::function<void(TimeOffset&)> TimeOffsetCallback;
typedef boost::function<void(ReceiverHardwareStatus&)> ReceiverHardwareStatusCallback;
typedef boost::function<void(Position&)> BestPositionCallback;
typedef boost::function<void(Position&)> BestPseudorangePositionCallback;
typedef boost::function<void(Position&)> BestRtkPositionCallback;


class Novatel
{
public:
	Novatel();
	~Novatel();

	/*!
	 * Connects to the Novatel receiver given a serial port.
	 *
	 * @param port Defines which serial port to connect to in serial mode.
	 * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
	 *
	 * @throws ConnectionFailedException connection attempt failed.
	 * @throws UnknownErrorCodeException unknown error code returned.
	 */
	bool Connect(std::string port, int baudrate=115200);

   /*!
    * Disconnects from the serial port
    */
    void Disconnect();

    /*!
     * Pings the GPS to determine if it is properly connected
     *
     * This method sends a ping to the GPS and waits for a response.
     *
     * @param num_attempts The number of times to ping the device
     * before giving up
     * @param timeout The time in milliseconds to wait for each reponse
     *
     * @return True if the GPS was found, false if it was not.
     */
     bool Ping(int num_attempts=5);


     /*!
      * Pings the GPS to determine if it is properly connected
      *
      * This method sends a ping to the GPS and waits for a response.
      *
      * @param num_attempts The number of times to ping the device
      * before giving up
      * @param timeout The time in milliseconds to wait for each reponse
      *
      * @return True if the GPS was found, false if it was not.
      */
     void set_time_handler(GetTimeCallback time_handler) {
         this->time_handler_ = time_handler;
     }

	void UnlogAll();

    /*!
     * Requests version information from the receiver
     *
     * This requests the VERSION message from the receiver and
     * uses the result to populate the receiver capapbilities
     *
     * @return True if the GPS was found, false if it was not.
     */
	bool UpdateVersion();

private:

	/*!
	 * Starts a thread to continuously read from the serial port.
	 *
	 * Starts a thread that runs 'ReadSerialPort' which constatly reads
	 * from the serial port.  When valid data is received, parse and then
	 *  the data callback functions are called.
	 *
	 * @see xbow440::DataCallback, xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StopReading
	 */
	void StartReading();

	/*!
	 * Starts the thread that reads from the serial port
	 *
	 * @see xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StartReading
	 */
	void StopReading();

	/*!
	 * Method run in a seperate thread that continuously reads from the
	 * serial port.  When a complete packet is received, the parse
	 * method is called to process the data
	 *
	 * @see xbow440::XBOW440::Parse, xbow440::XBOW440::StartReading, xbow440::XBOW440::StopReading
	 */
	void ReadSerialPort();


	void BufferIncomingData(unsigned char *message, unsigned int length);

	/*!
	 * Parses a packet of data from the GPS.  The
	 */
	void ParseBinary(unsigned char *message, BINARY_LOG_TYPE message_id);

	bool ParseVersion(std::string packet);

	//! Serial port object for communicating with sensor
	serial::Serial *serial_port_;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> read_thread_ptr_;
	bool reading_status_;  //!< True if the read thread is running, false otherwise.
	GetTimeCallback time_handler_; //!< Function pointer to callback function for timestamping

	HandleAcknowledgementCallback handle_acknowledgement_;

    //////////////////////////////////////////////////////
    // New Data Callbacks
    //////////////////////////////////////////////////////
    BestGpsPositionCallback best_gps_position_callback_;
    BestLeverArmCallback best_lever_arm_callback_;
    BestPositionCallback best_position_callback_;
    BestUtmPositionCallback best_utm_position_callback_;
    BestVelocityCallback best_velocity_callback_;
    BestPositionEcefCallback best_position_ecef_callback_;
    InsPositionVelocityAttitudeCallback ins_position_velocity_attitude_callback_;
    InsPositionVelocityAttitudeShortCallback ins_position_velocity_attitude_short_callback_;
    VehicleBodyRotationCallback vehicle_body_rotation_callback_;
    InsSpeedCallback ins_speed_callback_;
    RawImuCallback raw_imu_callback_;
    RawImuShortCallback raw_imu_short_callback_;
    InsCovarianceCallback ins_covariance_callback_;
    InsCovarianceShortCallback ins_covariance_short_callback_;

    // GPS Callbacks
    PseudorangeDopCallback pseudorange_dop_callback_;
    RtkDopCallback rtk_dop_callback_;
    BaselineEcefCallback baseline_ecef_callback_;
    IonosphericModelCallback ionospheric_model_callback_;
    RangeMeasurementsCallback range_measurements_callback_;
    CompressedRangeMeasurementsCallback compressed_range_measurements_callback_;
    GpsEphemerisCallback gps_ephemeris_callback_;
    SatellitePositionsCallback satellite_positions_callback_;
    TimeOffsetCallback time_offset_callback_;
    ReceiverHardwareStatusCallback receiver_hardware_status_callback_;
    BestPseudorangePositionCallback best_pseudorange_position_callback_;
    BestRtkPositionCallback best_rtk_position_callback_;



	//////////////////////////////////////////////////////
	// Incoming data buffers
	//////////////////////////////////////////////////////
	unsigned char data_buffer_[MAX_NOUT_SIZE];	//!< data currently being buffered to read
	unsigned char* data_read_;		//!< used only in BufferIncomingData - declared here for speed
	size_t bytes_remaining_;	//!< bytes remaining to be read in the current message
	size_t buffer_index_;		//!< index into data_buffer_
	size_t header_length_;	//!< length of the current header being read
	bool reading_acknowledgement_;	//!< true if an acknowledgement is being received
	double read_timestamp_; 		//!< time stamp when last serial port read completed
	double parse_timestamp_;		//!< time stamp when last parse began

	//////////////////////////////////////////////////////
	// Receiver capabilities
	//////////////////////////////////////////////////////
	std::string protocol_version_;		//!< Receiver version, OEM4, OEMV, OEM6, or UNKNOWN
	std::string serial_number_; //!< Receiver serial number
	std::string hardware_version_; //!< Receiver hardware version
	std::string software_version_; //!< Receiver hardware version
	std::string model_;				//!< Receiver model number

	bool l2_capable_; //!< Can the receiver handle L1 and L2 or just L1?
	bool raw_capable_; //!< Can the receiver output raw measurements?
	bool rtk_capable_; //!< Can the receiver compute RT2 and/or RT20 positions?
	bool glonass_capable_; //!< Can the receiver receive GLONASS frequencies?
	bool span_capable_;  //!< Is the receiver a SPAN unit?


};
}
#endif
