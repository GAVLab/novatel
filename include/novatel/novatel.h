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
//#include <boost/condition_variable.hpp>
// Serial Headers
#include "serial/serial.h"

namespace novatel {

// used to convert lat and long to UTM coordinates
#define GRAD_A_RAD(g) ((g)*0.0174532925199433)
#define CRC32_POLYNOMIAL 0xEDB88320L

// Constants for unpacking RANGECMP
#define CMP_MAX_VALUE         8388608.0
#define CMP_GPS_WAVELENGTH_L1 0.1902936727984
#define CMP_GPS_WAVELENGTH_L2 0.2442102134246

typedef boost::function<double()> GetTimeCallback;
typedef boost::function<void()> HandleAcknowledgementCallback;

// Messaging callbacks
typedef boost::function<void(const std::string&)> LogMsgCallback;
typedef boost::function<void(unsigned char *)> RawMsgCallback;

// INS Specific Callbacks
typedef boost::function<void(InsPositionVelocityAttitude&, double&)> InsPositionVelocityAttitudeCallback;
typedef boost::function<void(InsPositionVelocityAttitudeShort&, double&)> InsPositionVelocityAttitudeShortCallback;
typedef boost::function<void(VehicleBodyRotation&, double&)> VehicleBodyRotationCallback;
typedef boost::function<void(InsSpeed&, double&)> InsSpeedCallback;
typedef boost::function<void(RawImu&, double&)> RawImuCallback;
typedef boost::function<void(RawImuShort&, double&)> RawImuShortCallback;
typedef boost::function<void(Position&, double&)> BestGpsPositionCallback;
typedef boost::function<void(BestLeverArm&, double&)> BestLeverArmCallback;
typedef boost::function<void(InsCovariance&, double&)> InsCovarianceCallback;
typedef boost::function<void(InsCovarianceShort&, double&)> InsCovarianceShortCallback;

// GPS Callbacks
typedef boost::function<void(UtmPosition&, double&)> BestUtmPositionCallback;
typedef boost::function<void(Velocity&, double&)> BestVelocityCallback;
typedef boost::function<void(PositionEcef&, double&)> BestPositionEcefCallback;
typedef boost::function<void(Dop&, double&)> PseudorangeDopCallback;
typedef boost::function<void(Dop&, double&)> RtkDopCallback;
typedef boost::function<void(BaselineEcef&, double&)> BaselineEcefCallback;
typedef boost::function<void(IonosphericModel&, double&)> IonosphericModelCallback;
typedef boost::function<void(RangeMeasurements&, double&)> RangeMeasurementsCallback;
typedef boost::function<void(CompressedRangeMeasurements&, double&)> CompressedRangeMeasurementsCallback;
typedef boost::function<void(GpsEphemeris&, double&)> GpsEphemerisCallback;
typedef boost::function<void(RawEphemeris&, double&)> RawEphemerisCallback;
typedef boost::function<void(RawAlmanac&, double&)> RawAlmanacCallback;
typedef boost::function<void(Almanac&, double&)> AlmanacCallback;
typedef boost::function<void(SatellitePositions&, double&)> SatellitePositionsCallback;
typedef boost::function<void(SatelliteVisibility&, double&)> SatelliteVisibilityCallback;
typedef boost::function<void(TimeOffset&, double&)> TimeOffsetCallback;
typedef boost::function<void(TrackStatus&, double&)> TrackingStatusCallback;
typedef boost::function<void(ReceiverHardwareStatus&, double&)> ReceiverHardwareStatusCallback;
typedef boost::function<void(Position&, double&)> BestPositionCallback;
typedef boost::function<void(Position&, double&)> BestPseudorangePositionCallback;
typedef boost::function<void(Position&, double&)> RtkPositionCallback;


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
	 bool Connect(std::string port, int baudrate=115200, bool search=true);

   /*!
    * Disconnects from the serial port
    */
    void Disconnect();

  //! Indicates if a connection to the receiver has been established.
  bool IsConnected() {return is_connected_;}

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

    void setLogDebugCallback(LogMsgCallback debug_callback){log_debug_=debug_callback;};
    void setLogInfoCallback(LogMsgCallback info_callback){log_info_=info_callback;};
    void setLogWarningCallback(LogMsgCallback warning_callback){log_warning_=warning_callback;};
    void setLogErrorCallback(LogMsgCallback error_callback){log_error_=error_callback;};

    /*!
     * Request the given list of logs from the receiver.
     * Format: "[LOGNAME][MESSAGETYPE] [PORT] [LOGTYPE] [PERIOD] ..."
     * [MESSAGETYPE] - [A]=ASCII
     *               - [B]=Binary
     *               . [empty]=Abreviated ASCII
     * log_string format: "BESTUTMB ONTIME 1.0; BESTVELB ONTIME 1.0"
     */
    void ConfigureLogs(std::string log_string);
    void Unlog(std::string log); //!< Stop logging a specified log
    void UnlogAll(); //!< Stop logging all logs that aren't set with HOLD parameter

    /*!
     * SaveConfiguration() saves the current receiver configuration
     * in nonvolatile memory
     */
    void SaveConfiguration();

    void ConfigureInterfaceMode(std::string com_port,  
      std::string rx_mode, std::string tx_mode);

    void ConfigureBaudRate(std::string com_port, int baudrate);

    void SetBaudRate(int baudrate, std::string com_port="COM1");

    bool SendCommand(std::string cmd_msg, bool wait_for_ack=true);
    bool SendMessage(uint8_t* msg_ptr, size_t length);

    /*!
     * SetSvElevationCutoff
     * Sets the elevation cut-off angle. Svs below this angle
     * are not automatically searched for and are not used
     * in the position calculation. Angles < 5 deg are not
     * recommended except in specific situations
     * (Angle = +-90 deg)
     */
    bool SetSvElevationAngleCutoff(float angle);

    /*!
     * Pseudrange/Delta-Phase filter (PDPFILTER)- smooths positions
     * and bridges gaps in GPS coverage. Enabled by default on
     * OEMStar receiver.
     */
    void PDPFilterDisable();
    void PDPFilterEnable();
    void PDPFilterReset();
    void PDPModeConfigure(PDPMode mode, PDPDynamics dynamics);

    /*!
     * SetPositionTimeout (POSTIMEOUT) sets the timeout value for the
     * position calculation. In position logs, the position_type field
     * is set to NONE when this timeout expires (0 - 86400 sec)
     */
    void SetPositionTimeout(uint32_t seconds);

    bool SetInitialPosition(double latitude, double longitude, double height);
    bool SetInitialTime(uint32_t gps_week, double gps_seconds);
    bool InjectAlmanac(Almanac almanac);
    /*!
     * SetL1CarrierSmoothing sets the amount of smoothing to be performed on
     * code measurements. L2 smoothing is available in OEMV receivers, but
     * NOT in OEMStar Firmaware receivers.
     *      l1_time_constant : 2<= time constant <= 2000 [sec]
     *      l2_time_constant : 5<= time constant <= 2000 [sec] (firmware default = 100)
     */
    bool SetCarrierSmoothing(uint32_t l1_time_constant, uint32_t l2_time_constant);

    bool HardwareReset();
    /*!
     * HotStartReset
     * Restarts the GPS receiver, initialized with
     * Ephemeris, Almanac, Position, Time, etc.
     */
    bool HotStartReset();
    /*!
     * WarmStartReset
     * Restarts the GPS receiver, initialized with
     * Ephemeris, Almanac, NOT Position and Time info
     */
    bool WarmStartReset();
    /*!
     * ColdStartReset
     * Restarts the GPS receiver, initialized without
     * any initial or aiding data.
     */
    bool ColdStartReset();

    void SendRawEphemeridesToReceiver(RawEphemerides raw_ephemerides);

    /*!
     * Requests version information from the receiver
     *
     * This requests the VERSION message from the receiver and
     * uses the result to populate the receiver capapbilities
     *
     * @return True if the GPS was found, false if it was not.
     */
	bool UpdateVersion();

    bool ConvertLLaUTM(double Lat, double Long, double *northing, double *easting, int *zone, bool *north);

    void ReadFromFile(unsigned char* buffer, unsigned int length);

    // Set data callbacks
    void set_best_gps_position_callback(BestGpsPositionCallback handler){
        best_gps_position_callback_=handler;};
    void set_best_lever_arm_callback(BestLeverArmCallback handler){
        best_lever_arm_callback_=handler;};
    void set_best_position_callback(BestPositionCallback handler){
        best_position_callback_=handler;};
    void set_best_utm_position_callback(BestUtmPositionCallback handler){
        best_utm_position_callback_=handler;};
    void set_best_velocity_callback(BestVelocityCallback handler){
        best_velocity_callback_=handler;};
    void set_best_position_ecef_callback(BestPositionEcefCallback handler){
        best_position_ecef_callback_=handler;};
    void set_ins_position_velocity_attitude_callback(InsPositionVelocityAttitudeCallback handler){
        ins_position_velocity_attitude_callback_=handler;};
    void set_ins_position_velocity_attitude_short_callback(InsPositionVelocityAttitudeShortCallback handler){
        ins_position_velocity_attitude_short_callback_=handler;};
    void set_vehicle_body_rotation_callback(VehicleBodyRotationCallback handler){
        vehicle_body_rotation_callback_=handler;};
    void set_ins_speed_callback(InsSpeedCallback handler){
        ins_speed_callback_=handler;};
    void set_raw_imu_callback(RawImuCallback handler){
        raw_imu_callback_=handler;};
    void set_raw_imu_short_callback(RawImuShortCallback handler){
        raw_imu_short_callback_=handler;};
    void set_ins_covariance_callback(InsCovarianceCallback handler){
        ins_covariance_callback_=handler;};
    void set_ins_covariance_short_callback(InsCovarianceShortCallback handler){
        ins_covariance_short_callback_=handler;};
    void set_pseudorange_dop_callback(PseudorangeDopCallback handler){
        pseudorange_dop_callback_=handler;};
    void set_rtk_dop_callback(RtkDopCallback handler){
        rtk_dop_callback_=handler;};
    void set_baseline_ecef_callback(BaselineEcefCallback handler){
        baseline_ecef_callback_=handler;};
    void set_ionospheric_model_callback(IonosphericModelCallback handler){
        ionospheric_model_callback_=handler;};
    void set_range_measurements_callback(RangeMeasurementsCallback handler){
        range_measurements_callback_=handler;};
    void set_compressed_range_measurements_callback(CompressedRangeMeasurementsCallback handler){
        compressed_range_measurements_callback_=handler;};
    void set_gps_ephemeris_callback(GpsEphemerisCallback handler){
        gps_ephemeris_callback_=handler;};
    void set_raw_ephemeris_callback(RawEphemerisCallback handler){
        raw_ephemeris_callback_=handler;};
    void set_raw_almanc_callback(RawAlmanacCallback handler){
        raw_almanac_callback_=handler;};
    void set_almanac_callback(AlmanacCallback handler){
        almanac_callback_=handler;};
    void set_satellite_positions_callback(SatellitePositionsCallback handler){
        satellite_positions_callback_=handler;};
    void set_satellite_visibility_callback(SatelliteVisibilityCallback handler){
        satellite_visibility_callback_=handler;};
    void set_time_offset_callback(TimeOffsetCallback handler){
        time_offset_callback_=handler;};
    void set_tracking_status_callback(TrackingStatusCallback handler){
        tracking_status_callback_=handler;};
    void set_receiver_hardware_status_callback(ReceiverHardwareStatusCallback handler){
        receiver_hardware_status_callback_=handler;};
    void set_best_pseudorange_position_callback(BestPseudorangePositionCallback handler){
        best_pseudorange_position_callback_=handler;};
    void set_rtk_position_callback(RtkPositionCallback handler){
        rtk_position_callback_=handler;};

    void set_raw_msg_callback(RawMsgCallback handler) {
        raw_msg_callback_=handler;};
    RawEphemerides test_ephems_;
private:

  bool Connect_(std::string port, int baudrate);


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
	void ParseBinary(unsigned char *message, size_t length, BINARY_LOG_TYPE message_id);

	bool ParseVersion(std::string packet);

	void UnpackCompressedRangeData(const CompressedRangeData &cmp,
	                                     RangeData           &rng);

	double UnpackCompressedPsrStd(const uint16_t &val) const;

	double UnpackCompressedAccumulatedDoppler(
	    const CompressedRangeData &cmp,
	    const double              &uncmpPsr) const;

    bool SendBinaryDataToReceiver(uint8_t* msg_ptr, size_t length);

    unsigned long CRC32Value(int i);
    unsigned long CalculateBlockCRC32 ( unsigned long ulCount, /* Number of bytes in the data block */
                                        unsigned char *ucBuffer ); /* Data block */

    //////////////////////////////////////////////////////
    // Serial port reading members
    //////////////////////////////////////////////////////
	//! Serial port object for communicating with sensor
	serial::Serial *serial_port_;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> read_thread_ptr_;
	bool reading_status_;  //!< True if the read thread is running, false otherwise.

    //////////////////////////////////////////////////////
    // Diagnostic Callbacks
    //////////////////////////////////////////////////////
    HandleAcknowledgementCallback handle_acknowledgement_;
    LogMsgCallback log_debug_;
    LogMsgCallback log_info_;
    LogMsgCallback log_warning_;
    LogMsgCallback log_error_;

    GetTimeCallback time_handler_; //!< Function pointer to callback function for timestamping


    //////////////////////////////////////////////////////
    // New Data Callbacks
    //////////////////////////////////////////////////////
    RawMsgCallback raw_msg_callback_;

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
    RawEphemerisCallback raw_ephemeris_callback_;
    AlmanacCallback almanac_callback_;
    RawAlmanacCallback raw_almanac_callback_;
    SatellitePositionsCallback satellite_positions_callback_;
    SatelliteVisibilityCallback satellite_visibility_callback_;
    TimeOffsetCallback time_offset_callback_;
    TrackingStatusCallback tracking_status_callback_;
    ReceiverHardwareStatusCallback receiver_hardware_status_callback_;
    BestPseudorangePositionCallback best_pseudorange_position_callback_;
    RtkPositionCallback rtk_position_callback_;



	//////////////////////////////////////////////////////
	// Incoming data buffers
	//////////////////////////////////////////////////////
	unsigned char data_buffer_[MAX_NOUT_SIZE];	//!< data currently being buffered to read
	unsigned char* data_read_;		//!< used only in BufferIncomingData - declared here for speed
	size_t bytes_remaining_;	//!< bytes remaining to be read in the current message
	size_t buffer_index_;		//!< index into data_buffer_
	size_t header_length_;	//!< length of the current header being read
	bool reading_acknowledgement_;	//!< true if an acknowledgement is being received
    bool reading_reset_complete_;   //!< true if an {COM#} message confirming receiver reset if complete
	double read_timestamp_; 		//!< time stamp when last serial port read completed
	double parse_timestamp_;		//!< time stamp when last parse began
	BINARY_LOG_TYPE message_id_;	//!< message id of message currently being buffered

    //////////////////////////////////////////////////////
    // Mutex's
    //////////////////////////////////////////////////////
    boost::condition_variable ack_condition_;
    boost::mutex ack_mutex_;
    bool ack_received_;     //!< true if an acknowledgement has been received from the GPS
    boost::condition_variable reset_condition_;
    boost::mutex reset_mutex_;
    bool waiting_for_reset_complete_;     //!< true if GPS has finished resetting and is ready for input

    bool is_connected_; //!< indicates if a connection to the receiver has been established
	//////////////////////////////////////////////////////
    // Receiver information and capabilities
	//////////////////////////////////////////////////////
	std::string protocol_version_;		//!< Receiver version, OEM4, OEMV, OEM6, or UNKNOWN
	std::string serial_number_; //!< Receiver serial number
	std::string hardware_version_; //!< Receiver hardware version
	std::string software_version_; //!< Receiver software version
	std::string model_;				//!< Receiver model number

	bool l2_capable_; //!< Can the receiver handle L1 and L2 or just L1?
	bool raw_capable_; //!< Can the receiver output raw measurements?
	bool rtk_capable_; //!< Can the receiver compute RT2 and/or RT20 positions?
	bool glonass_capable_; //!< Can the receiver receive GLONASS frequencies?
	bool span_capable_;  //!< Is the receiver a SPAN unit?


};
}
#endif
