


/*!
 * \file SickPLS.hh
 * \brief Definition of class SickPLS.
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#ifndef SICK_PLS_HH
#define SICK_PLS_HH

/* Implementation dependencies */
#include <string>
#include <iostream>
#include <termios.h>

#include "SickLIDAR.hh"
#include "SickException.hh"

#include "SickPLSBufferMonitor.hh"
#include "SickPLSMessage.hh"

/* Macro definitions */
#define DEFAULT_SICK_PLS_SICK_BAUD                                       (B9600)  ///< Initial baud rate of the PLS (whatever is set in flash)
#define DEFAULT_SICK_PLS_HOST_ADDRESS                                     (0x80)  ///< Client/host default serial address
#define DEFAULT_SICK_PLS_SICK_ADDRESS                                     (0x00)  ///< Sick PLS default serial address
#define DEFAULT_SICK_PLS_SICK_PASSWORD                                "SICK_PLS"  ///< Password for entering installation mode
#define DEFAULT_SICK_PLS_SICK_MESSAGE_TIMEOUT                (unsigned int)(20e6)  ///< The max time to wait for a message reply (usecs)
#define DEFAULT_SICK_PLS_SICK_SWITCH_MODE_TIMEOUT            (unsigned int)(20e6)  ///< Can take the Sick LD up to 3 seconds to reply (usecs)
#define DEFAULT_SICK_PLS_SICK_CONFIG_MESSAGE_TIMEOUT        (unsigned int)(20e6)  ///< The sick can take some time to respond to config commands (usecs)
#define DEFAULT_SICK_PLS_BYTE_INTERVAL                                      (55)  ///< Minimum time in microseconds between transmitted bytes
//#define DEFAULT_SICK_PLS_BYTE_INTERVAL                                      (0)  ///< Minimum time in microseconds between transmitted bytes
#define DEFAULT_SICK_PLS_NUM_TRIES                                           (3)  ///< The max number of tries before giving up on a request

/* Associate the namespace */
namespace sickpls {


    /*!
     * \brief A general class for interfacing w/ SickPLS laser range finders
     *
     * This class implements the basic telegram protocol for SickPLS range finders.
     * It allows the setting of such parameters as angular resolution, fov, etc...
     */
    class SickPLS : public SickLIDAR<SickPLSBufferMonitor, SickPLSMessage> {

    public:

        /** Define the maximum number of measurements */
        /* TODO: I think this should be 180 * 2 */
        static const uint16_t SICK_MAX_NUM_MEASUREMENTS = 721;                     ///< Maximum number of measurements returned by the Sick PLS


        /*!
         * \enum sick_pls_scan_angle_t
         * \brief Defines the scan angle for the Sick PLS.
         */
        enum sick_pls_scan_angle_t {
            /** PLS only does 180 **/
            SICK_SCAN_ANGLE_180 = 180,                                               ///< Scanning angle of 180 degrees
            SICK_SCAN_ANGLE_UNKNOWN = 0xFF                                           ///< Unknown scanning angle
        };

        /*!
         * \enum sick_pls_scan_resolution_t
         * \brief Defines the available resolution settings for the Sick PLS.
         */
        enum sick_pls_scan_resolution_t {
            /** PLS only does 0.5 deg angular resolution **/
            SICK_SCAN_RESOLUTION_50 = 50,                                            ///< 0.50 degree angular resolution
            SICK_SCAN_RESOLUTION_UNKNOWN = 0xFF                                      ///< Unknown angular resolution
        };

        /*!
         * \enum sick_pls_measuring_units_t
         * \brief Defines the available Sick PLS measured value units.
         */
        enum sick_pls_measuring_units_t {

            /* Only in cm */
            SICK_MEASURING_UNITS_CM = 0x00,                                          ///< Measured values are in centimeters
            SICK_MEASURING_UNITS_UNKNOWN = 0xFF                                      ///< Unknown units
        };

        /*!
         * \enum sick_pls_status_t
         * \brief Defines the status of the Sick PLS unit.
         */
        enum sick_pls_status_t {
            SICK_STATUS_OK = 0x00,                                                   ///< PLS is OK
            SICK_STATUS_ERROR = 0x01,                                                ///< PLS has encountered an error
            SICK_STATUS_UNKNOWN = 0xFF                                               ///< Unknown PLS status
        };

        /*!
         * \enum sick_pls_operating_mode_t
         * \brief Defines the operating modes supported by Sick PLS.
         * See page 41 of the PLS telegram manual for additional descriptions of these modes.
         */
        enum sick_pls_operating_mode_t {

            /** TODO: Check what options PLS has **/

            SICK_OP_MODE_INSTALLATION = 0x00,                                        ///< Installation mode for writing EEPROM
            SICK_OP_MODE_DIAGNOSTIC = 0x10,                                          ///< Diagnostic mode for testing purposes
            SICK_OP_MODE_MONITOR_STREAM_MIN_VALUE_FOR_EACH_SEGMENT = 0x20,           ///< Streams minimum measured values for each segement
            SICK_OP_MODE_MONITOR_TRIGGER_MIN_VALUE_ON_OBJECT = 0x21,                 ///< Sends the min measured values when object is detected
            SICK_OP_MODE_MONITOR_STREAM_MIN_VERT_DIST_TO_OBJECT = 0x22,              ///< Streams min "vertical distance" to objects
            SICK_OP_MODE_MONITOR_TRIGGER_MIN_VERT_DIST_TO_OBJECT = 0x23,             ///< Sends min vertical distance to object when detected
            SICK_OP_MODE_MONITOR_STREAM_VALUES = 0x24,                               ///< Streams all measured values in a scan
            SICK_OP_MODE_MONITOR_REQUEST_VALUES = 0x25,                              ///< Sends measured range values on request (i.e. when polled)
            SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES = 0x26,                          ///< Streams mean values from a sample size of n consecutive scans
            SICK_OP_MODE_MONITOR_STREAM_VALUES_SUBRANGE = 0x27,                      ///< Streams data from given subrange
            SICK_OP_MODE_MONITOR_STREAM_MEAN_VALUES_SUBRANGE = 0x28,                 ///< Streams mean values over requested subrange
            SICK_OP_MODE_MONITOR_STREAM_VALUES_WITH_FIELDS = 0x29,                   ///< Streams measured values with associated flags
            SICK_OP_MODE_MONITOR_STREAM_VALUES_FROM_PARTIAL_SCAN = 0x2A,             ///< Streams measured values of partial scan directly after measurement
            SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT_FROM_PARTIAL_SCAN = 0x2B,  ///< Streams range and intensity from n partial scans
            SICK_OP_MODE_MONITOR_STREAM_MIN_VALUES_FOR_EACH_SEGMENT_SUBRANGE = 0x2C, ///< Streams minimum measured values for each segment in a sub-range
            SICK_OP_MODE_MONITOR_NAVIGATION = 0x2E,                                  ///< Sick outputs navigation data records
            SICK_OP_MODE_MONITOR_STREAM_RANGE_AND_REFLECT = 0x50,                    ///< Streams measured range from a scan and sub-range of reflectivity values
            SICK_OP_MODE_UNKNOWN = 0xFF                                              ///< Unknown operating mode
        };

        /*!
         * \enum sick_pls_baud_t
         * \brief Defines available Sick PLS baud rates
         */
        enum sick_pls_baud_t {
            SICK_BAUD_9600 = 0x42,                                                   ///< 9600 baud
            SICK_BAUD_19200 = 0x41,                                                  ///< 19200 baud
            SICK_BAUD_38400 = 0x40,                                                  ///< 38400 baud
            SICK_BAUD_500K = 0x48,                                                   ///< 500000 baud
            SICK_BAUD_UNKNOWN = 0xFF                                                 ///< Unknown baud rate
        };


        /*!
         * \struct sick_pls_operating_status_tag
         * \brief A structure for aggregating the data that
         *        collectively defines the operating status
         *        of the device.
         */
        /*!
         * \typedef sick_pls_operating_status_t
         * \brief Adopt c-style convention
         */
        typedef struct sick_pls_operating_status_tag {
            uint16_t sick_scan_angle;                                                ///< Sick scanning angle (deg)
            uint16_t sick_scan_resolution;                                           ///< Sick angular resolution (1/100 deg)
            uint16_t sick_num_motor_revs;                                            ///< Sick number of motor revs
            uint8_t sick_operating_mode;                                             ///< Sick operating mode
            uint8_t sick_laser_mode;                                                 ///< Sick laser is on/off
            uint8_t sick_measuring_units;                                            ///< Sick measuring units {cm,mm}
            uint8_t sick_address;                                                    ///< Sick device address
        } sick_pls_operating_status_t;



        /*!
         * \struct sick_pls_baud_status_tag
         * \brief A structure for aggregating the data that
         *        collectively define the baud config.
         */
        /*!
         * \typedef sick_pls_baud_status_t
         * \brief Adopt c-style convention
         */
        typedef struct sick_pls_baud_status_tag {
            uint16_t sick_baud_rate;                                                 ///< Sick baud as reported by the device
            uint8_t sick_permanent_baud_rate;                                        ///< 0 - When power is switched on baud rate is 9600/1 - configured transmission rate is used
        } sick_pls_baud_status_t;


        /*!
         * \struct sick_pls_scan_profile_b0_tag
         * \brief A structure for aggregating the data that
         *        define a scan profile obtained from reply
         *        B0 (See page 49 Telegram listing)
         */
        /*!
         * \typedef sick_pls_scan_profile_b0_t
         * \brief Adopt c-style convention
         */
        typedef struct sick_pls_scan_profile_b0_tag {
            uint16_t sick_num_measurements;                                          ///< Number of measurements
            uint16_t sick_measurements[SICK_MAX_NUM_MEASUREMENTS];                   ///< Range/reflectivity measurement buffer
            uint8_t sick_telegram_index;                                             ///< Telegram index modulo 256
            uint8_t sick_real_time_scan_index;                                       ///< If real-time scan indices are requested, this value is set (modulo 256)
            uint8_t sick_partial_scan_index;                                         ///< Indicates the start angle of the scan (This is useful for partial scans)
        } sick_pls_scan_profile_b0_t;


        /** Constructor */
        explicit SickPLS(std::string sick_device_path);

        /** Destructor */
        ~SickPLS() override;

        /** Initializes the Sick */
        void Initialize(sick_pls_baud_t desired_baud_rate)
        noexcept(false);

        /** Uninitializes the Sick */
        void Uninitialize() noexcept(false);

        /** Gets the Sick PLS device path */
        [[nodiscard]] std::string GetSickDevicePath() const;

        /** Gets the scan angle currently being used by the device */
        [[nodiscard]] double GetSickScanAngle() const noexcept(false);

        /** Gets the scan resolution currently being used by the device */
        [[nodiscard]] double GetSickScanResolution() const noexcept(false);

        /** Get the current measurement units of the device */
        [[nodiscard]] SickPLS::sick_pls_measuring_units_t GetSickMeasuringUnits() const noexcept(false);

        /** Get the current Sick PLS operating mode */
        [[nodiscard]] sick_pls_operating_mode_t GetSickOperatingMode() const noexcept(false);

        /** Gets measurement data from the Sick. NOTE: Data can be either range or reflectivity given the Sick mode. */
        void GetSickScan(unsigned int* measurement_values, unsigned int& num_measurement_values) noexcept(false);

        /** Acquire the Sick PLS status */
        sick_pls_status_t GetSickStatus() noexcept(false);

        /** Resets Sick PLS field values */
        void ResetSick() noexcept(false);

        /** Get Sick status as a string */
        [[nodiscard]] std::string GetSickStatusAsString() const;




        /*
         * NOTE: The following methods are given to make working with our
         *       predefined types a bit more manageable.
         */


        /** A utility function for converting integers to pls_sick_scan_angle_t */
        static sick_pls_scan_angle_t IntToSickScanAngle(int scan_angle_int);

        /** A utility function for converting ints to pls_sick_scan_resolution_t */
        static sick_pls_scan_resolution_t IntToSickScanResolution(int scan_resolution_int);

        /** A utility function for converting doubles to pls_sick_scan_resolution_t */
        static sick_pls_scan_resolution_t DoubleToSickScanResolution(double scan_resolution_double);

        /** Converts the given bad, returns a string representing that baud rate. */
        static std::string SickBaudToString(sick_pls_baud_t baud_rate);

        /** A utility function for converting integers to pls_baud_t */
        static sick_pls_baud_t IntToSickBaud(int baud_int);

        /** A utility function for converting baud strings to pls_baud_t */
        static sick_pls_baud_t StringToSickBaud(const std::string& baud_str);

        /** Converts the PLS's status to a corresponding string */
        static std::string SickStatusToString(sick_pls_status_t sick_status);

        /** Converts the PLS's measuring mode to a corresponding string */
        static std::string SickOperatingModeToString(sick_pls_operating_mode_t sick_operating_mode);

        /** Converts the PLS's measuring units to a corresponding string */
        static std::string SickMeasuringUnitsToString(sick_pls_measuring_units_t sick_units);

    protected:

        /** A path to the device at which the sick can be accessed. */
        std::string _sick_device_path;

        /** The baud rate at which to communicate with the Sick */
        sick_pls_baud_t _curr_session_baud;

        /** The desired baud rate for communicating w/ the Sick */
        sick_pls_baud_t _desired_session_baud;


        /** The operating parameters of the device */
        sick_pls_operating_status_t _sick_operating_status{};


        /** The baud configuration of the device */
        sick_pls_baud_status_t _sick_baud_status{};


        /** Stores information about the original terminal settings */
        struct termios _old_term{};

        /** Opens the terminal for serial communication. */
        void _setupConnection() noexcept(false) override;

        /** Closes the serial communication terminal. */
        void _teardownConnection() noexcept(false) override;

        /** Sends a message to the PLS and get the expected reply using th 0x80 rule.   @todo Check difference in comments? */
        void _sendMessageAndGetReply(const SickPLSMessage& sick_send_message,
                                     SickPLSMessage& sick_recv_message,
                                     unsigned int timeout_value,
                                     unsigned int num_tries) noexcept(false);

        /** Sends a message to the PLS and get the expected reply using th 0x80 rule. @todo Check difference in comments? */
        void _sendMessageAndGetReply(const SickPLSMessage& sick_send_message,
                                     SickPLSMessage& sick_recv_message,
                                     uint8_t reply_code,
                                     unsigned int timeout_value,
                                     unsigned int num_tries) noexcept(false);

        /** Flushes the terminal I/O buffers */
        void _flushTerminalBuffer() noexcept(false);

        /** Sets the baud rate for communication with the PLS. */
        void _setSessionBaud(sick_pls_baud_t baud_rate) noexcept(false);

        /** Tests communication wit the PLS at a particular baud rate. */
        bool _testSickBaud(sick_pls_baud_t baud_rate) noexcept(false);

        /** Changes the terminal's baud rate. */
        void _setTerminalBaud(sick_pls_baud_t sick_baud) noexcept(false);

        /** Gets the error status of the Sick PLS */
        void _getSickErrors(unsigned int* num_sick_errors = nullptr,
                            uint8_t* error_type_buffer = nullptr,
                            uint8_t* error_num_buffer = nullptr) noexcept(false);

        /** Switch Sick PLS to installation mode */
        void _setSickOpModeInstallation()
        noexcept(false);

        /** Switch Sick PLS to diagnostic mode */
        void _setSickOpModeDiagnostic()
        noexcept(false);

        /** Switch Sick PLS to monitor mode (request range data) */
        void _setSickOpModeMonitorRequestValues()
        noexcept(false);

        /** Switch Sick PLS to monitor mode (stream range) */
        void _setSickOpModeMonitorStreamValues()
        noexcept(false);

        /** Switches the operating mode of the PLS. */
        void _switchSickOperatingMode(uint8_t sick_mode, const uint8_t* mode_params = nullptr)
        noexcept(false);

        /** Parses the scan profile returned w/ message B0 */
        void
        _parseSickScanProfileB0(const uint8_t* src_buffer, sick_pls_scan_profile_b0_t& sick_scan_profile) const;


        /** Acquires the bit mask to extract the field bit values returned with each range measurement */
        static void _extractSickMeasurementValues(const uint8_t* byte_sequence, uint16_t num_measurements,
                                                  uint16_t* measured_values);

        /** Indicates whether the given scan angle is defined */
        static bool _validSickScanAngle(sick_pls_scan_angle_t sick_scan_angle);

        /** Indicates whether the given scan resolution is defined */
        static bool _validSickScanResolution(sick_pls_scan_resolution_t sick_scan_resolution);

        /** Given a baud rate as an integer, gets a PLS baud rate command. */
        static sick_pls_baud_t _baudToSickBaud(int baud_rate);

    };

    /*!
     * \typedef sick_pls_scan_angle_t
     * \brief Makes working w/ SickPLS::sick_pls_scan_angle_t a bit easier
     */
    typedef SickPLS::sick_pls_scan_angle_t sick_pls_scan_angle_t;

    /*!
     * \typedef sick_pls_scan_resolution_t
     * \brief Makes working w/ SickPLS::sick_pls_scan_resolution_t a bit easier
     */
    typedef SickPLS::sick_pls_scan_resolution_t sick_pls_scan_resolution_t;

    /*!
     * \typedef sick_pls_measuring_units_t
     * \brief Makes working w/ SickPLS::sick_pls_measuring_units_t a bit easier
     */
    typedef SickPLS::sick_pls_measuring_units_t sick_pls_measuring_units_t;


    /*!
     * \typedef sick_pls_operating_mode_t
     * \brief Makes working w/ SickPLS::sick_pls_operating_mode_t a bit easier
     */
    typedef SickPLS::sick_pls_operating_mode_t sick_pls_operating_mode_t;

    /*!
     * \typedef sick_pls_baud_t
     * \brief Makes working w/ SickPLS::sick_pls_baud_t a bit easier
     */
    typedef SickPLS::sick_pls_baud_t sick_pls_baud_t;

} //namespace sickpls

#endif //SICK_PLS_HH
