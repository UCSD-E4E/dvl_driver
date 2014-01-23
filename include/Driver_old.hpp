#ifndef DVL_TELEDYNE_DRIVER_HPP
#define DVL_TELEDYNE_DRIVER_HPP

#include "PD0Parser.hpp"
#include <iostream>

namespace dvl_teledyne
{
    class Driver : public PD0Parser
    {
        std::vector<uint8_t> buffer;
        int extractPacket (uint8_t const *buffer, size_t buffer_size) const;

        bool mConfMode;
        int mDesiredBaudrate;

        ros::Duration m_read_timeout;
        ros::Duration m_write_timeout;

        std::iostream m_stream;

        /** Tells the DVL to switch to the desired rate */
        void setDeviceBaudrate(int rate);

    public:
        Driver();

        /** Tries to access the DVL at the provided URI
         *
         * For now, only a serial port can be provided. It is assumed that the
         * DVL is using 9600 bauds (the manufacturer's default)
         */
        void open(std::string const& uri);
        void openURI(std::string const& uri);

        void setFileDescriptor(int fd, bool auto_close=true);
        void setMainStream(std::iostream& stream);
        bool openSerial(std::string const& port, int baud_rate);
        int openSerialIO(std::string const& port, int baud_rate);



        /** Once open using the baudrate specified in the URI, configures the
         * device to output at a different baud rate, and modifies the driver's
         * configuration accordingly
         */
        void setDesiredBaudrate(int rate);

        /** Configures the output coordinate system */
        void setOutputConfiguration(OutputConfiguration conf);
        
        /** Sends a text file that contains commands to the device
         *
         * The device is guaranteed to be in configuration mode afterwards
         * (regardless of whether the configuration file contains a CS
         * command). Use startAcquisition() to put it in acquisition mode
         */
        void sendConfigurationFile(std::string const& file_name);

        /** Sets the device into configuration mode (and make it stop pinging)
         * */
        void setConfigurationMode();

        /** Start acquisition
         *
         * Since the driver relies on receiving PD0 message frames, this method
         * requires the DVL to send into this format, and then starts pinging
         */
        void startAcquisition();

        /** Read available packets on the I/O */
        void read();

        /** Verifies that the DVL acked a configuration command
         *
         * Throws std::runtime_error if an error is reported by the device
         */
        void readConfigurationAck(ros::Duration const timeout = ros::Duration(1).toSec());

        /** Sets/Returns the read timeout variable which is used while reading
          * from the DVL
          */
        void setReadTimeout(ros::Duration const timeout);
        ros::Duration getReadTimeout();

        /** Sets/Returns the write timeout variable which is used while reading
          * from the DVL
          */
        void setWriteTimeout(ros::Duration const timeout);
        ros::Duration getWriteTimeout();
    };
}

#endif

