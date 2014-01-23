#include "Driver.hpp"
#include <sys/ioctl.h>
#include <termios.h>
#include <iostream>
#include <fstream>
#include <boost/lexical_cast.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace dvl_teledyne;

Driver::Driver(){
    m_stream = 0;
    mConfMode = false;
    mDesiredBaudrate = 9600;
    buffer.resize(1000000);
}

void Driver::open(std::string const& uri)
{
    openURI(uri);
    setConfigurationMode();
    if (mDesiredBaudrate != 9600)
        setDesiredBaudrate(mDesiredBaudrate);

    startAcquisition();
}

void Driver::openURI(std::string const& uri)
{
    // Modes:
    //   0 for serial
    //   1 for TCP
    //   2 for UDP
    //   3 for UDP server
    //   4 for file (either Unix sockets or named FIFOs)
    int mode_idx = -1;
    char const* modes[5] = { "serial://", "tcp://", "udp://", "udpserver://", "file://" };
    for (int i = 0; i < 5; ++i)
    {
        if (uri.compare(0, strlen(modes[i]), modes[i]) == 0)
        {
            mode_idx = i;
            break;
        }
    }

    if (mode_idx == -1)
        throw std::runtime_error("unknown URI " + uri);

    std::string device = uri.substr(strlen(modes[mode_idx]));

    // Find a :[additional_info] marker
    std::string::size_type marker = device.find_last_of(":");
    int additional_info = 0;
    if (marker != std::string::npos)
    {
        additional_info = boost::lexical_cast<int>(device.substr(marker + 1));
        device = device.substr(0, marker);
    }

    if (mode_idx == 0)
    { // serial://DEVICE:baudrate
        if (marker == std::string::npos)
            throw std::runtime_error("missing baudrate specification in serial:// URI");
        openSerial(device, additional_info);
        return;
    }
}

void Driver::setFileDescriptor(int fd, bool auto_close)
{
    setMainStream(new std::fdstream(fd, auto_close));
}

void Driver::setMainStream(std::iostream &stream)
{
    delete m_stream;
    m_stream = stream;
}

bool Driver::openSerial(std::string const& port, int baud_rate)
{
    setFileDescriptor(Driver::openSerialIO(port, baud_rate));
    return true;
}

int Driver::openSerialIO(std::string const& port, int baud_rate)
{
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK );
    if (fd == FDStream::INVALID_FD)
        throw std::runtime_error("cannot open device " + port);

    //FileGuard guard(fd);

    struct termios tio;
    memset(&tio, 0, sizeof(termios));
    tio.c_cflag = CS8 | CREAD;    // data bits = 8bit and enable receiver
    tio.c_iflag = IGNBRK; // don't use breaks by default

    // Commit
    if (tcsetattr(fd, TCSANOW, &tio)!=0)
        throw std::runtime_error("Driver::openSerial cannot set serial options");

    if (!setSerialBaudrate(fd, baud_rate))
        throw std::runtime_error("Driver::openSerial cannot set baudrate");

    //guard.release();
    return fd;
}

bool Driver::setSerialBaudrate(int brate) {
    return setSerialBaudrate(getFileDescriptor(), brate);
}

bool Driver::setSerialBaudrate(int fd, int brate) {
    int tc_rate = 0;
#ifdef __gnu_linux__
    bool custom_rate = false;
#endif
    switch(brate) {
    case(1200):
        tc_rate = B1200;
        break;
    case(2400):
        tc_rate = B2400;
        break;
    case(4800):
        tc_rate = B4800;
        break;
        case(9600):
            tc_rate = B9600;
            break;
        case(19200):
            tc_rate = B19200;
            break;
        case(38400):
            tc_rate = B38400;
            break;
        case(57600):
            tc_rate = B57600;
            break;
        case(115200):
            tc_rate = B115200;
            break;
    case(921600):
            tc_rate = B921600;
            break;
        default:
#ifdef __gnu_linux__
        tc_rate = B38400;
        custom_rate = true;
            std::cerr << "Using custom baud rate " << brate << std::endl;
#else
            std::cerr << "Non-standard baud rate selected. This is only supported on linux." << std::endl;
            return false;
#endif
    }

#ifdef __gnu_linux__
    struct serial_struct ss;
    ioctl(fd, TIOCGSERIAL, &ss);
    if( custom_rate )
    {
    ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
    ss.custom_divisor = (ss.baud_base + (brate / 2)) / brate;
    int closestSpeed = ss.baud_base / ss.custom_divisor;

    if (closestSpeed < brate * 98 / 100 || closestSpeed > brate * 102 / 100)
    {
        std::cerr << "Cannot set custom serial rate to " << brate
        << ". The closest possible value is " << closestSpeed << "."
        << std::endl;
    }
    }
    else
    {
    ss.flags &= ~ASYNC_SPD_MASK;
    }
    ioctl(fd, TIOCSSERIAL, &ss);
#endif

    struct termios termios_p;
    if(tcgetattr(fd, &termios_p)){
        perror("Failed to get terminal info \n");
        return false;
    }

    if(cfsetispeed(&termios_p, tc_rate)){
        perror("Failed to set terminal input speed \n");
        return false;
    }

    if(cfsetospeed(&termios_p, tc_rate)){
        perror("Failed to set terminal output speed \n");
        return false;
    }

    if(tcsetattr(fd, TCSANOW, &termios_p)) {
        perror("Failed to set speed \n");
        return false;
    }
    return true;
}

void Driver::sendConfigurationFile(std::string const& file_name)
{
    setConfigurationMode();

    std::ifstream file(file_name.c_str());

    char line_buffer[2000];
    while (!file.eof())
    {
        if (!file.getline(line_buffer, 2000) && !file.eof())
            throw std::runtime_error("lines longer than 2000 characters");

        std::string line(line_buffer);
        if (line == "CS")
            break;

        line += "\n";
        std::cout << iodrivers_base::Driver::printable_com(line) << std::endl;
        writePacket(reinterpret_cast<uint8_t const*>(line.c_str()), line.length());
        readConfigurationAck();
    }
}

void Driver::setDesiredBaudrate(int rate)
{
    if (getFileDescriptor() != iodrivers_base::Driver::INVALID_FD)
        setDeviceBaudrate(rate);
    mDesiredBaudrate = rate;
}

void Driver::setDeviceBaudrate(int rate)
{
    setConfigurationMode();

    int code = 0;
    switch(rate)
    {
        case 300: code = 0; break;
        case 1200: code = 1; break;
        case 2400: code = 2; break;
        case 4800: code = 3; break;
        case 9600: code = 4; break;
        case 19200: code = 5; break;
        case 38400: code = 6; break;
        case 57600: code = 7; break;
        case 115200: code = 8; break;
        default: throw std::runtime_error("invalid baud rate specified");
    }
    uint8_t data[7] = { 'C', 'B', '0' + code, '1', '1', '\n', 0 };
    writePacket(data, 6, 100);
    readConfigurationAck();
}

void Driver::read()
{
    int packet_size = readPacket(&buffer[0], buffer.size());
    if (packet_size)
        parseEnsemble(&buffer[0], packet_size);
}

void Driver::setReadTimeout(ros::Duration const timeout)
{ m_read_timeout = timeout; }
ros::Time Driver::getReadTimeout() const
{ return m_read_timeout; }

int Driver::readPacket(uint8_t* buffer, int buffer_size,
        ros::Time const& packet_timeout)
{
    return readPacket(buffer, buffer_size, packet_timeout,
            packet_timeout + ros::Time::fromSeconds(1));
}
int Driver::readPacket(uint8_t* buffer, int buffer_size,
        base::Time const& packet_timeout, ros::Time const& first_byte_timeout)
{
    return readPacket(buffer, buffer_size, packet_timeout.toMilliseconds(),
            first_byte_timeout.toMilliseconds());
}
int Driver::readPacket(uint8_t* buffer, int buffer_size, int packet_timeout, int first_byte_timeout)
{
    if (first_byte_timeout > packet_timeout)
        first_byte_timeout = -1;

    if (buffer_size < MAX_PACKET_SIZE)
        throw length_error("readPacket(): provided buffer too small (got " + boost::lexical_cast<string>(buffer_size) + ", expected at least " + boost::lexical_cast<string>(MAX_PACKET_SIZE) + ")");

    if (!isValid())
    {
        // No valid file descriptor. Assume that the user is using the raw data
        // interface (i.e. that the data is already in the internal read buffer)
        pair<int, bool> result = extractPacketFromInternalBuffer(buffer, buffer_size);
        if (result.first)
            return result.first;
        else
            throw TimeoutError(TimeoutError::PACKET, "readPacket(): no packet in the internal buffer and no FD to read from");
    }

    Timeout time_out;
    bool read_something = false;
    while(true) {
        // cerr << endl;

    pair<int, bool> read_state = readPacketInternal(buffer, buffer_size);

    int packet_size     = read_state.first;

    read_something = read_something || read_state.second;

    if (packet_size > 0)
            return packet_size;

        int timeout;
        TimeoutError::TIMEOUT_TYPE timeout_type;
        if (first_byte_timeout != -1 && !read_something)
        {
            timeout = first_byte_timeout;
            timeout_type = TimeoutError::FIRST_BYTE;
        }
        else
        {
            timeout = packet_timeout;
            timeout_type = TimeoutError::PACKET;
        }

        if (time_out.elapsed(timeout))
            throw TimeoutError(timeout_type, "readPacket(): timeout");

        try {
            int remaining_timeout = time_out.timeLeft(timeout);
            m_stream->waitRead(base::Time::fromMicroseconds(remaining_timeout * 1000));
        }
        catch(TimeoutError& e)
        {
            e.type = timeout_type;
            throw e;
        }
    }
}

int Driver::extractPacket (uint8_t const *buffer, size_t buffer_size) const
{
    if (mConfMode)
    {
        char const* buffer_as_string = reinterpret_cast<char const*>(buffer);
        if (buffer_as_string[0] == '>')
            return 1;
        else if (buffer_as_string[0] == 'E')
        {
            if (buffer_size > 1 && buffer_as_string[1] != 'R')
                return -1;
            else if (buffer_size > 2 && buffer_as_string[2] != 'R')
                return -1;

            // We have an error. Find \n> and return
            size_t eol = 2;
            for (eol = 2; eol < buffer_size - 1; ++eol)
            {
                if (buffer_as_string[eol] == '\n' && buffer_as_string[eol + 1] == '>')
                    return eol;
            }
            return 0;
        }
        else
            return -1;
    }
    else
    {
        // std::cout << iodrivers_base::Driver::printable_com(buffer, buffer_size) << std::endl;
        return PD0Parser::extractPacket(buffer, buffer_size);
    }
}

void Driver::setConfigurationMode()
{
    if (tcsendbreak(getFileDescriptor(), 0))
        throw std::runtime_error("failed to set configuration mode");
    mConfMode = true;

    // This is a tricky one. As usual with fiddling with serial lines, the
    // device is inaccessible "for a while" (which is unspecified)
    //
    // Repeatedly write a CR on the line and check for an ack (i.e. a prompt).
    // We do it repeatedly so that we are sure that the CR is not lost.
    clear();
    for (int i = 0; i < 12; ++i)
    {
        writePacket(reinterpret_cast<uint8_t const*>("\n"), 1, 100);
        try
        {
            readConfigurationAck(ros::Duration(0.1).toSec());
            clear();
            break;
        }
        catch(iodrivers_base::TimeoutError)
        {
            if (i == 11) throw;
        }
    }
}

void Driver::clear()
{
    if (m_stream)
        m_stream->clear();
}

void Driver::readConfigurationAck(ros::Duration const timeout)
{
    if (!mConfMode)
        throw std::runtime_error("not in configuration mode");
    int packet_size = readPacket(&buffer[0], buffer.size(), timeout);
    if (buffer[0] != '>')
        throw std::runtime_error(std::string(reinterpret_cast<char const*>(&buffer[0]), packet_size));
}

/** Configures the output coordinate system */
void Driver::setOutputConfiguration(OutputConfiguration conf)
{
    if (!mConfMode)
        throw std::runtime_error("not in configuration mode");

    uint8_t mode_codes_1[4] = { '0', '0', '1', '1' };
    uint8_t mode_codes_2[4] = { '0', '1', '0', '1' };
    uint8_t const cmd[7] = {
        'E', 'X',
        mode_codes_1[conf.coordinate_system], mode_codes_2[conf.coordinate_system],
        (conf.use_attitude       ? '1' : '0'),
        (conf.use_3beam_solution ? '1' : '0'),
        (conf.use_bin_mapping ? '1' : '0') };

    writePacket(cmd, 7, 500);
}

void Driver::startAcquisition()
{
    if (!mConfMode)
        throw std::logic_error("not in configuration mode");

    writePacket(reinterpret_cast<uint8_t const*>("PD0\n"), 4, 100);
    readConfigurationAck();
    writePacket(reinterpret_cast<uint8_t const*>("CS\n"), 3, 100);
    readConfigurationAck();
    mConfMode = false;
}

void Driver::setWriteTimeout(ros::Duration const timeout)
{ m_write_timeout = timeout; }
ros::Duration Driver::getWriteTimeout() const
{ return m_write_timeout; }

bool Driver::writePacket(uint8_t const* buffer, int buffer_size)
{
    return writePacket(buffer, buffer_size, getWriteTimeout());
}
bool Driver::writePacket(uint8_t const* buffer, int buffer_size, base::Time const& timeout)
{ return writePacket(buffer, buffer_size, timeout.toMilliseconds()); }
bool Driver::writePacket(uint8_t const* buffer, int buffer_size, int timeout)
{
    Timeout time_out(timeout);
    int written = 0;
    while(true) {
        int c = m_stream->write(buffer + written, buffer_size - written);
        for (set<IOListener*>::iterator it = m_listeners.begin(); it != m_listeners.end(); ++it)
            (*it)->writeData(buffer + written, c);
        written += c;

        if (written == buffer_size) {
            m_stats.stamp = base::Time::now();
        m_stats.tx += buffer_size;
            return true;
        }

        if (time_out.elapsed())
            throw TimeoutError(TimeoutError::PACKET, "writePacket(): timeout");

        int remaining_timeout = time_out.timeLeft();
        m_stream->waitWrite(base::Time::fromMicroseconds(remaining_timeout * 1000));
    }
}
