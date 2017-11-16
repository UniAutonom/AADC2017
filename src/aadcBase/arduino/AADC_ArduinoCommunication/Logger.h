/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra  $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include "arduino_protocol.h"

/*!
 * A Logger class which helps logging the arduino frames.
 *
 * This Logger class creates a log file with a timestamp and the arduino id as a suffix. The
 * error id in the member functions can be looked up in the arduino_protocol.h header.
 *
 */
class Logger
{
public:

    /*! Default constructor. */
    Logger() : _logging_active(false)
    {}

    /*! Destructor. */
    ~Logger()
    {}

    /*!
     * Starts.
     *
     * \param   arduino_id  Identifier for the arduino.
     */
    void start(int arduino_id)
    {
        boost::posix_time::ptime time = boost::posix_time::second_clock::local_time();
        std::stringstream ss;
        std::string s;
        ss << time.date().year() << std::setfill('0') << std::setw(2) << static_cast<int>(time.date().month()) << time.date().day()
           << std::setfill('0') << std::setw(2) << time.time_of_day().hours()
           << std::setfill('0') << std::setw(2) << time.time_of_day().minutes()
           << std::setfill('0') << std::setw(2) << time.time_of_day().seconds()
           << "-" << arduino_id;

        s = ss.str();
        if (!_logging_active)
        {
            _logging_file.open(s.c_str());
            _logging_active = true;
        }
    }

    /*! Stops logging. */
    void stop()
    {
        if (_logging_active)
        {
            _logging_file.close();
            _logging_active = false;
        }
    }

    /*!
     * Logs increment frame.
     *
     * \param           error_id    Identifier for the error.
     * \param [in,out]  frame       The frame.
     */
    void log_inc_frame(int error_id, std::vector<uint8_t>& frame)
    {
        if (_logging_active)
        {
            boost::lock_guard<boost::mutex> lock(_mutex);
            _logging_file << timestamp() << "[RECEIVED]";
            if (error_id < 0)
            {
                switch (error_id)
                {
                case ERROR_STX_NOT_FOUND:
                    _logging_file << "[STX NOT FOUND]";
                    break;
                case ERROR_ESC_BYTE_BROKEN:
                    _logging_file << "[ESC NOT FOUND]";
                    break;
                case ERROR_ETX_NOT_FOUND:
                    _logging_file << "[ETX NOT FOUND]";
                    break;
                case ERROR_NO_FRAME_DATA:
                    _logging_file << "[NO MORE DATA]";
                    break;
                case ERROR_CRC_INVALID:
                    _logging_file << "[INVALID CRC]";
                    break;
                case ERROR_NO_BYTES_AVAILABLE:
                    _logging_file << "[NO DATA]";
                    break;
                case ERROR_FRAME_DROPPED:
                    _logging_file << "[POPPED]";
                    break;
                default:
                    break;
                }
            }

            _logging_file << "[ ";
            for (size_t i = 0; i < frame.size(); i++)
            {
                _logging_file << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(frame.data()[i]) << " ";
            }
            _logging_file << "]" << std::dec << std::endl;
        }
    }

    /*!
     * Logs out frame.
     *
     * \param           error_id    Identifier for the error.
     * \param [in,out]  frame       The frame.
     */
    void log_out_frame(int error_id, std::vector<uint8_t>& frame)
    {
        if (_logging_active)
        {
            boost::lock_guard<boost::mutex> lock(_mutex);
            _logging_file << timestamp() << "[  SENT  ]";

            switch (error_id)
            {
            case ERROR_FRAME_NOT_WRITTEN:
                _logging_file << "[WRITE ERROR]";
                break;
            default:
                break;
            }

            _logging_file << "[ ";
            for (size_t i = 0; i < frame.size(); i++)
            {
                _logging_file << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(frame.data()[i]) << " ";
            }
            _logging_file << "]" << std::dec << std::endl;
        }
    }

    /*!
     * Gets the timestamp.
     *
     * \return  A std::string.
     */
    static std::string timestamp()
    {
        boost::posix_time::ptime local_time = boost::posix_time::second_clock::local_time();
        boost::posix_time::time_duration clock_time = local_time.time_of_day();

        boost::posix_time::ptime current_date_microseconds = boost::posix_time::microsec_clock::local_time();
        long milliseconds = long(current_date_microseconds.time_of_day().total_milliseconds());

        std::stringstream ss;
        ss << "[ " << clock_time << ":" << std::setfill('0') << std::setw(3) << milliseconds % 1000 << " ]";
        return ss.str();
    }

private:

    /*! True to logging active */
    bool            _logging_active;

    /*! The logging file */
    std::ofstream   _logging_file;

    /*! The mutex */
    boost::mutex    _mutex;
};
#endif // LOGGER_HPP
