#ifndef ARCEUS_CONTROL_ARDUINO_COMMS_HPP
#define ARCEUS_CONTROL_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  void send_msg(const std::string &msg_to_send, bool print_output = true)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    if (print_output)
    {
      std::clog << "Sent: " << msg_to_send <<  std::endl;
    }
  }


  //void send_empty_msg()
  //{
  //  std::string response = send_msg("\r");
  //}

  void read_encoder_values(int &val_1, int &val_2, int &val_3, bool print_output = false)
  {
    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '}', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    json jencoders;

    try
    {
      jencoders = json::parse(response);
      val_1 = jencoders["encoders"][0];
      val_2 = jencoders["encoders"][1];
      val_3 = jencoders["encoders"][2];
      if(print_output){
        std::clog << "Received: " << val_1  << " " << val_2 << " " << val_3 << std::endl;
      }
    }
    catch (const json::parse_error& e)
    {
      std::cerr << "Parse error: " << e.what();
    }
  }
  
  void set_motor_values(double val_1, double val_2, double val_3)
  {
    std::stringstream ss;
    ss << val_1 << " " << val_2 << " " << val_3 << "\r";
    std::clog << "Sent: " << val_1  << " " << val_2 << " " << val_3 << std::endl;
    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP