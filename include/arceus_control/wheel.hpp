#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double inches_per_pulse = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int pulses_per_rev)
    {
      setup(wheel_name, pulses_per_rev);
    }

    
    void setup(const std::string &wheel_name, int pulses_per_rev)
    {
      name = wheel_name;
      inches_per_pulse = (pulses_per_rev/1000)/(2*M_PI*2.5591);
    }

    double calc_enc_angle()
    {
      return enc / inches_per_pulse;
    }



};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP