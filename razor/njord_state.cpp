/******************************************************************************************
* Test Program: Mac OSX / Unix / Linux C++ Interface for Razor AHRS v1.4.2
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" and "9DOF Sensor Stick"
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
* Written by Peter Bartz (peter-bartz@gmx.de)
*
* Infos, updates, bug reports, contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
******************************************************************************************/

/******************************************************************************************
* Simple modification to include lcm quaternion datatype
* Nick Rypkema
* August 2014
******************************************************************************************/

/******************************************************************************************
* g++ njord_state.cpp RazorAHRS.cpp -Wall -D_REENTRANT -lpthread -o njord_state -llcm
******************************************************************************************/

#include <iostream>   // cout()
#include <iomanip>    // setprecision() etc.
#include <stdexcept>  // runtime_error
#include <cstdio>     // getchar()
#include <cmath>
#include "RazorAHRS.h"
#include <lcm/lcm-cpp.hpp>
#include "njord/quaternion_t.hpp"

using namespace std;

const string serial_port_name = "/dev/ttyUSB0"; // a good guess on linux
float yaw;
float pitch;
float roll;
lcm::LCM _lcm;
njord::quaternion_t quaternion;

// Razor error callback handler
// Will be called from (and in) Razor background thread!
void on_error(const string &msg)
{
  cout << "  " << "ERROR: " << msg << endl;

  // NOTE: make a copy of the message if you want to save it or send it to another thread. Do not
  // save or pass the reference itself, it will not be valid after this function returns!
}

// Euler angle to quaternion calculation from: http://onlineconversion.vbulletin.net/forum/main-forums/convert-and-calculate/3249-euler-angle-quaternion
// φ = roll
// θ = pitch
// ψ = yaw
// q0 = cos(φ/2)*cos(θ/2)*cos(ψ/2)+sin(φ/2)*sin(θ/2)*sin(ψ/2)
// q1 = sin(φ/2)*cos(θ/2)*cos(ψ/2)-cos(φ/2)*sin(θ/2)*sin(ψ/2)
// q2 = cos(φ/2)*sin(θ/2)*cos(ψ/2)+sin(φ/2)*cos(θ/2)*sin(ψ/2)
// q3 = cos(φ/2)*cos(θ/2)*sin(ψ/2)-sin(φ/2)*sin(θ/2)*cos(ψ/2)
// α = 2*acos(cos(φ/2)*cos(θ/2)*cos(ψ/2)+sin(φ/2)*sin(θ/2)*sin(ψ/2))
// βx = acos((sin(φ/2)*cos(θ/2)*cos(ψ/2)-cos(φ/2)*sin(θ/2)*sin(ψ/2))/sin(α/2))
// βy = acos((cos(φ/2)*sin(θ/2)*cos(ψ/2)+sin(φ/2)*cos(θ/2)*sin(ψ/2))/sin(α/2))
// βz = acos((cos(φ/2)*cos(θ/2)*sin(ψ/2)-sin(φ/2)*sin(θ/2)*cos(ψ/2))/sin(α/2))
void calc_quaternion()
{
  double q0 = cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);
  double q1 = sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
  double q2 = cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
  double q3 = cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
  double alpha = 2*acos(cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2));
  double beta_x = acos((sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2))/sin(alpha/2));
  double beta_y = acos((cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2))/sin(alpha/2));
  double beta_z = acos((cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2))/sin(alpha/2));
  quaternion.w = alpha;
  quaternion.x = beta_x;
  quaternion.y = beta_y;
  quaternion.z = beta_z;
  _lcm.publish("NJORD_QUATERNION", &quaternion);
  cout << "  " << fixed << setprecision(5)
  << "w = " << setw(6) << quaternion.w << "      x = " << setw(6) << quaternion.x << "      y = " << setw(6) << quaternion.y << "      z = " << setw(6) << quaternion.z << endl;
}

// Razor data callback handler
// Will be called from (and in) Razor background thread!
// 'data' depends on mode that was set when creating the RazorAHRS object. In this case 'data'
// holds 3 float values: yaw, pitch and roll.
void on_data(const float data[])
{
  cout << "  " << fixed << setprecision(1)
  << "Yaw = " << setw(6) << data[0] << "      Pitch = " << setw(6) << data[1] << "      Roll = " << setw(6) << data[2] << endl;
  yaw = data[0]*M_PI/180.0;
  pitch = data[1]*M_PI/180.0;;
  roll = data[2]*M_PI/180.0;;
  calc_quaternion();
  cout << "  " << fixed << setprecision(5)
  << "--> Yaw = " << setw(6) << yaw << "      Pitch = " << setw(6) << pitch << "      Roll = " << setw(6) << roll << endl;
  // NOTE: make a copy of the yaw/pitch/roll data if you want to save it or send it to another
  // thread. Do not save or pass the pointer itself, it will not be valid after this function
  // returns!

  // If you created the Razor object using RazorAHRS::ACC_MAG_GYR_RAW or RazorAHRS::ACC_MAG_GYR_CALIBRATED
  // instead of RazorAHRS::YAW_PITCH_ROLL, 'data' would contain 9 values that could be printed like this:

  // cout << "  " << fixed << setprecision(1)
  // << "ACC = " << setw(6) << data[0] << ", " << setw(6) << data[1] << ", " << setw(6) << data[2]
  // << "        MAG = " << setw(7) << data[3] << ", " << setw(7) << data[4] << ", " << setw(7) << data[5]
  // << "        GYR = " << setw(7) << data[6] << ", " << setw(7) << data[7] << ", " << setw(7) << data[8] << endl;

}

RazorAHRS *razor;
int main()
{
  cout << "  " << "Connecting to razor IMU..." << endl << endl;

  if(!_lcm.good())
    return 1;

  try
  {
    // Create Razor AHRS object. Serial I/O will run in background thread and report
    // errors and data updates using the callbacks on_data() and on_error().
    // We want to receive yaw/pitch/roll data. If we wanted the unprocessed raw or calibrated sensor
    // data, we would pass RazorAHRS::ACC_MAG_GYR_RAW or RazorAHRS::ACC_MAG_GYR_CALIBRATED
    // instead of RazorAHRS::YAW_PITCH_ROLL.
    razor = new RazorAHRS(serial_port_name, on_data, on_error, RazorAHRS::YAW_PITCH_ROLL);

    // NOTE: If these callback functions were members of a class and not global
    // functions, you would have to bind them before passing. Like this:

    // class Callback
    // {
    //   public:
    //     void on_data(const float ypr[]) { }
    //     void on_error(const string &msg) { }
    // };

    // Callback c;

    // razor = new RazorAHRS(serial_port_name,
    //    bind(&Callback::on_data, &c, placeholders::_1),
    //    bind(&Callback::on_error, &c, placeholders::_1),
    //    RazorAHRS::YAW_PITCH_ROLL);

    // If you're calling from inside of "c" you would of course use "this" instead of "&c".
  }
  catch(runtime_error &e)
  {
    cout << "  " << (string("Could not create tracker: ") + string(e.what())) << endl;
    cout << "  " << "Did you set your serial port correctly?" << endl;
    return 0;
  }

  getchar();  // wait for RETURN key
  return 0;
}

