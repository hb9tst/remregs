#include <iostream>
#include <cmath>
#include <sys/time.h>
#include "remregs_serial.h"
#include "wcompat.h"

using namespace std;

double time_d()
{
#ifdef _WIN32
  FILETIME ft;
  GetSystemTimeAsFileTime(&ft);
  int64_t* val = (int64_t*) &ft;
  return static_cast<double>(*val) / 10000000.0 - 11644473600.0;
#else
  timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + ((double) tv.tv_usec / 1000000.0);
#endif
}

int main(int argc, char* argv[])
{
  CSerialRemoteRegs regs;
  
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " port_name" << endl;
    return 1;
  }

  if (!regs.open(argv[1], PORT_SPEED)) {
    return 1;
  }

  // delay, as open() is resetting the Arduino and there's no way to prevent Linux from doing that

  cout << "Waiting for board start-up... " << flush;
  usleep(2100000);
  
  if (!regs.sync()) {
    cerr << "Synchronization failed on serial port, can't communicate with remote device." << endl;
    return 1;
  }
  
  cout << "synchronization ok!" << endl;
  
  double start = time_d();
  double led_sw = 0;
  uint8_t led = 0;
  int count(0);

  while (true) {
    
    // current time in seconds since startup
    double t = time_d() - start;
    
    // change the state of the LED every 0.5 seconds
    if (t > led_sw + 0.5) {
      led = 1 - led;
      regs.set_reg_b(1, led);
      float f = count / (t - led_sw);
      led_sw = t;
      count = 0;
      cout.precision(1);
      cout << "\rControl freq: " << fixed << f << " Hz, Arduino millis(): " << regs.get_reg_dw(1) << "  " << flush;
    }
    
    // outputs a 0.5 Hz sine wave to the servomotor
    double pos1 = 90 + 45 * sin(0.5 * 2 * M_PI * t);
    double pos2 = 90 + 45 * sin(2 * M_PI * (0.5 * t + M_PI / 4));
    count++;
    /*
    regs.set_reg_b(2, (int) pos1);
    regs.set_reg_b(3, (int) pos2);
    */
    uint8_t buffer[2];
    buffer[0] = (int) pos1;
    buffer[1] = (int) pos2;
    regs.set_reg_mb(1, buffer, 2);
    
  }

  return 0;
}
