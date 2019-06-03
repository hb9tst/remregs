#ifndef __REMREGS_SERIAL_H
#define __REMREGS_SERIAL_H

/**
 * \file   remregs_serial.h
 * \brief  Abstract class for register operations
 * \author Alessandro Crespi & Jeremie Knuesel
 * \date   August 2016
 */

#include <stdint.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <termios.h>
#endif

#include "remregs.h"
#include "mutex.h"

#ifdef _WIN32
typedef HANDLE handle_t;
#else
typedef int handle_t;
#endif

/// \brief Serial port implementation of CRemoteRegs
/// \note See documentation for methods in CRemoteRegs
class CSerialRemoteRegs: virtual public CRemoteRegs
{

public:

  CSerialRemoteRegs(const uint32_t timeout = 500);
  ~CSerialRemoteRegs();
  
  /// \brief Opens the communication with the device on the serial port
  /// \param portname Name of the port (e.g. "COM3" on Win32 systems, "/dev/ttyS2" on UNIX)
  /// \param spd Baud rate to use for the communication (e.g. 57600 for Win32, B57600 on UNIX)
  bool open(const char* portname, int spd);
  bool sync();
  void close();

  bool get_reg_b(const uint16_t addr, uint8_t& res);
  bool get_reg_w(const uint16_t addr, uint16_t& res);
  bool get_reg_dw(const uint16_t addr, uint32_t& res);
  bool get_reg_mb(const uint16_t addr, uint8_t* data, uint8_t& len);

  uint8_t get_reg_b(const uint16_t addr);
  uint16_t get_reg_w(const uint16_t addr);
  uint32_t get_reg_dw(const uint16_t addr);

  bool set_reg_b(const uint16_t addr, const uint8_t val);
  bool set_reg_w(const uint16_t addr, const uint16_t val);
  bool set_reg_dw(const uint16_t addr, const uint32_t val);
  bool set_reg_mb(const uint16_t addr, const uint8_t* data, const uint8_t len);

private:

  static const uint8_t ACK = 6;
  static const uint8_t NAK = 15;

  /// helper method for a generic register operation
  bool reg_op(const uint8_t op, const uint16_t addr, const uint8_t* data, const int len, const bool lock = false);

  /// helper method to read a given number of bytes from a port
  bool read_n(const handle_t file, void* buf, const size_t count);

  /// helper method to write a given number of bytes to a port
  bool write_n(const handle_t file, const void* buf, const size_t count);
  
  /// actual method doing the synchronization, expects the mutex do be locked
  bool do_sync();

#ifndef _WIN32
  struct termios oldtio;
#endif
  handle_t file;
  mutex_t mutex;
  bool connected;
  bool sync_state;
  
  uint32_t timeout;
};

#endif
