/**
 * \file   remregs_serial.cpp
 * \brief  Class for accessing the a register bank over a UART (RS-232, USB converter or local)
 * \author Alessandro Crespi & Jeremie Knuesel
 * \date   August 2016
 */

#include <iostream>
#include <sys/types.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "remregs_serial.h"
#include "wcompat.h"

CSerialRemoteRegs::CSerialRemoteRegs(const uint32_t timeout) : timeout(timeout)
{
  mutex_init(&mutex);
#ifdef _WIN32
  file = NULL;
#else
  file = -1;
#endif
  connected = false;
  sync_state = false;
}

CSerialRemoteRegs::~CSerialRemoteRegs()
{
  if (connected) {
    close();
  }
  mutex_destroy(&mutex);
}

bool CSerialRemoteRegs::read_n(const handle_t file, void* buf, const size_t count)
{
#ifdef _WIN32

  DWORD l;
  if (ReadFile(file, buf, count, &l, NULL)) {
    if (l == count) {
      return true;
    } else {
      SetLastError(ERROR_TIMEOUT);  // on timeout, ReadFile succeeds but returns a lower-or-zero byte count
      return false;
    }
  } else {
    return false;
  }

#else

  timeval to;
  to.tv_sec = (timeout / 1000);
  to.tv_usec = (timeout % 1000) * 1000;
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(file, &fds);

  for (unsigned int i(0); i < count; i++) {
    if (select(file + 1, &fds, NULL, NULL, &to) <= 0) {
      errno = ETIMEDOUT;
      return false;
    }
    if (read(file, ((char*) buf) + i, 1) <= 0) {
      return false;
    }
  }

  return true;
#endif
}

bool CSerialRemoteRegs::write_n(const handle_t file, const void* buf, const size_t count)
{
#ifdef _WIN32
  DWORD l;
  if (WriteFile(file, buf, count, &l, NULL)) {
    if (l == count) {
      return true;
    } else {
      SetLastError(ERROR_TIMEOUT);
      return false;
    }
  } else {
    return false;
  }
#else
  return (write(file, buf, count) == (signed) count);
#endif
}

bool CSerialRemoteRegs::open(const char* portname, int speed)
{
  if (connected) {
    std::cerr << "The serial port is already open." << std::endl;
    return false;
  }

#ifdef _WIN32
  // prepends "\\.\" to the port name (if not already done by the calling program) to make
  // sure we can also open ports higher than COM8 (e.g. COM21), see Win32 SDK documentation
  char* portbuf = new char[strlen(portname) + 6];
  if (portname[0] != '\\') {
    strcpy(portbuf, "\\\\.\\");
    strcat(portbuf, portname);
  } else {
    strcpy(portbuf, portname);
  }

  HANDLE h = CreateFile(portbuf, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, 0);
  if (h == INVALID_HANDLE_VALUE) {
    wperror(portbuf);
    delete[] portbuf;
    return false;
  }
  delete[] portbuf;

  DCB dcb;
  GetCommState(h, &dcb);
  dcb.DCBlength = sizeof(dcb);
  dcb.BaudRate = speed;
  dcb.ByteSize = 8;                      // standard UART settings, 8 data bit, 1 stop bit, no partity
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;  // enable RTS line in case the device requires it
  dcb.fDtrControl = DTR_CONTROL_ENABLE;  // this causes Arduinos to reboot, might be wanted or not, but that's what happens on Linux anyway
  dcb.fBinary = TRUE;                    // Win32 only supports binary data on comm ports anyway
  dcb.fOutX = dcb.fInX = FALSE;          // no in-band flow control (XON/XOFF)
  dcb.fNull = FALSE;                     // don't discard NULL bytes
  dcb.fOutxCtsFlow = FALSE;              // no hardware flow control
  if (!SetCommState(h, &dcb)) {
    wperror("SetCommState");
    CloseHandle(h);
    return false;
  }

  // use a small output buffer to make sure outgoing requests are not delayed
  SetupComm(h, 4096, 16);

  // setup timeouts
  COMMTIMEOUTS ct;
  ZeroMemory(&ct, sizeof(ct));
  ct.ReadTotalTimeoutMultiplier = 1;
  ct.ReadTotalTimeoutConstant = timeout;
  if (!SetCommTimeouts(h, &ct)) {
    wperror("SetCommTimeouts");
    CloseHandle(h);
    return false;
  }

  file = h;

#else
  // Open the serial port device
  int fd = ::open(portname, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    wperror(portname);
    return false;
  }

  // Save old serial port configuration to keep it on exit
  tcgetattr(fd, &oldtio);

  // Set the new configuration: specified baud rate, no parity, 8 bits
  struct termios iop;
  memset(&iop, 0, sizeof(iop));
  iop.c_cflag = CS8 | CREAD | CLOCAL;
  iop.c_cc[VTIME] = 0;
  iop.c_cc[VMIN] = 1;
  if (cfsetospeed(&iop, speed) < 0) {
    wperror("cfsetospeed");          // typical error: 19200 instead of B19200...
    ::close(fd);
    return false;
  }
  cfsetispeed(&iop, speed);
  if (tcsetattr(fd, TCSANOW, &iop) < 0) {
    wperror("tcsetattr");
    ::close(fd);
    return false;
  }
  file = fd;
#endif

  connected = true;
  return true;
}

void CSerialRemoteRegs::close()
{
  mutex_lock(&mutex);
#ifdef _WIN32
  if (file != NULL && file != INVALID_HANDLE_VALUE) {
    CloseHandle(file);
    file = NULL;
  }
#else
  if (connected) {
    tcsetattr(file, TCSAFLUSH, &oldtio);
    ::close(file);
  }
  file = -1;
#endif
  connected = false;
  mutex_unlock(&mutex);
}

bool CSerialRemoteRegs::do_sync()
{
  uint8_t b(0xff);

  sync_state = false;
  for (int i(0); i < 24; i++) {
    if (!write_n(file, &b, 1)) {
      wperror("write_n");
      return false;
    }
  }
  b = 0xAA;
  write_n(file, &b, 1);
  do {
    if (!read_n(file, &b, 1)) {
      wperror("read_n");
      return false;
    }
  } while (b != 0xAA && b != 0x55);      // accepts both 0xAA (BioRob radio interface) and 0x55 (as implemented by ARM-side of radio protocol or Arduino version)
  sync_state = true;
  return true;
}

bool CSerialRemoteRegs::sync()
{
  uint8_t b(0xff);

  mutex_lock(&mutex);
  bool result = do_sync();
  mutex_unlock(&mutex);

  return result;
}

bool CSerialRemoteRegs::reg_op(const uint8_t op, const uint16_t addr, const uint8_t* data, const int len, const bool lock)
{
  uint8_t req[MAX_MB_SIZE + 3];
  
  // prepare the request
  req[0] = (op << 2) | ((addr & 0x300) >> 8);
  req[1] = (addr & 0xFF);
  memcpy(req + 2, data, len);
  for (int i(0); i < len; i++) {
    req[i+2] = data[i];
  }

  // locks the mutex if required
  if (lock) {
    mutex_lock(&mutex);
  }
  
  // check if a sync operation is needed
  if (!sync_state) {
    if (!do_sync()) {
      if (lock) {
        mutex_unlock(&mutex);
      }
      return false;
    }
  }

  if (!write_n(file, req, len + 2)) {
    wperror("write_n");
    if (lock) {
      mutex_unlock(&mutex);
    }
    return false;
  }

  if (!read_n(file, req, 1)) {
    wperror("read_n");
    if (lock) {
      mutex_unlock(&mutex);
    }
    return false;
  }

  if (req[0] == 0xff) {
    sync_state = false;
  }

  if (lock) {
    mutex_unlock(&mutex);
  }

  return (req[0] == ACK);
}

bool CSerialRemoteRegs::get_reg_b(const uint16_t addr, uint8_t& res)
{
  uint8_t buf;

  mutex_lock(&mutex);
  if (!reg_op(ROP_READ_8, addr, NULL, 0)) {
    mutex_unlock(&mutex);
    return false;
  }
  if (!read_n(file, &buf, 1)) {
    wperror("read_n");
    mutex_unlock(&mutex);
    return false;
  }
  res = buf;
  mutex_unlock(&mutex);
  return true;
}

bool CSerialRemoteRegs::get_reg_w(const uint16_t addr, uint16_t& res)
{
  uint16_t buf;

  mutex_lock(&mutex);
  if (!reg_op(ROP_READ_16, addr, NULL, 0)) {
    mutex_unlock(&mutex);
    return false;
  }
  if (!read_n(file, &buf, 2)) {
    wperror("read_n");
    mutex_unlock(&mutex);
    return false;
  }
  res = buf;
  mutex_unlock(&mutex);
  return true;
}

bool CSerialRemoteRegs::get_reg_dw(const uint16_t addr, uint32_t& res)
{
  uint32_t buf;

  mutex_lock(&mutex);
  if (!reg_op(ROP_READ_32, addr, NULL, 0)) {
    mutex_unlock(&mutex);
    return false;
  }
  if (!read_n(file, &buf, 4)) {
    wperror("read_n");
    mutex_unlock(&mutex);
    return false;
  }
  res = buf;
  mutex_unlock(&mutex);
  return true;
}

bool CSerialRemoteRegs::get_reg_mb(const uint16_t addr, uint8_t* data, uint8_t& len)
{
  mutex_lock(&mutex);
  if (!reg_op(ROP_READ_MB, addr, NULL, 0)) {
    mutex_unlock(&mutex);
    return false;
  }
  if (!read_n(file, &len, 1)) {
    wperror("read_n");
    len = 0;
    mutex_unlock(&mutex);
    return false;
  }
  if (!read_n(file, data, len)) {
    wperror("read_n");
    len = 0;
    mutex_unlock(&mutex);
    return false;
  }

  mutex_unlock(&mutex);
  return true;
}

uint8_t CSerialRemoteRegs::get_reg_b(const uint16_t addr)
{
  uint8_t result;
  if (!get_reg_b(addr, result)) {
    return 0xFF;
  } else {
    return result;
  }
}

uint16_t CSerialRemoteRegs::get_reg_w(const uint16_t addr)
{
  uint16_t result;
  if (!get_reg_w(addr, result)) {
    return 0xFFFF;
  } else {
    return result;
  }
}

uint32_t CSerialRemoteRegs::get_reg_dw(const uint16_t addr)
{
  uint32_t result;
  if (!get_reg_dw(addr, result)) {
    return 0xFFFFFFFF;
  } else {
    return result;
  }
}

bool CSerialRemoteRegs::set_reg_b(const uint16_t addr, const uint8_t val)
{
  return (reg_op(ROP_WRITE_8, addr, &val, 1, true));
}

bool CSerialRemoteRegs::set_reg_w(const uint16_t addr, const uint16_t val)
{
  return (reg_op(ROP_WRITE_16, addr, (const uint8_t*) &val, 2, true));
}

bool CSerialRemoteRegs::set_reg_dw(const uint16_t addr, const uint32_t val)
{
  return (reg_op(ROP_WRITE_32, addr, (const uint8_t*) &val, 4, true));
}

bool CSerialRemoteRegs::set_reg_mb(const uint16_t addr, const uint8_t* data, const uint8_t len)
{
  uint8_t buf[MAX_MB_SIZE + 3];
  if (len > MAX_MB_SIZE) {
    return false;
  }
  buf[0] = len;
  for (int i(0); i<len; i++) {
    buf[i+1] = data[i];
  }
  return (reg_op(ROP_WRITE_MB, addr, buf, len + 1, true));
}
