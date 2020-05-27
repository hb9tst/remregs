#ifndef __REMREGS_H
#define __REMREGS_H

/**
 * \file   remregs.h
 * \brief  Abstract class for register operations
 * \author Alessandro Crespi & Jeremie Knuesel
 * \date   June 2016, updated May 2020
 */

#include <stdint.h>


/** Abstract class for a register bank with 8-bit, 16-bit, 32-bit and "multibyte"
  * (arbitrary length, maximum length depending on remote platform) registers.
  */
class CRemoteRegs
{

public:

  /// Destructor
  virtual ~CRemoteRegs() {};

  /// \brief Opens the connection to the register bank
  /// \param portname Filename of the device to open (for local ports) or host name (for remote interfaces)
  /// \param param Baud rate to use (local ports) or TCP port number (for TCP-based remote interfaces)
  virtual bool open(const char* portname, int param) = 0;

  /// Synchronizes the communication between the PC and the interface  
  virtual bool sync() = 0;
  
  /// Closes the connection
  virtual void close() = 0;

  /** \brief Reads a 8-bit register
    * \param addr The address of the register (0 - 1023)
    * \param res Reference to a variable that will contain the read value
    * \return true if the operation succeeded, false if not
    */
  virtual bool get_reg_b(const uint16_t addr, uint8_t& res) = 0;

  /** \brief Reads a 16-bit register
    * \param addr The address of the register (0 - 1023)
    * \param res Reference to a variable that will contain the read value
    * \return true if the operation succeeded, false if not
    */
  virtual bool get_reg_w(const uint16_t addr, uint16_t& res) = 0;
  
  /** \brief Reads a 32-bit register
    * \param addr The address of the register (0 - 1023)
    * \param res Reference to a variable that will contain the read value
    * \return true if the operation succeeded, false if not
    */
  virtual bool get_reg_dw(const uint16_t addr, uint32_t& res) = 0;

  /** \brief Reads a multibyte register
    * \param addr The address of the register (0 - 1023)
    * \param data A pointer to the output buffer (at least 29 bytes long)
    * \param len Reference to a variable that will contain the length of the returned data
    * \return true if the operation succeeded, false if not
    */
  virtual bool get_reg_mb(const uint16_t addr, uint8_t* data, uint8_t& len) = 0;

  /** \brief Reads a 8-bit register
    * \param addr The address of the register (0 - 1023)
    * \return The read value (0x00 - 0xff) or 0xff on failure
    */
  virtual uint8_t get_reg_b(const uint16_t addr) = 0;

  /** \brief Reads a 16-bit register
    * \param addr The address of the register (0 - 1023)
    * \return The read value (0x0000 - 0xffff) or 0xffff on failure
    */
  virtual uint16_t get_reg_w(const uint16_t addr) = 0;

  /** \brief Reads a 32-bit register
    * \param addr The address of the register (0 - 1023)
    * \return The read value (0x00000000 - 0xffffffff) or 0xffffffff on failure
    */
  virtual uint32_t get_reg_dw(const uint16_t addr) = 0;

  /** \brief Writes a 8-bit register
    * \param addr The address of the register (0 - 1023)
    * \param val The value to write to the register
    * \return true if the operation succeeded, false if not
    */  
  virtual bool set_reg_b(const uint16_t addr, const uint8_t val) = 0;
  
  /** \brief Writes a 16-bit register
    * \param addr The address of the register (0 - 1023)
    * \param val The value to write to the register
    * \return true if the operation succeeded, false if not
    */  
  virtual bool set_reg_w(const uint16_t addr, const uint16_t val) = 0;
  
  /** \brief Writes a 32-bit register
    * \param addr The address of the register (0 - 1023)
    * \param val The value to write to the register
    * \return true if the operation succeeded, false if not
    */
  virtual bool set_reg_dw(const uint16_t addr, const uint32_t val) = 0;
  
  /** \brief Writes a multibyte register
    * \param addr The address of the register (0 - 1023)
    * \param data Pointer to the data to write to the register
    * \param len Length of the data to write (0 - 29 bytes)
    * \return true if the operation succeeded, false if not
    */
  virtual bool set_reg_mb(const uint16_t addr, const uint8_t* data, const uint8_t len) = 0;
  
protected:

  /// 8-bit register read
  static const uint8_t ROP_READ_8 = 0;
  /// 16-bit register read
  static const uint8_t ROP_READ_16 = 1;
  /// 32-bit register read
  static const uint8_t ROP_READ_32 = 2;
  /// multibyte register read
  static const uint8_t ROP_READ_MB = 3;
  /// 8-bit register write
  static const uint8_t ROP_WRITE_8 = 4;
  /// 16-bit register write
  static const uint8_t ROP_WRITE_16 = 5;
  /// 32-bit register write
  static const uint8_t ROP_WRITE_32 = 6;
  /// multibyte register write
  static const uint8_t ROP_WRITE_MB = 7;

#ifndef REMREGS_MAX_MB_SIZE
  /// maximum size of a multibyte register
  static const uint8_t MAX_MB_SIZE = 128;
#else
  static const uint8_t MAX_MB_SIZE = REMREGS_MAX_MB_SIZE;
#endif

};

#endif
