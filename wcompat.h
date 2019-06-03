#ifndef __WCOMPAT_H
#define __WCOMPAT_H

/**
 * \file   wcompat.h
 * \brief  Win32 / POSIX compatibility header for remregs
 * \author Alessandro Crespi & Jeremie Knuesel
 * \date   August 2016
 */

#include <cstdio>
#include <cstdlib>

#ifdef _WIN32
#include <windows.h>
#include <winsock.h>

/// Default baud rate of the serial port
#define PORT_SPEED   115200

/// Definition of POSIX usleep() function using Win32 Sleep()
#define usleep(A)    Sleep((A) / 1000)

/// Macro implementation of a Win32 equivalent of perror()
#define wperror(STR) do          \
  {                              \
    DWORD err = GetLastError();  \
    char* msg = NULL;            \
    FormatMessage (FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | 70, NULL, err, LANG_NEUTRAL, (char*) &msg, 2048, NULL); \
    std::cerr << (STR) << ": " << msg << std::endl; \
    LocalFree(msg);              \
  } while (0)

/// Socket type
typedef SOCKET socket_t;

#else

#include <unistd.h>
#include <sys/socket.h>      // for socket(), bind(), and connect()
#include <arpa/inet.h>       // for sockaddr_in and inet_ntoa()
#include <pthread.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <errno.h>

/// \brief Default baud rate of the serial port
/// \warning Please make sure that PORT_SPEED is a valid speed constant (e.g. B19200) and not the speed itself
#define PORT_SPEED   B115200

/// On POSIX, use perror() for wperror()
#define wperror      perror

/// On POSIX, sockets are closed as any file
#define closesocket  ::close

/// Socket type
typedef unsigned int socket_t;

/// Equivalent of Win32 INVALID_SOCKET
#define INVALID_SOCKET (socket_t) (~0)

/// Return code for socket operation errors
#define SOCKET_ERROR -1

#endif  // not Win32

#endif
