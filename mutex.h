#ifndef __MUTEX_H
#define __MUTEX_H

/**
 * \file   mutex.h
 * \brief  Simple cross-platform macros for mutexes (Win32 or pthread)
 * \author Alessandro Crespi
 * \date   April 2010
 */

#ifdef _WIN32
#include <windows.h>
#define mutex_t CRITICAL_SECTION
#define mutex_init(x) InitializeCriticalSection(x)
#define mutex_lock(x) EnterCriticalSection(x)
#define mutex_unlock(x) LeaveCriticalSection(x)
#define mutex_destroy(x) DeleteCriticalSection(x)
#else
#include <pthread.h>
#define mutex_t pthread_mutex_t
#define mutex_init(x) pthread_mutex_init(x, NULL)
#define mutex_lock(x) pthread_mutex_lock(x)
#define mutex_unlock(x) pthread_mutex_unlock(x)
#define mutex_destroy(x) pthread_mutex_destroy(x)
#endif

#endif
