#ifndef CURRENT_H
#define CURRENT_H
#include <inttypes.h>

/** \file current.c Current mode */

/**
 * \param current [mA]
 * \return success/error
 */
int current_set(int16_t milliamps);

int current_halt(void);

/**
 * Tries to read feedback and blocks untill data arrives.
 * \param pos* [rad]
 * \param vel* [rpm]
 * \param curr* [mA]
 * \param timeout as in poll, typical in millisec, 0 is no timeout and -1
 * \return 0 on success, -1 (MOTOR_ERROR) or -2 (MOTOR_TIMEOUT) on failure
 */
int current_read(int32_t* pos, int32_t* vel, int16_t* curr, int timeout);

#endif // CURRENT_H
