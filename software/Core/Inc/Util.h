/**
 ******************************************************************************
 * @file Util.h
 * @brief Generic Utility Functions for System Operations
 ******************************************************************************
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include "stdbool.h"
#include "main.h"
#include "can.h"
#include <stdlib.h>
#include "QUTMS_can.h"

/**
 * @brief Map value within range to new range
 * @param x Integer value to map within current range
 * @param in_min Lower bounds of current range
 * @param in_max Upper bounds of current range
 * @param out_min Lower bounds of new range to map x to
 * @param out_max Upper bounds of new range to map x to
 * @return The newly converted x value within the new range
 */
int map(int x, int in_min, int in_max, int out_min, int out_max);

#endif /* INC_UTIL_H_ */
