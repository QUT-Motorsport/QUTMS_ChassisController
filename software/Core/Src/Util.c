/**
 ******************************************************************************
 * @file Util.c
 * @brief Generic Utility Functions for System Operations
 ******************************************************************************
 */

#include "Util.h"

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

