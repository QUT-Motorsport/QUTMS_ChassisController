/*
 * window_filtering.h
 *
 *  Created on: 9 Mar 2021
 *      Author: Calvin
 */

#ifndef INC_WINDOW_FILTERING_H_
#define INC_WINDOW_FILTERING_H_

#include <stdint.h>

#define WINDOW_FILTERING_COUNT 128

typedef struct window_filter {
	uint16_t current_filtered;
	uint16_t prev_values[WINDOW_FILTERING_COUNT];
	uint32_t running_sum;
	uint16_t current_idx;
	uint8_t initialized;
} window_filter_t;

void initialize_filtering(window_filter_t *filter_data, uint16_t initial_value);

void update_filtering(window_filter_t *filter_data, uint16_t new_value);

#endif /* INC_WINDOW_FILTERING_H_ */
