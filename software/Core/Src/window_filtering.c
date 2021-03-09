/*
 * window_filtering.c
 *
 *  Created on: 9 Mar 2021
 *      Author: Calvin
 */

#include "window_filtering.h"

void initialize_filtering(window_filter_t *filter_data, uint16_t initial_value) {
	filter_data->current_idx = 0;
	for (int i = 0; i < WINDOW_FILTERING_COUNT; i++) {
		filter_data->prev_values[i] = initial_value;
	}
	filter_data->running_sum = initial_value * WINDOW_FILTERING_COUNT;
	filter_data->current_filtered = initial_value;
	filter_data->initialized = 1;
}

void update_filtering(window_filter_t *filter_data, uint16_t new_value) {
	// simple window filter

	if (filter_data->initialized == 0) {
		initialize_filtering(filter_data, new_value);
	} else {
		// remove the oldest value from the running sum and add the new value
		filter_data->running_sum += new_value
				- filter_data->prev_values[filter_data->current_idx];

		// update the oldest value with the new value (round robin style)
		filter_data->prev_values[filter_data->current_idx] = new_value;

		// increment the index so it wraps around
		filter_data->current_idx = (filter_data->current_idx + 1)
				% WINDOW_FILTERING_COUNT;

		// we know PEDAL_FILTERING_COUNT is multiple of 2 so we can bit shift to do the division
		filter_data->current_filtered = filter_data->running_sum >> 7;
	}
}
