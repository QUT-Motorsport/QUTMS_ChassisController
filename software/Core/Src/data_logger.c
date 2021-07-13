/*
 * data_logger.c
 *
 *  Created on: 5 Dec 2020
 *      Author: Calvin Johnson
 */

#include "data_logger.h"
#include "fatfs.h"
#include "sdmmc.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <QUTMS_can.h>

message_queue_t queue_serial_log;
message_queue_t queue_CAN_log;
ms_timer_t timer_data_logger;

int current_dir_num = -1;
bool sd_init = false;

const char *serial_filename = "sl.txt";
const char *CAN_filename = "log.CC";
char f_name[80];
char working_folder[80];
FATFS fs;

#define SERIAL_BUFFER_SIZE 200

char serial_buffer[SERIAL_BUFFER_SIZE];

/**
 * store each log session in it's own folder, which each reboot of CC corresponding to new session
 * looks for folders in root directory, and finds highest numbered folder, and uses that num + 1
 * for name of new folder, or 0 if none are found
 *
 * returns folder number, or -1 if open_dir failed
 */
int get_latest_folder() {
	FRESULT res;
	DIR dir;
	UINT i;
	FILINFO fno;
	int dir_num = -1;
	int cur_folder = -1;

	// open root directory
	res = f_opendir(&dir, "");

	if (res == FR_OK) {
		for (;;) {
			// read next directory item
			res = f_readdir(&dir, &fno);
			if (res != FR_OK || fno.fname[0] == 0)
				break;  // Break on error or end of dir
			if (fno.fattrib & AM_DIR) {
				// found directory
				cur_folder = atoi(fno.fname);

				if (cur_folder > dir_num) {
					dir_num = cur_folder;
				}
			}
		}

		// exited for loop so have found latest folder, so increment for new folder
		dir_num = dir_num + 1;
	}

	return dir_num;
}

void log_can_to_sd(CAN_MSG_Generic_t *log_item, int folder) {
	snprintf(f_name, 80, "%d/%s", folder, CAN_filename);
	FIL logfile;
	FRESULT res;
	UINT num_used;

	res = f_mount(&fs, "", 1);
	if (res != FR_OK) {
		// somethings cooked so don't bother lol
		return;
	}

	// open log file to append to
	res = f_open(&logfile, f_name, FA_WRITE | FA_OPEN_APPEND);

	if (res != FR_OK) {
		// somethings cooked so don't bother lol
		return;
	}

	int log_len = 0;

	// prepare CAN msg for logging
	uint8_t data[18];
	// first four bytes are number of ticks
	for (int i = 0; i < 4; i++) {
		data[i] = (log_item->timestamp >> (i * 8)) & 0xFF;
	}
	log_len += 4;

	// next byte is CAN id type
	data[4] = log_item->ID_TYPE;
	log_len += 1;

	// next 4 bytes is CAN id
	uint32_t can_id = log_item->ID;
	for (int i = 0; i < 4; i++) {
		data[5 + i] = (can_id >> (i * 8)) & 0xFF;
	}
	log_len += 4;

	// next byte is data length
	data[9] = log_item->DLC;
	log_len += 1;

	// next 0-8 bytes is data
	for (int i = 0; i < log_item->DLC; i++) {
		data[10 + i] = log_item->data[i];
		log_len += 1;
	}

	// write
	res = f_write(&logfile, data, log_len, &num_used);

	if (num_used != log_len) {
		// damn bro that shit sucks we should probably log this idk
	}

	// close and save data
	res = f_close(&logfile);

}

void log_serial_to_sd(serial_log_t *log_item, int folder) {
	snprintf(f_name, 80, "%d/%s", folder, serial_filename);
	FIL logfile;
	FRESULT res;

	res = f_mount(&fs, "", 1);
	if (res != FR_OK) {
		// somethings cooked so don't bother lol
		return;
	}

	// open log file to append to
	res = f_open(&logfile, f_name, FA_WRITE | FA_OPEN_APPEND);

	if (res != FR_OK) {
		// somethings cooked so don't bother lol
		return;
	}

	// copy the string from data into our buffer
	memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
	memcpy(serial_buffer, log_item->data, log_item->len);

	// write
	res = f_printf(&logfile, "%s", serial_buffer);

	// close and save data
	res = f_close(&logfile);

}

void setup_data_logger() {
	FRESULT res;

	sd_init = false;

	// mount sd card
	res = f_mount(&fs, "", 0);
	if (res != FR_OK) {
		// big sad
		sd_init = false;
	}

	current_dir_num = get_latest_folder();

	// find folder name for logging
	if (current_dir_num == -1) {
		sd_init = false;
	}

	if (!sd_init) {
		printf("ERROR: unable to find folder\r\n");
	}

	sprintf(working_folder, "%d", current_dir_num);

	if (sd_init) {
		// create new directory for logging
		res = f_mkdir(working_folder);
		if (res != FR_OK) {
			// big sad
			sd_init = false;
		}
	}

	if (!sd_init) {
		printf("ERROR: unable to create directory\r\n");
	}

	// setup queues
	if (queue_init(&queue_CAN_log, sizeof(CAN_MSG_Generic_t),
	LOG_CAN_QUEUE_SIZE) == false) {
		printf("ERROR: FAILED TO SETUP CAN LOG QUEUE\r\n");
	}
	if (queue_init(&queue_serial_log, sizeof(serial_log_t),
	LOG_SERIAL_QUEUE_SIZE) == false) {
		printf("ERROR: FAILED TO SETUP SERIAL LOG QUEUE\r\n");
	}

	// setup timer
	timer_data_logger = timer_init(25, true, data_logger_timer_cb);

	// start timer
	timer_start(&timer_data_logger);
}

void data_logger_timer_cb(void *args) {
	// poll for CAN messages (higher priority than serial lol)

	int log_count = 0;
	int MAX_LOG = 8;

	CAN_MSG_Generic_t msg;
	while (queue_next(&queue_CAN_log, &msg) && log_count < MAX_LOG) {
		// only bother actually logging if sd card is working lol
		if (sd_init) {
			log_can_to_sd(&msg, current_dir_num);
		}
		log_count++;
	}

	serial_log_t log_item;
	while (queue_next(&queue_serial_log, &log_item) && log_count < MAX_LOG) {
		// only bother actually logging if sd card is working lol
		if (sd_init) {
			log_serial_to_sd(&log_item, current_dir_num);
		}
		log_count++;
	}
}
