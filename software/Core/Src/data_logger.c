/*
 * data_logger.c
 *
 *  Created on: 5 Dec 2020
 *      Author: Calvin Johnson
 */

#include "data_logger.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "sdmmc.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>



#define LOG_SERIAL_QUEUE_SIZE 20
#define LOG_CAN_QUEUE_SIZE 100

osMessageQueueId_t log_serial_queue;
osMessageQueueId_t log_CAN_queue;

const char *serial_filename = "slog.txt";
const char *CAN_filename = "log.CC";
char f_name[80];
char working_folder[80];
FATFS fs;

char serial_buffer[200];

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
	int cur_folder = 0;

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

void log_can_to_sd(CAN_log_t *log_item, int folder) {
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
		data[i] = (log_item->current_ticks >> (i * 8)) & 0xFF;
	}
	log_len += 4;

	// next byte is CAN id type
	data[4] = log_item->can_msg.header.IDE == CAN_ID_EXT;
	log_len += 1;

	// next 4 bytes is CAN id
	uint32_t can_id =
			(log_item->can_msg.header.IDE == CAN_ID_EXT) ?
					log_item->can_msg.header.ExtId :
					log_item->can_msg.header.StdId;
	for (int i = 0; i < 4; i++) {
		data[5 + i] = (can_id >> (i * 8)) & 0xFF;
	}
	log_len += 4;

	// next byte is data length
	data[9] = log_item->can_msg.header.DLC;
	log_len += 1;

	// next 0-8 bytes is data
	for (int i = 0; i < log_item->can_msg.header.DLC; i++) {
		data[10 + i] = log_item->can_msg.data[i];
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

	// copy the string from data into our buffer (so we null terminate string lol)
	strcpy(serial_buffer, log_item->data);

	// write
	res = f_printf(&logfile, "%d:\t%s", log_item->current_ticks,
			serial_buffer);

	// close and save data
	res = f_close(&logfile);

}

void thread_data_logger(void *argument) {
	FRESULT res;

	bool sd_init = true;

	// mount sd card
	res = f_mount(&fs, "", 0);
	if (res != FR_OK) {
		// big sad
		sd_init = false;
	}

	int dir_num = get_latest_folder();

	// find folder name for logging
	if (dir_num == -1) {
		sd_init = false;
	}

	sprintf(working_folder, "%d", dir_num);

	// create new directory for logging
	res = f_mkdir(working_folder);
	if (res != FR_OK) {
		// big sad
		sd_init = false;
	}

	for (;;) {

		// poll for CAN messages (higher priority than serial lol)
		while (osMessageQueueGetCount(log_CAN_queue) >= 1) {
			CAN_log_t log_item;
			if (osMessageQueueGet(log_CAN_queue, &log_item, 0U, 0U) == osOK) {
				// only bother actually logging if sd card is working lol
				if (sd_init) {
					log_can_to_sd(&log_item, dir_num);
				}
			}
		}

		// poll for serial messages
		while (osMessageQueueGetCount(log_serial_queue) >= 1) {
			serial_log_t log_item;
			if (osMessageQueueGet(log_serial_queue, &log_item, 0U, 0U)
					== osOK) {
				// only bother actually logging if sd card is working lol
				if (sd_init) {
					log_serial_to_sd(&log_item, dir_num);
				}
			}
		}

		// run this every 50ms
		osDelay(50);
	}

}

int setup_log_queues() {
	log_serial_queue = osMessageQueueNew(LOG_SERIAL_QUEUE_SIZE,
			sizeof(serial_log_t), NULL);
	log_CAN_queue = osMessageQueueNew(LOG_CAN_QUEUE_SIZE, sizeof(CAN_log_t),
	NULL);

	return ((log_serial_queue != NULL) && (log_CAN_queue != NULL));
}

void add_serial_log(serial_log_t *log_item) {
	if (log_serial_queue != NULL) {
		osMessageQueuePut(log_serial_queue, log_item, 0U, 0U);
	}
}

void add_CAN_log(CAN_log_t *log_item) {
	if (log_CAN_queue != NULL) {
		osMessageQueuePut(log_CAN_queue, log_item, 0U, 0U);
	}
}
