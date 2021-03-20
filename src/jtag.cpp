/*******************************************************************************
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * rbfloader
 * Copyright (c) 2020-2021, The Raetro Authors (see AUTHORS file)
 *
 * rbfloader is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 3 or (at your option) any later version.
 *
 * rbfloader is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with rbfloader. If not, see <https://www.gnu.org/licenses>.
 *
 ******************************************************************************/

/*!*****************************************************************************
 * @file jtag.cpp.c
 * @brief 
 * @copyright (c) 2021 - Marcus Andrade <marcus@raetro.org>
 ******************************************************************************/
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "config.h"
#include "wiringPi.h"
#include "jtag.h"

jtag_pins_conf_t jtag = {
	04, /**< Test Mode Select */
	17, /**< Test Clock */
	22, /**< Test Data In */
	27  /**< Test Data Out */
};

#define MAXIR_CHAINLENGTH 100

const char *rbf_filename;
int IRlen = 0;
int nDevices = 0;

/**
 * @brief
 */
struct codestr {
	Uchar onebit: 1;
	Uint32 manuf: 11;
	Uint32 size: 9;
	Uchar family: 7;
	Uchar rev: 4;
};

/**
 * @brief
 */
union {
	Uint64 code = 0;
	codestr b;
} idcode;

/**
 * @brief
 */
static void jtag_clock() {
	digitalWrite(jtag.tck_pin, HIGH);
	digitalWrite(jtag.tck_pin, LOW);
}

/**
 * @brief
 */
void jtag_setup() {
	pinMode(jtag.tck_pin, OUTPUT);
	pullUpDnControl(jtag.tdo_pin, PUD_UP);
	pinMode(jtag.tdo_pin, INPUT);
	pinMode(jtag.tms_pin, OUTPUT);
	pinMode(jtag.tdi_pin, OUTPUT);

	digitalWrite(jtag.tck_pin, LOW);
	digitalWrite(jtag.tms_pin, LOW);
	digitalWrite(jtag.tdi_pin, LOW);
}

/**
 * @brief
 */
void jtag_release() {
	pullUpDnControl(jtag.tck_pin, PUD_UP);
	pinMode(jtag.tck_pin, INPUT);
	pullUpDnControl(jtag.tdo_pin, PUD_UP);
	pinMode(jtag.tdo_pin, INPUT);
	pullUpDnControl(jtag.tms_pin, PUD_UP);
	pinMode(jtag.tms_pin, INPUT);
	pullUpDnControl(jtag.tdi_pin, PUD_UP);
	pinMode(jtag.tdi_pin, INPUT);
}

/**
 * @brief
 */
static void jtag_reset() {
	int i;
	digitalWrite(jtag.tms_pin, HIGH);
	// go to reset state
	for (i = 0; i < 10; i++) {
		jtag_clock();
	}
}

/**
 * @brief
 */
static void jtag_enter_select_dr() {
	// go to select DR
	digitalWrite(jtag.tms_pin, LOW);
	jtag_clock();
	digitalWrite(jtag.tms_pin, HIGH);
	jtag_clock();
}

/**
 * @brief
 */
static void jtag_enter_shift_ir() {
	digitalWrite(jtag.tms_pin, HIGH);
	jtag_clock();
	digitalWrite(jtag.tms_pin, LOW);
	jtag_clock();
	digitalWrite(jtag.tms_pin, LOW);
	jtag_clock();

}

/**
 * @brief
 */
static void jtag_enter_shift_dr() {
	digitalWrite(jtag.tms_pin, LOW);
	jtag_clock();
	digitalWrite(jtag.tms_pin, LOW);
	jtag_clock();
}

/**
 * @brief
 */
static void jtag_exit_shift() {
	digitalWrite(jtag.tms_pin, HIGH);
	jtag_clock();
	digitalWrite(jtag.tms_pin, HIGH);
	jtag_clock();
	digitalWrite(jtag.tms_pin, HIGH);
	jtag_clock();
}

/**
 * @brief
 * @param bit_length
 * @note: call this function only when in shift-IR or shift-DR state
 */
static void jtag_read_data(int bit_length) {
	int bitofs = 0;
	Uint64 temp;

	bit_length--;
	while (bit_length--) {
		digitalWrite(jtag.tck_pin, HIGH);

		temp = digitalRead(jtag.tdo_pin);
		temp = temp << bitofs;
		idcode.code |= temp;

		digitalWrite(jtag.tck_pin, LOW);
		bitofs++;
	}

	digitalWrite(jtag.tms_pin, HIGH);
	digitalWrite(jtag.tck_pin, HIGH);

	temp = digitalRead(jtag.tdo_pin);
	temp = temp << bitofs;
	idcode.code |= temp;

	digitalWrite(jtag.tck_pin, LOW);
	digitalWrite(jtag.tms_pin, HIGH);
	jtag_clock();
	digitalWrite(jtag.tms_pin, HIGH);
	jtag_clock();  // go back to select-DR
}

/**
 * @brief
 * @param bit_length
 */
static void jtag_read_dr(int bit_length) {
	jtag_enter_shift_dr();
	jtag_read_data(bit_length);
}

/**
 * @brief
 * @return
 */
static int jtag_determine_chain_length() {
	int i;
	// empty the chain (fill it with 0's)
	digitalWrite(jtag.tdi_pin, LOW);
	for (i = 0; i < MAXIR_CHAINLENGTH; i++) {
		digitalWrite(jtag.tms_pin, LOW);
		jtag_clock();
	}
	digitalWrite(jtag.tck_pin, LOW);

	// feed the chain with 1's
	digitalWrite(jtag.tdi_pin, HIGH);
	for (i = 0; i < MAXIR_CHAINLENGTH; i++) {
		digitalWrite(jtag.tck_pin, HIGH);
		if(digitalRead(jtag.tdo_pin) == HIGH) {
			break;
		}
		digitalWrite(jtag.tck_pin, LOW);
	}
	digitalWrite(jtag.tck_pin, LOW);
	jtag_exit_shift();

	return i;
}

/**
 * @brief
 * @return
 */
int jtag_scan() {
	int i;

	jtag_reset();
	jtag_enter_select_dr();
	jtag_enter_shift_ir();

	IRlen = jtag_determine_chain_length();

	jtag_enter_shift_dr();
	nDevices = jtag_determine_chain_length();

	if(IRlen == MAXIR_CHAINLENGTH || nDevices == MAXIR_CHAINLENGTH) {
		LOG("[JTAG] => ERROR!!!!");
		return 1;
	}

	// read the IDCODEs (assume all devices support IDCODE, so read 32 bits per device)
	jtag_reset();
	jtag_enter_select_dr();
	jtag_read_dr(32 * nDevices);

	for (i = 0; i < nDevices; i++) {
		LOG("[JTAG] => Device");
		LOG("[JTAG] =>   IDCODE: %i", idcode.code);
		LOG("[JTAG] =>   Rev   : %i", idcode.b.rev);
		LOG("[JTAG] =>   Family: %i", idcode.b.family);
		LOG("[JTAG] =>   Size  : %i", idcode.b.size);
		LOG("[JTAG] =>   Manuf : %i", idcode.b.manuf);
		LOG("[JTAG] =>   Onebit: %i", idcode.b.onebit);
	}

	return 0;
}

/**
 * @brief
 */
static void jtag_pre_program() {
	int n;
	jtag_reset();
	jtag_enter_select_dr();
	jtag_enter_shift_ir();
	// Here TMS is already low, no need for another command to lower it
	// IR = PROGRAM   = 00 0000 0010
	// IR = CONFIG_IO = 00 0000 1101
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, HIGH); jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);	jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);	jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);	jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);	jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);
	digitalWrite(jtag.tms_pin, HIGH); jtag_clock();
	// here exit ir is the current mode
	digitalWrite(jtag.tms_pin, HIGH); jtag_clock();
	// here update ir is the current mode
	// Drive TDI HIGH while moving to SHIFTDR */
	digitalWrite(jtag.tdi_pin, HIGH);
	digitalWrite(jtag.tms_pin, HIGH); jtag_clock();
	// here select dr scan is the current mode
	digitalWrite(jtag.tms_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tms_pin, LOW);  jtag_clock();
	// here shift dr is the current mode
	// todo: remove extra call?
	// digitalWrite(jtag.tms_pin, LOW); jtag_clock();
	// Issue MAX_JTAG_INIT_CLOCK clocks in SHIFTDR state
	digitalWrite(jtag.tdi_pin, HIGH);
	for (n = 0; n < 300; n++) { jtag_clock(); }
	digitalWrite(jtag.tdi_pin, LOW);
}

/**
 * @brief
 */
static void jtag_post_program() {
	int n;
	// in exit DR
	digitalWrite(jtag.tms_pin, HIGH); jtag_clock();
	// aqui esta no update DR
	digitalWrite(jtag.tms_pin, LOW); jtag_clock();
	// in RUN/IDLE
	jtag_enter_select_dr();
	jtag_enter_shift_ir();
	// in shift ir
	// IR = CHECK STATUS = 00 0000 0100
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, HIGH); jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  //
	digitalWrite(jtag.tms_pin, HIGH); jtag_clock();
	// in exit IR
	digitalWrite(jtag.tms_pin, HIGH); jtag_clock();
	digitalWrite(jtag.tms_pin, HIGH); jtag_clock();
	// in select dr scan
	digitalWrite(jtag.tms_pin, HIGH); jtag_clock();
	digitalWrite(jtag.tms_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tms_pin, LOW);  jtag_clock();
	// in shift IR
	// IR = START = 00 0000 0011
	digitalWrite(jtag.tdi_pin, HIGH); jtag_clock();
	digitalWrite(jtag.tdi_pin, HIGH); jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);  jtag_clock();
	digitalWrite(jtag.tdi_pin, LOW);
	digitalWrite(jtag.tms_pin, HIGH); jtag_clock();
	// in exit IR
	digitalWrite(jtag.tms_pin, HIGH); jtag_clock();
	digitalWrite(jtag.tms_pin, LOW);  jtag_clock();
	// in IDLE
	for (n = 0; n < 200; n++) { jtag_clock(); }
	jtag_reset();
}

/**
 * @brief
 */
void jtag_program_fpga() {
	Uint64 bit_count = 0;
	int n;

	LOG("[JTAG] => Programming ");
	jtag_pre_program();

	// "r" = open for reading / b = for binary
	FILE *fp = fopen(rbf_filename, "rb");
	// validate file open for reading
	if(!fp) {
		fprintf(stderr, "error: file open failed.\n");
		exit(1);
	}
	fseek(fp, 0L, SEEK_END);
	Uint64 total = ftell(fp);
	rewind(fp);

	LOG("[JTAG] => total %i bytes", total);
	int divisor = total / 32;

	while (bit_count < total) { //155224
		Uchar val = fgetc(fp);
		int value;

		if(bit_count % divisor == 0) {
			printf("*");
		}
		bit_count++;
#ifdef TARGET_CYCLONE_II
		// Skips the first 44 characters of the RBF (Cyclone II Header)
		if (bit_count<45) continue;
#endif
		for (n = 0; n <= 7; n++) {
			value = ((val >> n) & 0x01);
			digitalWrite(jtag.tdi_pin, value);
			jtag_clock();
		}
	}
	LOG("[JTAG] => FINISH");

	// AKL (Version1.7): Dump additional 16 bytes of 0xFF at the end of the RBF file
	for (n = 0; n < 127; n++) {
		digitalWrite(jtag.tdi_pin, HIGH);
		jtag_clock();
	}

	digitalWrite(jtag.tdi_pin, HIGH);
	digitalWrite(jtag.tms_pin, HIGH);

	jtag_clock();
	LOG("[JTAG] => Programmed %i bytes", bit_count);
	fclose(fp);  // close the file

	jtag_post_program();
}