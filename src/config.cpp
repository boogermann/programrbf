/*******************************************************************************
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2021 - Marcus Andrade <marcus@raetro.org>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the authors nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*!*****************************************************************************
 * @file config.cpp
 * @brief 
 * @copyright (c) 2021 - Marcus Andrade <marcus@raetro.org>
 ******************************************************************************/

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "config.h"

bool use_cyclone_2;

/**
 * @brief Parse Configuration file
 * @param[in] file_path
 * @param[in] gpio
 * @return
 */
int parse_config(const char *file_path, jtag_pins_conf_t *gpio) {
	// Setup default values
	gpio->tms_pin = 04;
	gpio->tck_pin = 17;
	gpio->tdi_pin = 22;
	gpio->tdo_pin = 27;

	FILE *fp;
	char buffer[1024];
	if((fp = fopen(file_path, "r")) != nullptr) {
		fgets(buffer, 1024, fp);
		while (!feof(fp)) {
			if(buffer[0] != '#' && buffer[0] != 0 && strcmp(buffer, "") != 0) {
				char *token = strtok(buffer, " ");

				/* Grab the TMS Settings */
				if(strcmp(token, "TMS") == 0) {
					token = strtok(nullptr, " ");
					if(token[strlen(token) - 1] == '\n') {
						token[strlen(token) - 1] = '\0';
					}
					gpio->tms_pin = atoi(token);
				}
				/* Grab the TCK Settings */
				if(strcmp(token, "TCK") == 0) {
					token = strtok(nullptr, " ");
					if(token[strlen(token) - 1] == '\n') {
						token[strlen(token) - 1] = '\0';
					}
					gpio->tck_pin = atoi(token);
				}
				/* Grab the TDI Settings */
				if(strcmp(token, "TDI") == 0) {
					token = strtok(nullptr, " ");
					if(token[strlen(token) - 1] == '\n') {
						token[strlen(token) - 1] = '\0';
					}
					gpio->tdi_pin = atoi(token);
				}
				/* Grab the TDO Settings */
				if(strcmp(token, "TDO") == 0) {
					token = strtok(nullptr, " ");
					if(token[strlen(token) - 1] == '\n') {
						token[strlen(token) - 1] = '\0';
					}
					gpio->tdo_pin = atoi(token);
				}
				/* Grab the TDO Settings */
				if(strcmp(token, "CYCLONE_II") == 0) {
					token = strtok(nullptr, " ");
					if(token[strlen(token) - 1] == '\n') {
						token[strlen(token) - 1] = '\0';
					}
					use_cyclone_2 = atoi(token);
				}
			}
			fgets(buffer, 1024, fp);
		}
	} else {
		return 0;
	}
	fclose(fp);
	return 1;
}