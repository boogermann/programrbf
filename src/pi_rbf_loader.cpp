/*******************************************************************************
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2017-2020 - Victor Trucco
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
 * @file fpga_rbf_load.cpp
 * @brief Implementation of a simple JTAG programmer for ARM CPUs
 * @author Victor Trucco
 *
 * @authors Carlos Palmero
 * @authors Fernando Mosquera (Benitoss)
 * @authors Marcus Andrade (Boogermann)
 * @note Code modified from STM32F103 MCU to ARM CPU using the wiringPi library
 ******************************************************************************/

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <unistd.h>

#include "wiringPi.h"
#include "config.h"
#include "jtag.h"
#include "version.h"

int main(int argc, char *argv[]) {
	printf("RBFLoader Version %s\n", PROJECT_VER);

	const char *configFilePath = "/usr/etc/rbfloader.conf";

	jtag_pins_conf_t config = {0};

	if(!parse_config(configFilePath, &config)) {
		printf("Failed to open config file at %s, using default values.\n", configFilePath);
	}

	// Initialize with gpio numbering if want wiringpi numbering change it
	wiringPiSetupGpio();
	//	wiringPiSetup ();

	if(argc <= 2) {
		printf("Usage: %s [rbf_file]\n", argv[0]);
		printf("       rbf_file: bitstream to load\n");
	} else {
		rbf_filename = argv[1];
		// Open rbf file.
		int rbf = open(rbf_filename, (O_RDONLY|O_SYNC));
		if (rbf < 0) {
			// Some error happened...
			printf("\n%s\n\n", "Error opening file. Check for an appropriate fpga_config_file.rbf file.");
			exit(-1);
		}

		printf("Loading rbf file: %s \n", argv[1]);

		jtag_setup();
		if(jtag_scan() == 0) {
			unsigned int startTime = millis();
			unsigned int duration = 0;
			jtag_program_fpga();
			duration = millis() - startTime;
			printf("Programming time: %i ms\n", duration);
		}

		close(rbf);
		jtag_release();

		printf("OK, finished\n");
	}

	return 0;
}
