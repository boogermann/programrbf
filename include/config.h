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
 * @brief Interface of JTAG programmer config file
 * @author Victor Trucco
 *
 * @authors Carlos Palmero
 * @authors Fernando Mosquera (Benitoss)
 * @authors Marcus Andrade (Boogermann)
 * @note Code modified from STM32F103 MCU to ARM CPU using the wiringPi library
 ******************************************************************************/

#ifndef PI_RBF_LOADER_CONFIG_H
#define PI_RBF_LOADER_CONFIG_H

// Set up for C function definitions, even when using C++.

#ifdef __cplusplus
extern "C" {
#endif

typedef wchar_t Wchar;       /**< An unsigned char type. */
typedef u_char Uchar;       /**< An unsigned char type. */
typedef uintptr_t UintP;    /**< An unsigned integer type that is capable of storing a data pointer. */
typedef int8_t Sint8;       /**< A signed 8-bit integer type. */
typedef uint8_t Uint8;      /**< An unsigned 8-bit integer type. */
typedef int16_t Sint16;     /**< A signed 16-bit integer type. */
typedef uint16_t Uint16;    /**< An unsigned 16-bit integer type. */
typedef int32_t Sint32;     /**< A signed 32-bit integer type. */
typedef uint32_t Uint32;    /**< An unsigned 32-bit integer type. */
#ifndef MIMIC_NO_64BIT_TYPE
typedef int64_t Sint64;     /**< A signed 64-bit integer type. */
typedef uint64_t Uint64;    /**< An unsigned 64-bit integer type. */
#else
/** This is really just a hack to prevent the compiler from complaining */
typedef struct {
	Uint32 hi;
	Uint32 lo;
} Uint64, Sint64;
#endif

#define LOG(arg, ...) printf(arg, ##__VA_ARGS__ );

/**
 * @brief for bit bang mode this structure provide value for each JTAG signals
 */
typedef struct jtag_pins_conf_t {
	Uint8 tms_pin {};    /**< TMS pin value */
	Uint8 tck_pin {};    /**< TCK pin value */
	Uint8 tdi_pin {};    /**< TDI pin value */
	Uint8 tdo_pin {};    /**< TDO pin value */
} jtag_pins_conf_t;

int parse_config(const char *file_path, jtag_pins_conf_t *gpio);
extern bool use_cyclone_2;

// Ends C function definitions when using C++.
#ifdef __cplusplus
}
#endif

#endif //PI_RBF_LOADER_CONFIG_H
