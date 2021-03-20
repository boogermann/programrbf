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
 * @file jtag.h
 * @brief 
 * @copyright (c) 2021 - Marcus Andrade <marcus@raetro.org>
 ******************************************************************************/

#ifndef RBFLOADER_JTAG_H
#define RBFLOADER_JTAG_H

// Set up for C function definitions, even when using C++.
#ifdef __cplusplus
extern "C" {
#endif

extern const char *rbf_filename;

void jtag_setup();
void jtag_release();
int jtag_scan();
void jtag_program_fpga();

// Ends C function definitions when using C++.
#ifdef __cplusplus
}
#endif

#endif //RBFLOADER_JTAG_H
