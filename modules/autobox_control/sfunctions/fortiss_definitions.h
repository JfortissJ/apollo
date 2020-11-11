/******************************************************************************
 * Apollo Autobox Control
 * Copyright (C) 2020 fortiss GmbH
 * Authors: Tobias Kessler, Jianjie Lin
 * 
 * This library is free software; you can redistribute it and/or modify it 
 * under the terms of the GNU Lesser General Public License as published by the 
 * Free Software Foundation; either version 2.1 of the License, or (at your 
 * option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License 
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License 
 * along with this library; if not, write to the Free Software Foundation, 
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *****************************************************************************/

#ifndef _FORTISS_DEFINITIONS_H
#define _FORTISS_DEFINITIONS_H

#define MAX_MSG_LEN 200
#if defined _DS1401
	#include "simstruc_types.h"
	#include <brtenv.h>
	#include "fortiss_rti.h"
#elif (defined MATLAB_MEX_FILE)
	#include "simstruc_types.h"
	#include "fortiss_matlab.h"
#elif defined QEMU_SIMULARTOR_GCC
	#include "fortiss_qemu.h"
#elif defined QEMU_SIMULATOR_POWERPC
	#include "fortiss_qemu.h"
#endif
#endif
