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
#ifndef MODULES_BRIDGE_COMMON_MACROC_H
#define MODULES_BRIDGE_COMMON_MACROC_H

#include <stdlib.h>

#ifdef linux

#include <unistd.h>

#endif
#ifdef _DS1401 //dspace
#include <brtenv.h>
#include <dstypes.h>
#else

#include <stdint.h>

#endif

static const uint32_t APOLLO_BRIDGE_MARCO_FRAME_SIZE = 1024u;
static const int APOLLO_BRIDGE_MARCO_MAXEPOLLSIZE = 100;

#define FREE_ARRY(arry) \
  if (arry) {           \
    delete[] arry;      \
  }                     \
  arry = NULL

#define FREE_POINTER(p) \
  if (p) {              \
    delete p;           \
  }                     \
  p = NULL
#if !defined(ARRAY_SIZE)
	#define ARRAY_SIZE(x) (sizeof((x)) / sizeof((x)[0]))
#endif

#endif

//#ifdef _DS1401 //dspace
//#define SWAPL_MOTOROLA_CUSTOM(value)   ((value & 0xFF000000) >> 24 | (value & 0x00FF0000) >> 8 | (value & 0x0000FF00) << 8 | (value & 0x000000FF) << 24)
//#define UINT32_CAST(value) SWAPL_MOTOROLA_CUSTOM(*((uint32_t*)(value)))
//#else
//#define SWAPL_MOTOROLA_CUSTOM(value) value
//#define UINT32_CAST(value) *((uint32_t*)(value))
//#endif



#if defined _DS1401
	#define REVERSE_DOUBLE_BYTES(result, value)                                  \
		  {                                                                   \
			int _makro_idx = 0;                                               \
			uint8_t* dest = (uint8_t*)&result;                                \
			uint8_t* source = (uint8_t*)&value;                               \
			for (_makro_idx = 0; _makro_idx < sizeof(double); ++_makro_idx) { \
			  dest[_makro_idx] = source[sizeof(double) - _makro_idx - 1];     \
			}                                                                 \
		  }
	#define SWAPL_MOTOROLA_CUSTOM(value)   ((value & 0xFF000000) >> 24 | (value & 0x00FF0000) >> 8 | (value & 0x0000FF00) << 8 | (value & 0x000000FF) << 24)
	#define UINT32_CAST(value) SWAPL_MOTOROLA_CUSTOM(*((uint32_t*)(value)))
	#define DOUBLE_CAST(result, value) REVERSE_DOUBLE_BYTES(result, *((const double*)value))
#elif defined MATLAB_MEX_FILE
	#define SWAPL_MOTOROLA_CUSTOM(value) value
	#define UINT32_CAST(value) *((uint32_t*)(value))
	#define DOUBLE_CAST(result, value) result=*((const double*)value)
#elif defined QEMU_SIMULARTOR_GCC
	#define SWAPL_MOTOROLA_CUSTOM(value) value
	#define UINT32_CAST(value) *((uint32_t*)(value))
	#define DOUBLE_CAST(result, value) result=*((const double*)value)
#elif defined QEMU_SIMULATOR_POWERPC
	#define REVERSE_DOUBLE_BYTES(result, value)                                  \
		{                                                                   \
		  int _makro_idx = 0;                                               \
		  uint8_t* dest = (uint8_t*)&result;                                \
		  uint8_t* source = (uint8_t*)&value;                               \
		  for (_makro_idx = 0; _makro_idx < sizeof(double); ++_makro_idx) { \
			dest[_makro_idx] = source[sizeof(double) - _makro_idx - 1];     \
		  }                                                                 \
		}
	#define SWAPL_MOTOROLA_CUSTOM(value)   ((value & 0xFF000000) >> 24 | (value & 0x00FF0000) >> 8 | (value & 0x0000FF00) << 8 | (value & 0x000000FF) << 24)
	#define UINT32_CAST(value) SWAPL_MOTOROLA_CUSTOM(*((uint32_t*)(value)))
	#define DOUBLE_CAST(result, value) REVERSE_DOUBLE_BYTES(result, *((const double*)value))
#else
	#define SWAPL_MOTOROLA_CUSTOM(value) value
	#define UINT32_CAST(value) *((uint32_t*)(value))
	#define DOUBLE_CAST(result, value) result=*((const double*)value)
#endif
