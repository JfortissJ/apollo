/** <!------------------------------------------------------------------------->
*
*  @file Dstypes.h
*
*  @brief Implementation platform dependent data type definitions.
*
*  @author
*    dSPACE
*
*  @description
*    Implementation platform dependent data type definitions.
*    Common version for all supported targets (PC and embedded).
*
*  @copyright
*    Copyright 1993-2011, dSPACE GmbH. All rights reserved.
*
*  @version
*    $RCSfile: Dstypes.h $ $Revision: 1.20 $ $Date: 2011/10/12 18:19:16MESZ $
*    $ProjectName: e:/ARC/Products/ImplSW/RTLibSW/RTLib1401/Develop/DS1401/RTLib/DSTYPES.pj $
*
*   <hr><br>
*<!-------------------------------------------------------------------------->*/

#ifndef __DSTYPES_H__
#define __DSTYPES_H__

/************************************************************************
  data type definitions for Host programs
************************************************************************/

#if defined _DSHOST || defined _MSC_VER || defined __TURBOC__ || defined __UNIX__ || defined(MATLAB_MEX_FILE)

/* defined identically for OpenInterface */
#ifndef _MCHPUB_
typedef unsigned char           Bool;
typedef char                    Int8;
typedef unsigned char           UInt8;
typedef short int               Int16;
typedef unsigned short int      UInt16;
typedef long int                Int32;
typedef unsigned long int       UInt32;

typedef Bool *                  BoolPtr;
typedef Int8 *                  Int8Ptr;
typedef UInt8 *                 UInt8Ptr;
typedef Int16 *                 Int16Ptr;
typedef UInt16 *                UInt16Ptr;
typedef Int32 *                 Int32Ptr;
typedef UInt32 *                UInt32Ptr;
#endif

typedef struct { UInt32 low; Int32 high;  } Int64;
typedef struct { UInt32 low; UInt32 high; } UInt64;

typedef float                   Float32;
typedef double                  Float64;

typedef Int64 *                 Int64Ptr;
typedef UInt64 *                UInt64Ptr;
typedef Float32 *               Float32Ptr;
typedef Float64 *               Float64Ptr;

#if _MSC_VER == 1200
// long long is not known on VC 6.0
typedef __int64                 Long64;
typedef unsigned __int64        ULong64;
#else
typedef long long               Long64;
typedef unsigned long long      ULong64;
#endif
typedef union { UInt64 ui64; ULong64 ul64; } DsULong64;
typedef Long64 *                Long64Ptr;
typedef ULong64 *               ULong64Ptr;
typedef double                  dsfloat;

/************************************************************************
  data type definitions for Alpha application programs
************************************************************************/

#elif defined _DS1004 || defined __alpha__

typedef double                  dsfloat;

typedef char                    Int8;
typedef unsigned char           UInt8;
typedef short int               Int16;
typedef unsigned short int      UInt16;
typedef int                     Int32;
typedef unsigned int            UInt32;
typedef long int                Int64;
typedef unsigned long int       UInt64;
typedef float                   Float32;
typedef double                  Float64;

typedef Int8 *                  Int8Ptr;
typedef UInt8 *                 UInt8Ptr;
typedef Int16 *                 Int16Ptr;
typedef UInt16 *                UInt16Ptr;
typedef Int32 *                 Int32Ptr;
typedef UInt32 *                UInt32Ptr;
typedef Int64 *                 Int64Ptr;
typedef UInt64 *                UInt64Ptr;
typedef Float32 *               Float32Ptr;
typedef Float64 *               Float64Ptr;

/************************************************************************
  data type definitions for DS1003, DS1201 and DS1102 application programs
************************************************************************/

#elif defined _DS1003 || defined _DS1102 || defined _DS1201 || defined _TMS320C40

typedef float                   dsfloat;

typedef char                    Int8;
typedef unsigned char           UInt8;
typedef int                     Int16;
typedef unsigned int            UInt16;
typedef long int                Int32;
typedef unsigned long int       UInt32;
typedef struct { UInt32 low; Int32 high;  } Int64;
typedef struct { UInt32 low; UInt32 high; } UInt64;
typedef float                   Float32;

typedef Int8 *                  Int8Ptr;
typedef UInt8 *                 UInt8Ptr;
typedef Int16 *                 Int16Ptr;
typedef UInt16 *                UInt16Ptr;
typedef Int32 *                 Int32Ptr;
typedef UInt32 *                UInt32Ptr;
typedef Int64 *                 Int64Ptr;
typedef UInt64 *                UInt64Ptr;
typedef float *                 Float32Ptr;

/************************************************************************
  data type definitions for PPC (big endian) targets with Microtec compiler
************************************************************************/

#elif defined _DS1005 || defined _DS1103 || defined _DS1104 || defined _DS1401 || defined _603 || defined _603e || defined _604 || defined _604e || defined _565 || defined _RP565 || defined _DS1602 || defined __x86_64__

typedef double                  dsfloat;

typedef unsigned char           Bool;
typedef char                    Int8;
typedef unsigned char           UInt8;
typedef short                   Int16;
typedef unsigned short          UInt16;
typedef int                     Int32;
typedef unsigned int            UInt32;
typedef struct { UInt32 low; Int32 high;  } Int64;
typedef struct { UInt32 low; UInt32 high; } UInt64;
typedef long long               Long64;
typedef unsigned long long      ULong64;
typedef float                   Float32;
typedef double                  Float64;

typedef Bool *                  BoolPtr;
typedef Int8 *                  Int8Ptr;
typedef UInt8 *                 UInt8Ptr;
typedef Int16 *                 Int16Ptr;
typedef UInt16 *                UInt16Ptr;
typedef Int32 *                 Int32Ptr;
typedef UInt32 *                UInt32Ptr;
typedef Int64 *                 Int64Ptr;
typedef UInt64 *                UInt64Ptr;
typedef Long64 *                Long64Ptr;
typedef ULong64 *               ULong64Ptr;
typedef Float32 *               Float32Ptr;
typedef Float64 *               Float64Ptr;

/************************************************************************
  data type definitions for 32bit little endian targets with GNU compiler
************************************************************************/

#elif (defined _DS1006) || ((defined DS_PLATFORM_X86) && (defined __GNUC__))
/* applies to all embedded x86 targets */

typedef double                  dsfloat;

typedef unsigned char           Bool;
typedef char                    Int8;
typedef unsigned char           UInt8;
typedef short                   Int16;
typedef unsigned short          UInt16;
typedef int                     Int32;
typedef unsigned int            UInt32;
typedef struct { UInt32 low; Int32 high;  } Int64;
typedef struct { UInt32 low; UInt32 high; } UInt64;
typedef long long               Long64;
typedef unsigned long long      ULong64;
typedef union { UInt64 ui64; ULong64 ul64; } DsULong64;
typedef float                   Float32;
typedef double                  Float64;

typedef Bool *                  BoolPtr;
typedef Int8 *                  Int8Ptr;
typedef UInt8 *                 UInt8Ptr;
typedef Int16 *                 Int16Ptr;
typedef UInt16 *                UInt16Ptr;
typedef Int32 *                 Int32Ptr;
typedef UInt32 *                UInt32Ptr;
typedef Int64 *                 Int64Ptr;
typedef UInt64 *                UInt64Ptr;
typedef Long64 *                Long64Ptr;
typedef ULong64 *               ULong64Ptr;
typedef Float32 *               Float32Ptr;
typedef Float64 *               Float64Ptr;

/************************************************************************
  data type definitions for 32bit big endian targets with GNU compiler
************************************************************************/

#elif (defined DS_PLATFORM_PPC) && (defined __GNUC__)
/* applies to all PowerPC targets using GNU compiler*/

typedef double                  dsfloat;

typedef unsigned char           Bool;
typedef signed char             Int8;
typedef unsigned char           UInt8;
typedef signed short            Int16;
typedef unsigned short          UInt16;
typedef signed int              Int32;
typedef unsigned int            UInt32;
typedef struct { Int32  high; UInt32 low; } Int64;
typedef struct { UInt32 high; UInt32 low; } UInt64;
typedef long long               Long64;
typedef unsigned long long      ULong64;
typedef union { UInt64 ui64; ULong64 ul64; } DsULong64;
typedef float                   Float32;
typedef double                  Float64;

typedef Bool *                  BoolPtr;
typedef Int8 *                  Int8Ptr;
typedef UInt8 *                 UInt8Ptr;
typedef Int16 *                 Int16Ptr;
typedef UInt16 *                UInt16Ptr;
typedef Int32 *                 Int32Ptr;
typedef UInt32 *                UInt32Ptr;
typedef Int64 *                 Int64Ptr;
typedef UInt64 *                UInt64Ptr;
typedef Long64 *                Long64Ptr;
typedef ULong64 *               ULong64Ptr;
typedef Float32 *               Float32Ptr;
typedef Float64 *               Float64Ptr;

/************************************************************************
  data type definitions for TMS320C2xx and C164 application programs
************************************************************************/

#elif defined _TMS320C2XX || defined _C166 || defined __C166__

typedef char                    Int8;
typedef unsigned char           UInt8;
typedef int                     Int16;
typedef unsigned int            UInt16;
typedef long                    Int32;
typedef unsigned long           UInt32;
typedef struct { UInt32 low; UInt32 high; } UInt64;

typedef Int8 *                  Int8Ptr;
typedef UInt8 *                 UInt8Ptr;
typedef Int16 *                 Int16Ptr;
typedef UInt16 *                UInt16Ptr;
typedef Int32 *                 Int32Ptr;
typedef UInt32 *                UInt32Ptr;

/************************************************************************
  data type definitions for Motorola CPU32 (MC68336)
************************************************************************/

#elif defined _MC68336 || defined _DS1401_DIO_TP1

typedef signed char             Int8;
typedef unsigned char           UInt8;
typedef short int               Int16;
typedef unsigned short int      UInt16;
typedef int                     Int32;
typedef unsigned int            UInt32;

typedef Int8 *                  Int8Ptr;
typedef UInt8 *                 UInt8Ptr;
typedef Int16 *                 Int16Ptr;
typedef UInt16 *                UInt16Ptr;
typedef Int32 *                 Int32Ptr;
typedef UInt32 *                UInt32Ptr;

/************************************************************************
  data type definitions PIC 18 (16bit program memory width)
************************************************************************/

#elif defined _PIC16

typedef signed char             Int8;
typedef unsigned char           UInt8;
typedef signed int              Int16;
typedef unsigned int            UInt16;
typedef signed short long       Int24;
typedef unsigned short long     UInt24;
typedef signed long             Int32;
typedef unsigned long           UInt32;
typedef float                   Float32;
typedef unsigned int            size_t;

typedef Int8 *                  Int8Ptr;
typedef UInt8 *                 UInt8Ptr;
typedef Int16 *                 Int16Ptr;
typedef UInt16 *                UInt16Ptr;
typedef Int24 *                 Int24Ptr;
typedef UInt24 *                UInt24Ptr;
typedef Int32 *                 Int32Ptr;
typedef UInt32 *                UInt32Ptr;

/************************************************************************
  data type definitions PIC 16 (14bit program memory width)
************************************************************************/

#elif defined _PIC14

typedef bit                     Bit;
typedef signed char             Int8;
typedef unsigned char           UInt8;
typedef signed int              Int16;
typedef unsigned int            UInt16;
typedef signed long             Int32;
typedef unsigned long           UInt32;
typedef float                   Float24;
typedef double                  Float32;
typedef unsigned int            size_t;

typedef Int8 *                  Int8Ptr;
typedef UInt8 *                 UInt8Ptr;
typedef Int16 *                 Int16Ptr;
typedef UInt16 *                UInt16Ptr;
typedef Int32 *                 Int32Ptr;
typedef UInt32 *                UInt32Ptr;

/************************************************************************
  data type definitions for PPC core e200z6 application programs
  Core does not support floating points with double precision in hardware
************************************************************************/

#elif defined _RP5554 || defined _DS1603

#define Void                    void

typedef float                   dsfloat;

typedef unsigned char           Bool;
typedef signed char             Int8;
typedef unsigned char           UInt8;
typedef signed short            Int16;
typedef unsigned short          UInt16;
typedef signed int              Int32;
typedef unsigned int            UInt32;
typedef struct { UInt32 low; Int32 high;  } Int64;
typedef struct { UInt32 low; UInt32 high; } UInt64;
typedef float                   Float32;

#if __OPTION_VALUE("-Kq")
#define Float64                 double
#else
typedef double                  Float64;
#endif

typedef Bool *                  BoolPtr;
typedef Int8 *                  Int8Ptr;
typedef UInt8 *                 UInt8Ptr;
typedef Int16 *                 Int16Ptr;
typedef UInt16 *                UInt16Ptr;
typedef Int32 *                 Int32Ptr;
typedef UInt32 *                UInt32Ptr;
typedef Int64 *                 Int64Ptr;
typedef UInt64 *                UInt64Ptr;
typedef Float32 *               Float32Ptr;

#if __OPTION_VALUE("-Kq")
#define Float64Ptr              double *
#else
typedef Float64 *               Float64Ptr;
#endif

#else
  #error target platform undefined
#endif

#endif /* __DSTYPES_H__ */

