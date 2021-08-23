/*

Module:	MCCI_Sigfox_Image.h

Function:
	Header file for Sigfox / ST SX1276 binary libararies for Arduino.

Copyright and License:
	This file copyright (C) 2020 by

		MCCI Corporation
		3520 Krums Corners Road
		Ithaca, NY  14850

	See accompanying LICENSE file for copyright and license information.

Author:
	Dhinesh Kumar Pitchai, MCCI Corporation	September 2020

*/
#ifdef ARDUINO_ARCH_STM32

#ifndef _MCCI_Sigfox_Image_h_
#define _MCCI_Sigfox_Image_h_	/* prevent multiple includes */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// We declare here the external references or function entry points
// that will cause the libraries to be pulled in. Generally we try
// to keep the image library as thin as possible, but it really depends
// on the contents.

#include <it_sdk/itsdk.h>
#include <arduino_wrapper.h>

#ifdef __cplusplus
}
#endif

#endif /* _MCCI_Sigfox_Image_h_ */

#endif // ARDUINO_ARCH_STM32
