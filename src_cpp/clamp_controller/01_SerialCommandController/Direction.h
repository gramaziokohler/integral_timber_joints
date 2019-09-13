// Direction.h

#ifndef _DIRECTION_h
#define _DIRECTION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#pragma once

enum direction {
	EXTEND,
	RETRACT
};

enum motor {
	M1,
	M2
};
#endif

