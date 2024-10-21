#pragma once

#include <limits.h>
#include <stdio.h>

#include <stdHeaderFiles.h>
#include <vtkHeaderFiles.h>

typedef unsigned char ubyte;

class _MAX
{
public:
	const unsigned char UINT8 = 0xFF;
	const unsigned char UI8 = 0xFF;
	const char INT8 = 0x7F;
	const char I8 = 0x7F;
	const unsigned short UINT16 = 0xFFFF;
	const unsigned short UI16 = 0xFFFF;
	const short INT16 = 0x7FFF;
	const short I16 = 0x7FFF;
	const unsigned int UINT32 = 0x7FFFFFFF;
	const unsigned int UI32 = 0x7FFFFFFF;
	const int INT32 = 0xFFFFFFFF;
	const int I32 = 0xFFFFFFFF;
	const unsigned long UINT64 = 0xFFFFFFFFFFFFFFFF;
	const unsigned long UI64 = 0xFFFFFFFFFFFFFFFF;
	const long INT64 = 0x7FFFFFFFFFFFFFFf;
	const long I64 = 0x7FFFFFFFFFFFFFFf;
	const float FLOAT = 0x7F7FFFFF;
	const float F = 0x7F7FFFFF;
	const float DOUBLE = 0x7FEFFFFFFFFFFFFF;
	const float D = 0x7FEFFFFFFFFFFFFF;
};

extern _MAX Max;