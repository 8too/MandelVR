#pragma once

#include <cstdlib>
#include <cstdarg>
#include <windows.h>
#include <compat.h>

static bool g_bPrintf = true;

// Outputs a set of optional arguments to debugging output, using
// the printf format setting specified in fmt*.
inline void dprintf(const char* fmt, ...)
{
	va_list args;
	char buffer[2048];

	va_start(args, fmt);
	vsprintf_s(buffer, fmt, args);
	va_end(args);

	if (g_bPrintf)
		printf("%s", buffer);

	OutputDebugStringA(buffer);
}