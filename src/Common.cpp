#include <Common.h>

namespace Time
{
	chrono::steady_clock::time_point Now()
	{
		return chrono::high_resolution_clock::now();
	}

	uint64_t Microseconds(chrono::steady_clock::time_point& from, chrono::steady_clock::time_point& now)
	{
		return std::chrono::duration_cast<std::chrono::microseconds>(now - from).count();
	}

	chrono::steady_clock::time_point End(chrono::steady_clock::time_point& from, const string& message)
	{
		auto now = chrono::high_resolution_clock::now();
		printf("[%s] %.4f ms from start\n", message.c_str(), (float)(Microseconds(from, now)) / 1000.0f);
		return now;
	}
}