#ifdef LOG1
#include "log1.h"
#else
#include "log.h"
#endif

#pragma once
class LogWrapper
{
public:

	LogWrapper()
	{
		try
		{
			FILELog::ReportingLevel() = FILELog::FromString("DEBUG1");

			FILE* log_fd;

			fopen_s(&log_fd, "mylogfile.txt", "w");

			Output2FILE::Stream() = log_fd;
		}
		catch (const std::exception& e)
		{
			FILE_LOG(logERROR) << e.what();
		}
	}

	void Log(const char* msg)
	{
		FILE_LOG(logDEBUG) << msg;
	}

	~LogWrapper()
	{
	}
};

