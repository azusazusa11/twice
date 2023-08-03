#include "common/logger.h"

#include <cstdarg>

namespace twice {

int logger_verbose_level;

void
LOG(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	vfprintf(stderr, format, args);
	va_end(args);
}

void
LOGV(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	if (logger_verbose_level >= 1) {
		vfprintf(stderr, format, args);
	}
	va_end(args);
}

void
LOGVV(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	if (logger_verbose_level >= 2) {
		vfprintf(stderr, format, args);
	}
	va_end(args);
}

} // namespace twice
