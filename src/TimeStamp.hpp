#ifndef TIMESTAMP_HPP_
#define TIMESTAMP_HPP_

#include "Config.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace Utils
{
	/**
	 *
	 */
	inline std::string TimeStamp()
	{
		auto microsecs = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());

		std::time_t timer  = std::chrono::duration_cast<std::chrono::seconds>(microsecs).count();
		struct tm tm;
		localtime_r(&timer, &tm); // thread safe version of std::localtime

		std::ostringstream os;
		os.fill('0');
		os 	<< tm.tm_year+1900 << '-'
			<< std::setw(2) << tm.tm_mon+1 << '-'
			<< std::setw(2) << tm.tm_mday << "-"
			<< std::setw(2) << tm.tm_hour << '-'
			<< std::setw(2) << tm.tm_min << '-'
			<< std::setw(2) << tm.tm_sec << '-'
			<< std::setw(6) << std::chrono::duration_cast<std::chrono::microseconds>(microsecs).count();
		return os.str();
	}
} /* namespace Utils */

#endif // SRC_TIMESTAMP_HPP_ 
