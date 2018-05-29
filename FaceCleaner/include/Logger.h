#ifndef FACECLEANER_LOGGER_H
#define FACECLEANER_LOGGER_H

/*****************************************************************************\
**                                                       _ __   ___ __ _     **
**    Face Cleaner                                      | '_ \ / __/ _` |    **
**    Created by:                                       | | | | (_| (_| |    **
**    	Tomas Nekvinda, tom(at)neqindi.cz               |_| |_|\___\__, |    **
**      Copyright (c) anno domini 2018                                |_|    **
**                                                                           **
**    This program is free software: you can redistribute it and/or modify   **
**    it under the terms of the GNU General Public License as published by   **
**    the  Free Software Foundation;  either version 3 of the License,  or   **
**    any later version.	                                                 **
**                                                                           **
**    This program is distributed in the hope that it will be useful, but    **
**    WITHOUT ANY WARRANTY; without even the implied warranty of             **
**    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the General   **
**    Public License (http://www.gnu.org/licenses/) for more details.        **
**                                                                           **
\*****************************************************************************/

/**
 * @brief   A lightweight logger.
 *
 * @file    Logger.h
 * @author  Tomas Nekvinda <tom(at)neqindi.cz>
 * 
 * The logger is slightly adapter code from stackoverflow, see:
 * https://stackoverflow.com/questions/5028302/small-logger-class
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>

// supported types of message importance levels
enum typelog { QUI, ERR, WRN, INF };
// structure which can hold configuration of the logger 
struct structlog { bool header = true; typelog level = INF; std::ofstream* of = nullptr; };

// THE GLOBAL LOOGER INTANCE 
extern structlog LOGCFG;

class LOG
{
public:
	LOG() {}

	/**
	 * @brief Construct a new LOG object
	 * 
	 * @param type 
	 */
	LOG(typelog type) {
		msglevel = type;
		// we should print a header
		if (LOGCFG.header) {
			// getting time out of the damn platform
			auto time = std::time(nullptr);
			std::stringstream ss;
			auto timer = std::time(0);
			std::tm bt{};
#if defined(__unix__)
			localtime_r(&timer, &bt);
#elif defined(_MSC_VER)
			localtime_s(&bt, &timer);
#else
			static std::mutex mtx;
			std::lock_guard<std::mutex> lock(mtx);
			bt = *std::localtime(&timer);
#endif
			ss << std::put_time(&bt, "%T");
			// print a label for the messhage importance level
			operator<<("[" + get_label(type) + " " + ss.str() + "] ");
		}
	}

	/**
	 * @brief Destroy the LOG object and print end of line at the end.
	 * 
	 */
	~LOG() {
		if (opened) {
			std::cout << '\n';
			if (LOGCFG.of != nullptr && LOGCFG.of->is_open()) {
				*LOGCFG.of << '\n';
			}		
		}
		opened = false;
	}

	/**
	 * @brief Write into the Logger.
	 * 
	 * @tparam T 	...
	 * @param msg 	Message to be printed.
	 * @return 		...
	 */
	template<class T>
	LOG &operator<<(const T &msg) {
		if (msglevel <= LOGCFG.level) {
			std::cout << msg;
			if (LOGCFG.of != nullptr && LOGCFG.of->is_open()) {
				*LOGCFG.of << msg;
			}	
			opened = true;
		}
		return *this;
	}

	/**
	 * @brief Add a padding between two strings to match a length.
	 * 
	 * @param str1 First string.
	 * @param str2 Second string.
	 * @param num  Length of the resulting string.
	 * @param pad  Char that will be used as the padding.
	 * @return 	   Concatenation of the strings @a str1 and @a str2 joined by the @a pad to match the length @a num .
	 */
	static std::string pad_string(const std::string &str1, const std::string &str2, const size_t num = 80, const char pad = '.') {
		std::string r = str1 + ' ';
		if (num > r.size()) r.insert(r.end(), num - str1.size() - str2.size() - 2, pad);
		r += ' ' + str2;
		return r;
	}

	/**
	 * @brief Appends padding to a string to match a length.
	 * 
	 * @param str String to be padded.
	 * @param num  Length of the resulting string.
	 * @param pad  Char that will be used as the padding.
	 * @return 	   Extension of the string @a str produced by appending @a pad .
	 */
	static std::string pad_right(const std::string &str, size_t num = 80, const char pad = ' ') {
		if (str.size() < num - 1) return str + ' ' + std::string(num - str.size() - 1, pad);
		else return str;
	}

	/**
	 * @brief Pads a number from left to match a length. 
	 * 
	 * @param n The number to be padded.
	 * @param s The desired length.
	 * @return 	String representing the number @a n but padded from left to match the length @a s .
	 */
	static std::string pad_number_left(size_t n, size_t s) {
		std::string str = std::to_string(n);
		if (str.size() < s) return std::string(s - str.size(), ' ') + str;
		else return str;
	}

private:
	/**
	 * @brief Get the label for a message importance level.
	 * 
	 * @param type Message importnace level.
	 * @return 	   String representing the level.
	 */
	inline std::string get_label(typelog type) {
		std::string label;
		switch (type) {
			case INF: label = "INF"; break;
			case WRN: label = "WRN"; break;
			case ERR: label = "ERR"; break;
			case QUI: label = "QUI"; break;
		}
		return label;
	}

	// keeps state of the logger to determine end and start of lines
	bool opened = false;
	// keeps info about message importance levels for one whole line 
	typelog msglevel = INF;
};

#endif //FACECLEANER_LOGGER_H