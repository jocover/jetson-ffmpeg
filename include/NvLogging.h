/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * <b>NVIDIA Multimedia API: Logging API</b>
 *
 * @brief Description: This file defines macros for logging messages.
 */

#ifndef __NV_LOGGING_H_
#define __NV_LOGGING_H_

#include <iostream>
#include <sstream>

/**
 *
 * @defgroup l4t_mm_nvlogging_group Logging API
 *
 * This file defines macros that provide message logging
 * functionality.
 *
 * @ingroup aa_framework_api_group
 * @{
 */


/**
 * Specifies the log level for Information messages.
 */
#define LOG_LEVEL_INFO  0
/**
 * Specifies the log level for Error messages.
 */
#define LOG_LEVEL_ERROR 1
/**
 * Specifies the log level for Warning messages.
 */
#define LOG_LEVEL_WARN  2
/**
 * Specifies the log level for Debug messages.
 */
#define LOG_LEVEL_DEBUG 3

/**
 * Holds the current log level at runtime by assignment of one of the
 * @c LOG_LEVEL_* values.
 */
extern int log_level;

/**
 * Specifies the default log level.
 */
#define DEFAULT_LOG_LEVEL LOG_LEVEL_ERROR

/**
 * @cond
 */
#define stringify(s) #s
#define xstringify(s) stringify(s)
#define __LINE_NUM_STR__ xstringify(__LINE__)

extern const char *log_level_name[];
/**
 * @endcond
 */

/**
 *
 * Prints log messages.
 *
 * Prints a log message only if the current log_level is greater
 * than or equal to the level of the message.
 *
 * Messages are in the following form:
 * [LEVEL] (FILE: LINE_NUM) Message
 *
 * @param[in] level The Log level of the message.
 * @param[in] str1 The NULL-terminated char array to print.
 */
#define PRINT_MSG(level, str1) if(level <= log_level) { \
                                  std::ostringstream ostr; \
                                  ostr << "[" << log_level_name[level] << "] ("  << \
                                  __FILE__ << ":" __LINE_NUM_STR__ ") " << \
                                  str1 << std::endl; \
                                  std::cerr << ostr.str(); \
                              }

/**
 * Prints a log message of level LOG_LEVEL_INFO.
 */
#define INFO_MSG(str) PRINT_MSG(LOG_LEVEL_INFO, str)
/**
 * Prints a component-specific log message of level LOG_LEVEL_INFO.
 * This is used by the components internally and should not be used by
 * the application.
 *
 * Messages are in the following form:
 * [LEVEL] (FILE: LINE_NUM) <comp_name> <message_content>
 */
#define COMP_INFO_MSG(str) INFO_MSG("<" << comp_name << "> " << str)
/**
 * Prints a category-specific (Component type) system error log
 * message of level LOG_LEVEL_INFO. This is used by the components
 * internally and should not be used by the application.
 *
 * Messages are in the following form:
 * [LEVEL] (FILE: LINE_NUM) <cat_name> <message_content>
 */
#define CAT_INFO_MSG(str) INFO_MSG("<" CAT_NAME "> " << str)

/**
 * Prints a log message of level LOG_LEVEL_ERROR.
 */
#define ERROR_MSG(str) PRINT_MSG(LOG_LEVEL_ERROR, str)
/**
 * Prints a component-specific log message of level
 * LOG_LEVEL_ERROR.  This is used by the components internally
 * and should not be used by the application.
 *
 * Messages are in the following form:
 * [LEVEL] (FILE:LINE_NUM) <comp_name> <message_content>
 */
#define COMP_ERROR_MSG(str) ERROR_MSG("<" << comp_name << "> " << str)
/**
 * Prints a category-specific (Component type) log message of level
 * LOG_LEVEL_ERROR. This is used by the components internally and
 * should not be used by the application.
 *
 * Messages are in the following form:
 * [LEVEL] (FILE:LINE_NUM) <cat_name> <message_content>
 */
#define CAT_ERROR_MSG(str) ERROR_MSG("<" CAT_NAME "> " << str)

/**
 * Prints a system error log message of level LOG_LEVEL_ERROR with
 * the string description of the errno value appended.
 */
#define SYS_ERROR_MSG(str) ERROR_MSG(str << ": " << strerror(errno))
/**
 * Prints a component-specific system error log message of level
 * LOG_LEVEL_ERROR. This is used by the components internally and
 * should not be used by the application.
 *
 * Messages are in the following form:
 * [LEVEL] (FILE:LINE_NUM) <comp_name> <message_content>
 */
#define COMP_SYS_ERROR_MSG(str) SYS_ERROR_MSG("<" << comp_name << "> " << str)
/**
 * Prints a category-specific (Component type) system error log
 * message of level LOG_LEVEL_ERROR. This is used by the components
 * internally and should not be used by the application.
 *
 * Messages are in the following form:
 * [LEVEL] (FILE:LINE_NUM) <cat_name> <message_content>
 */
#define CAT_SYS_ERROR_MSG(str) SYS_ERROR_MSG("<" CAT_NAME "> " << str)

/**
 * Prints a log message of level LOG_LEVEL_WARN.
 */
#define WARN_MSG(str) PRINT_MSG(LOG_LEVEL_WARN, str)
/**
 * Prints a component-specific log message of level LOG_LEVEL_WARN.
 * This is used by the components internally and should not be used by
 * the application.
 *
 * Messages are in the following form:
 * [LEVEL] (FILE:LINE_NUM) <comp_name> <message_content>
 */
#define COMP_WARN_MSG(str) WARN_MSG("<" << comp_name << "> :" << str)
/**
 * Print a category-specific (Component type) log message of level
 * LOG_LEVEL_WARN.
 * This is used by the components internally and should not be used by the
 * application.
 *
 * Messages are in the following form:
 * [LEVEL] (FILE:LINE_NUM) <cat_name> <message_content>
 */
#define CAT_WARN_MSG(str) WARN_MSG("<" CAT_NAME "> " << str)

/**
 * Prints a log message of level LOG_LEVEL_DEBUG.
 */
#define DEBUG_MSG(str) PRINT_MSG(LOG_LEVEL_DEBUG, str)
/**
 * Prints a component-specific log message of level LOG_LEVEL_DEBUG.
 * This is used by the components internally and should not be used by the
 * application.
 *
 * Messages are in the following form:
 * [LEVEL] (FILE:LINE_NUM) <comp_name> <message_content>
 */
#define COMP_DEBUG_MSG(str) DEBUG_MSG("<" << comp_name << "> :" << str)
/**
 * Prints a category-specific (Component type) log message of level
 * LOG_LEVEL_DEBUG. This is used by the components internally and
 * should not be used by the application.
 *
 * Messages are in the following form:
 * [LEVEL] (FILE:LINE_NUM) <cat_name> <message_content>
 */
#define CAT_DEBUG_MSG(str) DEBUG_MSG("<" CAT_NAME "> " << str)

#endif
/** @} */
