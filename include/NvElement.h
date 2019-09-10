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
 * <b>NVIDIA Multimedia API: %NvElement Base Class</b>
 *
 * @b This file declares the NvElement base class.
 */

#ifndef __NV_ELEMENT_H__
#define __NV_ELEMENT_H__

#include <iostream>
#include <sys/time.h>
#include <stdint.h>
#include <string.h>

#include "NvElementProfiler.h"

/**
 *
 * @defgroup l4t_mm_nvelement_base_group NvElement Base Class
 * @ingroup l4t_mm_nvelement_group
 *
 * This class is the class from which both V4L2 and non-V4L2
 * components are derived.
 *
 * @{
 */
/**
 * Every element has a unique name that can be used for identifying
 * the element in debug logs.
 *
 * @c %NvElement also provides other common functionality, such as keeping
 * track of errors.
 */
class NvElement
{
public:
    /**
     * Indicates whether the element encountered an error during its operation.
     *
     * @return 0 if no error was encountered, a non-zero value if an
     *            error was encountered.
     */
    virtual int isInError()
    {
        return is_in_error;
    }
    virtual ~NvElement()
    {
    }

    /**
     * Gets profiling data for the element.
     *
     * @return A constant reference to the element's profiling data.
     */
    void getProfilingData(NvElementProfiler::NvElementProfilerData &data);

    /**
     * Prints profiling data for the element to an output stream.
     *
     * @param[in] out_stream Output stream of type std::ostream to print the
     *              data to. It takes the default value std::cout if not specified.
     */
    void printProfilingStats(std::ostream &out_stream = std::cout);

    /**
     * Enables profiling for the element.
     */
    virtual void enableProfiling();

    /**
     * Checks whether profiling is enabled for the element.
     *
     * @return Boolean value indicating if profiling is enabled.
     */
    bool isProfilingEnabled();

protected:

    /**
     * Creates a new NvElement object with name @a name.
     *
     * If the @a name parameter is NULL, this method sets the internal
     * error variable.
     *
     * @param[in] name If non-NULL, a pointer to the name of the
     *                 element.
     */
    NvElement(const char *name, NvElementProfiler::ProfilerField = NvElementProfiler::PROFILER_FIELD_NONE);

    int is_in_error;        /**< Indicates if an error was encountered during
                               the operation of the element. */
    const char *comp_name;  /**< Specifies the name of the component,
                               for debugging. */
    NvElementProfiler profiler; /**< Profiler for the element. */

    /**
     * Disallows copy constructor.
     */
    NvElement(const NvElement& that);
    /**
     * Disallows assignment.
     */
    void operator=(NvElement const&);

};
/** @} */
#endif
