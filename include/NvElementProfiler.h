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

#ifndef __NV_ELEMENT_PROFILER_H__
#define __NV_ELEMENT_PROFILER_H__

#include <iostream>
#include <pthread.h>
#include <map>
#include <stdint.h>
#include <sys/time.h>

/**
 * @file
 * <b>NVIDIA Multimedia API: Element Profiler API</b>
 *
 * @b Description: This file profiles the performance of individual elements.
 */


/**
 *
 * Helper class for profiling the performance of individual elements.
 *
 * NvElementProfiler currently measures processing latencies, average processing rate, and
 * the number of units that arrived late at the element. Components should use this
 * information internally.
 *
 * If you require latency measurements,
 * you must call startProcessing() to indicate that a unit has been submitted
 * for processing and finishProcessing() to indicate that a unit has finished processing.
 * If you require only averaging processing rate or the number of units that
 * arrived late need not call startProcessing().
 *
 * You can get data from NvElementProfiler using getProfilerData(). This function
 * fills the [NvElementProfilerData](@ref NvElementProfiler::NvElementProfilerData)
 * structure. Components that do not support all
 * the fields available in the structure must use the variable
 * [valid_fields](@ref NvElementProfiler::NvElementProfilerData::valid_fields) of
 * type [ProfilerField](@ref NvElementProfiler::ProfilerField), which is also
 * included in the structure.
 *
 * @defgroup l4t_mm_nvelementprofiler_group  Element Profiler API
 * @ingroup aa_framework_api_group
 * @{
 */
class NvElementProfiler {
public:
    /**
     * @defgroup Defines @c valid_field values for the #NvElementProfilerData structure.
     * @ingroup l4t_mm_nvelementprofiler_group
     * @{
     */
    typedef int ProfilerField;
    static const ProfilerField PROFILER_FIELD_NONE = 0;
    static const ProfilerField PROFILER_FIELD_TOTAL_UNITS = 1;
    static const ProfilerField PROFILER_FIELD_LATE_UNITS = 2;
    static const ProfilerField PROFILER_FIELD_LATENCIES = 4;
    static const ProfilerField PROFILER_FIELD_FPS = 8;
    static const ProfilerField PROFILER_FIELD_ALL = (PROFILER_FIELD_FPS << 1) - 1;
    /** @} */

    /**
     * Holds profiling data for the element.
     *
     * Some elements may not support all the fields in the structure. User must check
     * the @a valid_fields flag to determine which fields are valid.
     */
    typedef struct {
        /** Valid Fields which are supported by the element. */
        ProfilerField valid_fields;

        /** Average latency of all processed units, in microseconds. */
        uint64_t average_latency_usec;
        /** Minimum of latencies for each processed units, in microseconds. */
        uint64_t min_latency_usec;
        /** Maximum of latencies for each processed units, in microseconds. */
        uint64_t max_latency_usec;

        /** Total units processed. */
        uint64_t total_processed_units;
        /** Number of units which arrived late at the element. */
        uint64_t num_late_units;

        /** Average rate at which the units were processed. */
        float average_fps;

        /** Total profiling time. */
        struct timeval profiling_time;
    } NvElementProfilerData;

    /**
     * Gets the profiling data for the element.
     *
     * @param[out] data Reference to the NvElementProfilerData structure which should be filled.
     */
    void getProfilerData(NvElementProfilerData &data);

    /**
     * Prints the element's profiling data to an output stream.
     *
     * @param[in] out_stream Reference to a std::ostream.
     */
    void printProfilerData(std::ostream &out_stream = std::cout);

    /**
     * Informs the profiler that processing has started.
     *
     * Has no effect if profiler is disabled.
     *
     * @return ID of the unit, to be supplied with finishProcessing();.
     */
    uint64_t startProcessing();

    /**
     * Informs the profiler that processing has finished.
     *
     * Has no effect if profiler is disabled.
     *
     * @param[in] id ID of the unit whose processing is finished,
     *          0 if the first unit in the profiler's queue should be picked.
     * @param[in] is_late Should be true if the frame arrived late at the element.
     */
    void finishProcessing(uint64_t id, bool is_late);

    /**
     * Enables the profiler.
     *
     * startProcessing() and finishProcessing() are ineffective until the profiler is enabled.
     *
     * @param[in] reset_data Reset the profiled data.
     */
    void enableProfiling(bool reset_data);

    /**
     * Disables the profiler.
     */
    void disableProfiling();
private:
    /**
     * Resets the profiler data.
     */
    void reset();

    pthread_mutex_t profiler_lock; /**< Mutex to synchronize multithreaded access to profiler data. */

    bool enabled; /**< Flag indicating if profiler is enabled. */

    const ProfilerField valid_fields; /**< Valid fields for the element. */

    struct NvElementProfilerDataInternal : NvElementProfilerData {
        /** Wall-clock time at which the first unit was processed. */
        struct timeval start_time;

        /** Wall-clock time at which the latest unit was processed. */
        struct timeval stop_time;

        /** Total accumulated time.
         *  When performance measurement is restarted @a start_time and @a stop_time
         *  are reset. This field is used to accumulate time before
         *  resetting. */
        struct timeval accumulated_time;

        /** Total accumulated latency for all units, in microseconds. */
        uint64_t total_latency;
    } data_int;

    /** Queue used to maintain the timestamps of when the unit
     *  processing started. Required to calculate latency. */
    std::map<uint64_t, struct timeval> unit_start_time_queue;

    uint64_t unit_id_counter; /**< Unique ID of the last unit. */

    /**
     * Constructor for NvElementProfiler.
     *
     * Initializes internal data structures. The profiler is disabled by default.
     * @param fields
     */
    NvElementProfiler(ProfilerField fields);

    /**
     * Disallow copy constructor.
     */
    NvElementProfiler(const NvElementProfiler& that);
    /**
     * Disallow assignment.
     */
    void operator=(NvElementProfiler const&);

    ~NvElementProfiler();

    friend class NvElement;
};

/** @} */

#endif
