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

#include <iostream>
#include <string.h>
#include <map>
#include <stdint.h>
#include "NvElementProfiler.h"

#define LOCK() pthread_mutex_lock(&profiler_lock)
#define UNLOCK() pthread_mutex_unlock(&profiler_lock)

#define RETURN_IF_DISABLED() \
    if (!enabled) { \
        UNLOCK(); \
        return; \
    }

#define GET_TIME(timeval) gettimeofday(timeval, NULL);

#define TIMESPEC_DIFF_USEC(timespec1, timespec2) \
    (((timespec1)->tv_sec - (timespec2)->tv_sec) * 1000000L + \
        (timespec1)->tv_usec - (timespec2)->tv_usec)

using namespace std;

NvElementProfiler::NvElementProfiler(ProfilerField fields)
    :valid_fields(fields)
{
    enabled = false;
    unit_id_counter = 0;

    reset();

    pthread_mutex_init(&profiler_lock, NULL);
}

NvElementProfiler::~NvElementProfiler()
{
    LOCK();
    reset();
    UNLOCK();
    pthread_mutex_destroy(&profiler_lock);
}
void
NvElementProfiler::enableProfiling(bool reset_data)
{
    LOCK();
    if (enabled)
    {
        UNLOCK();
        return;
    }

    if(reset_data)
    {
        reset();
    }

    enabled = true;
    UNLOCK();
}

void
NvElementProfiler::disableProfiling()
{
    LOCK();
    RETURN_IF_DISABLED();

    data_int.accumulated_time.tv_sec +=
        (data_int.stop_time.tv_sec - data_int.start_time.tv_sec);
    data_int.accumulated_time.tv_usec +=
        (data_int.stop_time.tv_usec - data_int.start_time.tv_usec);
    data_int.start_time.tv_sec = 0;
    data_int.start_time.tv_usec = 0;
    data_int.stop_time.tv_sec = 0;
    data_int.stop_time.tv_usec = 0;
    enabled = false;
    UNLOCK();
}

void NvElementProfiler::getProfilerData(NvElementProfiler::NvElementProfilerData &data)
{
    uint64_t total_time;

    LOCK();

    total_time = data_int.accumulated_time.tv_sec * 1000000L +
        data_int.accumulated_time.tv_usec +
        TIMESPEC_DIFF_USEC(&data_int.stop_time, &data_int.start_time);

    if (data_int.total_processed_units == 0 || total_time == 0)
    {
        data.average_fps = 0;
    }
    else
    {
        data.average_fps = ((float) (data_int.total_processed_units - 1)) *
            1000000 / total_time;
    }

    if (data_int.total_processed_units == 0)
    {
        data.max_latency_usec = 0;
        data.min_latency_usec = 0;
        data.average_latency_usec = 0;
    }
    else
    {
        data.max_latency_usec = data_int.max_latency_usec;
        data.min_latency_usec = data_int.min_latency_usec;
        data.average_latency_usec =
            data_int.total_latency / data_int.total_processed_units;
    }

    data.profiling_time.tv_sec =
        data_int.accumulated_time.tv_sec + data_int.stop_time.tv_sec -
                data_int.start_time.tv_sec;

    data.profiling_time.tv_usec =
        data_int.accumulated_time.tv_usec + data_int.stop_time.tv_usec -
                data_int.start_time.tv_usec;

    if (data.profiling_time.tv_usec < 0)
    {
        data.profiling_time.tv_usec += 1000000;
        data.profiling_time.tv_sec--;
    }

    if (data.profiling_time.tv_usec > 1000000)
    {
        data.profiling_time.tv_usec -= 1000000;
        data.profiling_time.tv_sec++;
    }

    data.total_processed_units = data_int.total_processed_units;
    data.num_late_units = data_int.num_late_units;
    data.valid_fields = valid_fields;
    UNLOCK();
}

void NvElementProfiler::printProfilerData(ostream &out_stream)
{
    NvElementProfilerData data;
    getProfilerData(data);

    if (data.valid_fields & PROFILER_FIELD_FPS)
    {
        out_stream << "Total Profiling time = " <<
            (data.profiling_time.tv_sec +
                (data.profiling_time.tv_usec / 1000000.0)) << endl;
        out_stream << "Average FPS = " << data.average_fps << endl;
    }
    if (data.valid_fields & PROFILER_FIELD_TOTAL_UNITS)
    {
        out_stream << "Total units processed = " <<
            data.total_processed_units << endl;
    }
    if (data.valid_fields & PROFILER_FIELD_LATE_UNITS)
    {
        out_stream << "Num. of late units = " <<
            data.num_late_units << endl;
    }
    if (data.valid_fields & PROFILER_FIELD_LATENCIES)
    {
        out_stream << "Average latency(usec) = " <<
            data.average_latency_usec << endl;
        out_stream << "Minimum latency(usec) = " <<
            data.min_latency_usec << endl;
        out_stream << "Maximum latency(usec) = " <<
            data.max_latency_usec << endl;
    }
}

void
NvElementProfiler::reset()
{
    memset(&data_int, 0, sizeof(data_int));
    data_int.min_latency_usec = (uint64_t) -1;

    unit_start_time_queue.clear();
}

uint64_t
NvElementProfiler::startProcessing()
{
    struct timeval time;
    uint64_t ret = 0;
    LOCK();
    if (enabled)
    {
        std::map<uint64_t,struct timeval>::iterator it =
            unit_start_time_queue.end();

        unit_id_counter++;
        GET_TIME(&time);

        unit_start_time_queue.insert(it,
                std::pair<uint64_t,struct timeval>(unit_id_counter, time));

        ret = unit_id_counter;
    }
    UNLOCK();
    return ret;
}

void
NvElementProfiler::finishProcessing(uint64_t id, bool is_late)
{
    struct timeval unit_start_time;
    struct timeval stop_time;
    uint64_t latency;

    LOCK();
    RETURN_IF_DISABLED();

    if ((valid_fields & PROFILER_FIELD_LATENCIES) &&
            unit_start_time_queue.empty())
    {
        UNLOCK();
        return;
    }

    GET_TIME(&stop_time);

    if (valid_fields & PROFILER_FIELD_LATENCIES)
    {
        std::map<uint64_t, struct timeval>::iterator it;
        if (id)
        {
            it = unit_start_time_queue.find(id);
        }
        else
        {
            it = unit_start_time_queue.begin();
        }

        if (it == unit_start_time_queue.end())
        {
            UNLOCK();
            return;
        }
        unit_start_time = it->second;
        unit_start_time_queue.erase(it);

        latency = TIMESPEC_DIFF_USEC(&stop_time, &unit_start_time);
        data_int.total_latency += latency;

        if (latency < data_int.min_latency_usec)
        {
            data_int.min_latency_usec = latency;
        }
        if(latency > data_int.max_latency_usec)
        {
            data_int.max_latency_usec = latency;
        }
    }

    data_int.stop_time = stop_time;

    if (!data_int.start_time.tv_sec && !data_int.start_time.tv_usec)
    {
        data_int.start_time = data_int.stop_time;
    }

    if (is_late)
    {
        data_int.num_late_units++;
    }
    data_int.total_processed_units++;

    UNLOCK();
}
