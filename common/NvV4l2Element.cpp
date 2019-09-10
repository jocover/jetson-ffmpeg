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

#include "NvV4l2Element.h"
#include "NvLogging.h"

#include <fcntl.h>
#include <cstring>
#include <errno.h>
#include <libv4l2.h>

#define CAT_NAME "V4l2Element"

using namespace std;

/*initialization of mutex for NvV4l2Element*/
pthread_mutex_t initializer_mutex = PTHREAD_MUTEX_INITIALIZER;

NvV4l2Element::NvV4l2Element(const char *comp_name, const char *dev_node, int flags, NvElementProfiler::ProfilerField fields)
    :NvElement(comp_name, fields),
      output_plane(V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, comp_name,
                  fd, !(flags & O_NONBLOCK), profiler),
      capture_plane(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, comp_name,
                  fd, !(flags & O_NONBLOCK), profiler)
{
    struct v4l2_capability caps;
    int ret;

    app_data = NULL;
    output_plane_pixfmt = 0;
    capture_plane_pixfmt = 0;

    /*Synchronization issue of libv4l2 open source library fixing here,adding lock for that*/
    pthread_mutex_lock(&initializer_mutex);
    fd = v4l2_open(dev_node, flags | O_RDWR);
    if (fd == -1)
    {
        COMP_SYS_ERROR_MSG("Could not open device '" << dev_node << "'");
        is_in_error = 1;
        pthread_mutex_unlock(&initializer_mutex);
        return;
    }
    pthread_mutex_unlock(&initializer_mutex);

    COMP_DEBUG_MSG("Opened, fd = " << fd);

    ret = v4l2_ioctl(fd, VIDIOC_QUERYCAP, &caps);
    if (ret != 0)
    {
        COMP_SYS_ERROR_MSG("Error in VIDIOC_QUERYCAP");
        is_in_error = 1;
        return;
    }
    if (!(caps.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE))
    {
        COMP_ERROR_MSG("Device does not support V4L2_CAP_VIDEO_M2M_MPLANE");
        is_in_error = 1;
        return;
    }
}

NvV4l2Element::~NvV4l2Element()
{
    output_plane.deinitPlane();
    capture_plane.deinitPlane();

    if (fd != -1)
    {
        v4l2_close(fd);
        CAT_DEBUG_MSG("Device closed, fd = " << fd);
    }
}

int
NvV4l2Element::dqEvent(struct v4l2_event &ev, uint32_t max_wait_ms)
{
    int ret;

    do
    {
        ret = v4l2_ioctl(fd, VIDIOC_DQEVENT, &ev);

        if (ret == 0)
        {
            COMP_DEBUG_MSG("DQed event " << hex << ev.type << dec);
        }
        else if (errno != EAGAIN)
        {
            COMP_SYS_ERROR_MSG("Error while DQing event");
            break;
        }
        else if (max_wait_ms-- == 0)
        {
            COMP_WARN_MSG("Error while DQing event: Resource temporarily unavailable");
            break;
        }
        else
        {
            usleep(1000);
        }
    }
    while (ret && (output_plane.getStreamStatus() || capture_plane.getStreamStatus()));

    return ret;
}

int
NvV4l2Element::setControl(uint32_t id, int32_t value)
{
    struct v4l2_control ctl;
    int ret;

    ctl.id = id;
    ctl.value = value;

    ret = v4l2_ioctl(fd, VIDIOC_S_CTRL, &ctl);

    if (ret < 0)
    {
        COMP_SYS_ERROR_MSG("Error setting value " << value << " on control " <<
                id);
    }
    else
    {
        COMP_DEBUG_MSG("Set value " << value << " on control " << id);
    }
    return ret;
}

int
NvV4l2Element::getControl(uint32_t id, int32_t &value)
{
    struct v4l2_control ctl;
    int ret;

    ctl.id = id;

    ret = v4l2_ioctl(fd, VIDIOC_G_CTRL, &ctl);

    if (ret < 0)
    {
        COMP_SYS_ERROR_MSG("Error getting value of control " << id);
    }
    else
    {
        COMP_DEBUG_MSG("Got value " << ctl.value << " for control " << id);
        value = ctl.value;
    }
    return ret;
}

int
NvV4l2Element::setExtControls(v4l2_ext_controls &ctl)
{
    int ret;

    ret = v4l2_ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctl);

    if (ret < 0)
    {
        COMP_SYS_ERROR_MSG("Error setting controls");
    }
    else
    {
        COMP_DEBUG_MSG("Set controls");
    }
    return ret;
}

int
NvV4l2Element::getExtControls(v4l2_ext_controls &ctl)
{
    int ret;

    ret = v4l2_ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctl);

    if (ret < 0)
    {
        COMP_SYS_ERROR_MSG("Error getting value of controls");
    }
    else
    {
        COMP_DEBUG_MSG("Got controls");
    }
    return ret;
}

int
NvV4l2Element::subscribeEvent(uint32_t type, uint32_t id, uint32_t flags)
{
    struct v4l2_event_subscription sub;
    int ret;

    memset(&sub, 0, sizeof(struct v4l2_event_subscription));

    sub.type = type;
    sub.id = id;
    sub.flags = flags;

    ret = v4l2_ioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    if (ret == 0)
    {
        COMP_DEBUG_MSG("Successfully subscribed to event " << type);
    }
    else
    {
        COMP_SYS_ERROR_MSG
            ("Error while subscribing to event " << type);
    }

    return ret;
}

int
NvV4l2Element::abort()
{
    int ret = 0;

    ret |= output_plane.setStreamStatus(false);
    ret |= capture_plane.setStreamStatus(false);

    return ret;
}

int
NvV4l2Element::waitForIdle(uint32_t max_wait_ms)
{
    COMP_ERROR_MSG("wait_for_idle not implemented");
    return 0;
}

int
NvV4l2Element::isInError()
{
    int error = is_in_error;
    error |= capture_plane.isInError();
    error |= output_plane.isInError();
    return error;
}

void
NvV4l2Element::enableProfiling()
{
    if (output_plane_pixfmt || capture_plane_pixfmt)
    {
        COMP_ERROR_MSG("Profiling must be enabled before setting formats on either plane");
        return;
    }
    profiler.enableProfiling(true);
}
