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

#include "NvVideoConverter.h"
#include "NvLogging.h"

#include <cstring>
#include <errno.h>

#define CONVERTER_DEV "/dev/nvhost-vic"
#define CAT_NAME "NVVIDCONV"

#define CHECK_V4L2_RETURN(ret, str)              \
    if (ret < 0) {                               \
        COMP_SYS_ERROR_MSG(str << ": failed");   \
        return -1;                               \
    } else {                                     \
        COMP_DEBUG_MSG(str << ": success");      \
        return 0;                                \
    }

#define RETURN_ERROR_IF_FORMATS_SET() \
    if (output_plane_pixfmt != 0 || capture_plane_pixfmt != 0) { \
        COMP_ERROR_MSG("Should be called before setting plane formats") \
        return -1; \
    }

#define RETURN_ERROR_IF_BUFFERS_REQUESTED() \
    if (output_plane.getNumBuffers() != 0 && capture_plane.getNumBuffers() != 0) { \
        COMP_ERROR_MSG("Should be called before requesting buffers on either plane") \
        return -1; \
    }

#define RETURN_ERROR_IF_FORMATS_NOT_SET() \
    if (output_plane_pixfmt == 0 || capture_plane_pixfmt == 0) { \
        COMP_ERROR_MSG("Should be called after setting plane formats") \
        return -1; \
    }

using namespace std;

NvVideoConverter::NvVideoConverter(const char *name, int flags)
    :NvV4l2Element(name, CONVERTER_DEV, flags, valid_fields)
{
}

NvVideoConverter *
NvVideoConverter::createVideoConverter(const char *name, int flags)
{
    NvVideoConverter *conv = new NvVideoConverter(name, flags);
    if (conv->isInError())
    {
        delete conv;
        return NULL;
    }
    return conv;
}

NvVideoConverter::~NvVideoConverter()
{
}

int
NvVideoConverter::setCapturePlaneFormat(uint32_t pixfmt, uint32_t width,
        uint32_t height, enum v4l2_nv_buffer_layout type)
{
    struct v4l2_format format;
    uint32_t num_bufferplanes;
    NvBuffer::NvBufferPlaneFormat planefmts[MAX_PLANES];

    if (setCapturePlaneBufferLayout(type) < 0)
    {
        return -1;
    }

    capture_plane_pixfmt = pixfmt;
    NvBuffer::fill_buffer_plane_format(&num_bufferplanes, planefmts, width,
            height, pixfmt);
    capture_plane.setBufferPlaneFormat(num_bufferplanes, planefmts);

    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = capture_plane.getBufType();
    format.fmt.pix_mp.width = width;
    format.fmt.pix_mp.height = height;
    format.fmt.pix_mp.pixelformat = pixfmt;
    format.fmt.pix_mp.num_planes = num_bufferplanes;

    CHECK_V4L2_RETURN(capture_plane.setFormat(format),
            "Setting capture plane format");
}

int
NvVideoConverter::setOutputPlaneFormat(uint32_t pixfmt, uint32_t width,
        uint32_t height, enum v4l2_nv_buffer_layout type)
{
    struct v4l2_format format;
    uint32_t num_bufferplanes;
    NvBuffer::NvBufferPlaneFormat planefmts[MAX_PLANES];

    if (setOutputPlaneBufferLayout(type) < 0)
    {
        return -1;
    }

    output_plane_pixfmt = pixfmt;
    NvBuffer::fill_buffer_plane_format(&num_bufferplanes, planefmts, width,
            height, pixfmt);
    output_plane.setBufferPlaneFormat(num_bufferplanes, planefmts);

    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = output_plane.getBufType();
    format.fmt.pix_mp.width = width;
    format.fmt.pix_mp.height = height;
    format.fmt.pix_mp.pixelformat = pixfmt;
    format.fmt.pix_mp.num_planes = num_bufferplanes;

    CHECK_V4L2_RETURN(output_plane.setFormat(format),
            "Setting output plane format");
}

int
NvVideoConverter::waitForIdle(uint32_t max_wait_ms)
{
    struct timespec timeToWait;
    struct timeval now;
    int return_val = 0;
    int ret;

    gettimeofday(&now, NULL);

    timeToWait.tv_nsec = (now.tv_usec + (max_wait_ms % 1000) * 1000L) * 1000L;
    timeToWait.tv_sec = now.tv_sec + max_wait_ms / 1000 +
        timeToWait.tv_nsec / 1000000000L;
    timeToWait.tv_nsec = timeToWait.tv_nsec % 1000000000L;

    pthread_mutex_lock(&capture_plane.plane_lock);
    while (output_plane.getTotalQueuedBuffers() >
           capture_plane.getTotalDequeuedBuffers())
    {
        if (!capture_plane.getStreamStatus())
        {
            return_val = -2;
            break;
        }
        ret = pthread_cond_timedwait(&capture_plane.plane_cond,
                &capture_plane.plane_lock, &timeToWait);
        if (ret == ETIMEDOUT)
        {
            return_val = -1;
            break;
        }
    }
    pthread_mutex_unlock(&capture_plane.plane_lock);

    return return_val;
}

int
NvVideoConverter::setOutputPlaneBufferLayout(enum v4l2_nv_buffer_layout type)
{
    struct v4l2_ext_control ctl;
    struct v4l2_ext_controls controls;

    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&ctl, 0, sizeof(ctl));
    memset(&controls, 0, sizeof(controls));

    ctl.id = V4L2_CID_VIDEO_CONVERT_OUTPUT_PLANE_LAYOUT;
    ctl.value = (uint32_t) type;
    controls.controls = &ctl;
    controls.count = 1;

    CHECK_V4L2_RETURN(setExtControls(controls),
            "Setting output plane buffer layout to " << type);
}

int
NvVideoConverter::setCapturePlaneBufferLayout(enum v4l2_nv_buffer_layout type)
{
    struct v4l2_ext_control ctl;
    struct v4l2_ext_controls controls;

    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&ctl, 0, sizeof(ctl));
    memset(&controls, 0, sizeof(controls));

    ctl.id = V4L2_CID_VIDEO_CONVERT_CAPTURE_PLANE_LAYOUT;
    ctl.value = (uint32_t) type;
    controls.controls = &ctl;
    controls.count = 1;

    CHECK_V4L2_RETURN(setExtControls(controls),
            "Setting capture plane buffer layout to " << type);
}

int
NvVideoConverter::setInterpolationMethod(enum v4l2_interpolation_method method)
{
    struct v4l2_ext_control ctl;
    struct v4l2_ext_controls controls;

    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&ctl, 0, sizeof(ctl));
    memset(&controls, 0, sizeof(controls));

    ctl.id = V4L2_CID_VIDEO_CONVERT_INTERPOLATION_METHOD;
    ctl.value = (uint32_t) method;
    controls.controls = &ctl;
    controls.count = 1;

    CHECK_V4L2_RETURN(setExtControls(controls),
            "Setting interpolation method to " << method);
}

int
NvVideoConverter::setFlipMethod(enum v4l2_flip_method method)
{
    struct v4l2_ext_control ctl;
    struct v4l2_ext_controls controls;

    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&ctl, 0, sizeof(ctl));
    memset(&controls, 0, sizeof(controls));

    ctl.id = V4L2_CID_VIDEO_CONVERT_FLIP_METHOD;
    ctl.value = (uint32_t) method;
    controls.controls = &ctl;
    controls.count = 1;

    CHECK_V4L2_RETURN(setExtControls(controls),
            "Setting flip method to " << method);
}

int
NvVideoConverter::setTnrAlgorithm(enum v4l2_tnr_algorithm algorithm)
{
    struct v4l2_ext_control ctl;
    struct v4l2_ext_controls controls;

    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&ctl, 0, sizeof(ctl));
    memset(&controls, 0, sizeof(controls));

    ctl.id = V4L2_CID_VIDEO_CONVERT_TNR_ALGORITHM;
    ctl.value = (uint32_t) algorithm;
    controls.controls = &ctl;
    controls.count = 1;

    CHECK_V4L2_RETURN(setExtControls(controls),
            "Setting TNR algorithm to " << algorithm);
}

int
NvVideoConverter::setYUVRescale(enum v4l2_yuv_rescale_method method)
{
    struct v4l2_ext_control ctl;
    struct v4l2_ext_controls controls;

    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&ctl, 0, sizeof(ctl));
    memset(&controls, 0, sizeof(controls));

    ctl.id = V4L2_CID_VIDEO_CONVERT_YUV_RESCALE_METHOD;
    ctl.value = (uint32_t) method;
    controls.controls = &ctl;
    controls.count = 1;

    CHECK_V4L2_RETURN(setExtControls(controls),
            "Setting YUV Rescale mothod to " << method);
}

int
NvVideoConverter::setCropRect(uint32_t left, uint32_t top, uint32_t width,
            uint32_t height)
{
    struct v4l2_rect rect;

    rect.left = left;
    rect.top = top;
    rect.width = width;
    rect.height = height;

    CHECK_V4L2_RETURN(capture_plane.setSelection(V4L2_SEL_TGT_CROP, 0, rect),
            "Setting crop rectangle to left=" << left << ", top=" << top <<
            ", width=" << width << ", height=" << height);
}

int
NvVideoConverter::setDestRect(uint32_t left, uint32_t top, uint32_t width,
            uint32_t height)
{
    struct v4l2_rect rect;

    rect.left = left;
    rect.top = top;
    rect.width = width;
    rect.height = height;

    CHECK_V4L2_RETURN(output_plane.setSelection(V4L2_SEL_TGT_COMPOSE, 0, rect),
            "Setting compose rectangle to left=" << left << ", top=" << top <<
            ", width=" << width << ", height=" << height);
}
