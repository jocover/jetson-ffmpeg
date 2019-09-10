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

#include "NvVideoDecoder.h"
#include "NvLogging.h"

#include <cstring>
#include <errno.h>
#include <libv4l2.h>

#define DECODER_DEV "/dev/nvhost-nvdec"
#define CAT_NAME "NVDEC"

#define CHECK_V4L2_RETURN(ret, str)              \
    if (ret < 0) {                               \
        COMP_SYS_ERROR_MSG(str << ": failed");   \
        return -1;                               \
    } else {                                     \
        COMP_DEBUG_MSG(str << ": success");      \
        return 0;                                \
    }

#define RETURN_ERROR_IF_FORMATS_SET() \
    if (output_plane_pixfmt != 0) { \
        COMP_ERROR_MSG("Should be called before setting plane formats") \
        return -1; \
    }

#define RETURN_ERROR_IF_BUFFERS_REQUESTED() \
    if (output_plane.getNumBuffers() != 0 && capture_plane.getNumBuffers() != 0) { \
        COMP_ERROR_MSG("Should be called before requesting buffers on either plane") \
        return -1; \
    }

#define RETURN_ERROR_IF_FORMATS_NOT_SET() \
    if (output_plane_pixfmt == 0) { \
        COMP_ERROR_MSG("Should be called after setting plane formats") \
        return -1; \
    }

using namespace std;

NvVideoDecoder::NvVideoDecoder(const char *name, int flags)
    :NvV4l2Element(name, DECODER_DEV, flags, valid_fields)
{
}

NvVideoDecoder *
NvVideoDecoder::createVideoDecoder(const char *name, int flags)
{
    NvVideoDecoder *dec = new NvVideoDecoder(name, flags);
    if (dec->isInError())
    {
        delete dec;
        return NULL;
    }
    return dec;
}

NvVideoDecoder::~NvVideoDecoder()
{
}

int
NvVideoDecoder::setCapturePlaneFormat(uint32_t pixfmt, uint32_t width,
        uint32_t height)
{
    struct v4l2_format format;
    uint32_t num_bufferplanes;
    NvBuffer::NvBufferPlaneFormat planefmts[MAX_PLANES];

    if (! ((pixfmt == V4L2_PIX_FMT_NV12M) || (pixfmt == V4L2_PIX_FMT_P010M)))
    {
        COMP_ERROR_MSG("Only V4L2_PIX_FMT_NV12M or V4L2_PIX_FMT_P010M is supported");
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

    return capture_plane.setFormat(format);
}

int
NvVideoDecoder::setOutputPlaneFormat(uint32_t pixfmt, uint32_t sizeimage)
{
    struct v4l2_format format;

    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    switch (pixfmt)
    {
        case V4L2_PIX_FMT_H264:
        case V4L2_PIX_FMT_H265:
        case V4L2_PIX_FMT_VP8:
        case V4L2_PIX_FMT_VP9:
        case V4L2_PIX_FMT_MPEG2:
        case V4L2_PIX_FMT_MPEG4:
            output_plane_pixfmt = pixfmt;
            break;
        default:
            COMP_ERROR_MSG("Unsupported pixel format for decoder output plane "
                    << pixfmt);
            return -1;
    }
    format.fmt.pix_mp.pixelformat = pixfmt;
    format.fmt.pix_mp.num_planes = 1;
    format.fmt.pix_mp.plane_fmt[0].sizeimage = sizeimage;

    return output_plane.setFormat(format);
}

int
NvVideoDecoder::disableDPB()
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;

    control.id = V4L2_CID_MPEG_VIDEO_DISABLE_DPB;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Disabling decoder DPB");
}

int
NvVideoDecoder::disableCompleteFrameInputBuffer()
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;

    control.id = V4L2_CID_MPEG_VIDEO_DISABLE_COMPLETE_FRAME_INPUT;
    control.value = 1;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Disabling decoder complete frame input buffer");
}

int
NvVideoDecoder::setFrameInputMode(unsigned int ctrl_value)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;

    control.id = V4L2_CID_MPEG_VIDEO_DISABLE_COMPLETE_FRAME_INPUT;
    control.value = ctrl_value;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting decoder frame input mode to " << ctrl_value);
}


int
NvVideoDecoder::getMinimumCapturePlaneBuffers(int &num)
{
    CHECK_V4L2_RETURN(getControl(V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, num),
            "Getting decoder minimum capture plane buffers (" << num << ")");
}

int
NvVideoDecoder::setSkipFrames(enum v4l2_skip_frames_type skip_frames)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;

    control.id = V4L2_CID_MPEG_VIDEO_SKIP_FRAMES;
    control.value = skip_frames;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting decoder skip frames to " << skip_frames);
}

int
NvVideoDecoder::setMaxPerfMode(int flag)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;

    control.id = V4L2_CID_MPEG_VIDEO_MAX_PERFORMANCE;
    control.value = flag;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Enabling Maximum Performance ");
}

int
NvVideoDecoder::enableMetadataReporting()
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;

    control.id = V4L2_CID_MPEG_VIDEO_ERROR_REPORTING;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Enabling decoder output metadata reporting");
}

int
NvVideoDecoder::getMetadata(uint32_t buffer_index,
        v4l2_ctrl_videodec_outputbuf_metadata &dec_metadata)
{
    v4l2_ctrl_video_metadata metadata;
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    ctrls.count = 1;
    ctrls.controls = &control;

    metadata.buffer_index = buffer_index;
    metadata.VideoDecMetadata = &dec_metadata;

    control.id = V4L2_CID_MPEG_VIDEODEC_METADATA;
    control.string = (char *)&metadata;

    CHECK_V4L2_RETURN(getExtControls(ctrls),
            "Getting decoder output metadata for buffer " << buffer_index);
}

int
NvVideoDecoder::getInputMetadata(uint32_t buffer_index,
        v4l2_ctrl_videodec_inputbuf_metadata &dec_input_metadata)
{
    v4l2_ctrl_video_metadata metadata;
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    ctrls.count = 1;
    ctrls.controls = &control;

    metadata.buffer_index = buffer_index;
    metadata.VideoDecHeaderErrorMetadata = &dec_input_metadata;

    control.id = V4L2_CID_MPEG_VIDEODEC_INPUT_METADATA;
    control.string = (char *)&metadata;

    CHECK_V4L2_RETURN(getExtControls(ctrls),
            "Getting decoder input metadata for buffer " << buffer_index);
}

int
NvVideoDecoder::checkifMasteringDisplayDataPresent(v4l2_ctrl_video_displaydata
        &displaydata)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    ctrls.count = 1;
    ctrls.controls = &control;

    control.id = V4L2_CID_VIDEODEC_DISPLAYDATA_PRESENT;
    control.string = (char *)&displaydata;

    CHECK_V4L2_RETURN(getExtControls(ctrls),
            "Getting decoder output displaydata for buffer ");
}

int
NvVideoDecoder::MasteringDisplayData(v4l2_ctrl_video_hdrmasteringdisplaydata
        *hdrmasteringdisplaydata)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    ctrls.count = 1;
    ctrls.controls = &control;

    control.id = V4L2_CID_VIDEODEC_HDR_MASTERING_DISPLAY_DATA;
    control.string = (char *)hdrmasteringdisplaydata;

    CHECK_V4L2_RETURN(getExtControls(ctrls),
            "Getting decoder output hdrdisplaydata for buffer ");
}

int
NvVideoDecoder::DevicePoll(v4l2_ctrl_video_device_poll *devicepoll)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;

    control.id = V4L2_CID_MPEG_VIDEO_DEVICE_POLL;
    control.string = (char *)devicepoll;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Done calling video device poll ");
}

int
NvVideoDecoder::SetPollInterrupt()
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;

    control.id = V4L2_CID_MPEG_SET_POLL_INTERRUPT;
    control.value = 1;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting decoder poll interrupt to 1 ");
}

int
NvVideoDecoder::ClearPollInterrupt()
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;

    control.id = V4L2_CID_MPEG_SET_POLL_INTERRUPT;
    control.value = 0;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting decoder poll interrupt to 0 ");
}

