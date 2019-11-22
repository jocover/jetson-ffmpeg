/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION. All rights reserved.
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

#include "NvVideoEncoder.h"
#include "NvLogging.h"

#include <cstring>
#include <errno.h>
#include <libv4l2.h>

#define ENCODER_DEV "/dev/nvhost-msenc"
#define ENCODER_COMP_NAME "NVENC"

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

#define RETURN_ERROR_IF_BUFFERS_NOT_REQUESTED() \
    if (output_plane.getNumBuffers() == 0 || capture_plane.getNumBuffers() == 0) { \
        COMP_ERROR_MSG("Should be called before requesting buffers on either plane") \
        return -1; \
    }

#define RETURN_ERROR_IF_FORMATS_NOT_SET() \
    if (output_plane_pixfmt == 0 || capture_plane_pixfmt == 0) { \
        COMP_ERROR_MSG("Should be called after setting plane formats") \
        return -1; \
    }

using namespace std;

NvVideoEncoder::NvVideoEncoder(const char *name, int flags)
    :NvV4l2Element(name, ENCODER_DEV, flags, valid_fields)
{
}

NvVideoEncoder *
NvVideoEncoder::createVideoEncoder(const char *name, int flags)
{
    NvVideoEncoder *enc = new NvVideoEncoder(name, flags);
    if (enc->isInError())
    {
        delete enc;
        return NULL;
    }
    return enc;
}

NvVideoEncoder::~NvVideoEncoder()
{
}

int
NvVideoEncoder::setOutputPlaneFormat(uint32_t pixfmt, uint32_t width,
        uint32_t height)
{
    struct v4l2_format format;
    uint32_t num_bufferplanes;
    NvBuffer::NvBufferPlaneFormat planefmts[MAX_PLANES];

    if (pixfmt != V4L2_PIX_FMT_YUV420M && pixfmt != V4L2_PIX_FMT_YUV444M &&
        pixfmt != V4L2_PIX_FMT_P010M)
    {
        COMP_ERROR_MSG("Only YUV420M, YUV444M and P010M are supported");
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

    return output_plane.setFormat(format);
}

int
NvVideoEncoder::setCapturePlaneFormat(uint32_t pixfmt, uint32_t width,
        uint32_t height, uint32_t sizeimage)
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
            capture_plane_pixfmt = pixfmt;
            break;
        default:
            ERROR_MSG("Unknown supported pixel format for encoder " << pixfmt);
            return -1;
    }

    format.fmt.pix_mp.pixelformat = pixfmt;
    format.fmt.pix_mp.width = width;
    format.fmt.pix_mp.height = height;
    format.fmt.pix_mp.num_planes = 1;
    format.fmt.pix_mp.plane_fmt[0].sizeimage = sizeimage;

    return capture_plane.setFormat(format);
}

int
NvVideoEncoder::setFrameRate(uint32_t framerate_num, uint32_t framerate_den)
{
    struct v4l2_streamparm parms;

    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&parms, 0, sizeof(parms));
    parms.parm.output.timeperframe.numerator = framerate_den;
    parms.parm.output.timeperframe.denominator = framerate_num;

    CHECK_V4L2_RETURN(output_plane.setStreamParms(parms),
            "Setting framerate to " << framerate_num << "/" << framerate_den);
}

int
NvVideoEncoder::setBitrate(uint32_t bitrate)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    cout << capture_plane_pixfmt <<endl;
    cout << output_plane_pixfmt <<endl;
    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEO_BITRATE;
    control.value = bitrate;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder bitrate to " << bitrate);
}

int
NvVideoEncoder::setEncoderCommand(int cmd, int flags)
{
   int ret=0;
   struct v4l2_encoder_cmd v4l2_enc_cmd;
   v4l2_enc_cmd.cmd = cmd;
   v4l2_enc_cmd.flags = flags;

   ret = v4l2_ioctl(fd, VIDIOC_ENCODER_CMD, v4l2_enc_cmd);
   if (ret < 0)
     printf(" Error in encoder command \n");

  return ret;
}

int
NvVideoEncoder::setPeakBitrate(uint32_t peak_bitrate)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    cout << capture_plane_pixfmt <<endl;
    cout << output_plane_pixfmt <<endl;
    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEO_BITRATE_PEAK;
    control.value = peak_bitrate;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder peak bitrate to " << peak_bitrate);
}

int
NvVideoEncoder::setProfile(uint32_t profile)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    switch (capture_plane_pixfmt)
    {
        case V4L2_PIX_FMT_H264:
            control.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
            break;
        case V4L2_PIX_FMT_H265:
            control.id = V4L2_CID_MPEG_VIDEO_H265_PROFILE;
            break;
        default:
            COMP_ERROR_MSG("Unsupported encoder type");
            return -1;
    }
    control.value = profile;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder profile to " << profile);
}

int
NvVideoEncoder::setLevel(enum v4l2_mpeg_video_h264_level level)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    if (capture_plane_pixfmt != V4L2_PIX_FMT_H264)
    {
        COMP_WARN_MSG("Currently only supported for H.264");
        return 0;
    }

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL;
    control.value = level;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder level to " << level);
}

int
NvVideoEncoder::setConstantQp(int qp_value)
{
    struct v4l2_ext_control control[3];
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 3;
    ctrls.controls = &control[0];
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control[0].id = V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE;
    control[0].value = 0; // disable rate control

    control[1].id = V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP;
    control[1].value = qp_value;

    control[2].id = V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP;
    control[2].value = qp_value;
    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder constant qp to " << qp_value);
}

int
NvVideoEncoder::setRateControlMode(enum v4l2_mpeg_video_bitrate_mode mode)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEO_BITRATE_MODE;
    control.value = mode;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder rate control mode to " << mode);
}

int
NvVideoEncoder::setMaxPerfMode(int flag)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEO_MAX_PERFORMANCE;
    control.value = flag;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Enabling Maximum Performance ");
}

int
NvVideoEncoder::setIFrameInterval(uint32_t interval)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEO_GOP_SIZE;
    control.value = interval;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder I-frame interval to " << interval);
}

int
NvVideoEncoder::setIDRInterval(uint32_t interval)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEO_IDR_INTERVAL;
    control.value = interval;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder IDR interval to " << interval);
}

int
NvVideoEncoder::forceIDR()
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Forcing IDR");
}

int
NvVideoEncoder::setTemporalTradeoff(v4l2_enc_temporal_tradeoff_level_type level)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_TEMPORAL_TRADEOFF_LEVEL;
    control.value = level;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder temporal tradeoff level to " << level);
}

int
NvVideoEncoder::setSliceLength(v4l2_enc_slice_length_type type, uint32_t length)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;
    v4l2_enc_slice_length_param param = {type, length};

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_SLICE_LENGTH_PARAM;
    control.string = (char *) &param;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder packet type to " << type << ", length to " <<
            length);
}

int
NvVideoEncoder::setHWPresetType(v4l2_enc_hw_preset_type type)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_HW_PRESET_TYPE_PARAM;
    control.value = type;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder HW Preset type to " << type);
}

int
NvVideoEncoder::setROIParams(uint32_t buffer_index,
        v4l2_enc_frame_ROI_params &params)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_NOT_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_ROI_PARAMS;
    control.string = (char *) &params;
    params.config_store = buffer_index;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder ROI params for buffer " << buffer_index);
}

int
NvVideoEncoder::SetInputMetaParams(uint32_t buffer_index,
        v4l2_ctrl_videoenc_input_metadata &params)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_NOT_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_INPUT_METADATA;
    control.string = (char *) &params;
    params.config_store = buffer_index;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder input metadata Params");
}

int
NvVideoEncoder::enableROI(v4l2_enc_enable_roi_param &params)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_ENABLE_ROI_PARAM;
    control.string = (char *) &params;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Enabling encoder ROI");
}

int
NvVideoEncoder::enableReconCRC(v4l2_enc_enable_reconcrc_param &params)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_ENABLE_RECONCRC_PARAM;
    control.string = (char *) &params;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Enabling encoder ReconCRC");
}

int
NvVideoEncoder::enableExternalRPS(v4l2_enc_enable_ext_rps_ctr &params)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_ENABLE_EXTERNAL_RPS_CONTROL;
    control.string = (char *) &params;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Enabling encoder External RPS");
}

int
NvVideoEncoder::enableExternalRC(v4l2_enc_enable_ext_rate_ctr &params)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_ENABLE_EXTERNAL_RATE_CONTROL;
    control.string = (char *) &params;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Enabling encoder External RC");
}

int
NvVideoEncoder::setVirtualBufferSize(uint32_t size)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;
    v4l2_enc_virtual_buffer_size buffer_size = {size};

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_VIRTUALBUFFER_SIZE;
    control.string = (char *) &buffer_size;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder virtual buffer size to " << size);
}

int
NvVideoEncoder::setNumReferenceFrames(uint32_t num_frames)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;
    v4l2_enc_num_ref_frames frames = { num_frames };

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_NUM_REFERENCE_FRAMES;
    control.string = (char *) &frames;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder number of reference frames to " << num_frames);
}

int
NvVideoEncoder::setSliceIntrarefresh(uint32_t interval)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;
    v4l2_enc_slice_intrarefresh_param param = {interval};

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_SLICE_INTRAREFRESH_PARAM;
    control.string = (char *) &param;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder slice intrarefresh interval to " << interval);
}

int
NvVideoEncoder::setNumBFrames(uint32_t num)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_NUM_BFRAMES;
    control.value = num;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder number of B frames to " << num);
}

int
NvVideoEncoder::setInsertSpsPpsAtIdrEnabled(bool enabled)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_INSERT_SPS_PPS_AT_IDR;
    control.value = enabled;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder SPSPPSatIDR to " << enabled);
}

int
NvVideoEncoder::enableMotionVectorReporting()
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_ENABLE_METADATA_MV;
    control.value = 1;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Enabling encoder motion vector reporting");
}

int
NvVideoEncoder::getMetadata(uint32_t buffer_index,
        v4l2_ctrl_videoenc_outputbuf_metadata &enc_metadata)
{
    v4l2_ctrl_video_metadata metadata;
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_BUFFERS_NOT_REQUESTED();

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    metadata.buffer_index = buffer_index;
    metadata.VideoEncMetadata = &enc_metadata;

    control.id = V4L2_CID_MPEG_VIDEOENC_METADATA;
    control.string = (char *)&metadata;

    CHECK_V4L2_RETURN(getExtControls(ctrls),
            "Getting encoder output metadata for buffer " << buffer_index);
}

int
NvVideoEncoder::getMotionVectors(uint32_t buffer_index,
        v4l2_ctrl_videoenc_outputbuf_metadata_MV &enc_mv_metadata)
{
    v4l2_ctrl_video_metadata metadata;
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_BUFFERS_NOT_REQUESTED();

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    metadata.buffer_index = buffer_index;
    metadata.VideoEncMetadataMV = &enc_mv_metadata;

    control.id = V4L2_CID_MPEG_VIDEOENC_METADATA_MV;
    control.string = (char *)&metadata;

    CHECK_V4L2_RETURN(getExtControls(ctrls),
            "Getting encoder output MV metadata for buffer " << buffer_index);
}

int
NvVideoEncoder::setQpRange(uint32_t MinQpI, uint32_t MaxQpI, uint32_t MinQpP,
            uint32_t MaxQpP, uint32_t MinQpB, uint32_t MaxQpB)
{
    v4l2_ctrl_video_qp_range qprange;
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    qprange.MinQpI = MinQpI;
    qprange.MaxQpI = MaxQpI;
    qprange.MinQpP = MinQpP;
    qprange.MaxQpP = MaxQpP;
    qprange.MinQpB = MinQpB;
    qprange.MaxQpB = MaxQpB;

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_QP_RANGE;
    control.string = (char *)&qprange;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder Qp range " << ctrls.count);
}

int
NvVideoEncoder::setInsertVuiEnabled(bool enabled)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_INSERT_VUI;
    control.value = enabled;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder InsertVUI to " << enabled);
}

int
NvVideoEncoder::setExtendedColorFormat(bool enabled)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_EXTEDED_COLORFORMAT;
    control.value = enabled;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder extended colorformat to " << enabled);
}

int
NvVideoEncoder::setInsertAudEnabled(bool enabled)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_INSERT_AUD;
    control.value = enabled;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder InsertAUD to " << enabled);
}

int
NvVideoEncoder::setAlliFramesEncode(bool enabled)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();
    RETURN_ERROR_IF_BUFFERS_REQUESTED();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEOENC_ENABLE_ALLIFRAME_ENCODE;
    control.value = enabled;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder InsertAUD to " << enabled);
}

int
NvVideoEncoder::DevicePoll(v4l2_ctrl_video_device_poll *devicepoll)
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_VIDEO_DEVICE_POLL;
    control.string = (char *)devicepoll;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Done calling video device poll ");
}

int
NvVideoEncoder::SetPollInterrupt()
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_SET_POLL_INTERRUPT;
    control.value = 1;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder poll interrupt to 1 ");
}

int
NvVideoEncoder::ClearPollInterrupt()
{
    struct v4l2_ext_control control;
    struct v4l2_ext_controls ctrls;

    RETURN_ERROR_IF_FORMATS_NOT_SET();

    memset(&control, 0, sizeof(control));
    memset(&ctrls, 0, sizeof(ctrls));

    ctrls.count = 1;
    ctrls.controls = &control;
    ctrls.ctrl_class = V4L2_CTRL_CLASS_MPEG;

    control.id = V4L2_CID_MPEG_SET_POLL_INTERRUPT;
    control.value = 0;

    CHECK_V4L2_RETURN(setExtControls(ctrls),
            "Setting encoder poll interrupt to 0 ");
}
