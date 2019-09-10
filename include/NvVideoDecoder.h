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
 * <b>NVIDIA Multimedia API: Video Decode API</b>
 *
 */

/**
 * @defgroup l4t_mm_nvvideodecoder_group Video Decoder
 * @ingroup l4t_mm_nvvideo_group
 *
 * Helper class that creates new V4L2
 * video decoders, and it sets decoder capture and output plane
 * formats.
 * @{
 */

#ifndef __NV_VIDEO_DECODER_H__
#define __NV_VIDEO_DECODER_H__

#include "NvV4l2Element.h"

/**
 * @brief Defines a helper class for V4L2 Video Decoder.
 *
 * The video decoder device node is `/dev/nvhost-nvdec`. The category name
 * for the decoder is \c "NVDEC".
 *
 * Refer to [V4L2 Video Decoder](group__V4L2Dec.html) for more information on the decoder.
 */
class NvVideoDecoder:public NvV4l2Element
{
public:
    /**
     * Creates a new V4L2 Video Decoder object named \a name.
     *
     * This method internally calls \c v4l2_open on the decoder dev node
     * \c "/dev/nvhost-nvdec" and checks for \c V4L2_CAP_VIDEO_M2M_MPLANE
     * capability on the device. This method allows the caller to specify
     * additional flags with which the device should be opened.
     *
     * The device is opened in blocking mode, which can be modified by passing
     * the @a O_NONBLOCK flag to this method.
     *
     * @returns Reference to the newly created decoder object else \a NULL in
     *          case of failure during initialization.
     */
    static NvVideoDecoder *createVideoDecoder(const char *name, int flags = 0);

     ~NvVideoDecoder();
    /**
     * Sets the format on the decoder output plane.
     *
     * Calls \c VIDIOC_S_FMT IOCTL internally on the capture plane.
     *
     * @param[in] pixfmt One of the raw V4L2 pixel formats.
     * @param[in] width Width of the output buffers in pixels.
     * @param[in] height Height of the output buffers in pixels.
     * @return 0 for success, -1 otherwise.
     */
    int setCapturePlaneFormat(uint32_t pixfmt, uint32_t width, uint32_t height);
    /**
     * Sets the format on the decoder output plane.
     *
     * Calls the \c VIDIOC_S_FMT IOCTL internally on the output plane.
     *
     * @param[in] pixfmt One of the coded V4L2 pixel formats.
     * @param[in] sizeimage Maximum size of the buffers on the output plane.
                            containing encoded data in bytes.
     * @return 0 for success, -1 otherwise.
     */
    int setOutputPlaneFormat(uint32_t pixfmt, uint32_t sizeimage);

    /**
     * Informs the decoder that input buffers may not contain complete frames.
     * Deprecated interface, Use setFrameInputMode instead.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * V4L2_CID_MPEG_VIDEO_DISABLE_COMPLETE_FRAME_INPUT.
     *
     * @return 0 for success, -1 otherwise.
     */
    int disableCompleteFrameInputBuffer();

    /**
     * Informs the decoder that input buffers may not contain complete frames.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * V4L2_CID_MPEG_VIDEO_DISABLE_COMPLETE_FRAME_INPUT.
     *
     * @param[in] ctrl_value control value to disable
     *                       complete frame input buffer.
     * @return 0 for success, -1 otherwise.
     */
    int setFrameInputMode(unsigned int ctrl_value);

    /**
     * Disables the display picture buffer.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * V4L2_CID_MPEG_VIDEO_DISABLE_DPB. Must be called after setFormat on both
     * the planes and before requestBuffers on any of the planes.
     *
     * @return 0 for success, -1 otherwise.
     */
    int disableDPB();

    /**
     * Gets the minimum number of buffers to be requested on the decoder capture plane.
     *
     * Calls the VIDIOC_G_CTRL IOCTL internally with Control ID
     * V4L2_CID_MIN_BUFFERS_FOR_CAPTURE. It is valid after the first
     * V4L2_RESOLUTION_CHANGE_EVENT and may change after each subsequent
     * event.
     *
     * @param[out] num A reference to the integer to return the number of buffers.
     *
     * @return 0 for success, -1 otherwise.
     */
    int getMinimumCapturePlaneBuffers(int & num);

    /**
     * Sets the skip-frames parameter of the decoder.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * V4L2_CID_MPEG_VIDEO_SKIP_FRAMES. Must be called after setFormat on both
     * the planes and before requestBuffers on any of the planes.
     *
     * @param[in] skip_frames Type of frames to skip decoding, one of
     *                        enum v4l2_skip_frames_type.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setSkipFrames(enum v4l2_skip_frames_type skip_frames);

    /**
     * Sets the decoder for maximum performance.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * V4L2_CID_MPEG_VIDEO_MAX_PERFORMANCE. Must be called after setFormat on both
     * the planes and before requestBuffers on any of the planes.
     *
     * @param[in] flag Integer variable to indicate whether max performance is to be
     *                        enabled/disabled.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setMaxPerfMode(int flag);

    /**
     * Enables video decoder output metadata reporting.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * V4L2_CID_MPEG_VIDEO_ERROR_REPORTING. Must be called after setFormat on
     * both the planes and before requestBuffers on any of the planes.
     *
     * @return 0 for success, -1 otherwise.
     */
    int enableMetadataReporting();

    int checkifMasteringDisplayDataPresent(v4l2_ctrl_video_displaydata &displaydata);
    int MasteringDisplayData(v4l2_ctrl_video_hdrmasteringdisplaydata *hdrmasteringdisplaydata);

    /**
     * Gets metadata for the decoded capture plane buffer.
     *
     * Calls the VIDIOC_G_EXT_CTRLS IOCTL internally with Control ID
     * V4L2_CID_MPEG_VIDEODEC_METADATA. Must be called for a buffer that has
     * been dequeued from the capture plane. The returned metadata corresponds
     * to the last dequeued buffer with index @a buffer_index.
     *
     * @param[in] buffer_index Index of the capture plane buffer whose metadata
     *              is required.
     * @param[in,out] metadata Reference to the metadata structure
     *              v4l2_ctrl_videodec_outputbuf_metadata to be filled.
     *
     * @return 0 for success, -1 otherwise.
     */
    int getMetadata(uint32_t buffer_index,
        v4l2_ctrl_videodec_outputbuf_metadata &metadata);

    /**
     * Gets metadata for the decoder output plane buffer.
     *
     * Calls the VIDIOC_G_EXT_CTRLS IOCTL internally with Control ID
     * V4L2_CID_MPEG_VIDEODEC_INPUT_METADATA. Must be called for a buffer that has
     * been dequeued from the output plane. The returned metadata corresponds
     * to the last dequeued buffer with index @a buffer_index.
     *
     * @param[in] buffer_index Index of the output plane buffer whose metadata
     *              is required.
     * @param[in,out] input_metadata Reference to the metadata structure
     *              v4l2_ctrl_videodec_inputbuf_metadata to be filled.
     *
     * @return 0 for success, -1 otherwise.
     */
    int getInputMetadata(uint32_t buffer_index,
        v4l2_ctrl_videodec_inputbuf_metadata &input_metadata);

    /**
     * Issues Poll on the device which blocks until :
     * a) Either there is something to dequeue from capture or output plane or any events.
     * b) Poll was interrupted by a call to the device using V4L2_CID_SET_POLL_INTERRUPT
     * c) Application has already interrupted polling by V4L2_CID_SET_POLL_INTERRUPT
     */
    int DevicePoll(v4l2_ctrl_video_device_poll *devicepoll);

    /**
     * Sets the polling interrupt, now if the application calls Poll, the device should
     * not block, in other words polling is disabled.
     */
    int SetPollInterrupt();

    /**
     * Clears the polling interrupt, now if the application calls Poll, the device should
     * block until the event is triggered, in other words polling is enabled.
     */
    int ClearPollInterrupt();

private:
    /**
     * Constructor used by #createVideoDecoder.
     */
    NvVideoDecoder(const char *name, int flags);

    static const NvElementProfiler::ProfilerField valid_fields =
            NvElementProfiler::PROFILER_FIELD_TOTAL_UNITS |
            NvElementProfiler::PROFILER_FIELD_FPS;
};
/** @} */
#endif
