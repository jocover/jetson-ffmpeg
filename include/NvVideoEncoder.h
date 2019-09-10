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

/**
 * @file
 * <b>NVIDIA Multimedia API: Video Encode API</b>
 *
 */

/**
 * @defgroup l4t_mm_nvvideoencoder_group Video Encoder
 * @ingroup l4t_mm_nvvideo_group
 *
 * Helper class that creates new V4L2
 * video encoders, and it sets encoder capture and output plane
 * formats.
 * @{
 */

#ifndef __NV_VIDEO_ENCODER_H__
#define __NV_VIDEO_ENCODER_H__

#include "NvV4l2Element.h"

/**
 * @brief Defines a helper class for V4L2 Video Encoder.
 *
 * The video encoder device node is \c "/dev/nvhost-msenc". The category name
 * for the encoder is \c "NVENC".
 *
 * Refer to [V4L2 Video Encoder](group__V4L2Enc.html) for more information on the encoder.
 */

class NvVideoEncoder:public NvV4l2Element
{
public:
    /**
     * Creates a new V4L2 Video Encoder object named \a name.
     *
     * This method internally calls \c v4l2_open on the encoder dev node
     * \c "/dev/nvhost-msenc" and checks for \c V4L2_CAP_VIDEO_M2M_MPLANE
     * capability on the device. This method allows the caller to specify
     * additional flags with which the device must be opened.
     *
     * The device is opened in blocking mode, which can be modified by passing
     * the @a O_NONBLOCK flag to this method.
     *
     * @returns Reference to the newly created encoder object, else NULL in
     *          case of failure during initialization.
     */
    static NvVideoEncoder *createVideoEncoder(const char *name, int flags = 0);

     ~NvVideoEncoder();

    /**
     * Sets the format on the encoder output plane.
     *
     * Calls \c VIDIOC_S_FMT IOCTL internally on the output plane.
     *
     * @pre Applications must set the capture plane format using #setCapturePlaneFormat before calling this method.
     *
     * @param[in] pixfmt One of the raw V4L2 pixel formats.
     * @param[in] width Width of the input buffers in pixels.
     * @param[in] height Height of the input buffers in pixels.
     * @return 0 for success, -1 otherwise.
     */
    int setOutputPlaneFormat(uint32_t pixfmt, uint32_t width, uint32_t height);
    /**
     * Sets the format on the converter capture plane.
     *
     * Calls \c VIDIOC_S_FMT IOCTL internally on the capture plane.
     *
     * @param[in] pixfmt One of the coded V4L2 pixel formats.
     * @param[in] width Width of the input buffers in pixels.
     * @param[in] height Height of the input buffers in pixels.
     * @param[in] sizeimage Maximum size of the encoded buffers on the capture.
     *                      plane in bytes
     * @return 0 for success, -1 otherwise.
     */
    int setCapturePlaneFormat(uint32_t pixfmt, uint32_t width,
                              uint32_t height, uint32_t sizeimage);

    /**
     * Sets the encode framerate.
     *
     * Calls the VIDIOC_S_PARM IOCTL on the encoder capture plane. Can be
     * called any time after setFormat on both the planes.
     *
     * @param[in] framerate_num Numerator part of the framerate fraction.
     * @param[in] framerate_den Denominator part of the framerate fraction.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setFrameRate(uint32_t framerate_num, uint32_t framerate_den);

    /**
     * Sets the encoder bitrate.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * \c V4L2_CID_MPEG_VIDEO_BITRATE. Can be called any time after setFormat on
     * both the planes.
     *
     * @param[in] bitrate Bitrate of the encoded stream, in bits per second.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setBitrate(uint32_t bitrate);

    /**
     * Sets the encoder peak bitrate.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * \c V4L2_CID_MPEG_VIDEO_BITRATE_PEAK. Can be called any time after setFormat on
     * both the planes. Takes effect in VBR mode
     *
     * @param[in] peak_bitrate Peak Bitrate of the encoded stream, in bits per second.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setPeakBitrate(uint32_t peak_bitrate);

    /**
     * Sets the encoder profile.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * \c V4L2_CID_MPEG_VIDEO_H264_PROFILE or #V4L2_CID_MPEG_VIDEO_H265_PROFILE,
     * depending on the encoder type. Must be called after setFormat on both
     * the planes and before \c requestBuffers on any of the planes.
     *
     * @param[in] profile Profile to be used for encoding.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setProfile(uint32_t profile);

   /**
     * Sets the encoder command.
     *
     * Calls the VIDIOC_ENCODER_CMD internally with encoder commands.
     *
     * @return 0 for succes, -1 otherwise.
     */
    int setEncoderCommand(int cmd, int flags);

   /**
     * Set the encoder level.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * \c V4L2_CID_MPEG_VIDEO_H264_LEVEL. Must be called after setFormat on both
     * the planes and before \c requestBuffers on any of the planes.
     *
     * @param[in] level Level to be used for encoding, one of enum
     *                  v4l2_mpeg_video_h264_level
     *
     * @return 0 for success, -1 otherwise.
     */
    int setLevel(enum v4l2_mpeg_video_h264_level level);

    /**
     * Sets the encoder for maximum performance.
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
     * Sets the encoder constant qp.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * \c V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
     * \c and V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP. Must be called after setFormat on both
     * the planes and before \c requestBuffers on any of the planes.
     *
     * @param[in] qp_value Qp value
     *
     * @return 0 for success, -1 otherwise.
     */
    int setConstantQp(int qp_value);

    /**
     * Sets the encoder rate control mode.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * \c V4L2_CID_MPEG_VIDEO_BITRATE_MODE. Must be called after setFormat on both
     * the planes and before \c requestBuffers on any of the planes.
     *
     * @param[in] mode Type of rate control, one of enum
     *              v4l2_mpeg_video_bitrate_mode.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setRateControlMode(enum v4l2_mpeg_video_bitrate_mode mode);

    /**
     * Sets the encoder I-frame interval.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * \c V4L2_CID_MPEG_VIDEO_GOP_SIZE. Must be called after setFormat on both
     * the planes and before \c requestBuffers on any of the planes.
     *
     * @param[in] interval Interval between two I frames, in number of frames.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setIFrameInterval(uint32_t interval);

    /**
     * Sets the encoder IDR interval.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEO_IDR_INTERVAL. Must be called after setFormat on both
     * the planes and before \c requestBuffers on any of the planes.
     *
     * @param[in] interval Interval between two IDR frames, in number of frames.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setIDRInterval(uint32_t interval);

    /**
     * Forces an IDR frame.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * \c V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE. Must be called after
     * setFormat on both the planes.
     *
     * @return 0 for success, -1 otherwise.
     */
    int forceIDR();

    /**
     * Sets the encoder Temporal Tradeoff.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_TEMPORAL_TRADEOFF_LEVEL. Must be called after
     * setFormat on both the planes and before \c requestBuffers on any of the
     * planes.
     *
     * @param[in] level Temporal tradeoff level, one of
     *                  v4l2_enc_temporal_tradeoff_level_type.
     * @return 0 for success, -1 otherwise.
     */
    int setTemporalTradeoff(v4l2_enc_temporal_tradeoff_level_type level);

    /**
     * Sets the encoder output slice length.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_SLICE_LENGTH_PARAM. Must be called after setFormat on
     * both the planes and before \c requestBuffers on any of the planes.
     *
     * @param[in] type Slice length type, one of enum v4l2_enc_slice_length_type.
     * @param[in] length Length of the slice, in bytes if the type is
     *      #V4L2_ENC_SLICE_LENGTH_TYPE_BITS, else in number of MBs if the type is
     *      #V4L2_ENC_SLICE_LENGTH_TYPE_MBLK.
     * @return 0 for success, -1 otherwise.
     */
    int setSliceLength(v4l2_enc_slice_length_type type, uint32_t length);

    /**
     * Sets the encoder HW Preset Type.
     *
     * Calls the VIDIOC_S_EXT_CTRLS ioctl internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_HW_PRESET_TYPE_PARAM. Must be called after setFormat() on
     * both the planes and before \c requestBuffers on any of the planes.
     *
     * @param[in] type HW Preset Type, one of
     *                         enum v4l2_enc_hw_preset_type_param.
     * @return 0 for success, -1 otherwise.
     */
    int setHWPresetType(v4l2_enc_hw_preset_type type);

    /**
     * Sets the Region of Interest (ROI) parameters for the next buffer, which will
     * be queued on the output plane with index \a buffer_index.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_ROI_PARAMS. Must be called after
     * requesting buffer on both the planes.
     *
     * @param[in] buffer_index Index of the output plane buffer to apply the ROI
     *                         params.
     * @param[in] params A reference to the parameters to be applied on the frame, structure of
     *                   type v4l2_enc_frame_ROI_params.
     * @return 0 for success, -1 otherwise.
     */
    int setROIParams(uint32_t buffer_index, v4l2_enc_frame_ROI_params & params);

    /**
     * Enables  External ROI.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_ENABLE_ROI_PARAM. Must be called after
     * requesting buffer on both the planes.
     *
     * @param[in] params Parameters to be applied on the frame, structure of
     *                   type #v4l2_enc_enable_roi_param.
     * @return 0 for success, -1 otherwise.
     */
    int enableROI(v4l2_enc_enable_roi_param &params);

    /**
     * Enables Recon CRC.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_ENABLE_RECONCRC_PARAM. Must be called after
     * requesting buffer on both the planes.
     *
     * @param[in] params Parameters to be applied on the frame, structure of
     *                   type #v4l2_enc_enable_reconcrc_param.
     * @return 0 for success, -1 otherwise.
     */
    int enableReconCRC(v4l2_enc_enable_reconcrc_param &params);

    /**
     * Enable External RPS
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_ENABLE_EXTERNAL_RPS_CONTROL. Must be called after
     * requesting buffer on both the planes.
     *
     * @param[in] params Parameters to be applied on the frame, structure of
     *                   type #v4l2_enc_enable_ext_rps_ctr
     * @return 0 for success, -1 otherwise.
     */
    int enableExternalRPS(v4l2_enc_enable_ext_rps_ctr &params);

    /**
     * Enable External Picture RC
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_ENABLE_EXTERNAL_RATE_CONTROL. Must be called after
     * requesting buffer on both the planes.
     *
     * @param[in] params Parameters to be applied on the frame, structure of
     *                   type ##v4l2_enc_enable_ext_rate_ctr
     * @return 0 for success, -1 otherwise.
     */
    int enableExternalRC(v4l2_enc_enable_ext_rate_ctr &params);

    /**
     * Set input Metadata parameters for the next buffer which will
     * be queued on output plane with index \a buffer_index
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_INPUT_METADATA. Must be called after
     * requesting buffer on both the planes.
     *
     * @param[in] buffer_index Index of output plane buffer on which the external
     *                         RC params should be applied.
     * @param[in] params Parameters to be applied on the frame, structure of
     *                   type #v4l2_ctrl_videoenc_input_metadata
     * @return 0 for success, -1 otherwise.
     */
    int SetInputMetaParams(uint32_t buffer_index, v4l2_ctrl_videoenc_input_metadata &params);

    /**
     * Sets the virtual buffer size of the encoder.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_VIRTUALBUFFER_SIZE. Must be called after
     * setFormat on both the planes.
     *
     * @param[in] size Virtual buffer size, in bytes.
     * @return 0 for success, -1 otherwise.
     */
    int setVirtualBufferSize(uint32_t size);

    /**
     * Sets the number of reference frames of the encoder.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_NUM_REFERENCE_FRAMES. Must be called after
     * setFormat on both the planes.
     *
     * @param[in] num_frames Number of reference frames.
     * @return 0 for success, -1 otherwise.
     */
    int setNumReferenceFrames(uint32_t num_frames);

    /**
     * Sets slice intra-refresh interval params.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_SLICE_INTRAREFRESH_PARAM. Must be called after
     * setFormat on both the planes.
     *
     * @param[in] interval Slice intra-refresh interval, in number of slices.
     * @return 0 for success, -1 otherwise.
     */
    int setSliceIntrarefresh(uint32_t interval);

    /**
     * Sets the number of B frames to P frames.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_NUM_BFRAMES. Must be called after
     * setFormat on both the planes.
     *
     * @param[in] num Number of B frames.
     * @return 0 for success, -1 otherwise.
     */
    int setNumBFrames(uint32_t num);

    /**
     * Enables/disables insert SPS PPS at every IDR.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_INSERT_SPS_PPS_AT_IDR. Must be called after
     * setFormat on both the planes.
     *
     * @param[in] enabled Boolean value indicating whether to enable/disable
     *                    the control.
     * @return 0 for success, -1 otherwise.
     */
    int setInsertSpsPpsAtIdrEnabled(bool enabled);

    /**
     * Enables video encoder output motion vector metadata reporting.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_METADATA_MV. Must be called after setFormat on
     * both the planes and before \c requestBuffers on any of the planes.
     *
     * @return 0 for success, -1 otherwise.
     */
    int enableMotionVectorReporting();

    /**
     * Gets metadata for the encoded capture plane buffer.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_METADATA. Must be called for a buffer that has
     * been dequeued from the capture plane. The returned metadata corresponds
     * to the last dequeued buffer with index @a buffer_index.
     *
     * @param[in] buffer_index Index of the capture plane buffer whose metadata
     *              is required.
     * @param[in,out] enc_metadata Reference to the metadata structure
     *              v4l2_ctrl_videoenc_outputbuf_metadata to be filled.
     *
     * @return 0 for success, -1 otherwise.
     */
    int getMetadata(uint32_t buffer_index,
            v4l2_ctrl_videoenc_outputbuf_metadata &enc_metadata);

    /**
     * Gets motion vector metadata for the encoded capture plane buffer.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_MPEG_VIDEOENC_METADATA_MV. Must be called for a buffer that has
     * been dequeued from the capture plane. The returned metadata corresponds
     * to the last dequeued buffer with index @a buffer_index.
     *
     * @param[in] buffer_index Index of the capture plane buffer whose metadata
     *              is required.
     * @param[in,out] enc_mv_metadata Reference to the metadata structure
     *              v4l2_ctrl_videoenc_outputbuf_metadata_MV to be filled.
     *
     * @return 0 for success, -1 otherwise.
     */
    int getMotionVectors(uint32_t buffer_index,
            v4l2_ctrl_videoenc_outputbuf_metadata_MV &enc_mv_metadata);

    /**
     * Sets QP values for I/P/B frames.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with control Id
     * %V4L2_CID_MPEG_VIDEOENC_QP_RANGE. Must be called after
     * setFormat on both the planes.
     *
     * @param[in] MinQpI Minimum Qp Value for I frame.
     * @param[in] MaxQpI Minimum Qp Value for I frame.
     * @param[in] MinQpP Minimum Qp Value for P frame.
     * @param[in] MaxQpP Minimum Qp Value for P frame.
     * @param[in] MinQpB Minimum Qp Value for B frame.
     * @param[in] MaxQpB Minimum Qp Value for B frame.
     * @returns 0 for success, -1 otherwise.
     */
    int setQpRange(uint32_t MinQpI, uint32_t MaxQpI, uint32_t MinQpP,
            uint32_t MaxQpP, uint32_t MinQpB, uint32_t MaxQpB);

    /**
     * Enables/disables insert VUI.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * @c V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE. Must be called after
     * setFormat on both the planes.
     *
     * @param[in] enabled Boolean value indicating whether to enable/disable
     *                    the control.
     * @return 0 for success, -1 otherwise.
     */
    int setInsertVuiEnabled(bool enabled);

    /**
     * Enables/disables extended color format.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * @c V4L2_CID_MPEG_VIDEOENC_EXTEDED_COLORFORMAT. Must be called after
     * setFormat on both the planes.
     *
     * @param[in] enabled Boolean value indicating whether to enable/disable
     *                    the control.
     * @return 0 for success, -1 otherwise.
     */
    int setExtendedColorFormat(bool enabled);

    /**
     * Enables/disables insert AUD.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * @c V4L2_CID_MPEG_VIDEO_H264_AUD_SAR_ENABLE. Must be called after
     * setFormat on both the planes.
     *
     * @param[in] enabled Boolean value indicating whether to enable/disable
     *                    the control.
     * @return 0 for success, -1 otherwise.
     */
    int setInsertAudEnabled(bool enabled);

    /**
     * Enables/disables all i-frame encode.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * @c V4L2_CID_MPEG_VIDEOENC_ENABLE_ALLIFRAME_ENCODE. Must be called after
     * setFormat on both the planes.
     *
     * @param[in] enabled Boolean value indicating whether to enable/disable
     *                    the control.
     * @return 0 for success, -1 otherwise.
     */
    int setAlliFramesEncode(bool enabled);

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
     * Constructor used by #createVideoEncoder.
     */
    NvVideoEncoder(const char *name, int flags);

    static const NvElementProfiler::ProfilerField valid_fields =
            NvElementProfiler::PROFILER_FIELD_TOTAL_UNITS |
            NvElementProfiler::PROFILER_FIELD_LATENCIES |
            NvElementProfiler::PROFILER_FIELD_FPS;
};
/** @} */

#endif
