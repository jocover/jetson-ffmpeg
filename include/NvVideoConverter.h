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
 * <b>NVIDIA Multimedia API: Video Converter</b>
 *
 * @b Description: This file declares a helper class for V4L2 Video Converter.
 */

/**
 * @defgroup l4t_mm_nvvideoconverter_group Video Converter
 * @ingroup l4t_mm_nvvideo_group
 *
 * Helper class that creates new V4L2
 * video converters, and it sets converter capture and output plane
 * formats.
 * @{
 */

#ifndef __NV_VIDEO_CONVERTER_H__
#define __NV_VIDEO_CONVERTER_H__

#include "NvV4l2Element.h"

/**
 * @brief Defines a helper class for V4L2 Video Converter.
 *
 * Use video converter for color space conversion, scaling, and
 * conversion between hardware buffer memory (<code>V4L2_MEMORY_MMAP/
 * V4L2_MEMORY_DMABUF</code>) and software buffer memory (\c V4L2_MEMORY_USERPTR).
 *
 * The video converter device node is \c "/dev/nvhost-vic". The category name
 * for the converter is \c "NVVIDCONV".
 *
 * Refer to [V4L2 Video Converter](group__V4L2Conv.html) for more information on the converter.
 */

class NvVideoConverter:public NvV4l2Element
{
public:

    /**
     * Creates a new V4L2 Video Converter object named \a name.
     *
     * This method internally calls \c v4l2_open on the converter dev node
     * \c "/dev/nvhost-vic" and checks for \c V4L2_CAP_VIDEO_M2M_MPLANE
     * capability on the device. This method allows the caller to specify
     * additional flags with which the device should be opened.
     *
     * The device is opened in blocking mode, which can be modified by passing
     * the O_NONBLOCK flag to this method.
     *
     * @returns Reference to the newly created converter object, else \a NULL in
     *          case of failure during initialization.
     */
    static NvVideoConverter *createVideoConverter(const char *name, int flags = 0);

     ~NvVideoConverter();
    /**
     * Sets the format on the converter output plane.
     *
     * Calls \c VIDIOC_S_FMT IOCTL internally on the capture plane.
     *
     * @param[in] pixfmt One of the raw V4L2 pixel formats.
     * @param[in] width Width of the output buffers in pixels.
     * @param[in] height Height of the output buffers in pixels.
     * @param[in] type Layout of the buffers in plane, one of
     *                   enum v4l2_nv_buffer_layout.
     * @return 0 for success, -1 otherwise.
     */
    int setCapturePlaneFormat(uint32_t pixfmt, uint32_t width, uint32_t height,
                              enum v4l2_nv_buffer_layout type);
    /**
     * Sets the format on the converter output plane.
     *
     * Calls \c VIDIOC_S_FMT IOCTL internally on the output plane.
     *
     * @param[in] pixfmt One of the raw V4L2 pixel formats.
     * @param[in] width Width of the output buffers in pixels.
     * @param[in] height Height of the output buffers in pixels.
     * @param[in] type Layout of the buffers in plane, one of
     *                   enum v4l2_nv_buffer_layout.
     * @return 0 for success, -1 otherwise.
     */
    int setOutputPlaneFormat(uint32_t pixfmt, uint32_t width, uint32_t height,
                             enum v4l2_nv_buffer_layout type);

    /**
     * Sets the buffer layout of the output plane buffers.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with control ID
     * #V4L2_CID_VIDEO_CONVERT_OUTPUT_PLANE_LAYOUT. Must be called before
     * setFormat() on any of the planes.
     *
     * @param[in] type Type of layout, one of enum v4l2_nv_buffer_layout.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setOutputPlaneBufferLayout(enum v4l2_nv_buffer_layout type);

    /**
     * Sets the buffer layout of the capture plane buffers.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_VIDEO_CONVERT_CAPTURE_PLANE_LAYOUT. Must be called before
     * setFormat() on any of the planes.
     *
     * @param[in] type Type of layout, one of enum v4l2_nv_buffer_layout.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setCapturePlaneBufferLayout(enum v4l2_nv_buffer_layout type);

    /**
     * Sets the interpolation(filter) method used for scaling.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_VIDEO_CONVERT_INTERPOLATION_METHOD. Must be called before
     * setFormat() on any of the planes.
     *
     * @param[in] method Type of interpolation method, one of enum
     *                   v4l2_interpolation_method.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setInterpolationMethod(enum v4l2_interpolation_method method);

    /**
     * Sets the flip method.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_VIDEO_CONVERT_FLIP_METHOD. Must be called before
     * setFormat() on any of the planes.
     *
     * @param[in] method Type of flip method, one of enum v4l2_flip_method.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setFlipMethod(enum v4l2_flip_method method);

    /**
     * Sets the TNR(Temporal Noise Reduction) algorithm to use.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_VIDEO_CONVERT_TNR_ALGORITHM. Must be called before
     * setForma() on any of the planes.
     *
     * @param[in] algorithm Type of TNR algorithm to use, one of enum
     *                      v4l2_tnr_algorithm.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setTnrAlgorithm(enum v4l2_tnr_algorithm algorithm);

    /**
     * Sets the YUV Rescale method to use.
     *
     * Calls the VIDIOC_S_EXT_CTRLS IOCTL internally with Control ID
     * #V4L2_CID_VIDEO_CONVERT_YUV_RESCALE_METHOD. Must be called before
     * setFormat() on any of the planes.
     *
     * @param[in] method Type of YUV Rescale method to use, one of enum
     *                      v4l2_yuv_rescale_method.
     *
     * @return 0 for success, -1 otherwise.
     */
    int setYUVRescale(enum v4l2_yuv_rescale_method method);

    /**
     * Set the cropping rectangle for the converter
     *
     * Calls the VIDIOC_S_SELECTION internally on the capture plane. Must be called
     * before setFormat() on any of the planes.
     *
     * @param[in] left Horizontal offset of the rectangle, in pixels.
     * @param[in] top  Verticaal offset of the rectangle, in pixels.
     * @param[in] width Width of the rectangle, in pixels.
     * @param[in] height Height of the rectangle, in pixels.
     *
     * @returns 0 for success, -1 for failure
     */
    int setCropRect(uint32_t left, uint32_t top, uint32_t width,
            uint32_t height);

    /**
     * Sets the destnation rectangle for the converter.
     *
     * Calls the VIDIOC_S_SELECTION internally on output plane. Must be called
     * before setFormat() on any of the planes.
     *
     * @param[in] left Horizontal offset of the rectangle, in pixels
     * @param[in] top  Verticaal offset of the rectangle, in pixels
     * @param[in] width Width of the rectangle, in pixels
     * @param[in] height Height of the rectangle, in pixels
     *
     * @returns 0 for success, -1 for failure
     */
    int setDestRect(uint32_t left, uint32_t top, uint32_t width,
            uint32_t height);

    /**
     * Waits until all buffers queued on the output plane are converted and
     * dequeued from the capture plane. This is a blocking call.
     *
     * @param[in] max_wait_ms Maximum time to wait in milliseconds
     * @returns 0 for success, -1 for timeout.
     */
    int waitForIdle(uint32_t max_wait_ms);

private:
    /**
     * Constructor used by #createVideoConverter.
     */
    NvVideoConverter(const char *name, int flags);

    static const NvElementProfiler::ProfilerField valid_fields =
            NvElementProfiler::PROFILER_FIELD_TOTAL_UNITS |
            NvElementProfiler::PROFILER_FIELD_LATENCIES |
            NvElementProfiler::PROFILER_FIELD_FPS;
};
/** @} */
#endif
