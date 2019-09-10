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
 * <b>NVIDIA Multimedia API: V4L2 Helper Class</b>
 *
 * @b Description: This file declares a helper class for V4L2-based components.
 */

/**
 * @defgroup l4t_mm_nvv4l2element_group V4L2 Element Class
 * @ingroup l4t_mm_nvelement_group
 *
 * Helper class that provides common functionality for V4L2-based components,
 * such as encoder and decoder. Objects instantiated from this class create
 * new V4L2 elements, subscribes to V4L2 events, dequeues an event from
 * an element, and sets/gets control values.
 * @{
 */
#ifndef __NV_V4L2_ELEMENT_H__
#define __NV_V4L2_ELEMENT_H__

#include "NvElement.h"
#include "NvV4l2ElementPlane.h"

#include "v4l2_nv_extensions.h"

/**
 * Defines a helper class for V4L2 based components.
 *
 * This derived class provides common functionality for V4L2 components. V4L2-based
 * components such as encoder/decoder extend from this class.
 *
 * This class is modeled on V4L2 M2M devices. It includes the file descriptor (FD) of the device
 * opened using %v4l2_open, two planes (NvV4l2ElementPlane), output plane,
 * capture plane, and other helper methods, such as setting/getting controls,
 * subscribing/dequeueing events, etc.
 */
class NvV4l2Element:public NvElement
{
public:
    virtual ~NvV4l2Element();

    /**
     * Subscribes to an V4L2 event.
     *
     * Calls \c VIDIOC_SUBSCRIBE_EVENT IOCTL internally.
     *
     * @param[in] type Type of the event.
     * @param[in] id ID of the event source.
     * @param[in] flags Event flags.
     * @return 0 for success, -1 otherwise.
     */
    int subscribeEvent(uint32_t type, uint32_t id, uint32_t flags);
    /**
     * Dequeues an event from the element.
     *
     * Calls \c VIDIOC_DQEVENT IOCTL internally. The caller can specify the maximum time
     * to wait for dequeuing the event. The call blocks until an event is
     * dequeued successfully or timeout is reached.
     *
     * @param[in,out] event A reference to the \c v4l2_event structure to fill.
     * @param[in] max_wait_ms Specifies the max wait time for dequeuing an event,
     *                        in milliseconds.
     * @return 0 for success, -1 otherwise.
     */
    int dqEvent(struct v4l2_event &event, uint32_t max_wait_ms);

    /**
     * Sets the value of a control.
     *
     * Calls \c VIDIOC_S_CTRL IOCTL internally.
     *
     * @param[in] id ID of the control to be set.
     * @param[in] value Value to be set on the control.
     * @return 0 for success, -1 otherwise.
     */
    int setControl(uint32_t id, int32_t value);
    /**
     * Gets the value of a control.
     *
     * Calls \c VIDIOC_G_CTRL IOCTL internally.
     *
     * @param[in] id ID of the control to get.
     * @param[out] value A reference to the variable into which the control value
                         is read.
     * @return 0 for success, -1 otherwise.
     */
    int getControl(uint32_t id, int32_t &value);

    /**
     * Sets the value of several controls.
     *
     * Calls \c VIDIOC_S_EXT_CTRLS IOCTL internally.
     *
     * @param[in] ctl A pointer to the controls to set.
     * @return 0 for success, -1 otherwise.
     */
    int setExtControls(struct v4l2_ext_controls &ctl);
    /**
     * Gets the value of several controls.
     *
     * Calls \c VIDIOC_G_EXT_CTRLS IOCTL internally.
     *
     * @param[in,out] ctl A pointer to the controls to get.
     * @return 0 for success, -1 otherwise.
     */
    int getExtControls(struct v4l2_ext_controls &ctl);

    virtual int isInError();

    /**
     * Sets the output plane.
     */
    NvV4l2ElementPlane output_plane;  /**< Output plane of the element */
    /**
     * Sets the capture plane.
     */
    NvV4l2ElementPlane capture_plane; /**< Capture plane of the element */

    /**
     *
     * Terminates processing of queued buffers immediately. All the buffers are
     * returned to the application.
     *
     * Calls VIDIOC_STREAMOFF IOCTL on both of the planes internally.
     *
     * @return 0 for success, -1 otherwise.
     */
    int abort();

    /**
     * Waits until the element processes all the output plane buffers.
     *
     * Objects extending @c V4l2Element must implement this because the idle
     * condition is component-specific.
     *
     * @param[in] max_wait_ms Max time to wait in milliseconds.
     * @return 0 for success, -1 otherwise.
     */
    virtual int waitForIdle(uint32_t max_wait_ms);

    void *app_data; /**< A pointer to the application-specific data. */

    /**
     * Enables profiling for the V4l2Element.
     *
     * Must be called before setting either plane formats.
     */
    void enableProfiling();

protected:
    int fd;         /**< Specifies the FD of the device opened using \c v4l2_open. */

    uint32_t output_plane_pixfmt;  /**< Pixel format of output plane buffers */
    uint32_t capture_plane_pixfmt; /**< Pixel format of capture plane buffers */

    /**
     * Creates a new V4l2Element named \a name.
     *
     * This constructor calls v4l2_open on the \a dev_node. It sets an error if
     * v4l2_open fails.
     *
     * This function also checks if the device supports V4L2_CAP_VIDEO_M2M_MPLANE
     * capability.
     *
     * @param[in] comp_name A pointer to the unique name to identity the element instance.
     * @param[in] dev_node A pointer to /dev/ * node of the device.
     * @param[in] flags Flags with which to open the device.
     * @param[in] fields Profiler fields that are valid for the element.
     */
    NvV4l2Element(const char *comp_name, const char *dev_node, int flags, NvElementProfiler::ProfilerField fields);
};
/** @} */
#endif
