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
 * <b>NVIDIA Multimedia API: V4L2 Element Plane</b>
 *
 * @b Description: This file declares a helper class for operations
 * performed on a V4L2 Element plane.
 */

/**
 * @defgroup l4t_mm_nvv4lelementplane_group NvV4l2ElementPlane Class
 * @ingroup l4t_mm_nvelement_group
 *
 * Helper class for operations performed on a V4L2 element plane.
 * This includes getting/setting plane formats, plane buffers,
 * and cropping.
 * @{
 */

#ifndef __NV_V4L2_ELELMENT_PLANE_H__
#define __NV_V4L2_ELELMENT_PLANE_H__

#include <pthread.h>
#include "NvElement.h"
#include "NvLogging.h"
#include "NvBuffer.h"

/**
 * Prints a plane-specific message of level LOG_LEVEL_DEBUG.
 * Must not be used by applications.
 */
#define PLANE_DEBUG_MSG(str) COMP_DEBUG_MSG(plane_name << ":" << str);
/**
 * Prints a plane-specific message of level LOG_LEVEL_INFO.
 * Must not be used by applications.
 */
#define PLANE_INFO_MSG(str) COMP_INFO_MSG(plane_name << ":" << str);
/**
 * Prints a plane-specific message of level LOG_LEVEL_WARN.
 * Must not be used by applications.
 */
#define PLANE_WARN_MSG(str) COMP_WARN_MSG(plane_name << ":" << str);
/**
 * Prints a plane-specific message of level LOG_LEVEL_ERROR.
 * Must not be used by applications.
 */
#define PLANE_ERROR_MSG(str) COMP_ERROR_MSG(plane_name << ":" << str);
/**
 * Prints a plane-specific system error message of level LOG_LEVEL_ERROR.
 * Must not be used by applications.
 */
#define PLANE_SYS_ERROR_MSG(str) COMP_SYS_ERROR_MSG(plane_name << ":" << str);

/**
 * @brief Defines a helper class for operations performed on a V4L2 Element plane.
 *
 * This derived class is modeled on the planes of a V4L2 Element. It provides
 * convenient wrapper methods around V4L2 IOCTLs associated with plane
 * operations such as <code>VIDIOC_G_FMT/VIDIOC_S_FMT</code>, \c VIDIOC_REQBUFS,
 * <code>VIDIOC_STREAMON/VIDIOC_STREAMOFF</code>, etc.
 *
 * The plane buffer type can be either \c V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE (for
 * the output plane) or \c V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE (for the capture
 * plane).
 *
 * The plane has an array of NvBuffer object pointers that is allocated and
 * initialized during reqbuf call. These \c NvBuffer objects are similar to the
 * \c v4l2_buffer structures that are queued/dequeued.
 *
 * This class provides another feature useful for multi-threading. On calling
 * #startDQThread, it internally spawns a thread that runs infinitely until
 * signaled to stop. This thread keeps trying to dequeue a buffer from the
 * plane and calls a #dqThreadCallback method specified by the user on
 * successful dequeue.
 *
 */
class NvV4l2ElementPlane
{

public:
    /**
     * Gets the plane format.
     *
     * Calls @b VIDIOC_G_FMT \c IOCTL internally.
     *
     * @param[in,out] format A reference to the \c v4l2_format structure to be filled.
     * @return 0 for success, -1 otherwise.
     */
    int getFormat(struct v4l2_format & format);
    /**
     * Sets the plane format.
     *
     * Calls @b VIDIOC_S_FMT \c IOCTL internally.
     *
     * @param[in] format A reference to the \c v4l2_format structure to be set on the plane.
     * @return 0 for success, -1 otherwise.
     */
    int setFormat(struct v4l2_format & format);

    /**
     * Maps the NvMMBuffer to NvBuffer for V4L2_MEMORY_DMABUF.
     *
     * @param[in] v4l2_buf Address of the NvBuffer to which the NvMMBuffer is mapped.
     * @param[in] dmabuff_fd Index to the field that holds NvMMBuffer attributes.
     * @return 0 for success, -1 otherwise.
     */

    int mapOutputBuffers(struct v4l2_buffer &v4l2_buf, int dmabuff_fd);

    /**
     * Unmaps the NvMMBuffer for V4L2_MEMORY_DMABUF.
     *
     * @param[in] index for the current buffer index.
     * @param[in] dmabuff_fd Index to the field that holds NvMMBuffer attributes.
     * @return 0 for success, -1 otherwise.
     */

    int unmapOutputBuffers(int index, int dmabuff_fd);

    /**
     * Gets the cropping rectangle for the plane.
     *
     * Calls @b VIDIOC_G_CROP \c IOCTL internally.
     *
     * @param[in] crop A reference to the \c v4l2_crop structure to be filled.
     * @return 0 for success, -1 otherwise.
     */
    int getCrop(struct v4l2_crop & crop);

    /**
     * Sets the selection rectangle for the plane.
     *
     * Calls @b VIDIOC_S_SELECTION IOCTL internally.
     *
     * @param[in] target Specifies the rectangle selection type.
     * @param[in] flags Specifies the flags to control selection adjustments.
     * @param[in] rect A reference to the selection rectangle.
     * @return 0 for success, -1 otherwise.
     */
    int setSelection(uint32_t target, uint32_t flags, struct v4l2_rect & rect);

    /**
     * Requests for buffers on the plane.
     *
     * Calls \c VIDIOC_REQBUFS IOCTL internally. Creates an array of NvBuffer of
     * length equal to the count returned by the IOCTL.
     *
     * @param[in] mem_type Specifies the type of V4L2 memory to be requested.
     * @param[in] num Specifies the number of buffers to request on the plane.
     * @return 0 for success, -1 otherwise.
     */
    int reqbufs(enum v4l2_memory mem_type, uint32_t num);
    /**
     * Queries the status of the buffer at the index.
     *
     * @warning This method works only for \c V4L2_MEMORY_MMAP memory.
     *
     * Calls \c VIDIOC_QUERYBUF IOCTL internally. Populates the \a length and \a
     * mem_offset members of all the NvBuffer::NvBufferPlane members of the
     * \c %NvBuffer object at index \a buf_index.
     *
     * @param[in] buf_index Specifies the index of the buffer to query.
     * @return 0 for success, -1 otherwise.
     */
    int queryBuffer(uint32_t buf_index);
    /**
     * Exports the buffer as DMABUF FD.
     *
     * @warning This method works only for \c V4L2_MEMORY_MMAP memory.
     *
     * Calls \c VIDIOC_EXPBUF IOCTL internally. Populates the \a fd member of all
     * the \c NvBuffer::NvBufferPlane members of \c NvBuffer object at index \a buf_in.
     *
     * @param[in] buf_index Specifies the index of the buffer to export.
     * @return 0 for success, -1 otherwise.
     */
    int exportBuffer(uint32_t buf_index);

    /**
     * Starts or stops streaming on the plane.
     *
     * Calls \c VIDIOC_STREAMON/VIDIOC_STREAMOFF IOCTLs internally.
     *
     * @param[in] status Must be TRUE to start the stream, FALSE to stop the stream.
     * @return 0 for success, -1 otherwise.
     */
    int setStreamStatus(bool status);

    /**
     * Checks whether the plane is streaming.
     *
     * @returns true if the plane is streaming, false otherwise.
     */
    bool getStreamStatus();

    /**
     * Sets streaming parameters.
     *
     * Calls \c VIDIOC_S_PARM IOCTL internally.
     *
     * @param[in] parm A reference to the \c v4l2_streamparm structure to be set on the
     *                 plane.
     * @return 0 for success, -1 otherwise.
     */
    int setStreamParms(struct v4l2_streamparm & parm);

    /**
     * Helper method that encapsulates all the method calls required to
     * set up the plane for streaming.
     *
     * Calls reqbuf internally. Then, for each of the buffers, calls #queryBuffer,
     * #exportBuffer and maps the buffer/allocates the buffer memory depending
     * on the memory type.
     *
     * @sa deinitPlane
     *
     * @param[in] mem_type V4L2 Memory to use on the buffer.
     * @param[in] num_buffers Number of buffer to request on the plane.
     * @param[in] map boolean value indicating if the buffers should be mapped to
                      memory (Only for V4L2_MEMORY_MMAP).
     * @param[in] allocate boolean valued indicating whether the buffers should be
                           allocated memory (Only for V4L2_MEMORY_USERPTR).
     * @return 0 for success, -1 otherwise.
     */
    int setupPlane(enum v4l2_memory mem_type, uint32_t num_buffers, bool map, bool allocate);
    /**
     * Helper method that encapsulates all the method calls required to
     * deinitialize the plane for streaming.
     *
     * For each of the buffers, unmaps/deallocates memory depending on the
     * memory type. Then, calls reqbuf with count zero.
     *
     * @sa setupPlane
     */
    void deinitPlane();

    /**
     * Gets the streaming/buffer type of this plane.
     *
     * @returns Type of the buffer belonging to enum \c v4l2_buf_type.
     */
    inline enum v4l2_buf_type getBufType()
    {
        return buf_type;
    }

    /**
     * Gets the \c NvBuffer object at index n.
     *
     * @returns \c %NvBuffer object at index n, NULL if n >= number of buffers.
     */
    NvBuffer *getNthBuffer(uint32_t n);

    /**
     * Dequeues a buffer from the plane.
     *
     * This is a blocking call. This call returns when a buffer is successfully
     * dequeued or timeout is reached. If \a buffer is not NULL, returns the
     * \c NvBuffer object at the index returned by the \c VIDIOC_DQBUF IOCTL. If this
     * plane shares a buffer with other elements and \a shared_buffer is not
     * NULL, returns the shared \c %NvBuffer object in \a shared_buffer.
     *
     * @param[in] v4l2_buf A reference to the \c v4l2_buffer structure to use for dequeueing.
     * @param[out] buffer Returns a pointer to a pointer to the \c %NvBuffer object associated with the dequeued
     *                    buffer. Can be NULL.
     * @param[out] shared_buffer Returns a pointer to a pointer to the shared \c %NvBuffer object if the queued
                   buffer is shared with other elements. Can be NULL.
     * @param[in] num_retries Number of times to try dequeuing a buffer before
     *                        a failure is returned. In case of non-blocking
     *                        mode, this is equivalent to the number of
     *                        milliseconds to try to dequeue a buffer.
     * @return 0 for success, -1 otherwise.
     */
    int dqBuffer(struct v4l2_buffer &v4l2_buf, NvBuffer ** buffer,
                 NvBuffer ** shared_buffer, uint32_t num_retries);
    /**
     * Queues a buffer on the plane.
     *
     * This method calls \c VIDIOC_QBUF internally. If this plane is sharing a
     * buffer with other elements, the application can pass the pointer to the
     * shared NvBuffer object in \a shared_buffer.
     *
     * @param[in] v4l2_buf A reference to the \c v4l2_buffer structure to use for queueing.
     * @param[in] shared_buffer A pointer to the shared \c %NvBuffer object.
     * @return 0 for success, -1 otherwise.
     */
    int qBuffer(struct v4l2_buffer &v4l2_buf, NvBuffer * shared_buffer);

    /**
     * Gets the number of buffers allocated/requested on the plane.
     *
     * @returns Number of buffers.
     */
    inline uint32_t getNumBuffers()
    {
        return num_buffers;
    }

    /**
     * Gets the number of planes buffers on this plane for the currently
     * set format.
     *
     * @returns Number of planes.
     */
    inline uint32_t getNumPlanes()
    {
        return n_planes;
    }

    /**
     * Sets the format of the planes of the buffer that is used with this
     * plane.
     *
     * The buffer plane format must be set before calling reqbuf since
     * these are needed by the NvBuffer constructor.
     *
     * @sa reqbufs
     *
     * @param[in] n_planes Number of planes in the buffer.
     * @param[in] planefmts A pointer to the array of \c NvBufferPlaneFormat that describes the
     *            format of each of the plane. The array length must be at
     *            least @a n_planes.
     */
    void setBufferPlaneFormat(int n_planes, NvBuffer::NvBufferPlaneFormat * planefmts);

    /**
     * Gets the number of buffers currently queued on the plane.
     *
     * @returns Number of buffers currently queued on the plane.
     */
    inline uint32_t getNumQueuedBuffers()
    {
        return num_queued_buffers;
    }

    /**
     * Gets the total number of buffers dequeued from the plane.
     *
     * @returns Total number of buffers dequeued from the plane.
     */
    inline uint32_t getTotalDequeuedBuffers()
    {
        return total_dequeued_buffers;
    }

    /**
     * Gets the total number of buffers queued on the plane.
     *
     * @returns Total number of buffers queued on the plane.
     */
    inline uint32_t getTotalQueuedBuffers()
    {
        return total_queued_buffers;
    }

    /**
     * Waits until all buffers of the plane are queued.
     *
     * This is a blocking call that returns when all the buffers are queued
     * or timeout is reached.

     * @param[in] max_wait_ms Maximum time to wait, in milliseconds.
     * @return 0 for success, -1 otherwise.
     */
    int waitAllBuffersQueued(uint32_t max_wait_ms);
    /**
     * Waits until all buffers of the plane are dequeued.
     *
     * This is a blocking call that returns when all the buffers are dequeued
     * or timeout is reached.

     * @param[in] max_wait_ms Maximum time to wait, in milliseconds
     * @return 0 for success, -1 otherwise.
     */
    int waitAllBuffersDequeued(uint32_t max_wait_ms);

    /**
     * This is a callback function type method that is called by the DQ Thread when
     * it successfully dequeues a buffer from the plane. Applications must implement
     * this and set the callback using #setDQThreadCallback.
     *
     * Setting the stream to off automatically stops this thread.
     *
     * @sa setDQThreadCallback, #startDQThread
     *
     * @param v4l2_buf A pointer to the \c v4l2_buffer structure that is used for dequeueing.
     * @param buffer A pointer to the NvBuffer object at the \c index contained in \c v4l2_buf.
     * @param shared_buffer A pointer to the NvBuffer object if the plane shares a buffer with other elements,
     *         else NULL.
     * @param data A pointer to application specific data that is set with
     *             #startDQThread.
     * @returns If the application implementing this call returns FALSE,
     *          the DQThread is stopped; else, the DQ Thread continues running.
     */
    typedef bool(*dqThreadCallback) (struct v4l2_buffer * v4l2_buf,
            NvBuffer * buffer, NvBuffer * shared_buffer,
            void *data);

    /**
     * Sets the DQ Thread callback method.
     *
     * The callback method is called from the DQ Thread once a buffer is
     * successfully dequeued.

     * @param[in] callback Method to be called upon succesful dequeue.
     * @returns TRUE for success, FALSE for failure.
     */
    bool setDQThreadCallback(dqThreadCallback callback);
    /**
     * Starts DQ Thread.
     *
     * This method starts a thread internally. On successful dequeue of a
     * buffer from the plane, the #dqThreadCallback method set using
     * #setDQThreadCallback is called.
     *
     * Setting the stream to off automatically stops the thread.
     *
     * @sa stopDQThread, waitForDQThread
     *
     * @param[in] data A pointer to the application data. This is provided as an
     *                 argument in the \c dqThreadCallback method.
     * @return 0 for success, -1 otherwise.
     */
    int startDQThread(void *data);
    /**
     * Force stops the DQ Thread if it is running.
     *
     * Does not work when the device is opened in blocking mode.
     *
     * @sa startDQThread, waitForDQThread
     *
     * @return 0 for success, -1 otherwise.
     */
    int stopDQThread();
    /**
     * Waits for the DQ Thread to stop.
     *
     * This method waits until the DQ Thread stops or timeout is reached.
     *
     * @sa startDQThread, stopDQThread
     *
     * @param[in] max_wait_ms Maximum wait time, in milliseconds.
     * @return 0 for success, -1 otherwise.
     */
    int waitForDQThread(uint32_t max_wait_ms);

    pthread_mutex_t plane_lock; /**< Mutex lock used along with #plane_cond. */
    pthread_cond_t plane_cond; /**< Plane condition that application can wait on
                                    to receive notifications from
                                    #qBuffer/#dqBuffer. */

private:
    int &fd;     /**< A reference to the FD of the V4l2 Element the plane is associated with. */

    const char *plane_name; /**< A pointer to the name of the plane. Could be "Output Plane" or
                                 "Capture Plane". Used only for debug logs. */
    enum v4l2_buf_type buf_type; /**< Speciifes the type of the stream. */

    bool blocking;  /**< Specifies whether the V4l2 element is opened with
                         blocking mode. */

    uint32_t num_buffers;   /**< Holds the number of buffers returned by \c VIDIOC_REQBUFS
                                 IOCTL. */
    NvBuffer **buffers;     /**< A pointer to an array of NvBuffer object pointers. This array is
                                 allocated and initialized in #reqbufs. */

    uint8_t n_planes;       /**< Specifies the number of planes in the buffers. */
    NvBuffer::NvBufferPlaneFormat planefmts[MAX_PLANES];
                            /**< Format of the buffer planes. This must be
                                 initialized before calling #reqbufs since this
                                 is required by the \c %NvBuffer constructor. */

    enum v4l2_memory memory_type; /**< Specifies the V4l2 memory type of the buffers. */

    uint32_t num_queued_buffers;  /**< Holds the number of buffers currently queued on the
                                       plane. */
    uint32_t total_queued_buffers;  /**< Holds the total number of buffers queued on the
                                         plane. */
    uint32_t total_dequeued_buffers;  /**< Holds the total number of buffers dequeued from
                                           the plane. */

    bool streamon;  /**< Specifies whether the plane is streaming. */

    bool dqthread_running;  /**< Specifies whether DQ Thread is running.
                                 Its value is toggled by the DQ Thread. */
    bool stop_dqthread; /**< Specifies the value used to signal the DQ Thread to stop. */

    pthread_t dq_thread; /**< Speciifes the pthread ID of the DQ Thread. */

    dqThreadCallback callback; /**< Specifies the callback method used by the DQ Thread. */

    void *dqThread_data;    /**< Application supplied pointer provided as an
                                argument in #dqThreadCallback. */

    /**
     * The DQ thread method.
     *
     * This method runs indefinitely until it is signaled to stop
     * by #stopDQThread or the #dqThreadCallback method returns FALSE.
     * It keeps on trying to dequeue a buffer from the plane and calls the
     * #dqThreadCallback method on successful dequeue.
     *
     * @param[in] v4l2_element_plane A pointer to the NvV4l2ElementPlane object
     *                 for which the thread started.
     */
    static void *dqThread(void *v4l2_element_plane);

    NvElementProfiler &v4l2elem_profiler; /**< A reference to the profiler belonging
                                            to the plane's parent element. */

    /**
     * Indicates whether the plane encountered an error during its operation.
     *
     * @return 0 if no error was encountered, a non-zero value if an
     *            error was encountered.
     */
    inline int isInError()
    {
        return is_in_error;
    }

    /**
     * Creates a new V4l2Element plane.
     *
     *
     * @param[in] buf_type Type of the stream.
     * @param[in] device_name A pointer to the name of the element the plane belongs to.
     * @param[in] fd A reference to the FD of the device opened using v4l2_open.
     * @param[in] blocking A flag that indicates whether the device has been opened with blocking mode.
     * @param[in] profiler The profiler.
     */
    NvV4l2ElementPlane(enum v4l2_buf_type buf_type, const char *device_name,
                     int &fd, bool blocking, NvElementProfiler &profiler);

    /**
     * Disallows copy constructor.
     */
    NvV4l2ElementPlane(const NvV4l2ElementPlane& that);
    /**
     * Disallows assignment.
     */
    void operator=(NvV4l2ElementPlane const&);

    /**
     * NvV4l2ElementPlane destructor.
     *
     * Calls #deinitPlane internally.
     */
     ~NvV4l2ElementPlane();

    int is_in_error;        /**< Indicates if an error was encountered during
                               the operation of the element. */
    const char *comp_name;  /**< Specifies the name of the component,
                               for debugging. */

    friend class NvV4l2Element;
};
/** @} */
#endif
