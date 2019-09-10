/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer.
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
 * <b>NVIDIA Multimedia API: Buffer API</b>
 *
 * @b Description: This file declares the NvBuffer APIs.
 */

#ifndef __NV_BUFFER_H__
#define __NV_BUFFER_H__

#include <linux/videodev2.h>
#include <pthread.h>
#include <stdint.h>

#include "v4l2_nv_extensions.h"

/**
 *
 * @defgroup l4t_mm_nvbuffer_group Buffer API
 *
 * The @c %NvBuffer API provides buffer functionality, including reference
 * count functionality and convenience methods.
 * @ingroup aa_framework_api_group
 * @{
 */

/**
 * Specifies the maximum number of planes a buffer can contain.
 */
#define MAX_PLANES 3

/**
 * @brief Class representing a buffer.
 *
 * The NvBuffer class is modeled on the basis of the @c v4l2_buffer
 * structure. The buffer has @c buf_type @c v4l2_buf_type, @c
 * memory_type @c v4l2_memory, and an index. It contains an
 * NvBufferPlane array similar to the array of @c v4l2_plane
 * structures in @c v4l2_buffer.m.planes. It also contains a
 * corresponding NvBufferPlaneFormat array that describes the
 * format of each of the planes.
 *
 * Even though @c %NvBuffer closely resembles v4l2 structures, it can
 * be easily used with other non-v4l2 components. @c %NvBuffer
 * contains data pointers, buffer length, file descriptor (FD) of
 * buffer planes, buffer format (height, width, stride, etc.), and
 * other members that are required by such components.
 *
 * This class also provides buffer reference count functionality. This
 * is useful when the same buffer is being used by multiple elements.
 *
 * In the case of a V4L2 MMAP, this class provides convenience methods
 * for mapping or unmapping the contents of the buffer to or from
 * memory, allocating or deallocating software memory depending on its
 * format.
 */
class NvBuffer
{
public:
    /**
     * Holds the buffer plane format.
     */
    typedef struct
    {
        uint32_t width;             /**< Holds the width of the plane in pixels. */
        uint32_t height;            /**< Holds the height of the plane in pixels. */

        uint32_t bytesperpixel;     /**< Holds the bytes used to represent one
                                      pixel in the plane. */
        uint32_t stride;            /**< Holds the stride of the plane in bytes. */
        uint32_t sizeimage;         /**< Holds the size of the plane in bytes. */
    } NvBufferPlaneFormat;

    /**
     * Holds the buffer plane parameters.
     */
    typedef struct
    {
        NvBufferPlaneFormat fmt;    /**< Holds the format of the plane. */

        unsigned char *data;        /**< Holds a pointer to the plane memory. */
        uint32_t bytesused;         /**< Holds the number of valid bytes in the plane. */

        int fd;                     /**< Holds the file descriptor (FD) of the plane of the
                                      exported buffer, in the case of V4L2 MMAP buffers. */
        uint32_t mem_offset;        /**< Holds the offset of the first valid byte
                                      from the data pointer. */
        uint32_t length;            /**< Holds the size of the buffer in bytes. */
    } NvBufferPlane;

    /**
     * Creates a new NvBuffer object.
     *
     * This convenience method for V4L2 elements creates a new buffer
     * with the planes array memset to zero and the refcount
     * initialized to zero.
     *

     * @param[in] buf_type Type of buffer, enumerated as @c
     *                     v4l2_buf_type.
     * @param[in] memory_type @c %NvBuffer memory, enumerated as an
     *                        @c v4l2_memory enum.
     * @param[in] n_planes Number of planes in the buffer.
     * @param[in] fmt Specifies a pointer to the array of buffer plane formats.
     *      Should contain at least @a n_planes elements.
     * @param[in] index Index of the buffer in the plane.
     */
    NvBuffer(enum v4l2_buf_type buf_type, enum v4l2_memory memory_type,
           uint32_t n_planes, NvBufferPlaneFormat *fmt, uint32_t index);

    /**
     * Creates a new NvBuffer for raw pixel formats.
     *
     * This convenience method for V4L2 elements is an @c %NvBuffer
     * constructor for raw pixel formats only. It requires width,
     * height, and pixel format to be specified.
     *
     * The planes array is memset to zero and the refcount is
     * initialized to zero.
     *
     * @attention The memory must be allocated by the application
     * by calling NvBuffer::allocateMemory.
     *
     * @param[in] pixfmt Pixel format of the buffer.
     * @param[in] width Width of the buffer in pixels.
     * @param[in] height Height of the buffer in pixels.
     * @param[in] index Index/ID of the buffer.
     */
    NvBuffer(uint32_t pixfmt, uint32_t width, uint32_t height, uint32_t index);

    /**
     * Creates a new NvBuffer object for non-raw pixel formats.
     *
     * This convenience method for V4L2 elements is an @c %NvBuffer
     * constructor for non raw pixel formats. It requires size of the
     * buffer to be supplied.
     *
     * The planes array is memset to zero and refcount initialized to
     * zero.
     *
     * @attention The memory needs to be allocated by the application
     * by calling NvBuffer::allocateMemory.
     *
     * @param[in] size Size of the buffer in bytes.
     * @param[in] index Index/ID of the buffer.
     */
    NvBuffer(uint32_t size, uint32_t index);

    /**
     * Destroys an NvBuffer object.
     *
     * This method cleans up class instances, unmapping any mapped
     * planes.
     */
    ~NvBuffer();

    /**
     * Maps the contents of the buffer to memory.
     *
     * This method maps the file descriptor (FD) of the planes to
     * a data pointer of @c planes. (MMAP buffers only.)
     *
     * @return 0 on success, -1 otherwise.
     */
    int map();
    /**
     * Unmaps the contents of the buffer from memory. (MMAP buffers only.)
     *
     */
    void unmap();

    /**
     * Allocates software memory for the buffer.
     *
     * @warning This method works only for @c V4L2_MEMORY_USERPTR memory.
     *
     * This method allocates memory on the basis of the buffer format:
     * @a height, @a width, @a bytesperpixel, and @a sizeimage.
     *
     * @return 0 for success, -1 otherwise.
     */
    int allocateMemory();
    /**
     * Deallocates buffer memory.
     *
     * @warning This method works only for @c V4L2_MEMORY_USERPTR memory and if
     *          the memory was previously allocated using NvBuffer::allocateMemory.
     */
    void deallocateMemory();

    /**
     * Increases the reference count of the buffer.
     *
     * This method is thread safe.
     *
     * @return Reference count of the buffer after the operation.
     */
    int ref();
    /**
     * Decreases the reference count of the buffer.
     *
     * This thread-safe method decreases the buffer reference count if the
     * buffer reference count is above 0.
     *
     * @return Reference count of the buffer after the operation.
     */
    int unref();

    const enum v4l2_buf_type buf_type;  /**< Type of the buffer. */
    const enum v4l2_memory memory_type; /**< Type of memory associated
                                           with the buffer. */

    const uint32_t index;               /**< Holds the buffer index. */

    uint32_t n_planes;            /**< Holds the number of planes in the buffer. */
    NvBufferPlane planes[MAX_PLANES];     /**< Holds the data pointer, plane file
                                             descriptor (FD), plane format, etc. */

    /**
     * Fills the NvBuffer::NvBufferPlaneFormat array.
     *
     * This convenience method populates the
     * @c %NvBuffer::NvBufferPlaneFormat array on the basis of @a width,
     * @a height and pixel format (@a raw_pixfmt). It also returns the number of planes
     * required for the pixel format in @a num_planes.
     *
     *
     * @param[out] num_planes The number of planes. Must not be NULL.
     * @param[in,out] planefmts Array of %NvBuffer::NvBufferPlaneFormat to
     *                fill. Must be at least \a num_planes in length. For best
     *                results, pass an array of length #MAX_PLANES.
     * @param[in] width Width of the buffer in pixels.
     * @param[in] height Height of the buffer in pixels.
     * @param[in] raw_pixfmt Raw V4L2 pixel formats.
     * @return 0 for success, -1 for an unsupported pixel format.
     */
    static int fill_buffer_plane_format(uint32_t *num_planes,
            NvBuffer::NvBufferPlaneFormat *planefmts,
            uint32_t width, uint32_t height, uint32_t raw_pixfmt);
private:
    uint32_t ref_count;             /**< Holds the reference count of the buffer. */
    pthread_mutex_t ref_lock;       /**< Mutex to synchronize increment/
                                         decrement operations of @c ref_count. */

    bool mapped;                    /**< Indicates if the buffer is mapped to
                                         memory. */
    bool allocated;                 /**< Indicates if the buffer is allocated
                                         memory. */
    NvBuffer *shared_buffer; /**< If this is a DMABUF buffer, @c shared_buffer
                                points to the MMAP @c NvBuffer whose FD was
                                sent when this buffer was queued. */

    /**
     * Disallows copy constructor.
     */
    NvBuffer(const NvBuffer& that);
    /**
     * Disallows assignment.
     */
    void operator=(NvBuffer const&);

    friend class NvV4l2ElementPlane;
};
/** @} */
#endif
