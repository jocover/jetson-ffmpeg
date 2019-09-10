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

#include "NvBuffer.h"
#include "NvLogging.h"

#include <cstring>
#include <errno.h>
#include <sys/mman.h>
#include <libv4l2.h>

#define CAT_NAME "Buffer"

#define MAX(a,b) (a > b ? a : b)

NvBuffer::NvBuffer(enum v4l2_buf_type buf_type, enum v4l2_memory memory_type,
        uint32_t n_planes, NvBufferPlaneFormat * fmt, uint32_t index)
        :buf_type(buf_type),
         memory_type(memory_type),
         index(index),
         n_planes(n_planes)
{
    uint32_t i;

    mapped = false;
    allocated = false;

    memset(planes, 0, sizeof(planes));
    for (i = 0; i < n_planes; i++)
    {
        this->planes[i].fd = -1;
        this->planes[i].fmt = fmt[i];
    }

    ref_count = 0;
    pthread_mutex_init(&ref_lock, NULL);
    shared_buffer = NULL;
}

NvBuffer::NvBuffer(uint32_t pixfmt, uint32_t width, uint32_t height,
        uint32_t index)
        :buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE),
         memory_type(V4L2_MEMORY_USERPTR),
         index(index)
{
    uint32_t i;
    NvBuffer::NvBufferPlaneFormat fmt[MAX_PLANES];

    mapped = false;
    allocated = false;

    fill_buffer_plane_format(&n_planes, fmt, width, height, pixfmt);

    for (i = 0; i < MAX_PLANES; i++)
    {
        this->planes[i].fd = -1;
        this->planes[i].data = NULL;
        this->planes[i].bytesused = 0;
        this->planes[i].mem_offset = 0;
        this->planes[i].length = 0;
        this->planes[i].fmt = fmt[i];
        this->planes[i].fmt.sizeimage = fmt[i].width * fmt[i].height *
                                        fmt[i].bytesperpixel;
        this->planes[i].fmt.stride = fmt[i].width * fmt[i].bytesperpixel;
    }

    ref_count = 0;
    pthread_mutex_init(&ref_lock, NULL);
    shared_buffer = NULL;
}

NvBuffer::NvBuffer(uint32_t size, uint32_t index)
        :buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE),
         memory_type(V4L2_MEMORY_USERPTR),
         index(index)
{
    uint32_t i;

    mapped = false;
    allocated = false;

    n_planes = 1;
    for (i = 0; i < n_planes; i++)
    {
        this->planes[i].fd = -1;
        this->planes[i].data = NULL;
        this->planes[i].bytesused = 0;
        this->planes[i].mem_offset = 0;
        this->planes[i].length = 0;
        this->planes[i].fmt.sizeimage = size;
    }

    ref_count = 0;
    pthread_mutex_init(&ref_lock, NULL);
    shared_buffer = NULL;
}

NvBuffer::~NvBuffer()
{
    if (mapped)
    {
        unmap();
    }
    if (allocated)
    {
        deallocateMemory();
    }

    pthread_mutex_destroy(&ref_lock);
}

int
NvBuffer::map()
{
    uint32_t j;

    if (memory_type != V4L2_MEMORY_MMAP)
    {
        CAT_WARN_MSG("Buffer " << index << "already mapped");
        return -1;
    }

    if (mapped)
    {
        CAT_WARN_MSG("Buffer " << index << "already mapped");
        return 0;
    }

    for (j = 0; j < n_planes; j++)
    {
        if (planes[j].fd == -1)
        {
            return -1;
        }

        planes[j].data = (unsigned char *) mmap(NULL,
                                                planes[j].length,
                                                PROT_READ | PROT_WRITE,
                                                MAP_SHARED,
                                                planes[j].fd,
                                                planes[j].mem_offset);
        if (planes[j].data == MAP_FAILED)
        {
            CAT_ERROR_MSG("Could not map buffer " << index << ", plane " << j);
            return -1;
        }
        else
        {
            CAT_DEBUG_MSG("Mapped buffer " << index << ", plane " << j << " to "
                    << planes[j].data);
        }
    }
    mapped = true;
    return 0;
}

void
NvBuffer::unmap()
{
    if (memory_type != V4L2_MEMORY_MMAP || !mapped)
    {
        CAT_WARN_MSG("Cannot Unmap Buffer " << index <<
                ". Only mapped MMAP buffer can be unmapped");
        return;
    }

    for (uint32_t j = 0; j < n_planes; j++)
    {
        if (planes[j].data)
        {
            munmap(planes[j].data, planes[j].length);
        }
        planes[j].data = NULL;
    }
    mapped = false;
    CAT_DEBUG_MSG("Buffer " << index << " unmapped ");
}

int
NvBuffer::allocateMemory()
{
    uint32_t j;

    if (memory_type != V4L2_MEMORY_USERPTR)
    {
        CAT_ERROR_MSG("Only USERPTR buffers can be allocated");
        return -1;
    }

    if (allocated)
    {
        CAT_WARN_MSG("Buffer " << index << "already allocated memory");
        return 0;
    }

    for (j = 0; j < n_planes; j++)
    {
        if (planes[j].data)
        {
            ERROR_MSG("Buffer " << index << ", Plane " << j <<
                            " already allocated");
            return -1;
        }

        planes[j].length = MAX(planes[j].fmt.sizeimage,
                               planes[j].fmt.width *
                               planes[j].fmt.bytesperpixel *
                               planes[j].fmt.height);
        planes[j].data = new unsigned char [planes[j].length];

        if (planes[j].data == MAP_FAILED)
        {
            SYS_ERROR_MSG("Error while allocating buffer " << index <<
                    " plane " << j);
            return -1;
        }
        else
        {
            DEBUG_MSG("Buffer " << index << ", Plane " << j <<
                    " allocated to " << (void *) planes[j].data);
        }
    }
    allocated = true;
    return 0;
}

void
NvBuffer::deallocateMemory()
{
    uint32_t j;

    if (memory_type != V4L2_MEMORY_USERPTR || !allocated)
    {
        ERROR_MSG("Only allocated USERPTR buffers can be deallocated");
        return;
    }

    for (j = 0; j < n_planes; j++)
    {
        if (!planes[j].data)
        {
            DEBUG_MSG("Buffer " << index << ", Plane " << j <<
                    " not allocated");
            continue;
        }
        delete[] planes[j].data;
        planes[j].data = NULL;
    }
    allocated = false;
    DEBUG_MSG("Buffer " << index << " deallocated");
}

int
NvBuffer::ref()
{
    int ref_count;

    pthread_mutex_lock(&ref_lock);
    ref_count = ++this->ref_count;
    pthread_mutex_unlock(&ref_lock);
    return ref_count;
}

int
NvBuffer::unref()
{
    int ref_count;

    pthread_mutex_lock(&ref_lock);
    if (this->ref_count > 0)
    {
        --this->ref_count;
    }
    ref_count = this->ref_count;
    pthread_mutex_unlock(&ref_lock);
    return ref_count;
}

int
NvBuffer::fill_buffer_plane_format(uint32_t *num_planes,
        NvBuffer::NvBufferPlaneFormat *planefmts,
        uint32_t width, uint32_t height, uint32_t raw_pixfmt)
{
    switch (raw_pixfmt)
    {
        case V4L2_PIX_FMT_YUV444M:
            *num_planes = 3;

            planefmts[0].width = width;
            planefmts[1].width = width;
            planefmts[2].width = width;

            planefmts[0].height = height;
            planefmts[1].height = height;
            planefmts[2].height = height;

            planefmts[0].bytesperpixel = 1;
            planefmts[1].bytesperpixel = 1;
            planefmts[2].bytesperpixel = 1;
            break;
        case V4L2_PIX_FMT_YUV422M:
            *num_planes = 3;

            planefmts[0].width = width;
            planefmts[1].width = width / 2;
            planefmts[2].width = width / 2;

            planefmts[0].height = height;
            planefmts[1].height = height;
            planefmts[2].height = height;

            planefmts[0].bytesperpixel = 1;
            planefmts[1].bytesperpixel = 1;
            planefmts[2].bytesperpixel = 1;
            break;
        case V4L2_PIX_FMT_YUV422RM:
            *num_planes = 3;

            planefmts[0].width = width;
            planefmts[1].width = width;
            planefmts[2].width = width;

            planefmts[0].height = height;
            planefmts[1].height = height / 2;
            planefmts[2].height = height / 2;

            planefmts[0].bytesperpixel = 1;
            planefmts[1].bytesperpixel = 1;
            planefmts[2].bytesperpixel = 1;
            break;
        case V4L2_PIX_FMT_YUV420M:
        case V4L2_PIX_FMT_YVU420M:
            *num_planes = 3;

            planefmts[0].width = width;
            planefmts[1].width = width / 2;
            planefmts[2].width = width / 2;

            planefmts[0].height = height;
            planefmts[1].height = height / 2;
            planefmts[2].height = height / 2;

            planefmts[0].bytesperpixel = 1;
            planefmts[1].bytesperpixel = 1;
            planefmts[2].bytesperpixel = 1;
            break;
        case V4L2_PIX_FMT_NV12M:
            *num_planes = 2;

            planefmts[0].width = width;
            planefmts[1].width = width / 2;

            planefmts[0].height = height;
            planefmts[1].height = height / 2;

            planefmts[0].bytesperpixel = 1;
            planefmts[1].bytesperpixel = 2;
            break;
        case V4L2_PIX_FMT_GREY:
            *num_planes = 1;

            planefmts[0].width = width;

            planefmts[0].height = height;

            planefmts[0].bytesperpixel = 1;
            break;
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_YVYU:
        case V4L2_PIX_FMT_UYVY:
        case V4L2_PIX_FMT_VYUY:
            *num_planes = 1;

            planefmts[0].width = width;

            planefmts[0].height = height;

            planefmts[0].bytesperpixel = 2;
            break;
        case V4L2_PIX_FMT_ABGR32:
        case V4L2_PIX_FMT_XRGB32:
            *num_planes = 1;

            planefmts[0].width = width;

            planefmts[0].height = height;

            planefmts[0].bytesperpixel = 4;
            break;
        case V4L2_PIX_FMT_P010M:
            *num_planes = 2;

            planefmts[0].width = width;
            planefmts[1].width = width / 2;

            planefmts[0].height = height;
            planefmts[1].height = height / 2;

            planefmts[0].bytesperpixel = 2;
            planefmts[1].bytesperpixel = 4;
            break;
        default:
            ERROR_MSG("Unsupported pixel format " << raw_pixfmt);
            return -1;
    }
    return 0;
}
