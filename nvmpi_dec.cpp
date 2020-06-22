
#include "nvmpi.h"
#include "NvVideoDecoder.h"
#include "nvbuf_utils.h"
#include <vector>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <queue>
#include <mutex>
#include <condition_variable>

#define CHUNK_SIZE 4000000
#define MAX_BUFFERS 32

#define TEST_ERROR(condition, message, errorCode)    \
	if (condition)                               \
{                                                    \
	std::cout<< message;			     \
}

using namespace std;

struct nvmpictx
{
	NvVideoDecoder *dec{nullptr};
	bool eos{false};
	bool got_res_event{false};
	int index{0};
	unsigned int coded_width{0};
	unsigned int coded_height{0};
	int dst_dma_fd{0};
	int numberCaptureBuffers{0};
	int dmaBufferFileDescriptor[MAX_BUFFERS];
	nvPixFormat out_pixfmt;
	unsigned int decoder_pixfmt{0};
	std::thread * dec_capture_loop{nullptr};
	std::mutex* mutex{nullptr};
	std::condition_variable* has_frame_cv{nullptr};
	std::queue<int> * frame_pools{nullptr};
	unsigned char * bufptr_0[MAX_BUFFERS];
	unsigned char * bufptr_1[MAX_BUFFERS];
	unsigned char * bufptr_2[MAX_BUFFERS];
	unsigned int frame_size[MAX_NUM_PLANES];
	unsigned int frame_linesize[MAX_NUM_PLANES];
	unsigned long long timestamp[MAX_BUFFERS];
};

void respondToResolutionEvent(v4l2_format &format, v4l2_crop &crop,nvmpictx* ctx){
	
	int32_t minimumDecoderCaptureBuffers;
	int ret=0;
	NvBufferCreateParams input_params = {0};
	NvBufferCreateParams cParams = {0};

	ret = ctx->dec->capture_plane.getFormat(format);	
	TEST_ERROR(ret < 0, "Error: Could not get format from decoder capture plane", ret);

	ret = ctx->dec->capture_plane.getCrop(crop);
	TEST_ERROR(ret < 0, "Error: Could not get crop from decoder capture plane", ret);

	ctx->coded_width=crop.c.width;
	ctx->coded_height=crop.c.height;

	if(ctx->dst_dma_fd != -1)
	{
		NvBufferDestroy(ctx->dst_dma_fd);
		ctx->dst_dma_fd = -1;
	}

	input_params.payloadType = NvBufferPayload_SurfArray;
	input_params.width = crop.c.width;
	input_params.height = crop.c.height;
	input_params.layout = NvBufferLayout_Pitch;
	input_params.colorFormat = ctx->out_pixfmt==NV_PIX_NV12?NvBufferColorFormat_NV12: NvBufferColorFormat_YUV420;
	input_params.nvbuf_tag = NvBufferTag_VIDEO_DEC;

	ctx->dec->capture_plane.deinitPlane();

	for (int index = 0; index < ctx->numberCaptureBuffers; index++)
	{
		if (ctx->dmaBufferFileDescriptor[index] != 0)
		{	
			ret = NvBufferDestroy(ctx->dmaBufferFileDescriptor[index]);
			TEST_ERROR(ret < 0, "Failed to Destroy NvBuffer", ret);
		}

	}


	ret=ctx->dec->setCapturePlaneFormat(format.fmt.pix_mp.pixelformat,format.fmt.pix_mp.width,format.fmt.pix_mp.height);
	TEST_ERROR(ret < 0, "Error in setting decoder capture plane format", ret);

	ctx->dec->getMinimumCapturePlaneBuffers(minimumDecoderCaptureBuffers);
	TEST_ERROR(ret < 0, "Error while getting value of minimum capture plane buffers",ret);

	ctx->numberCaptureBuffers = minimumDecoderCaptureBuffers + 5;



	switch (format.fmt.pix_mp.colorspace)
	{
		case V4L2_COLORSPACE_SMPTE170M:
			if (format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT)
			{
				// "Decoder colorspace ITU-R BT.601 with standard range luma (16-235)"
				cParams.colorFormat = NvBufferColorFormat_NV12;
			}
			else
			{
				//"Decoder colorspace ITU-R BT.601 with extended range luma (0-255)";
				cParams.colorFormat = NvBufferColorFormat_NV12_ER;
			}
			break;
		case V4L2_COLORSPACE_REC709:
			if (format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT)
			{
				//"Decoder colorspace ITU-R BT.709 with standard range luma (16-235)";
				cParams.colorFormat = NvBufferColorFormat_NV12_709;
			}
			else
			{
				//"Decoder colorspace ITU-R BT.709 with extended range luma (0-255)";
				cParams.colorFormat = NvBufferColorFormat_NV12_709_ER;
			}
			break;
		case V4L2_COLORSPACE_BT2020:
			{
				//"Decoder colorspace ITU-R BT.2020";
				cParams.colorFormat = NvBufferColorFormat_NV12_2020;
			}
			break;
		default:
			if (format.fmt.pix_mp.quantization == V4L2_QUANTIZATION_DEFAULT)
			{
				//"Decoder colorspace ITU-R BT.601 with standard range luma (16-235)";
				cParams.colorFormat = NvBufferColorFormat_NV12;
			}
			else
			{
				//"Decoder colorspace ITU-R BT.601 with extended range luma (0-255)";
				cParams.colorFormat = NvBufferColorFormat_NV12_ER;
			}
			break;
	}



	ret = NvBufferCreateEx (&ctx->dst_dma_fd, &input_params);
	TEST_ERROR(ret == -1, "create dst_dmabuf failed", error);

	for (int index = 0; index < ctx->numberCaptureBuffers; index++)
	{
		cParams.width = crop.c.width;
		cParams.height = crop.c.height;
		cParams.layout = NvBufferLayout_BlockLinear;
		cParams.payloadType = NvBufferPayload_SurfArray;
		cParams.nvbuf_tag = NvBufferTag_VIDEO_DEC;

		ret = NvBufferCreateEx(&ctx->dmaBufferFileDescriptor[index], &cParams);
		TEST_ERROR(ret < 0, "Failed to create buffers", ret);

	}	

	ctx->dec->capture_plane.reqbufs(V4L2_MEMORY_DMABUF, ctx->numberCaptureBuffers);
	TEST_ERROR(ret < 0, "Error in decoder capture plane streamon", ret);

	ctx->dec->capture_plane.setStreamStatus(true);
	TEST_ERROR(ret < 0, "Error in decoder capture plane streamon", ret);


	for (uint32_t i = 0; i < ctx->dec->capture_plane.getNumBuffers(); i++)
	{
		struct v4l2_buffer v4l2_buf;
		struct v4l2_plane planes[MAX_PLANES];

		memset(&v4l2_buf, 0, sizeof(v4l2_buf));
		memset(planes, 0, sizeof(planes));

		v4l2_buf.index = i;
		v4l2_buf.m.planes = planes;
		v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		v4l2_buf.memory = V4L2_MEMORY_DMABUF;
		v4l2_buf.m.planes[0].m.fd = ctx->dmaBufferFileDescriptor[i];

		ret = ctx->dec->capture_plane.qBuffer(v4l2_buf, NULL);
		TEST_ERROR(ret < 0, "Error Qing buffer at output plane", ret);
	}

	ctx->got_res_event = true;
}

void *dec_capture_loop_fcn(void *arg){
	nvmpictx* ctx=(nvmpictx*)arg;
	
	struct v4l2_format v4l2Format;
	struct v4l2_crop v4l2Crop;
	struct v4l2_event v4l2Event;
	int ret,buf_index=0;

	while (!(ctx->dec->isInError()||ctx->eos)){
		NvBuffer *dec_buffer;

		ret = ctx->dec->dqEvent(v4l2Event, ctx->got_res_event ? 0 : 500);
		if (ret == 0)
		{
			switch (v4l2Event.type)
			{
				case V4L2_EVENT_RESOLUTION_CHANGE:
					respondToResolutionEvent(v4l2Format, v4l2Crop,ctx);
					continue;
			}
		}	

		if (!ctx->got_res_event) {
			continue;
		}

		while(!ctx->eos){
			struct v4l2_buffer v4l2_buf;
			struct v4l2_plane planes[MAX_PLANES];
			v4l2_buf.m.planes = planes;

			if (ctx->dec->capture_plane.dqBuffer(v4l2_buf, &dec_buffer, NULL, 0)){
				if (errno == EAGAIN)
				{
					usleep(1000);
				}
				else
				{

					ERROR_MSG("Error while calling dequeue at capture plane");
					ctx->eos=true;
				}
				break;

			}

			dec_buffer->planes[0].fd = ctx->dmaBufferFileDescriptor[v4l2_buf.index];
			NvBufferRect src_rect, dest_rect;
			src_rect.top = 0;
			src_rect.left = 0;
			src_rect.width = ctx->coded_width;
			src_rect.height = ctx->coded_height;
			dest_rect.top = 0;
			dest_rect.left = 0;
			dest_rect.width = ctx->coded_width;
			dest_rect.height = ctx->coded_height;

			NvBufferTransformParams transform_params;
			memset(&transform_params,0,sizeof(transform_params));
			transform_params.transform_flag = NVBUFFER_TRANSFORM_FILTER;
			transform_params.transform_flip = NvBufferTransform_None;
			transform_params.transform_filter = NvBufferTransform_Filter_Smart;
			transform_params.src_rect = src_rect;
			transform_params.dst_rect = dest_rect;

			ctx->mutex->lock();

			if(!ctx->eos){

				ret = NvBufferTransform(dec_buffer->planes[0].fd, ctx->dst_dma_fd, &transform_params);
				TEST_ERROR(ret==-1, "Transform failed",ret);

				NvBufferParams parm;
				ret = NvBufferGetParams(ctx->dst_dma_fd, &parm);

				if(!ctx->frame_size[0]){

					for(int index=0;index<MAX_BUFFERS;index++){
						ctx->bufptr_0[index]=new unsigned char[parm.psize[0]];//Y
						ctx->bufptr_1[index]=new unsigned char[parm.psize[1]];//UV or UU
						ctx->bufptr_2[index]=new unsigned char[parm.psize[2]];//VV
					}
				}


				ctx->frame_linesize[0]=parm.width[0];
				ctx->frame_size[0]=parm.psize[0];

				ctx->frame_linesize[1]=parm.width[1];
				ctx->frame_size[1]=parm.psize[1];
				ctx->frame_linesize[2]=parm.width[2];
				ctx->frame_size[2]=parm.psize[2];


				ret=NvBuffer2Raw(ctx->dst_dma_fd,0,parm.width[0],parm.height[0],ctx->bufptr_0[buf_index]);
				ret=NvBuffer2Raw(ctx->dst_dma_fd,1,parm.width[1],parm.height[1],ctx->bufptr_1[buf_index]);	
				if(ctx->out_pixfmt==NV_PIX_YUV420)
					ret=NvBuffer2Raw(ctx->dst_dma_fd,2,parm.width[2],parm.height[2],ctx->bufptr_2[buf_index]);	

				ctx->frame_pools->push(buf_index);
				ctx->timestamp[buf_index]= (v4l2_buf.timestamp.tv_usec % 1000000) + (v4l2_buf.timestamp.tv_sec * 1000000UL);

				buf_index=(buf_index+1)%MAX_BUFFERS;

			}
			
			ctx->mutex->unlock();

			if (ctx->eos) {
				break;
			}

			ctx->has_frame_cv->notify_one();

			v4l2_buf.m.planes[0].m.fd = ctx->dmaBufferFileDescriptor[v4l2_buf.index];
			if (ctx->dec->capture_plane.qBuffer(v4l2_buf, NULL) < 0){
				ERROR_MSG("Error while queueing buffer at decoder capture plane");
			}
		}
	}

	// Wake all waiting threads at EOS or decoder error
	ctx->has_frame_cv->notify_all();
}

nvmpictx* nvmpi_create_decoder(nvCodingType codingType,nvPixFormat pixFormat){
	
	int ret;
	log_level = LOG_LEVEL_INFO;

	nvmpictx* ctx=new nvmpictx;

	ctx->dec = NvVideoDecoder::createVideoDecoder("dec0");
	TEST_ERROR(!ctx->dec, "Could not create decoder",ret);

	ret=ctx->dec->subscribeEvent(V4L2_EVENT_RESOLUTION_CHANGE, 0, 0);
	TEST_ERROR(ret < 0, "Could not subscribe to V4L2_EVENT_RESOLUTION_CHANGE", ret);

	switch(codingType){
		case NV_VIDEO_CodingH264:
			ctx->decoder_pixfmt=V4L2_PIX_FMT_H264;
			break;
		case NV_VIDEO_CodingHEVC:
			ctx->decoder_pixfmt=V4L2_PIX_FMT_H265;
			break;
		case NV_VIDEO_CodingMPEG4:
			ctx->decoder_pixfmt=V4L2_PIX_FMT_MPEG4;
			break;
		case NV_VIDEO_CodingMPEG2:
			ctx->decoder_pixfmt=V4L2_PIX_FMT_MPEG2;
			break;
		case NV_VIDEO_CodingVP8:
			ctx->decoder_pixfmt=V4L2_PIX_FMT_VP8;
			break;
		case NV_VIDEO_CodingVP9:
			ctx->decoder_pixfmt=V4L2_PIX_FMT_VP9;
			break;
		default:
			ctx->decoder_pixfmt=V4L2_PIX_FMT_H264;
			break;
	}

	ret=ctx->dec->setOutputPlaneFormat(ctx->decoder_pixfmt, CHUNK_SIZE);

	TEST_ERROR(ret < 0, "Could not set output plane format", ret);

	//ctx->nalu_parse_buffer = new char[CHUNK_SIZE];
	ret = ctx->dec->setFrameInputMode(0);
	TEST_ERROR(ret < 0, "Error in decoder setFrameInputMode for NALU", ret);

	ret = ctx->dec->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 10, false, true);
	TEST_ERROR(ret < 0, "Error while setting up output plane", ret);

	ctx->dec->output_plane.setStreamStatus(true);
	TEST_ERROR(ret < 0, "Error in output plane stream on", ret);

	ctx->out_pixfmt=pixFormat;
	ctx->dst_dma_fd=-1;
	ctx->eos=false;
	ctx->got_res_event=false;
	ctx->index=0;
	ctx->frame_size[0]=0;
	ctx->frame_pools=new std::queue<int>;
	ctx->mutex = new std::mutex();
	ctx->has_frame_cv = new std::condition_variable();
	for(int index=0;index<MAX_BUFFERS;index++)
		ctx->dmaBufferFileDescriptor[index]=0;
	for(int index=0;index<MAX_BUFFERS;index++){
		ctx->bufptr_0[index] = nullptr;
		ctx->bufptr_1[index] = nullptr;
		ctx->bufptr_2[index] = nullptr;
	}
	ctx->numberCaptureBuffers=0;
	ctx->dec_capture_loop=new thread(dec_capture_loop_fcn,ctx);

	return ctx;
}




int nvmpi_decoder_put_packet(nvmpictx* ctx,nvPacket* packet){
	int ret;
	
	struct v4l2_buffer v4l2_buf;
	struct v4l2_plane planes[MAX_PLANES];
	NvBuffer *nvBuffer;

	memset(&v4l2_buf, 0, sizeof(v4l2_buf));
	memset(planes, 0, sizeof(planes));

	v4l2_buf.m.planes = planes;

	if (ctx->index < (int)ctx->dec->output_plane.getNumBuffers()) {
		nvBuffer = ctx->dec->output_plane.getNthBuffer(ctx->index);
	} else {
		ret = ctx->dec->output_plane.dqBuffer(v4l2_buf, &nvBuffer, NULL, -1);
		if (ret < 0) {
			cout << "Error DQing buffer at output plane" << std::endl;
			return false;
		}
	}

	memcpy(nvBuffer->planes[0].data,packet->payload,packet->payload_size);
	nvBuffer->planes[0].bytesused=packet->payload_size;



	if (ctx->index < ctx->dec->output_plane.getNumBuffers())
	{
		v4l2_buf.index = ctx->index ;
		v4l2_buf.m.planes = planes;
	}

	v4l2_buf.m.planes[0].bytesused = nvBuffer->planes[0].bytesused;

	v4l2_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
	v4l2_buf.timestamp.tv_sec = packet->pts / 1000000;
	v4l2_buf.timestamp.tv_usec = packet->pts % 1000000;


	ret = ctx->dec->output_plane.qBuffer(v4l2_buf, NULL);
	if (ret < 0) {
		std::cout << "Error Qing buffer at output plane" << std::endl;
		return false;
	}

	if (ctx->index < ctx->dec->output_plane.getNumBuffers())
		ctx->index++;

	if (v4l2_buf.m.planes[0].bytesused == 0) {
		ctx->eos=true;
		std::cout << "Input file read complete" << std::endl;
	}


	return 0;

}


int nvmpi_decoder_get_frame(nvmpictx* ctx,nvFrame* frame,bool wait){
	
	int ret,picture_index;
	std::unique_lock<std::mutex> lock(*ctx->mutex);

	if (wait) {
		while (ctx->frame_pools->empty() && !ctx->eos && !ctx->dec->isInError()) {
			ctx->has_frame_cv->wait(lock);
		}
	}

	if (ctx->frame_pools->empty()) {
		return -1;
	}

	picture_index=ctx->frame_pools->front();
	ctx->frame_pools->pop();

	frame->width=ctx->coded_width;
	frame->height=ctx->coded_height;

	frame->linesize[0]=ctx->frame_linesize[0];
	frame->linesize[1]=ctx->frame_linesize[1];
	frame->linesize[2]=ctx->frame_linesize[2];

	frame->payload[0]=ctx->bufptr_0[picture_index];
	frame->payload[1]=ctx->bufptr_1[picture_index];
	frame->payload[2]=ctx->bufptr_2[picture_index];

	frame->payload_size[0]=ctx->frame_size[0];
	frame->payload_size[1]=ctx->frame_size[1];
	frame->payload_size[2]=ctx->frame_size[2];
	frame->timestamp=ctx->timestamp[picture_index];

	return 0;

}

int nvmpi_decoder_close(nvmpictx* ctx){

	ctx->mutex->lock();
	ctx->eos=true;
	ctx->mutex->unlock();
	
	ctx->dec->capture_plane.setStreamStatus(false);
	
	if (ctx->dec_capture_loop) {
		ctx->dec_capture_loop->join();
		delete ctx->dec_capture_loop;
		ctx->dec_capture_loop = nullptr;
	}

	if(ctx->dst_dma_fd != -1)
	{
		NvBufferDestroy(ctx->dst_dma_fd);
		ctx->dst_dma_fd = -1;
	}

	for (int index = 0; index < ctx->numberCaptureBuffers; index++)
	{
		if (ctx->dmaBufferFileDescriptor[index] != 0)
		{	
			int ret = NvBufferDestroy(ctx->dmaBufferFileDescriptor[index]);
			TEST_ERROR(ret < 0, "Failed to Destroy NvBuffer", ret);
		}

	}
	
	delete ctx->dec; ctx->dec = nullptr;

	for(int index=0;index<MAX_BUFFERS;index++){
		delete[] ctx->bufptr_0[index];
		delete[] ctx->bufptr_1[index];
		delete[] ctx->bufptr_2[index];
	}

	delete ctx->mutex; ctx->mutex = nullptr;
	delete ctx->has_frame_cv; ctx->has_frame_cv = nullptr;
	delete ctx->frame_pools; ctx->frame_pools = nullptr;

	delete ctx; ctx = nullptr;

	return 0;
}


