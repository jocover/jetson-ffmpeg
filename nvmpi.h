#ifndef __NVMPI_H__
#define __NVMPI_H__
#include <stdlib.h>
#include <stdbool.h>

typedef struct nvmpictx nvmpictx;

typedef enum {
	NV_PIX_NV12,
	NV_PIX_YUV420
}nvPixFormat;

typedef struct _NVENCPARAM{
	unsigned int width;
	unsigned int height;
	unsigned int profile;
	unsigned int level;
	unsigned int bitrate;
	unsigned int peak_bitrate;
	char enableLossless;
	char mode_vbr;
	char insert_spspps_idr;
	unsigned int iframe_interval;
	unsigned int idr_interval;
	unsigned int fps_n;
	unsigned int fps_d;
	int capture_num;
	unsigned int max_b_frames;
	unsigned int refs;
	unsigned int qmax;
	unsigned int qmin;
	unsigned int hw_preset_type;
	
} nvEncParam;

typedef struct _NVPACKET{
	unsigned long flags;
	unsigned long payload_size;
	unsigned char *payload;
	unsigned long  pts;
} nvPacket;

typedef struct _NVFRAME{
	unsigned long flags;
	unsigned long payload_size[3];
	unsigned char *payload[3];
	unsigned int linesize[3];
	nvPixFormat type;
	unsigned int width;
	unsigned int height;
	time_t timestamp;
}nvFrame;



typedef enum {
	NV_VIDEO_CodingUnused,
	NV_VIDEO_CodingH264,             /**< H.264 */
	NV_VIDEO_CodingMPEG4,              /**< MPEG-4 */
	NV_VIDEO_CodingMPEG2,              /**< MPEG-2 */
	NV_VIDEO_CodingVP8,                /**< VP8 */
	NV_VIDEO_CodingVP9,                /**< VP9 */
	NV_VIDEO_CodingHEVC,               /**< H.265/HEVC */
} nvCodingType;

#ifdef __cplusplus
extern "C" {
#endif

	nvmpictx* nvmpi_create_decoder(nvCodingType codingType,nvPixFormat pixFormat);

	int nvmpi_decoder_put_packet(nvmpictx* ctx,nvPacket* packet);

	int nvmpi_decoder_get_frame(nvmpictx* ctx,nvFrame* frame,bool wait);

	int nvmpi_decoder_close(nvmpictx* ctx);

	nvmpictx* nvmpi_create_encoder(nvCodingType codingType,nvEncParam * param);
		
	int nvmpi_encoder_put_frame(nvmpictx* ctx,nvFrame* frame);

	int nvmpi_encoder_get_packet(nvmpictx* ctx,nvPacket* packet);

	int nvmpi_encoder_close(nvmpictx* ctx);

#ifdef __cplusplus
}
#endif

#endif /*__NVMPI_H__*/
