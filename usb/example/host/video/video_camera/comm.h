#ifndef __COMM_H__
#define __COMM_H__

#pragma pack(1)
typedef struct
{
	uint8_t  flag;
	uint16_t len;
	uint8_t  type;
	uint16_t frame;
	uint8_t  rawdata[256];
} video_data_t;
#pragma pack()

#define MAX_VIDEO_DATA_BUF		  2
extern video_data_t g_video_data_pool[];
extern uint8_t g_video_data_tx_index;
extern uint8_t g_video_data_rx_index;

void comm_init(void);

#endif