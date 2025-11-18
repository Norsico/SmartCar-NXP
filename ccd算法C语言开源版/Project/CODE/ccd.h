#ifndef __CCD_H_
#define __CCD_H_

#define CCD1_set_width 38 //近端
#define CCD2_set_width 20 //远端

extern int i;
extern uint8 Threshold_multiple_1,Threshold_multiple_2;
extern uint8 Threshold_1,Threshold_2;
typedef struct
{
	uint16 max;
	uint16 min;
	uint32 threshold;
	uint16 aver;
	uint16 bin_thrd;
}CCDInformation;//CCD原始图像信息

extern CCDInformation CCD1;//CCD1原始图像相关数据
extern CCDInformation CCD2;//CCD2原始图像相关数据
typedef struct
{
	//CCD1原图像
	uint8 left_sideline1;
	uint8 right_sideline1;
	
	float middle_sideline1;
	float middle_sideline1_last;
	uint8 Width1;
	
	//CCD2原图像
	uint8 left_sideline2;
	
	uint8 right_sideline2;
	
	float middle_sideline2;
	float middle_sideline2_last;
	uint8 Width2;
	
	float right_qulu;
	float left_qulu;
}TrackInformation;

extern TrackInformation Trk;//赛道信息

extern uint8 Straight;
extern uint8 curve;

extern uint8 black_write_1;
extern uint8 black_write_2;

extern uint8 CCD1_left_flag;
extern uint8 CCD1_right_flag;
extern uint8 CCD2_left_flag;
extern uint8 CCD2_right_flag;

void CCDimage_init();
void CCD1_get();
void CCD2_get();
void left_right_sideline();
void middle_sideline();
void CCD_Per();

void CCD_Processing();

#endif