#ifndef _ENCODER_H_
#define _ENCODER_H_

#define SPEEDL_PLUSE   CTIM0_P34
#define SPEEDR_PLUSE   CTIM3_P04
//定义方向引脚
#define SPEEDL_DIR     P35
#define SPEEDR_DIR     P53

extern int encoder_L;
extern int encoder_R;
extern float encoder_integral;
extern float encoder;
extern float encoder_integra2;

void EncoderCount(void);//编码器计数函数

#endif