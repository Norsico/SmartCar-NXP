#ifndef __ELEMENT_H_
#define __ELEMENT_H_

#define No_Ring            0			//没环
#define Find_Ring    			 1			//找到环
#define Ready_In_Ring    	 2			//找到环
#define In_Ring      			 3			//在环里
#define Ready_Out_Ring     4			//准备
#define Out_Ring     	 	   5			//出环
#define Ready_No_Ring      6

extern uint8 Ring_big;
extern uint8 Ring_small;

extern int8 Ring_num;
extern int8 Ring_flag[4];
extern int8 Ring_open_flag[4];
extern int8 Ring_stop;
void Ring_all();

extern uint8 Ring_state;
extern uint16 encoder_Rin;
extern uint16 Angle_Ring;

extern uint8 Ring_left;
extern uint8 Ring_right;
void Ring();

//ui
extern uint8 Find_Ring_qulv;
extern uint8 Fing_Ring_sideline;

extern uint8 Ready_In_Ring_encoder;
extern uint8 Ready_In_Ring_sideline;

extern uint8 In_Ring_encoder;
extern int16 Ready_Out_Ring_angle;
extern int16 Out_Ring_angle;
extern uint8 No_Ring_encoder;

extern uint8 Ramp_flag;
extern uint8 Ramp_angle;
extern uint16 encoder_ramp;
void Ramp();

//ui
extern uint8 Find_Ramp_dis;
extern uint8 Find_Ramp_width1;
extern uint8 Find_Ramp_width2;
extern uint8 Find_Ramp_qulv;
extern uint8 Up_Ramp_encoder;
extern uint8 Out_Ramp_encoder;


extern uint8 Cross_flag;
extern uint16 encoder_cross;
void Cross();

//ui
extern uint8 Cross_sideline;
extern uint8 Cross_encoder;

extern uint8 zebra_1;
extern uint8 Zebra_flag;
extern uint16 Zebra_encoder;
void Zebra();

extern uint8 Out_Zebra_encoder;

void element();

#endif