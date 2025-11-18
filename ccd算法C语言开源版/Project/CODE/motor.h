#ifndef _MOTOR_H_
#define _MOTOR_H_

#define PWM_1 PWMA_CH2N_P13
#define PWM_2 PWMA_CH4P_P26
#define DIR_1 P10
#define DIR_2 P24

typedef struct
{
	float speed;
	float speed_L;
	float speed_R;
	
  float L_Ki;    
	float L_Kp;  
	float R_Ki;  
	float R_Kp;

	float Dif_P;
	float Dif_Speed;  
}MOTOR_PID;

extern float Desire_Speed;
extern int16 Motor_Pid_speed_Z; //ui调节的速
extern int16 Motor_Pid_Z_L_Ki;
extern int16 Motor_Pid_Z_L_Kp;
extern int16 Motor_Pid_Z_R_Ki;
extern int16 Motor_Pid_Z_R_Kp;
extern int16 Motor_Pid_Dif_P;
extern MOTOR_PID Motor_Pid;

extern int16 Linear_speed;//直道速度
extern int16 Curve_speed; //弯道速度

extern int16 Up_Ramp_speed;//上坡速度
extern int16 Out_Ramp_speed;//下坡速度

//环岛速度
extern uint8 Ring_Speed;

extern int16 Ready_In_50_Ring_speed;
extern int16 In_50_Ring_speed;
extern int16 Ready_Out_50_Ring_speed;

extern int16 Ready_In_60_Ring_speed;
extern int16 In_60_Ring_speed;
extern int16 Ready_Out_60_Ring_speed;

extern int16 Ready_In_90_Ring_speed;
extern int16 In_90_Ring_speed;
extern int16 Ready_Out_90_Ring_speed;
/***************Motor_Control************/
extern uint8 stop_flag;
extern uint8 block_time; //堵转时间记录
extern uint8 stop_time;

/***************Motor_Control************/
extern uint8 start1;
extern int16 PWM_L,PWM_R;

/*************Incremental_PI************/
extern int16 Motor_error_L,Motor_last_error_L;
extern int16 Motor_error_R,Motor_last_error_R;

void Motor_Pid_init();
void Motor_PID_subsection();
void Motor_Control();
void Motor_Control_L(int Example_L);
void Motor_Control_R(int Example_R);
void Incremental_PI_L (int Encoder1,int Target_L);
void Incremental_PI_R (int Encoder2,int Target_R);

#endif