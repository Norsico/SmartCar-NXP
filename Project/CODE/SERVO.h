#ifndef _SERVO_H_
#define _SERVO_H_

extern int16 SERVO_duty;
extern int8 error,error_last,error_2;

extern int16 Pid_Kp;
extern int16 Pid_Kd;

extern uint8 Pid_Ring_flag;

extern int16 Pid_Ring_50_P;
extern int16 Pid_Ring_50_D;

extern int16 Pid_Ring_60_P;
extern int16 Pid_Ring_60_D;

extern int16 Pid_Ring_90_P;
extern int16 Pid_Ring_90_D;
typedef struct
{
	float z_p;
	float z_d;
	//»·µºpd
	float R_50_P;
	float R_50_D;
	
	float R_60_P;
	float R_60_D;
	
	float R_90_P;
	float R_90_D;
}PID;
extern PID Pid;

void PID_init();
void servo_init();
void PID_subsection();
void servo_ccd();
void servo_Control();
#endif