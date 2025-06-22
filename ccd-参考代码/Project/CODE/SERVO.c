#include "headfile.h"

int16 SERVO_duty=0;
int8 error=0,error_last=0,error_2=0;

int16 Pid_Kp = 0;
int16 Pid_Kd = 0;

PID Pid;
//环岛的pd与标志位
uint8 Pid_Ring_flag=0;

int16 Pid_Ring_50_P=0;
int16 Pid_Ring_50_D=0;

int16 Pid_Ring_60_P=0;
int16 Pid_Ring_60_D=0;

int16 Pid_Ring_90_P=0;
int16 Pid_Ring_90_D=0;

void servo_init()
{
	pwm_init(PWMB_CH3_P33,300,Servo_Center);
	pwm_duty(PWMB_CH3_P33,Servo_Center);
}
void PID_init()
{
	Pid.z_p =0.0;
	Pid.z_d =0.0;
	
	Pid.R_50_P =0.0;
	Pid.R_50_D =0.0;
	
	Pid.R_60_P =0.0;
	Pid.R_60_D =0.0;
	
	Pid.R_90_P =0.0;
	Pid.R_90_D =0.0;
}
void PID_subsection()
{
	if(Pid_Ring_flag==0)
	{
		Pid.z_p =0.01*Pid_Kp;
		Pid.z_d =0.01*Pid_Kd;
	}
	
	
	if(Pid_Ring_flag==1)
	{
		Pid.z_p =0.01*Pid_Ring_50_P;
		Pid.z_d =0.01*Pid_Ring_50_D;
	}
	if(Pid_Ring_flag==2)
	{
		Pid.z_p =0.01*Pid_Ring_60_P;
		Pid.z_d =0.01*Pid_Ring_60_D;
	}
	if(Pid_Ring_flag==3)
	{
		Pid.z_p =0.01*Pid_Ring_90_P;
		Pid.z_d =0.01*Pid_Ring_90_D;
	}
	//ips显示的参数
	Pid.R_50_P =0.01*Pid_Ring_50_P;
	Pid.R_50_D =0.01*Pid_Ring_50_D;
	
	Pid.R_60_P =0.01*Pid_Ring_60_P;
	Pid.R_60_D =0.01*Pid_Ring_60_D;
	
	Pid.R_90_P =0.01*Pid_Ring_90_P;
	Pid.R_90_D =0.01*Pid_Ring_90_D;
}
void servo_ccd()
{
	PID_subsection();
	
	error=(float)(Trk.middle_sideline1-63);
	error_2=(float)(Trk.middle_sideline2-63);
	
	SERVO_duty=error*Pid.z_p+(error-error_last)*Pid.z_d;//
	error_last=error;
}

void servo_Control()
{
	if(SERVO_duty>Servo_delatduty_max)    //舵机PWM限幅
	{
		SERVO_duty = Servo_delatduty_max;
	}
	else if(SERVO_duty<(-Servo_delatduty_max))
	{
		SERVO_duty = (-Servo_delatduty_max);
	}
	pwm_duty(PWMB_CH3_P33,Servo_Center-SERVO_duty);	
}