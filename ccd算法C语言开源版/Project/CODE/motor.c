#include "headfile.h"

/***************Motor_Control************/
uint8 stop_flag=0;
uint8 block_time=0; //堵转时间记录
uint8 stop_time=0;
/***************Motor_Control************/
uint8 start1=0;
int16 PWM_L=0,PWM_R=0;
/*************Incremental_PI************/
int16 Motor_error_L=0,Motor_last_error_L=0;
int16 Motor_error_R=0,Motor_last_error_R=0;
MOTOR_PID Motor_Pid;
/**************UI调节的速度环参数************/
float Desire_Speed=0;
int16 Motor_Pid_speed_Z=0; //ui调节的速

int16 Motor_Pid_Z_L_Ki= 0;
int16 Motor_Pid_Z_L_Kp= 0;
int16 Motor_Pid_Z_R_Ki= 0;
int16 Motor_Pid_Z_R_Kp= 0;

//差速1
int16 Motor_Pid_Dif_P=0;
//直道弯道速度
int16 Linear_speed=0;//直道速度
int16 Curve_speed=0; //弯道速度
//坡道速度
int16 Up_Ramp_speed=0;//上坡速度
int16 Out_Ramp_speed=0;//下坡速度
//环岛速度
uint8 Ring_Speed=0;

int16 Ready_In_50_Ring_speed=0;
int16 In_50_Ring_speed=0;
int16 Ready_Out_50_Ring_speed=0;

int16 Ready_In_60_Ring_speed=0;
int16 In_60_Ring_speed=0;
int16 Ready_Out_60_Ring_speed=0;

int16 Ready_In_90_Ring_speed=0;
int16 In_90_Ring_speed=0;
int16 Ready_Out_90_Ring_speed=0;

void Motor_PID_subsection()
{
	Motor_Pid.L_Ki=0.01*Motor_Pid_Z_L_Ki; 	
	Motor_Pid.L_Kp=0.1*Motor_Pid_Z_L_Kp; 	
	Motor_Pid.R_Ki=0.01*Motor_Pid_Z_R_Ki; 
	Motor_Pid.R_Kp=0.1*Motor_Pid_Z_R_Kp; 

	Motor_Pid.speed=1.0*Desire_Speed;
	
	Motor_Pid.Dif_P=0.00001*Motor_Pid_Dif_P;
}
void Motor_Pid_init()
{
	Motor_Pid.speed=0.0;

	Motor_Pid.speed_L=0.0;
	Motor_Pid.speed_R=0.0;

	Motor_Pid.L_Ki=0.0;    
	Motor_Pid.L_Kp=0.0;  
	Motor_Pid.R_Ki=0.0;  
	Motor_Pid.R_Kp=0.0;  
	
	Motor_Pid.Dif_Speed=0.0;
}

void Motor_Control()
{
	if(black_write_1==1 && black_write_2==1)
	{
		stop_time++;
	}
	else
	{
		stop_time=0;
	}
	/***************堵转保护***********/
	if((encoder_L<=5&&PWM_L>9300)||(encoder_R<=5&&PWM_R>9300)&&start1==1)   //疯转保护，
	{
		block_time++;
	}
	else
	{
		block_time=0;
	}
	if(block_time==4 || stop_time==8)//   
	{
		stop_flag=1;
	}
	/***********停车保护************/
	if(stop_flag==1)
	{
		start1=0;
		duty_pwm_flag=0;
		Duct_stop();
	}
	
	//速度决策
	Desire_Speed=Motor_Pid_speed_Z;
	
	if((encoder>Desire_Speed)&&start1==1) //缓减
	{
		Motor_Pid.speed-=20;
		if(Motor_Pid.speed-Desire_Speed<20)
		{
			Motor_Pid.speed=Desire_Speed;
		}
	}
	
	if(Straight==1 && Ramp_flag==0 && Cross_flag==0 && Ring_state==0 && Ring_left==0 && Ring_right==0 && Ring_Speed==0)
	{
		Desire_Speed=Motor_Pid_speed_Z+Linear_speed;
	}
	if(curve==1 && Ramp_flag==0 && Cross_flag==0 && Ring_state==0 && Ring_left==0 && Ring_right==0 && Ring_Speed==0)
	{
		Desire_Speed=Motor_Pid_speed_Z-Curve_speed;
	}
	
	
	if(Ramp_flag==1)Desire_Speed=Up_Ramp_speed; 
	if(Ramp_flag==2)Desire_Speed=Out_Ramp_speed;
	
	if(Ring_Speed==1)Desire_Speed=Ready_In_50_Ring_speed;
	if(Ring_Speed==11)Desire_Speed=In_50_Ring_speed;
	if(Ring_Speed==12)Desire_Speed=Ready_Out_50_Ring_speed;
	
	if(Ring_Speed==2)Desire_Speed=Ready_In_60_Ring_speed;
	if(Ring_Speed==21)Desire_Speed=In_60_Ring_speed;
	if(Ring_Speed==22)Desire_Speed=Ready_Out_60_Ring_speed;
	
	if(Ring_Speed==3)Desire_Speed=Ready_In_90_Ring_speed;
	if(Ring_Speed==31)Desire_Speed=In_90_Ring_speed;
	if(Ring_Speed==32)Desire_Speed=Ready_Out_90_Ring_speed;
	
	
	//涵道关闭
	if(Zebra_flag==2)
	{
		Desire_Speed=0;
		duty_pwm_flag=0;
		Duct_stop();
	}
	
	Motor_PID_subsection(); //速度环PID赋值
	
	Motor_Pid.Dif_Speed=0.47*SERVO_duty*Motor_Pid.speed;
	
	if(SERVO_duty>=0)
	{
		Motor_Pid.speed_L=Motor_Pid.speed;
		Motor_Pid.speed_R=Motor_Pid.speed-Motor_Pid.Dif_Speed*Motor_Pid.Dif_P;
	}
	if(SERVO_duty<0)
	{
		Motor_Pid.speed_L=Motor_Pid.speed+Motor_Pid.Dif_Speed*Motor_Pid.Dif_P;
		Motor_Pid.speed_R=Motor_Pid.speed;
	}
	
	Motor_Control_L(Motor_Pid.speed_L);
	Motor_Control_R(Motor_Pid.speed_R);
}

void Motor_Control_L(int16 Example_L)  
{
	if(start1==1)
	{
		Incremental_PI_L(encoder_L,Example_L);
	}
	else
	{
		PWM_L=0;
	}
	if(0<=PWM_L) //电机1   正转 设置占空比为 百分之 (9900/GTM_ATOM0_PWM_DUTY_MAX*100)
	{
		if(PWM_L>9900)
		{
			PWM_L=9900;
		}
		DIR_2 = 0;
		pwm_duty(PWM_2, PWM_L);
	}
	else                //电机1   反转
	{
		if(PWM_L<-9900)
		{
			PWM_L=-9900;
		}
		DIR_2 = 1;
		pwm_duty(PWM_2, -PWM_L);
	}
}  
void Motor_Control_R(int16 Example_R) 
{
	if(start1==1)
	{
		Incremental_PI_R(encoder_R,Example_R);
	}
	else
	{
		PWM_R=0;
	}
	if(0<=PWM_R) //电机2   正转 设置占空比为 百分之 (9900/GTM_ATOM0_PWM_DUTY_MAX*100)
	{
		if(PWM_R>9900)
		{
			PWM_R=9900;
		}
		DIR_1 = 0;
		pwm_duty(PWM_1, PWM_R);  
	}
	else                //电机2   反转
	{
		if(PWM_R<-9900)
		{
			PWM_R=-9900;
		}
		DIR_1 = 1;
		pwm_duty(PWM_1, -PWM_R);  
	}
}
void Incremental_PI_L (int encoder_L,int Target_L)  //速度环
{		
	Motor_error_L=Target_L-encoder_L;                //求出速度偏差，由测量值减去目标值
	PWM_L+=Motor_Pid.L_Kp*(Motor_error_L-Motor_last_error_L)+Motor_Pid.L_Ki*Motor_error_L;   //使用增量 PI 控制器求出电机 PWM。
	Motor_last_error_L=Motor_error_L;                       //保存上一次偏差
}

void Incremental_PI_R (int encoder_R,int Target_R)
{
	Motor_error_R=Target_R-encoder_R;                //求出速度偏差，由测量值减去目标值
	PWM_R+=Motor_Pid.R_Kp*(Motor_error_R-Motor_last_error_R)+Motor_Pid.R_Ki*Motor_error_R;   //使用增量 PI 控制器求出电机 PWM。
	Motor_last_error_R=Motor_error_R;                       //保存上一次偏差
}
