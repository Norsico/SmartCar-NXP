#include "headfile.h"

uint16 duty_pwm=0,duty_pwm_left=0,duty_pwm_right=0;
uint8 duty_pwm_flag=0;
uint16 duty_pwm_error=0;
uint16 duty_pwm_stop=0;
void Duct_init()
{
	pwm_init(PWMB_CH1_P00,300,0); //右
	pwm_init(PWMB_CH2_P01,300,0); //左
}
void Duct_start()
{
	pwm_duty(PWMB_CH1_P00,duty_pwm); //右
	pwm_duty(PWMB_CH2_P01,duty_pwm); //左
	duty_pwm_stop=duty_pwm;
	duty_pwm_flag=1;
}
void Duct_stop()//缓减涵道pwm占空比防止停转瞬间大电流反灌电调
{
	if(duty_pwm_stop>0)
	{
		duty_pwm_stop-=50;
	}
	else duty_pwm_stop=0;
	pwm_duty(PWMB_CH1_P00,duty_pwm_stop); //右
	pwm_duty(PWMB_CH2_P01,duty_pwm_stop); //左
}