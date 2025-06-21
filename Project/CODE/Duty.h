#ifndef _DUCT_H_
#define _DUCT_H_

extern uint16 duty_pwm,duty_pwm_left,duty_pwm_right;
extern uint16 duty_pwm_error;
extern uint16 duty_pwm_stop;
extern uint8 duty_pwm_flag;
void Duct_init();
void Duct_start();
void Duct_stop();

#endif