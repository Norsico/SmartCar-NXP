#ifndef _ALLINIT_H_
#define _ALLINIT_H_

#define Servo_Center 5080 //¶æ»úÖÐÖµ  200hz 2650 2400 3150 5225
#define Servo_delatduty_max 450   //580

void beep_off();
void beep_on();
void Timer_Init(void);
void Gpio_En_Init(void);
void All_Init(void);

#endif