#include "headfile.h"

void Timer_Init(void)
{
	ctimer_count_init(SPEEDL_PLUSE);	//初始化定时器0作为外部计数
	ctimer_count_init(SPEEDR_PLUSE);	//初始化定时器3作为外部计数
	pit_timer_ms(TIM_4 ,5);
	
}
void Gpio_En_Init(void)
{
	gpio_mode(P4_4,GPIO);  //按键初始化
	gpio_mode(P5_1,GPIO);
	gpio_mode(P3_7,GPIO);
	gpio_mode(P4_5,GPIO);
	gpio_mode(P4_6,GPIO);
	gpio_mode(P3_6,GPIO);

	
	adc_init(ADC_P11,ADC_SYSclk_DIV_2); //红外测距初始化
	adc_init(ADC_P14,ADC_SYSclk_DIV_2);

	gpio_mode(P3_2, GPO_PP);//蜂鸣器初始化
	P32=1;//关闭蜂鸣器
	
  pwm_init(PWMA_CH2N_P13, 17000, 0); 	//初始化PWM1  使用P60引脚  初始化频率为17Khz
	pwm_init(PWMA_CH4P_P26, 17000, 0); 	//初始化PWM2  使用P62引脚  初始化频率为17Khz
	
	gpio_mode(P1_0, GPO_PP);	// P10引脚设置为推挽输出 右
	gpio_mode(P2_4, GPO_PP);	// P24引脚设置为推挽输出 左
}
void beep_on()
{
	P32 = 1; //蜂鸣器初始化
}

void beep_off()
{
	P32 = 0;
}
void All_Init(void)
{
	imu660ra_init(); //陀螺仪
//	wireless_uart_init();//无线串口初始化
	
	Gpio_En_Init(); //外设
	servo_init(); //舵机
	Duct_init();  //电调
	
	IPS114Init(); //屏幕
	
	iap_init();          
	iap_set_tps();     
	EepromRead(); //flash读取
	
	ccd_init();	
	ccd_collect();
	delay_ms(5);
	ccd_collect();    //采集两个ccd数据  //再采集
	delay_ms(5);
	CCDimage_init();
	
	Timer_Init();
	
	PID_init();	
	Motor_Pid_init();
}