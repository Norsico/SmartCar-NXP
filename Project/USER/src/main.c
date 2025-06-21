#include "headfile.h"
void main()
{
	clock_init(SYSTEM_CLOCK_52M);	// 初始化系统频率,勿删除此句代码。
	board_init();					// 初始化寄存器,勿删除此句代码。

	All_Init();
  beep_off();
	
	EnableGlobalIRQ();
	
	while(1)
	{
		if(key_flag==0)//按键ui屏幕显示
		{
			key();
			ParameterExchange();
			ips114_show();
		}
		if(tsl1401_finish_flag==1)
		{
			CCD_Processing();//ccd的图象处理
			//ccd_send_data(UART_1, ccd_data_ch1);
			tsl1401_finish_flag = 0;	
		}
	}
	
}


