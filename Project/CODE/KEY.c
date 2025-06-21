#include "headfile.h"


//开关标志位
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;
uint8 key5_flag;
uint8 key6_flag;

uint8 Model=1;          //光标上下移动标志位
uint8 grade_flag=0;     //确认标志位
uint8 ips114_show_flag=0; //二级菜单标志
uint8 lose=0;           //减
uint8 plus=0;           //加
uint8 return_flag=0;    //返回

uint8 Augle_flag=0; //二级菜单标志

uint8 key_flag=0;
uint8 data_buff[85]={0};

uint8 ips114_show_element_flag=0;
uint8 ips114_show_ring_flag=0;
void key(void)
{
		//开关状态变量
	static uint8 key1_status = 1;
	static uint8 key2_status = 1;
	static uint8 key3_status = 1;
	static uint8 key4_status = 1;
	static uint8 key5_status = 1;
	static uint8 key6_status = 1;
	//上一次开关状态变量
	static uint8 key1_last_status;
	static uint8 key2_last_status;
	static uint8 key3_last_status;
	static uint8 key4_last_status;
	static uint8 key5_last_status;
	static uint8 key6_last_status;
	//使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费
	//保存按键状态
	key1_last_status = key1_status;
	key2_last_status = key2_status;
	key3_last_status = key3_status;
	key4_last_status = key4_status;
	key5_last_status = key5_status;
	key6_last_status = key6_status;
	//读取当前按键状态
	key1_status = KEY1_PIN;
	key2_status = KEY2_PIN;
	key3_status = KEY3_PIN;
	key4_status = KEY4_PIN;
	key5_status = KEY5_PIN;
	key6_status = KEY6_PIN;
	//检测到按键按下之后  并放开置位标志位
	if(key1_status && !key1_last_status)    key1_flag = 1;//返回
	if(key2_status && !key2_last_status)    key2_flag = 1;//上
	if(key3_status && !key3_last_status)    key3_flag = 1;//下
	if(key4_status && !key4_last_status)    key4_flag = 1;//左
	if(key5_status && !key5_last_status)    key5_flag = 1;//右
	if(key6_status && !key6_last_status)    key6_flag = 1;//确认
	//标志位置位之后，可以使用标志位执行自己想要做的事件
	if(key1_flag==1)   
	{
		key1_flag = 0;//使用按键之后，应该清除标志位
		return_flag=1;//返回	
	}
	if(key2_flag==1)   
	{
		key2_flag = 0;//使用按键之后，应该清除标志位
		Model--;  //光标--
	}
	if(key3_flag==1)   
	{
		key3_flag = 0;//使用按键之后，应该清除标志位
		Model++;  //光标++
	}
	if(key4_flag==1)   
	{
		key4_flag = 0;//使用按键之后，应该清除标志位
		lose++;  //减
	}
	if(key5_flag==1)   
	{
		key5_flag = 0;//使用按键之后，应该清除标志位
		plus++;	  //加
	}
	if(key6_flag==1)   
	{
		key6_flag = 0;//使用按键之后，应该清除标志位
		grade_flag++;	//确定
	}
}
void ParameterExchange(void)
{
/******************光标******************/  
	ips114_showstr(0,Model - 1,">");
	ips114_showstr(0,(Model + 14) % 16," ");
	ips114_showstr(0,Model == 16 ? 1 : Model," ");
	
	if(grade_flag==1)                  
	{
		switch(Model)
		{
			case 1: ips114_show_flag=100;grade_flag=2; Model=0; cls_flag=0;Duct_start();    break; //发车标志位 ;
			case 2: ips114_show_flag=1; grade_flag=2; Model=0; cls_flag=0;   break;
			case 3: ips114_show_flag=2; grade_flag=2; Model=0; cls_flag=0;   break;
			case 4: ips114_show_flag=3; grade_flag=2; Model=0; cls_flag=0;   break;
			default:Model=1;break;
		}
	}
	if(ips114_show_flag==100)
	{
		if(grade_flag==3)
		{
			start1=1;
			Augle_flag=1;
  		key_flag=~key_flag;
		}
		if(return_flag==1) //返回
		{
			return_flag=0;
			Model=1;
			cls_flag=0;
			ips114_show_flag=0;
			grade_flag=0;
		}
	}
	if(ips114_show_flag==1)
  {
    if(plus==1)
    {
			switch(Model)
			{
				case 1: Pid_Kp        +=10; break; 
				case 2: Pid_Kd        +=100; break;
				case 3: Motor_Pid_speed_Z   +=5;		
							if(Motor_Pid_speed_Z==180)
							{
								Pid_Kp=1500;
								Pid_Kd=2000;
								Motor_Pid_Dif_P=4;
								duty_pwm=4000;
							}
							if(Motor_Pid_speed_Z==200)
							{
								Pid_Kp=1500;
								Pid_Kd=3000;
								Motor_Pid_Dif_P=6;
								duty_pwm=4500;
							}
							if(Motor_Pid_speed_Z==220)
							{
								Pid_Kp=1500;
								Pid_Kd=4200;
								Motor_Pid_Dif_P=5;
								duty_pwm=5000;
							}
							if(Motor_Pid_speed_Z==230)
							{
								Pid_Kp=1550;
								Pid_Kd=6200;
								Motor_Pid_Dif_P=7;
								duty_pwm=5100;
							}
							if(Motor_Pid_speed_Z==250)
							{
								Pid_Kp=1700;
								Pid_Kd=6800;
								Motor_Pid_Dif_P=9;
								duty_pwm=5600;
							}
							if(Motor_Pid_speed_Z==260)
							{
								Pid_Kp=1800;
								Pid_Kd=9000;
								Motor_Pid_Dif_P=12;
								duty_pwm=5900;
							}
				break; 
				case 4: Motor_Pid_Dif_P     +=1; 		 break;
				case 5: duty_pwm            +=100; break;
				case 6: duty_pwm_error      +=100; break;
				
				case 7: Linear_speed        +=5; break; 
				case 8: Curve_speed         +=5; break; 
				
				case 9: Motor_Pid_Z_L_Ki    +=10; break;
				case 10: Motor_Pid_Z_L_Kp   +=10; break;
				case 11: Motor_Pid_Z_R_Ki   +=10; break;
				case 12: Motor_Pid_Z_R_Kp   +=10; break;

			}
			EepromWrite();
		}
		if(lose==1)
    {
			switch(Model)
			{
				case 1: Pid_Kp    	-=10; break; 
				case 2: Pid_Kd    	-=100;break;
				case 3: Motor_Pid_speed_Z -=5; 
							if(Motor_Pid_speed_Z==180)
							{
								Pid_Kp=1500;
								Pid_Kd=2000;
								Motor_Pid_Dif_P=4;
								duty_pwm=4000;
							}
							if(Motor_Pid_speed_Z==200)
							{
								Pid_Kp=1500;
								Pid_Kd=3000;
								Motor_Pid_Dif_P=6;
								duty_pwm=4500;
							}
							if(Motor_Pid_speed_Z==220)
							{
								Pid_Kp=1500;
								Pid_Kd=4200;
								Motor_Pid_Dif_P=5;
								duty_pwm=5000;
							}
							if(Motor_Pid_speed_Z==230)
							{
								Pid_Kp=1550;
								Pid_Kd=6200;
								Motor_Pid_Dif_P=7;
								duty_pwm=5100;
							}
							if(Motor_Pid_speed_Z==250)
							{
								Pid_Kp=1700;
								Pid_Kd=6800;
								Motor_Pid_Dif_P=9;
								duty_pwm=5600;
							}
							if(Motor_Pid_speed_Z==260)
							{
								Pid_Kp=1800;
								Pid_Kd=9000;
								Motor_Pid_Dif_P=12;
								duty_pwm=5900;
							}
							break; 
				case 4: Motor_Pid_Dif_P       -=1; break;
				case 5: duty_pwm          -=100; break;
				case 6: duty_pwm_error      -=100; break;
				
				case 7: Linear_speed -=5; break; 
				case 8: Curve_speed  -=5; break; 
				
				case 9: Motor_Pid_Z_L_Ki  -=10; break;
				case 10: Motor_Pid_Z_L_Kp  -=10; break;
				case 11: Motor_Pid_Z_R_Ki  -=10; break;
				case 12: Motor_Pid_Z_R_Kp  -=10; break;
				
			}
			EepromWrite();
		}
		lose=0; plus=0;
		if(return_flag==1)   //返回
		{
			return_flag=0;
			Model=1;
			cls_flag=0;
			ips114_show_flag=0;
			grade_flag=0;
		}
  }
	if(ips114_show_flag==2)
	{
		if(plus==1)
    {
			switch(Model)
			{
				case 1: Threshold_multiple_1 +=5; break;
				case 2: Threshold_1 +=5; break;
				
				case 6: Threshold_multiple_2 +=5; break;	
				case 7: Threshold_2 +=5; break;				
			}
			EepromWrite();
		}
		if(lose==1)
    {
			switch(Model)
			{
				case 1: Threshold_multiple_1    -=1; break; 
				case 2: Threshold_1 -=1; break;
				
				case 6: Threshold_multiple_2 -=1; break;	
				case 7: Threshold_2 -=1; break;					
			}
			EepromWrite();
		}
		lose=0; plus=0;
		if(return_flag==1)   //返回
		{
			return_flag=0;
			Model=1;
			cls_flag=0;
			ips114_show_flag=0;
			grade_flag=0;
		}
	}
	if(ips114_show_flag==3||ips114_show_flag==31)
	{
		if(grade_flag==3)                
		{
			switch(Model)
			{
				case 1: ips114_show_element_flag=1; grade_flag=4; Model=0;cls_flag=0; ips114_show_flag=31; break;
				case 2: ips114_show_element_flag=2; grade_flag=4; Model=0;cls_flag=0; ips114_show_flag=31; break;
				case 3: ips114_show_element_flag=3; grade_flag=4; Model=0;cls_flag=0; ips114_show_flag=31; break;
				default:Model=1;break;
			}
		}
		if(ips114_show_element_flag==1||ips114_show_element_flag==41) //环岛
		{
			if(grade_flag==5)
			{
				switch(Model)
				{
					case 1: ips114_show_ring_flag=1; grade_flag=4; Model=0;cls_flag=0; ips114_show_element_flag=41; break;
					case 2: ips114_show_ring_flag=2; grade_flag=4; Model=0;cls_flag=0; ips114_show_element_flag=41; break;
					case 3: ips114_show_ring_flag=3; grade_flag=4; Model=0;cls_flag=0; ips114_show_element_flag=41; break;
					case 4: ips114_show_ring_flag=4; grade_flag=4; Model=0;cls_flag=0; ips114_show_element_flag=41; break;
					case 5: ips114_show_ring_flag=5; grade_flag=4; Model=0;cls_flag=0; ips114_show_element_flag=41; break;
					default:Model=1;break;
				}
			}
			if(ips114_show_ring_flag==1 && ips114_show_element_flag==41)
			{
				if(plus==1)
				{
					switch(Model)
					{	
						case 1: Ring_num 													+=1; break;
						case 2: Ring_flag[0] 											+=1; break;
						case 3: Ring_flag[1] 											+=1; break;
						case 4: Ring_flag[2] 											+=1; break;
						case 5: Ring_flag[3] 											+=1; break;
					}
					EepromWrite();
				}
				if(lose==1)
				{
					switch(Model)
					{
						case 1: Ring_num 													-=1; break;
						case 2: Ring_flag[0] 											-=1; break;
						case 3: Ring_flag[1] 											-=1; break;
						case 4: Ring_flag[2] 											-=1; break;
						case 5: Ring_flag[3] 											-=1; break;	
					}
					EepromWrite();
				}
				lose=0; plus=0;
			}
		
			if(ips114_show_ring_flag==2 && ips114_show_element_flag==41)
			{
				if(plus==1)
				{
					switch(Model)
					{	
						case 1: Find_Ring_qulv 												+=2; break;
						case 2: Fing_Ring_sideline 										+=1; break;
						case 3: Ready_In_Ring_encoder 								+=5; break;
						case 4: Ready_In_Ring_sideline 								+=5; break;
						case 5: In_Ring_encoder 											+=5; break;
						case 6: Ready_Out_Ring_angle 								+=5; break;
						case 7: Out_Ring_angle 											+=5; break;
						case 8: No_Ring_encoder 											+=5; break;		
					}
					EepromWrite();
				}
				if(lose==1)
				{
					switch(Model)
					{
						case 1: Find_Ring_qulv 												-=1; break;
						case 2: Fing_Ring_sideline 										-=1; break;
						case 3: Ready_In_Ring_encoder 								-=1; break;
						case 4: Ready_In_Ring_sideline 								-=1; break;
						case 5: In_Ring_encoder 											-=1; break;
						case 6: Ready_Out_Ring_angle 								-=1; break;
						case 7: Out_Ring_angle 											-=1; break;
						case 8: No_Ring_encoder 											-=1; break;		
					}
					EepromWrite();
				}
				lose=0; plus=0;
			}
			if(ips114_show_ring_flag==3 && ips114_show_element_flag==41)
			{
				if(plus==1)
				{
					switch(Model)
					{
						case 1: Pid_Ring_50_P	        		+=10; break;
						case 2: Pid_Ring_50_D	  	  			+=10; break;	
						case 3: Ready_In_50_Ring_speed	  +=5; break;
						case 4: In_50_Ring_speed	  			+=5; break;	
						case 5: Ready_Out_50_Ring_speed 	+=5; break;					
					}
					EepromWrite();
				}
				if(lose==1)
				{
					switch(Model)
					{
						case 1: Pid_Ring_50_P	        		-=10; break;
						case 2: Pid_Ring_50_D	  	  			-=100; break;	
						case 3: Ready_In_50_Ring_speed	  -=5; break;
						case 4: In_50_Ring_speed	  			-=5; break;	
						case 5: Ready_Out_50_Ring_speed 	-=5; break;				
					}
					EepromWrite();
				}
				lose=0; plus=0;
			}
			if(ips114_show_ring_flag==4 && ips114_show_element_flag==41)
			{
				if(plus==1)
				{
					switch(Model)
					{
						case 1: Pid_Ring_60_P 						+=10; break;	
						case 2: Pid_Ring_60_D 						+=10; break;		
						case 3: Ready_In_60_Ring_speed 		+=5; break;	
						case 4: In_60_Ring_speed	        +=5; break;
						case 5: Ready_Out_60_Ring_speed	  +=5; break;					
					}
					EepromWrite();
				}
				if(lose==1)
				{
					switch(Model)
					{
						case 1: Pid_Ring_60_P 						-=10; break;	
						case 2: Pid_Ring_60_D 						-=10; break;		
						case 3: Ready_In_60_Ring_speed 		-=5; break;	
						case 4: In_60_Ring_speed	        -=5; break;
						case 5: Ready_Out_60_Ring_speed	  -=5; break;					
					}
					EepromWrite();
				}
				lose=0; plus=0;
			}
			if(ips114_show_ring_flag==5 && ips114_show_element_flag==41)
			{
				if(plus==1)
				{
					switch(Model)
					{
						case 1: Pid_Ring_90_P	      			+=10; break;
						case 2: Pid_Ring_90_D	  					+=10; break;	
						case 3: Ready_In_90_Ring_speed 		+=5; break;	
						case 4: In_90_Ring_speed 					+=5; break;	
						case 5: Ready_Out_90_Ring_speed 	+=5; break;						
					}
					EepromWrite();
				}
				if(lose==1)
				{
					switch(Model)
					{
						case 1: Pid_Ring_90_P	      		-=5; break;
						case 2: Pid_Ring_90_D	  				-=5; break;	
						case 3: Ready_In_90_Ring_speed 	-=5; break;	
						case 4: In_90_Ring_speed 				-=5; break;	
						case 5: Ready_Out_90_Ring_speed 	-=5; break;						
					}
					EepromWrite();
				}
				lose=0; plus=0;
			}
		}
		if(ips114_show_flag==31&&ips114_show_element_flag==2) //坡道
		{
			if(plus==1)
			{
				switch(Model)
				{
					case 1: Up_Ramp_speed	        +=5; break;
					case 2: Out_Ramp_speed	  	  +=5; break;	
					case 3: Find_Ramp_dis	        +=5; break;
					case 4: Find_Ramp_width1	  	+=5; break;	
					case 5: Find_Ramp_width2 	 		+=5; break;	
					case 6: Find_Ramp_qulv 				+=5; break;	
					case 7: Up_Ramp_encoder 			+=5; break;		
					case 8: Out_Ramp_encoder 			+=5; break;							
				}
				EepromWrite();
			}
			if(lose==1)
			{
				switch(Model)
				{
					case 1: Up_Ramp_speed    			-=5; break; 
					case 2: Out_Ramp_speed    	  -=5; break; 
					case 3: Find_Ramp_dis    			-=2; break; 
					case 4: Find_Ramp_width1    	-=1; break; 
					case 5: Find_Ramp_width2    	-=1; break; 
					case 6: Find_Ramp_qulv    		-=1; break; 
					case 7: Up_Ramp_encoder    	  -=5; break; 
					case 8: Out_Ramp_encoder 			-=5; break;							
				}
				EepromWrite();
			}
			lose=0; plus=0;
		}
		if(ips114_show_flag==31&&ips114_show_element_flag==3) //十字
		{
			if(plus==1)
			{
				switch(Model)
				{
					case 1: Cross_encoder 				+=5; break;
					case 2: Out_Zebra_encoder 		+=5; break;			
				}
				EepromWrite();
			}
			if(lose==1)
			{
				switch(Model)
				{
					case 1: Cross_encoder 			  -=1; break;
					case 2: Out_Zebra_encoder 		-=1; break; 			
				}
				EepromWrite();
			}
			lose=0; plus=0;
		}
		if(return_flag==1 && ips114_show_ring_flag!=0)    //三级菜单退出
		{
			return_flag=0;
			Model=1;
			cls_flag=0;
			ips114_show_ring_flag=0;
			ips114_show_element_flag=1;
			grade_flag=4;
		}
		if(return_flag==1 && ips114_show_element_flag!=0)    //三级菜单退出
		{
			return_flag=0;
			Model=1;
			cls_flag=0;
			ips114_show_element_flag=0;
			ips114_show_flag=3;
			grade_flag=2;
		}
		if(return_flag==1)   //返回
		{
			return_flag=0;
			Model=1;
			cls_flag=0;
			ips114_show_flag=0;
			grade_flag=0;
		}
	}
}
/*************************************************************************
* 函数名称： Flash_Write
* 功能说明： 将数据写入flash
* 修改时间： 2019-11-2
*************************************************************************/
void EepromWrite(void)
{
	data_buff[0] = Motor_Pid_speed_Z;
	data_buff[1] = Motor_Pid_speed_Z>>8;
	data_buff[2] = Motor_Pid_Z_L_Ki;
	data_buff[3] = Motor_Pid_Z_L_Ki>>8;
	data_buff[4] = Motor_Pid_Z_L_Kp;
	data_buff[5] = Motor_Pid_Z_L_Kp>>8;
	data_buff[6] = Motor_Pid_Z_R_Ki;
	data_buff[7] = Motor_Pid_Z_R_Ki>>8;
	data_buff[8] = Motor_Pid_Z_R_Kp;
	data_buff[9] = Motor_Pid_Z_R_Kp>>8;
	data_buff[10] = Pid_Kp;
	data_buff[11] = Pid_Kp>>8;
	data_buff[12] = Pid_Kd;
	data_buff[13] = Pid_Kd>>8;

	data_buff[14] = Motor_Pid_Dif_P;
	data_buff[15] = Motor_Pid_Dif_P>>8;
	
	data_buff[16] = Threshold_multiple_1;
	data_buff[17] = Threshold_multiple_2;
	
	data_buff[18] = duty_pwm;
	data_buff[19] = duty_pwm>>8;
	
	//元素ui
	data_buff[20] = Out_Zebra_encoder;
	
	data_buff[21] = Cross_encoder;
	
	data_buff[22] = Find_Ramp_dis;
	data_buff[23] = Find_Ramp_width1;
	data_buff[24] = Find_Ramp_width2;
	data_buff[25] = Find_Ramp_qulv;
	data_buff[26] = Up_Ramp_encoder;
	data_buff[27] = Out_Ramp_encoder;

	data_buff[28] = Find_Ring_qulv;
	data_buff[29] = Fing_Ring_sideline;
	data_buff[30] = Ready_In_Ring_encoder;
	data_buff[31] = Ready_In_Ring_sideline;
	data_buff[32] = In_Ring_encoder;
	data_buff[33] = Ready_Out_Ring_angle;
	data_buff[34] = Ready_Out_Ring_angle>>8;
	data_buff[35] = Out_Ring_angle;
	data_buff[36] = Out_Ring_angle>>8;
	data_buff[37] = No_Ring_encoder;
	
	data_buff[38] = Up_Ramp_speed;
	data_buff[39] = Up_Ramp_speed>>8;
	data_buff[40] = Out_Ramp_speed;
	data_buff[41] = Out_Ramp_speed>>8;
	
	data_buff[42] = Pid_Ring_50_P;
	data_buff[43] = Pid_Ring_50_P>>8;
	data_buff[44] = Pid_Ring_50_D;
	data_buff[45] = Pid_Ring_50_D>>8;
	data_buff[46] = Ready_In_50_Ring_speed;
	data_buff[47] = Ready_In_50_Ring_speed>>8;
	data_buff[48] = In_50_Ring_speed;
	data_buff[49] = In_50_Ring_speed>>8;
	data_buff[50] = Ready_Out_50_Ring_speed;
	data_buff[51] = Ready_Out_50_Ring_speed>>8;
	
	data_buff[52] = Pid_Ring_60_P;
	data_buff[53] = Pid_Ring_60_P>>8;
	data_buff[54] = Pid_Ring_60_D;
	data_buff[55] = Pid_Ring_60_D>>8;
	data_buff[56] = Ready_In_60_Ring_speed;
	data_buff[57] = Ready_In_60_Ring_speed>>8;
	data_buff[58] = In_60_Ring_speed;
	data_buff[59] = In_60_Ring_speed>>8;
	data_buff[60] = Ready_Out_60_Ring_speed;
	data_buff[61] = Ready_Out_60_Ring_speed>>8;
	
	data_buff[62] = Pid_Ring_90_P;
	data_buff[63] = Pid_Ring_90_P>>8;
	data_buff[64] = Pid_Ring_90_D;
	data_buff[65] = Pid_Ring_90_D>>8;
	data_buff[66] = Ready_In_90_Ring_speed;
	data_buff[67] = Ready_In_90_Ring_speed>>8;
	data_buff[68] = In_90_Ring_speed;
	data_buff[69] = In_90_Ring_speed>>8;
	data_buff[70] = Ready_Out_90_Ring_speed;
	data_buff[71] = Ready_Out_90_Ring_speed>>8;

	data_buff[72] = Ring_num;
	data_buff[73] = Ring_flag[0];
	data_buff[74] = Ring_flag[1];
	data_buff[75] = Ring_flag[2];
	data_buff[76] = Ring_flag[3];
	
	data_buff[77] = Linear_speed;
	data_buff[78] = Linear_speed>>8;
	data_buff[79] = Curve_speed;
	data_buff[80] = Curve_speed>>8;
	
	data_buff[81] = Threshold_1;
	data_buff[82] = Threshold_2;
	data_buff[83] = duty_pwm_error;
	data_buff[84] = duty_pwm_error>>8;
	extern_iap_write_bytes(0x0000,data_buff,85);
}
/*************************************************************************
* 函数名称： Flash_Read
* 功能说明： 读数据
* 修改时间： 2019-11-2
*************************************************************************/
void EepromRead(void)
{
	iap_read_bytes(0x0000,data_buff,85);
	Motor_Pid_speed_Z 		= data_buff[0]+( data_buff[1]<<8);
	Motor_Pid_Z_L_Ki  		= data_buff[2]+( data_buff[3]<<8);
	Motor_Pid_Z_L_Kp  		= data_buff[4]+( data_buff[5]<<8);
	Motor_Pid_Z_R_Ki  		= data_buff[6]+( data_buff[7]<<8);
	Motor_Pid_Z_R_Kp  		= data_buff[8]+( data_buff[9]<<8);
	Pid_Kp      		= data_buff[10]+( data_buff[11]<<8);
	Pid_Kd					= data_buff[12]+(data_buff[13]<<8);

	Motor_Pid_Dif_P   	 	= data_buff[14]+(data_buff[15]<<8);//
	
	Threshold_multiple_1 	=	data_buff[16];
	Threshold_multiple_2 	= data_buff[17];
	
	duty_pwm    	   			= data_buff[18]+(data_buff[19]<<8);	
	
	Out_Zebra_encoder 		=	data_buff[20];
	Cross_encoder 				=	data_buff[21];
	Find_Ramp_dis 				= data_buff[22];
	Find_Ramp_width1 			=	data_buff[23];
	Find_Ramp_width2 			= data_buff[24];
	Find_Ramp_qulv 				=	data_buff[25];
	Up_Ramp_encoder 			= data_buff[26];
	Out_Ramp_encoder 			= data_buff[27];
	
	Find_Ring_qulv 				=	data_buff[28];
	Fing_Ring_sideline 		= data_buff[29];
	Ready_In_Ring_encoder =	data_buff[30];
	Ready_In_Ring_sideline= data_buff[31];
	In_Ring_encoder 			=	data_buff[32];
	Ready_Out_Ring_angle 	= data_buff[33]+(data_buff[34]<<8);	
	Out_Ring_angle 				=	data_buff[35]+(data_buff[36]<<8);	
	No_Ring_encoder 			= data_buff[37];
	
	Up_Ramp_speed 	      = data_buff[38]+(data_buff[39]<<8);	
	Out_Ramp_speed 				=	data_buff[40]+(data_buff[41]<<8);

	Pid_Ring_50_P 					=	data_buff[42]+(data_buff[43]<<8);
	Pid_Ring_50_D 					=	data_buff[44]+(data_buff[45]<<8);
	Ready_In_50_Ring_speed 	=	data_buff[46]+(data_buff[47]<<8);
	In_50_Ring_speed 				=	data_buff[48]+(data_buff[49]<<8);
	Ready_Out_50_Ring_speed =	data_buff[50]+(data_buff[51]<<8);

	Pid_Ring_60_P 					=	data_buff[52]+(data_buff[53]<<8);
	Pid_Ring_60_D 					=	data_buff[54]+(data_buff[55]<<8);
	Ready_In_60_Ring_speed 	=	data_buff[56]+(data_buff[57]<<8);
	In_60_Ring_speed 				=	data_buff[58]+(data_buff[59]<<8);
	Ready_Out_60_Ring_speed =	data_buff[60]+(data_buff[61]<<8);

	Pid_Ring_90_P 					=	data_buff[62]+(data_buff[63]<<8);
	Pid_Ring_90_D 					=	data_buff[64]+(data_buff[65]<<8);
	Ready_In_90_Ring_speed 	=	data_buff[66]+(data_buff[67]<<8);
	In_90_Ring_speed 				=	data_buff[68]+(data_buff[69]<<8);
	Ready_Out_90_Ring_speed =	data_buff[70]+(data_buff[71]<<8);
	
	Ring_num =data_buff[72];
	Ring_flag[0] = data_buff[73];
	Ring_flag[1] = data_buff[74];
	Ring_flag[2] = data_buff[75];
	Ring_flag[3] = data_buff[76];
	
	Linear_speed 				=	data_buff[77]+(data_buff[78]<<8);
	Curve_speed					=	data_buff[79]+(data_buff[80]<<8);
	
	Threshold_1 = data_buff[81];
	Threshold_2 = data_buff[82];
	
	duty_pwm_error 			=	data_buff[83]+(data_buff[84]<<8);
}