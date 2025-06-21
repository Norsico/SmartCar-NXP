#include "headfile.h"

uint8 cls_flag=0;

void IPS114Init()
{
	ips114_init();
	ips114_clear(WHITE);
	delay_us(5);
	ips114_clear(WHITE);
	delay_us(5);
}
void ips114_show()
{
	if(cls_flag==0)     
	{
	 ips114_init();
	 cls_flag=1;
	}

	if(ips114_show_flag==0)   
	{
		ips114_showstr(8,0," start ");
		ips114_showstr(8,1,"  pid  ");
		ips114_showstr(8,2,"  ccd  ");
		ips114_showstr(8,3,"element");	
		
		ips114_showfloat(8,4,distance_L,3,1);
		ips114_showfloat(8,5,distance_R,3,1);	
		ips114_showfloat(8,6,encoder_L,3,1);
		ips114_showfloat(8,7,encoder_R,3,1);
		
		ips114_showfloat(8,8,Desire_Speed,3,1);	
		ips114_showfloat(8,9,Angle_gz,3,1);
		ips114_showfloat(8,10,zebra_1,4,1);	
		ips114_showfloat(8,11,SERVO_duty,4,1);
		
		ips114_showfloat(8,12,Motor_Pid.speed_L,4,1);
		ips114_showfloat(8,13,Motor_Pid.speed_R,4,1);			
	}
	
	if(ips114_show_flag==100)ips114_show_start();
	if(ips114_show_flag==1)ips114_show_pid(); 
	if(ips114_show_flag==2)ips114_show_ccd();
	if(ips114_show_flag==3)ips114_show_element();
	
	if(ips114_show_element_flag==1)ips114_show_Ring();
 	if(ips114_show_element_flag==2)ips114_show_Ramp();
	if(ips114_show_element_flag==3)ips114_show_Cross();
	
	if(ips114_show_ring_flag==1)ips114_Ring_num();
 	if(ips114_show_ring_flag==2)ips114_Ring_Can();
	if(ips114_show_ring_flag==3)ips114_Ring_50();
	if(ips114_show_ring_flag==4)ips114_Ring_60();
 	if(ips114_show_ring_flag==5)ips114_Ring_90();
}
void ips114_show_start()
{
	ips114_showstr(8,0," start1 ");
}
void ips114_show_pid()
{
	ips114_showstr(8,0," KP ");
	ips114_showstr(8,1," KD ");
	ips114_showstr(8,2," SD ");
	ips114_showstr(8,3," CS ");
	
	ips114_showstr(8,4,"DUTY");
	ips114_showstr(8,5,"DUer");
	
	ips114_showstr(8,6,"Z-SP");
	ips114_showstr(8,7,"W-SP");
	
	ips114_showstr(8,8,"M-LI");
	ips114_showstr(8,9,"M-LP");
	ips114_showstr(8,10,"M-RI");
	ips114_showstr(8,11,"M-RP");
	
	ips114_showfloat(40,0,Pid.z_p,2,3);
	ips114_showfloat(40,1,Pid.z_d,2,3);
	ips114_showfloat(40,2,Motor_Pid_speed_Z,3,1);
	ips114_showfloat(40,3,Motor_Pid.Dif_P,1,5);
	
	ips114_showfloat(40,4,duty_pwm,4,1);
	ips114_showfloat(40,5,duty_pwm_error,4,1);
	
	ips114_showfloat(40,6,Linear_speed,3,1);
	ips114_showfloat(40,7,Curve_speed,3,1);
	
	ips114_showfloat(40,8,Motor_Pid.L_Ki,2,3);
	ips114_showfloat(40,9,Motor_Pid.L_Kp,3,3);
	ips114_showfloat(40,10,Motor_Pid.R_Ki,2,3);
	ips114_showfloat(40,11,Motor_Pid.R_Kp,3,3);
}
void ips114_show_ccd()
{
	ips114_showfloat(8,0,Threshold_multiple_1,2,1);
	ips114_showfloat(8,1,Threshold_1,3,1);
	ips114_showfloat(8,2,Trk.Width1,3,1);
	ips114_showfloat(8,3,CCD1.max,3,1);
	ips114_showfloat(8,4,Trk.left_qulu,3,1);
	
	ips114_showfloat(64,0,Trk.left_sideline1,3,1);
	ips114_showfloat(64,1,Trk.right_sideline1,3,1);
  ips114_showfloat(64,2,Trk.middle_sideline1,3,1);
	ips114_showfloat(64,3,CCD1.min,3,1);
	ips114_showfloat(64,4,CCD1.threshold,3,1);
	
	ips114_showfloat(8,5,Threshold_multiple_2,2,1);
	ips114_showfloat(8,6,Threshold_2,3,1);
	ips114_showfloat(8,7,Trk.Width2,3,1);
	ips114_showfloat(8,8,CCD2.max,3,1);
	ips114_showfloat(8,9,Trk.right_qulu,3,1);
	
	ips114_showfloat(64,5,Trk.left_sideline2,3,1);
	ips114_showfloat(64,6,Trk.right_sideline2,3,1);
  ips114_showfloat(64,7,Trk.middle_sideline2,3,1);
	ips114_showfloat(64,8,CCD2.min,3,1);
	ips114_showfloat(64,9,CCD2.threshold,3,1);

	for(i=0;i<128;i++)
	{
		if(i<=Trk.left_sideline1 || i>=Trk.right_sideline1 || i==Trk.middle_sideline1)// 
		{
			ips114_drawpoint(i+1,200,BLACK);
		}
		else ips114_drawpoint(i+1,200,WHITE);
		
		if(i<=Trk.left_sideline2 || i>=Trk.right_sideline2 || i==Trk.middle_sideline2)
		{
			ips114_drawpoint(i+1,220,BLACK);
		}
		else ips114_drawpoint(i+1,220,WHITE);
	}
}	
void ips114_show_element()
{
	ips114_showstr(8,0," Ring ");
	ips114_showstr(8,1," Ramp");
	ips114_showstr(8,2," Cross");

	ips114_showfloat(72,0,Ring_state,1,1);
	ips114_showfloat(72,1,Ramp_flag,1,1);
	ips114_showfloat(72,2,Cross_flag,1,1);
}
void ips114_show_Ring()
{
	ips114_showstr(8,0,"Ring-num");
	ips114_showstr(8,1,"Ring   ");
	ips114_showstr(8,2,"Ring-50");
	ips114_showstr(8,3,"Ring-60");
	ips114_showstr(8,4,"Ring-90");
	
}
void ips114_Ring_num()
{	
	ips114_showstr(8,0,"Ring_num");
	ips114_showstr(8,1,"Ring_flag");
	ips114_showstr(8,2,"Ring_flag");
	ips114_showstr(8,3,"Ring_flag");
	ips114_showstr(8,4,"Ring_flag");
	
	ips114_showfloat(80,0,Ring_num,1,1);
	
	ips114_showfloat(80,1,Ring_flag[0],1,1);
	if(Ring_flag[0]==0)ips114_showstr(120,1," ");
	if(Ring_flag[0]==1)ips114_showstr(120,1,"5");
	if(Ring_flag[0]==2)ips114_showstr(120,1,"6");
	if(Ring_flag[0]==3)ips114_showstr(120,1,"9");
	
	ips114_showfloat(80,2,Ring_flag[1],1,1);
	if(Ring_flag[1]==0)ips114_showstr(120,2," ");
	if(Ring_flag[1]==1)ips114_showstr(120,2,"5");
	if(Ring_flag[1]==2)ips114_showstr(120,2,"6");
	if(Ring_flag[1]==3)ips114_showstr(120,2,"9");
	
	ips114_showfloat(80,3,Ring_flag[2],1,1);
	if(Ring_flag[2]==0)ips114_showstr(120,3," ");
	if(Ring_flag[2]==1)ips114_showstr(120,3,"5");
	if(Ring_flag[2]==2)ips114_showstr(120,3,"6");
	if(Ring_flag[2]==3)ips114_showstr(120,3,"9");
	
	ips114_showfloat(80,4,Ring_flag[3],1,1);
	if(Ring_flag[3]==0)ips114_showstr(120,4," ");
	if(Ring_flag[3]==1)ips114_showstr(120,4,"5");
	if(Ring_flag[3]==2)ips114_showstr(120,4,"6");
	if(Ring_flag[3]==3)ips114_showstr(120,4,"9");
	
}
void ips114_Ring_Can()
{
	ips114_showstr(8,0,"F_R_ql  ");
	ips114_showstr(8,1,"F_R_side");
	ips114_showstr(8,2,"R_I_R_en");
	ips114_showstr(8,3,"R_I_R_si");
	ips114_showstr(8,4,"I_R_en  ");
	ips114_showstr(8,5,"R_O_R_an");
	ips114_showstr(8,6,"O_R_an  ");
	ips114_showstr(8,7,"N_R_en  ");
	

	ips114_showfloat(72,0,Find_Ring_qulv,2,1);
	ips114_showfloat(72,1,Fing_Ring_sideline,2,1);
	ips114_showfloat(72,2,Ready_In_Ring_encoder,2,1);
	ips114_showfloat(72,3,Ready_In_Ring_sideline,2,1);
	ips114_showfloat(72,4,In_Ring_encoder,2,1);
	ips114_showfloat(72,5,Ready_Out_Ring_angle,3,1);
	ips114_showfloat(72,6,Out_Ring_angle,3,1);
	ips114_showfloat(72,7,No_Ring_encoder,3,1);
}
void ips114_Ring_50()
{
	ips114_showstr(8,0,"R_50_P");
	ips114_showstr(8,1,"R_50_D");
	ips114_showstr(8,2,"R_I_50_S");
	ips114_showstr(8,3,"I_50_SPE");
	ips114_showstr(8,4,"R_O_50_S");
	
	ips114_showfloat(72,0,Pid.R_50_P,2,3);
	ips114_showfloat(72,1,Pid.R_50_D,2,3);
	
	ips114_showfloat(72,2,Ready_In_50_Ring_speed,3,1);
	ips114_showfloat(72,3,In_50_Ring_speed,3,1);
	ips114_showfloat(72,4,Ready_Out_50_Ring_speed,3,1);
	

}
void ips114_Ring_60()
{
	ips114_showstr(8,0,"R_60_P");
	ips114_showstr(8,1,"R_60_D");
	ips114_showstr(8,2,"R_I_60_S");
	ips114_showstr(8,3,"I_60_SPE");
	ips114_showstr(8,4,"R_O_60_S");
	
	ips114_showfloat(72,0,Pid.R_60_P,2,3);
	ips114_showfloat(72,1,Pid.R_60_D,2,3);
	
	ips114_showfloat(72,2,Ready_In_60_Ring_speed,3,1);
	ips114_showfloat(72,3,In_60_Ring_speed,3,1);
	ips114_showfloat(72,4,Ready_Out_60_Ring_speed,3,1);
	
}
void ips114_Ring_90()
{
	ips114_showstr(8,0,"R_90_P");
	ips114_showstr(8,1,"R_90_D");
	ips114_showstr(8,2,"R_I_90_S");
	ips114_showstr(8,3,"I_90_SPE");
	ips114_showstr(8,4,"R_O_90_S");
	
	ips114_showfloat(72,0,Pid.R_90_P,2,3);
	ips114_showfloat(72,1,Pid.R_90_D,2,3);

	ips114_showfloat(72,2,Ready_In_90_Ring_speed,3,1);
	ips114_showfloat(72,3,In_90_Ring_speed,3,1);
	ips114_showfloat(72,4,Ready_Out_90_Ring_speed,3,1);
}
void ips114_show_Ramp()
{
	ips114_showstr(8,0,"U_R_spe");
	ips114_showstr(8,1,"O_R_spe");
	
	ips114_showstr(8,2,"F_R_dis ");
	ips114_showstr(8,3,"F_R_wid1");
	ips114_showstr(8,4,"F_R_wid2");
	ips114_showstr(8,5,"F_R_qulv");
	ips114_showstr(8,6,"U_R_en ");
	ips114_showstr(8,7,"O_R_en ");

	ips114_showfloat(72,0,Up_Ramp_speed,3,1);
	ips114_showfloat(72,1,Out_Ramp_speed,3,1);
	
	ips114_showfloat(72,2,Find_Ramp_dis,2,1);
	ips114_showfloat(72,3,Find_Ramp_width1,2,1);
	ips114_showfloat(72,4,Find_Ramp_width2,2,1);
	ips114_showfloat(72,5,Find_Ramp_qulv,2,1);
	ips114_showfloat(72,6,Up_Ramp_encoder,3,1);
	ips114_showfloat(72,7,Out_Ramp_encoder,3,1);
}
void ips114_show_Cross()
{
	ips114_showstr(8,0,"O_C_en ");
	ips114_showstr(8,1,"O_Z_en ");
	
	ips114_showfloat(72,0,Cross_encoder,2,1);
	ips114_showfloat(72,1,Out_Zebra_encoder,3,1);
}