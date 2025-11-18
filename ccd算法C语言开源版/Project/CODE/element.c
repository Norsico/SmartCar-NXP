#include "headfile.h"

uint8 Ring_50=0;//降速
uint8 Ring_60=0;//跟70环参数一样
uint8 Ring_90=0;//应该可以跟80环一样

int8 Ring_num=0;
int8 Ring_flag[4]={0};
int8 Ring_open_flag[4]={0};
int8 Ring_stop=0;

void Ring_all()
{
  if(Ring_num>0 && Ring_open_flag[0]!=1)
  {
    if(Ring_flag[0]==1) {Ring_50=1;Ring_60=0;Ring_90=0;}
    if(Ring_flag[0]==2) {Ring_50=0;Ring_60=1;Ring_90=0;}
		if(Ring_flag[0]==3) {Ring_50=0;Ring_60=0;Ring_90=1;}
    if(Ring_stop==1)
    {
      Ring_stop=0;
      Ring_open_flag[0]=1;
    }
	}
  if(Ring_num>1 && Ring_open_flag[1]!=1 &&  Ring_open_flag[0]==1)
  {
    if(Ring_flag[1]==1) {Ring_50=1;Ring_60=0;Ring_90=0;}
    if(Ring_flag[1]==2) {Ring_50=0;Ring_60=1;Ring_90=0;}
		if(Ring_flag[1]==3) {Ring_50=0;Ring_60=0;Ring_90=1;}
    if(Ring_stop==1)
    {
      Ring_stop=0;
      Ring_open_flag[1]=1;
    }
  }
	if(Ring_num>2 && Ring_open_flag[2]!=1 &&  Ring_open_flag[1]==1)
  {
    if(Ring_flag[2]==1) {Ring_50=1;Ring_60=0;Ring_90=0;}
    if(Ring_flag[2]==2) {Ring_50=0;Ring_60=1;Ring_90=0;}
		if(Ring_flag[2]==3) {Ring_50=0;Ring_60=0;Ring_90=1;}
    if(Ring_stop==1)
    {
      Ring_stop=0;
      Ring_open_flag[2]=1;
    }
  }
	if(Ring_num>3 && Ring_open_flag[3]!=1 &&  Ring_open_flag[2]==1)
  {
    if(Ring_flag[3]==1) {Ring_50=1;Ring_60=0;Ring_90=0;}
    if(Ring_flag[3]==2) {Ring_50=0;Ring_60=1;Ring_90=0;}
		if(Ring_flag[3]==3) {Ring_50=0;Ring_60=0;Ring_90=1;}
    if(Ring_stop==1)
    {
      Ring_stop=0;
      Ring_open_flag[3]=1;
    }
  }
	if(Ring_num>4 && Ring_open_flag[4]!=1 &&  Ring_open_flag[3]==1)
  {
    if(Ring_flag[4]==1) {Ring_50=1;Ring_60=0;Ring_90=0;}
    if(Ring_flag[4]==2) {Ring_50=0;Ring_60=1;Ring_90=0;}
		if(Ring_flag[4]==3) {Ring_50=0;Ring_60=0;Ring_90=1;}
    if(Ring_stop==1)
    {
      Ring_stop=0;
      Ring_open_flag[4]=1;
    }
  }
}

uint8 Ring_state=0;
uint16 encoder_Ring=0;
uint16 Angle_Ring=0;

uint8 Ring_left=0;
uint8 Ring_right=0;

//环岛ui
uint8 Find_Ring_qulv=0;
uint8 Fing_Ring_sideline=0;

uint8 Ready_In_Ring_encoder=0; 
uint8 Ready_In_Ring_sideline=0;

uint8 In_Ring_encoder=0; 
int16 Ready_Out_Ring_angle=0; 
int16 Out_Ring_angle=0; 
uint8 No_Ring_encoder=0;

void Ring()
{
	switch(Ring_state)
	{
		case No_Ring: //做防误判弯道
			if(CCD1_left_flag==0 && CCD1_right_flag==1 && Trk.right_qulu<=12 && black_write_2==0)
			{
				Ring_state=Find_Ring;
				Ring_left=1;
				beep_on();
				encoder_integral=100;
				encoder_Ring=encoder_integral;
			}
			if(CCD1_left_flag==1 && CCD1_right_flag==0 && Trk.left_qulu<=12 && black_write_2==0)
			{
				Ring_state=Find_Ring;
				Ring_right=1;
				beep_on();
				encoder_integral=100;
				encoder_Ring=encoder_integral;
			}
			//因为弯入十字会导致环岛误判，防误判
			if((Ring_left==1 || Ring_right==1) && CCD1_left_flag==0 && CCD1_right_flag==0 && black_write_2==0 && black_write_1==0)
			{
				Ring_state=No_Ring;
				Ring_left=0;
				Ring_right=0;
				Cross_flag=1;
				Ramp_flag=0;
				beep_on();
				encoder_integral=100;
				encoder_cross=encoder_integral;
			}
			break;
		case Find_Ring:
				if(Ring_left==1)
				{
					if(abs(encoder_Ring-encoder_integral)>Ready_In_Ring_encoder && Trk.left_sideline2<27)
					{
						Ring_state=Ready_In_Ring;
						beep_on();
						Angle_gz=0;
						Angle_Ring=0;
						if(Ring_50==1)Ring_Speed=1;
						if(Ring_60==1)Ring_Speed=2;
						if(Ring_90==1)Ring_Speed=3;
						encoder_Ring=encoder_integral;
					}
					else if(Trk.right_qulu>30)
					{
						Ring_state=No_Ring;
						beep_off();
						Ring_left=0;
					}
				}
				
				if(Ring_right==1)
				{
					if(abs(encoder_Ring-encoder_integral)>Ready_In_Ring_encoder && Trk.right_sideline2>100)
					{
						Ring_state=Ready_In_Ring;
						beep_on();
						Angle_gz=0;
						Angle_Ring=0;
						if(Ring_50==1)Ring_Speed=1;
						if(Ring_60==1)Ring_Speed=2;
						if(Ring_90==1)Ring_Speed=3;
						encoder_Ring=encoder_integral;
					}
					else if(Trk.left_qulu>30)
					{
						Ring_state=No_Ring;
						Ring_right=0;
						beep_off();
					}
				}
				
			break;
		case Ready_In_Ring:
			if(Ring_left==1 && abs(encoder_Ring-encoder_integral)>In_Ring_encoder)
			{
				Ring_state=In_Ring; //准备分大小环pd与速度
				if(Ring_50==1)
				{
					Ring_Speed=11;
					Pid_Ring_flag=1;
				}
				if(Ring_60==1)
				{
					Ring_Speed=21;
					Pid_Ring_flag=2;
				}
				if(Ring_90==1)
				{
					Ring_Speed=31;
					Pid_Ring_flag=3;
				}
			}
			
			if(Ring_right==1 && abs(encoder_Ring-encoder_integral)>In_Ring_encoder)
			{
				Ring_state=In_Ring; //准备分大小环pd与速度
				if(Ring_50==1)
				{
					Ring_Speed=11;
					Pid_Ring_flag=1;
				}
				if(Ring_60==1)
				{
					Ring_Speed=21;
					Pid_Ring_flag=2;
				}
				if(Ring_90==1)
				{
					Ring_Speed=31;
					Pid_Ring_flag=3;
				}
			}
			break;
		case In_Ring:
			if(abs(Angle_Ring-Angle_gz)>Ready_Out_Ring_angle && Ring_left==1)
			{
				Ring_state=Ready_Out_Ring;
				if(Ring_50==1){Ring_Speed=12;}
				if(Ring_60==1){Ring_Speed=22;}
				if(Ring_90==1){Ring_Speed=32;}
				encoder_Ring=encoder_integral;
			}

			if(abs(Angle_Ring-Angle_gz)>Ready_Out_Ring_angle && Ring_right==1)
			{
				Ring_state=Ready_Out_Ring;
				if(Ring_50==1){Ring_Speed=12;}
				if(Ring_60==1){Ring_Speed=22;}
				if(Ring_90==1){Ring_Speed=32;}
				encoder_Ring=encoder_integral;
			}

			break;
		case Ready_Out_Ring:
			if(Ring_left==1)
			{
				if(abs(Angle_Ring-Angle_gz)>Out_Ring_angle)
				{
					Ring_state=Out_Ring;
					Ring_Speed=0;
					Pid_Ring_flag=0;
					encoder_Ring=encoder_integral;
				}
			}
			if(Ring_right==1)
			{
				if(abs(Angle_Ring-Angle_gz)>Out_Ring_angle)
				{
					Ring_state=Out_Ring;
					Ring_Speed=0;
					Pid_Ring_flag=0;
					encoder_Ring=encoder_integral;
				}
			}
			break;
		case Out_Ring:
			if(Ring_left==1)
			{
				if(abs(encoder_Ring-encoder_integral)>No_Ring_encoder)// || Trk.left_qulu<8 || Trk.right_qulu<8
				{
					Ring_state=Ready_No_Ring;
				}
			}
			if(Ring_right==1)
			{
				if(abs(encoder_Ring-encoder_integral)>No_Ring_encoder)
				{
					Ring_state=Ready_No_Ring;
				}
			}
			break;			
    case Ready_No_Ring:
      Ring_stop=1;
      Ring_left=0;
      Ring_right=0;
      Angle_gz=0;
			encoder_Ring=0;
			encoder_integral=0;
			beep_off();
			Ring_Speed=0;
			Pid_Ring_flag=0;
      Ring_state=No_Ring;
		break;
	}
}

uint8 Ramp_flag=0;
uint16 encoder_ramp=0;

//ui
uint8 Find_Ramp_dis=0;
uint8 Find_Ramp_width1=0;
uint8 Find_Ramp_width2=0;
uint8 Find_Ramp_qulv=0;
uint8 Up_Ramp_encoder=0;
uint8 Out_Ramp_encoder=0;

void Ramp()
{

	if(Ramp_flag==0 && Trk.Width1>Find_Ramp_width1 && Trk.Width2>Find_Ramp_width2 && Trk.right_qulu<Find_Ramp_qulv && Trk.left_qulu<Find_Ramp_qulv && distance<=Find_Ramp_dis
		&& CCD1_left_flag==1 && CCD1_right_flag==1 && CCD2_left_flag==1 && CCD2_right_flag==1 && black_write_2==0 && black_write_1==0)
	{
		Ramp_flag=1;
		beep_on();
		encoder_integral=100;
		encoder_ramp=encoder_integral;
	}
	//上坡到下坡
	if(Ramp_flag==1 && abs(encoder_ramp-encoder_integral)>Up_Ramp_encoder)
	{
		Ramp_flag=2;
		encoder_ramp=encoder_integral;
	}
	//下坡 出坡 清标志位
	if(Ramp_flag==2 && abs(encoder_ramp-encoder_integral)>Out_Ramp_encoder)
	{
		Ramp_flag=0;
		encoder_ramp=0;
		encoder_integral=0;
		beep_off();
	}	
}

uint8 Cross_flag=0;
uint16 encoder_cross=0;

//ui
uint8 Cross_encoder=0;

void Cross()
{
	if(Cross_flag==0 && CCD1_left_flag==0 && CCD1_right_flag==0 && black_write_1==0 && black_write_2==0)
	{
		Cross_flag=1;
		beep_on();
		encoder_integral=100;
		encoder_cross=encoder_integral;
	}
	if(Cross_flag==1 && abs(encoder_cross-encoder_integral)>Cross_encoder)
	{
		Cross_flag=0;
		beep_off();
		encoder_integral=0;
		encoder_cross=0;
	}
}

uint8 zebra_1=0;
uint8 Zebra_flag=0;
uint16 Zebra_encoder=0;

//ui
uint8 Out_Zebra_encoder=0;

void Zebra()
{
	//z+ebra_1=0;
	for(i=Trk.left_sideline1+5;i<=Trk.right_sideline1-5;i++)
	{
		if((uint32)(abs(ccd_data_ch1[i]-ccd_data_ch1[i+3])*100)/(ccd_data_ch1[i]+ccd_data_ch1[i+3])>CCD1.threshold)
		{
			zebra_1++;
		}
	}	
	
	if(zebra_1>=8 && Zebra_flag==0)
	{
		Zebra_flag=1;
		encoder_integra2=500;
		Zebra_encoder=encoder_integra2;
	}
	else if(zebra_1<8)
	{zebra_1=0;}
	
	if(Zebra_flag==1 && abs(Zebra_encoder-encoder_integra2)>Out_Zebra_encoder)
	{
		Zebra_flag=2;
		beep_on();
	}
}	

void element()
{	
	if(Cross_flag==0 && Ramp_flag==0)
	{
		Ring_all();
		Ring();
	}
	if(Cross_flag==0 &&Ring_state==0)Ramp();
	if(Ramp_flag==0 && Ring_state==0)Cross();

}