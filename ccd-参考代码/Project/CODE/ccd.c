#include "headfile.h"

int i=0;

CCDInformation CCD1 = {0};//CCD1原始图像相关数据
CCDInformation CCD2 = {0};//CCD2原始图像相关数据

TrackInformation Trk = {0};//赛道信息

uint8 Threshold_multiple_1=0,Threshold_multiple_2=0;
uint8 Threshold_1=0,Threshold_2=0;

void CCDimage_init()
{
	Trk.middle_sideline1=63;
	Trk.middle_sideline2=63;
	CCD1.bin_thrd=0;
	CCD2.bin_thrd=0;
}

void CCD1_get()
{
	CCD1.max=0;CCD1.min=ccd_data_ch1[4];
	CCD1.aver=0;
	for(i=5;i<=122;i++)
	{
		if(ccd_data_ch1[i]>CCD1.max)CCD1.max=ccd_data_ch1[i];
		if(ccd_data_ch1[i]<CCD1.min)CCD1.min=ccd_data_ch1[i];
	}
	for(i=48;i<78;i++)
	{
		CCD1.aver=CCD1.aver+(ccd_data_ch1[i]/30);
	}
	if(CCD1.bin_thrd == 0){CCD1.bin_thrd = 1;}
  else if(CCD1.bin_thrd == 1)
  {CCD1.bin_thrd = (uint32)(CCD1.aver*Threshold_1/100);}
	CCD1.threshold=(((uint32)(CCD1.max-CCD1.min)*100)/(CCD1.max+CCD1.min))*Threshold_multiple_1/100;
}

void CCD2_get()
{
	CCD2.max=0;CCD2.min=ccd_data_ch2[4];
	CCD2.aver=0;
	for(i=10;i<=117;i++)
	{
		if(ccd_data_ch2[i]>CCD2.max)CCD2.max=ccd_data_ch2[i];
		if(ccd_data_ch2[i]<CCD2.min)CCD2.min=ccd_data_ch2[i];
	}
	for(i=53;i<73;i++)
	{
		CCD2.aver=CCD2.aver+(ccd_data_ch2[i]/20);
	}
	if(CCD2.bin_thrd == 0){CCD2.bin_thrd = 1;}
  else if(CCD2.bin_thrd == 1)
  {CCD2.bin_thrd = (uint32)(CCD2.aver*Threshold_2/100);}
	CCD2.threshold=(((uint32)(CCD2.max-CCD2.min)*100)/(CCD2.max+CCD2.min))*Threshold_multiple_2/100;
}
uint8 CCD1_left_flag=0;/
uint8 CCD1_right_flag=0;
uint8 CCD2_left_flag=0;
uint8 CCD2_right_flag=0;

uint8 black_write_1=0;
uint8 black_write_2=0;

void left_right_sideline()
{
	Trk.middle_sideline1_last=Trk.middle_sideline1;
	Trk.middle_sideline2_last=Trk.middle_sideline2;
	
	for(i=Trk.middle_sideline1_last;i>5;i--)
	{
		if((abs((uint32)(ccd_data_ch1[i]-ccd_data_ch1[i-5]))*100/(ccd_data_ch1[i]+ccd_data_ch1[i-5]))>CCD1.threshold 
			&& ccd_data_ch1[i]>ccd_data_ch1[i-5])
		{
			Trk.left_sideline1=i;
			CCD1_left_flag=1;  //找到邊界
			break;
		}
		else 
		{
			Trk.left_sideline1=5;
			CCD1_left_flag=0;  //没找到边界
		}
	}
	for(i=Trk.middle_sideline1_last;i<122;i++)
	{
		if((abs((uint32)(ccd_data_ch1[i]-ccd_data_ch1[i+5]))*100/(ccd_data_ch1[i]+ccd_data_ch1[i+5]))>CCD1.threshold
			&& ccd_data_ch1[i]>ccd_data_ch1[i+5])
		{
			Trk.right_sideline1=i;
			CCD1_right_flag=1; //找到邊界
			break;
		}
		else 
		{
			Trk.right_sideline1=122;
			CCD1_right_flag=0;//没找到边界
		}
	}	
	
	if(CCD1.aver<CCD1.bin_thrd)
	{
		black_write_1=1;
	}
	else black_write_1=0;
	
	if(CCD1_left_flag==1&&CCD1_right_flag==0)
	{
		for(i=Trk.left_sideline1;i<122;i++)
		{
			if((abs((uint32)(ccd_data_ch1[i]-ccd_data_ch1[i+5]))*100/(ccd_data_ch1[i]+ccd_data_ch1[i+5]))>CCD1.threshold
				&& ccd_data_ch1[i]>ccd_data_ch1[i+5])
			{
				Trk.right_sideline1=i;
				CCD1_right_flag=1;
				break;
			}
			else 
			{
				Trk.right_sideline1=122;
				CCD1_right_flag=0;
			}
		}	
	}
	else if(CCD1_left_flag==0 && CCD1_right_flag==1)
	{
		for(i=Trk.right_sideline1;i>5;i--)
		{
			if((abs((uint32)(ccd_data_ch1[i]-ccd_data_ch1[i-5]))*100/(ccd_data_ch1[i]+ccd_data_ch1[i-5]))>CCD1.threshold
				&& ccd_data_ch1[i]>ccd_data_ch1[i-5])
			{
				Trk.left_sideline1=i;
				CCD1_left_flag=1;
				break;
			}
			else 
			{
				Trk.left_sideline1=5;
				CCD1_left_flag=0;
			}
		}
	}
	
	for(i=Trk.middle_sideline2_last;i>10;i--)
	{
		if((abs((uint32)(ccd_data_ch2[i]-ccd_data_ch2[i-5]))*100/(ccd_data_ch2[i]+ccd_data_ch2[i-5]))>CCD2.threshold
			&& ccd_data_ch2[i]>ccd_data_ch2[i-5])
		{
			Trk.left_sideline2=i;
			CCD2_left_flag=1;
			break;
		}
		else
		{
			Trk.left_sideline2=10;
			CCD2_left_flag=0;
		}
	}
	for(i=Trk.middle_sideline2_last;i<117;i++)
	{
		if((abs((uint32)(ccd_data_ch2[i]-ccd_data_ch2[i+5]))*100/(ccd_data_ch2[i]+ccd_data_ch2[i+5]))>CCD2.threshold
			&& ccd_data_ch2[i]>ccd_data_ch2[i+5])
		{
			Trk.right_sideline2=i;
			CCD2_right_flag=1;
			break;
		}
		else
		{
			Trk.right_sideline2=117;
			CCD2_right_flag=0;
		}			
	}		
	
	if(CCD2_left_flag==1&&CCD2_right_flag==0)
	{
		for(i=Trk.left_sideline2;i<117;i++)
		{
			if((abs((uint32)(ccd_data_ch2[i]-ccd_data_ch2[i+10]))*100/(ccd_data_ch2[i]+ccd_data_ch2[i+10]))>CCD2.threshold
				&& ccd_data_ch2[i]>ccd_data_ch2[i+10])
			{
				Trk.right_sideline2=i;
				CCD2_right_flag=1;
				break;
			}
			else 
			{
				Trk.right_sideline2=117;
				CCD2_right_flag=0;
			}
		}	
	}
	else if(CCD2_left_flag==0&&CCD2_right_flag==1)
	{
		for(i=Trk.right_sideline2;i>10;i--)
		{
			if((abs((uint32)(ccd_data_ch2[i]-ccd_data_ch2[i-10]))*100/(ccd_data_ch2[i]+ccd_data_ch2[i-10]))>CCD2.threshold
				&& ccd_data_ch2[i]>ccd_data_ch2[i-10])
			{
				Trk.left_sideline2=i;
				CCD2_left_flag=1;
				break;
			}
			else 
			{
				Trk.left_sideline2=10;
				CCD2_left_flag=0;
			}
		}
	}	
		
	if(CCD2.aver<CCD2.bin_thrd)
	{
		black_write_2=1;
	}
	else black_write_2=0;	
	

}
void middle_sideline()
{	
	Trk.middle_sideline1=(Trk.left_sideline1+Trk.right_sideline1)/2;
	Trk.middle_sideline2=(Trk.left_sideline2+Trk.right_sideline2)/2;
	
	Trk.Width1=Trk.right_sideline1-Trk.left_sideline1;
	Trk.Width2=Trk.right_sideline2-Trk.left_sideline2;

	if(Cross_flag==1)
	{
		Trk.middle_sideline1=Trk.middle_sideline2;
	}
	
	if((Ring_state==1 || Ring_state==5) && Ring_left==1)//入环出环补线Ring_state==1 || 
	{
		Trk.middle_sideline1=Trk.right_sideline1-(CCD1_set_width/2);
	}
	if((Ring_state==2 || Ring_state==4) && Ring_left==1)
	{
		Trk.middle_sideline1=Trk.left_sideline1+(CCD1_set_width/2);
	}
	if((Ring_state==1 || Ring_state==5) && Ring_right==1)//入环出环补线Ring_state==1 ||
	{
		Trk.middle_sideline1=Trk.left_sideline1+(CCD1_set_width/2);
	}
	if((Ring_state==2 || Ring_state==4) && Ring_right==1) 
	{
		Trk.middle_sideline1=Trk.right_sideline1-(CCD1_set_width/2);
	}

}

void CCD_Processing()
{
  CCD1_get();
  CCD2_get();

  left_right_sideline();
	
	if(encoder_integral>25)
	{
		element();
	}
	
  middle_sideline();
	
}