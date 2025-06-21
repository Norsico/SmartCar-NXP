#include "headfile.h"
#include "math.h"

float  AD_Right=0,AD_Lift=0;
float  RightAverage=0,LeftAverage=0;
float  distance,distance_L,distance_R;


void infrared_detection()
{  
	int16   RightADC[20],LeftADC[20]; 
	int i;
	int Right_Sum=0,Left_Sum=0;
	for(i=0;i<20;i++)
	{   
		LeftADC[i]=adc_once(ADC_P11, ADC_12BIT);   
		Left_Sum+=LeftADC[i];
		RightADC[i]=adc_once(ADC_P14, ADC_12BIT);  
		Right_Sum+=RightADC[i];
	}
	LeftAverage=Left_Sum/20;
	RightAverage=Right_Sum/20; 
	AD_Lift =LeftAverage*3.30/4096;
	AD_Right=RightAverage*3.30/4096;
 
	distance_L=26.481*pow(AD_Lift,(-1.05));   
	distance_R=26.481*pow(AD_Right,(-1.05));   

	if(distance_L>150.00) distance_L=150.00;        
	if(distance_L<20.00) distance_L=20.00;
	if(distance_R>150.00) distance_R=150.00;        
	if(distance_R<20.00) distance_R=20.00;
		
	distance=(distance_L+distance_R)*0.5;
}