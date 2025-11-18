#ifndef _KALMAN_H_
#define _KALMAN_H_

extern float Gyro_z;                        //Z轴陀螺仪数据暂存
extern float Angle_gz;    //由角速度计算的倾斜角度


void Angle_Calcu(void);

#endif