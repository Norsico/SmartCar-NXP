#include "headfile.h"
#include "math.h"
//float Accel_x;       //X轴加速度值暂存
//float Accel_y;       //Y轴加速度值暂存
//float Accel_z;       //Z轴加速度值暂存

//float Gyro_x;        //X轴陀螺仪数据暂存
//float Gyro_y;        //Y轴陀螺仪数据暂存
float Gyro_z;        //Z轴陀螺仪数据暂存

//float Angle_gx=0;    //由角速度计算的倾斜角度
//float Angle_gy=0;    //由角速度计算的倾斜角度
float Angle_gz=0;    //由角速度计算的倾斜角度

//float Angle_x_temp;  //由加速度计算的x倾斜角度
//float Angle_y_temp;  //由加速度计算的y倾斜角度
//float Angle_z_temp;  //由加速度计算的z倾斜角度

//float Angle_X_Final; //X最终倾斜角度
//float Angle_Y_Final; //Y最终倾斜角度
//float Angle_Z_Final; //Z最终倾斜角度
//角度计算
void Angle_Calcu(void)
{
    //范围为2g时，换算关系：16384 LSB/g
//    float rad;
//    float deg = rad*180/3.14;
//    float x,y,z;
                                  // 获取 IMU660RA 加速度计数据
		imu660ra_get_gyro();  

//	  Accel_x = imu660ra_acc_x;    //x轴加速度值暂存
//    Accel_y = imu660ra_acc_y;    //y轴加速度值暂存
//    Accel_z = imu660ra_acc_z;    //z轴加速度值暂存
//    Gyro_x  = imu660ra_gyro_x;  //x轴陀螺仪值暂存
//    Gyro_y  = imu660ra_gyro_y;  //y轴陀螺仪值暂存
    Gyro_z  = imu660ra_gyro_z;  //z轴陀螺仪值暂存

//    //处理x轴加速度
//    if(Accel_x<32764) x=Accel_x/16384;
//    else              x=1-(Accel_x-49152)/16384;

//    //处理y轴加速度
//    if(Accel_y<32764) y=Accel_y/16384;
//    else              y=1-(Accel_y-49152)/16384;

//    //处理z轴加速度
//    if(Accel_z<32764) z=Accel_z/16384;
//    else              z=(Accel_z-49152)/16384;

//    //用加速度计算三个轴和水平面坐标系之间的夹角
//    Angle_x_temp=(atan(y/z))*180/3.1415926;
//    Angle_y_temp=(atan(x/z))*180/3.1415926;
//    Angle_z_temp=(atan(x/y))*180/3.1415926;

//    //角度的正负号
////    if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
////    if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
////    if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
////    if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;
//    if(Accel_z<32764) Angle_z_temp = +Angle_z_temp;
//    if(Accel_z>32764) Angle_z_temp = -Angle_z_temp;

    //角速度
    //向前运动
//    if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
//    //向后运动
//    if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
//    //向前运动
//    if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
//    //向后运动
//    if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
    //向前运动
    if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
    //向后运动
    if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;

   
    Angle_gz = Angle_gz + Gyro_z*0.005;  //角速度积分得到倾斜角度.越大积分出来的角度越大
//		Angle_gy = Angle_gy + Gyro_y*0.005;  //角速度积分得到倾斜角度.越大积分出来的角度越大
}