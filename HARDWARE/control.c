#include "control.h"






/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity
入口参数：左轮速度、右轮速度
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity(int velocity_left,int velocity_right)
{  
      static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Target_Velocity=5000;
	  static float Encoder_Integral;  
    
		if(1==Flag_Qian)    	Movement=Target_Velocity;	           //===前进标志位置1 
		else if(1==Flag_Hou)	Movement=-Target_Velocity;           //===后退标志位置1
        else  Movement=0;
    
//	  if(Bi_zhang==1&&Distance<500&&Flag_Left!=1&&Flag_Right!=1)        //避障标志位置1且非遥控转弯的时候，进入避障模式
//    Movement=-Target_Velocity;
   //=============速度PI控制器=======================//	
    
		Encoder_Least=Mean_Filter(velocity_left,velocity_right);          //速度滤波  
		Encoder *= 0.7;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 
		Encoder_Integral +=Movement;                                      //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>320000)  	Encoder_Integral=320000;            //===积分限幅
		if(Encoder_Integral<-320000)	Encoder_Integral=-320000;              //===积分限幅	
		Velocity=Encoder*Velocity_Kp/100+Encoder_Integral*Velocity_Ki/100;     //===速度控制	
  	    if(Turn_Off(Angle_Balance)==1)    Encoder_Integral=0;      //===电机关闭后清除积分
	  return Velocity;
}

/**************************************************************************
函数功能：速度滤波
入口参数：速度
返回  值：滤波后的速度
**************************************************************************/
int Mean_Filter(int moto1,int moto2)
{
    u8 i;
    s32 Sum_Speed = 0; 
    s16 Filter_Speed;
    
  static  s16 Speed_Buf[FILTERING_TIMES]={0};
  
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Speed_Buf[i - 1] = Speed_Buf[i];
  }
  Speed_Buf[FILTERING_TIMES - 1] =moto1+moto2;

  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Speed += Speed_Buf[i];
  }
  Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);
	return Filter_Speed;
}




/**************************************************************************
函数功能：获取角度 
入口参数：获取角度的算法 1：DMP 
返回  值：无
**************************************************************************/
void Get_Angle(void) 
{
    
        Read_DMP();
        Angle_Balance=-Roll;             //===更新平衡倾角
        Gyro_Balance=gyro[0];             //===更新平衡角速度
        Acceleration_Z=accel[2];         //===更新Z轴加速度计
//        printf("Pitch: %f ,%f ，%f\n\r", Angle_Balance,Gyro_Balance,Acceleration_Z);

}

/**************************************************************************
函数功能：直立PD控制
入口参数：角度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balance(float Angle ,float Gyro)
{  
     float Bias;    //这里D为零
	 int balancel;
	 Bias=Angle-1;                          //===求出平衡的角度中值 和机械相关
	 balancel=Balance_Kp*Bias+Balance_Kd*Gyro;      //===计算平衡控制的电机PWM 
	 return balancel;
}

/**************************************************************************
函数功能：自适应中值
入口参数：无
返回  值：无
**************************************************************************/
void  Get_Zhongzhi(void)
{
	    static int  count;
        if(Flag_Stop==0 && Flag_Zhongzhi==0)
		 {
		    if(myabs(Moto1)<100 && myabs(Moto2)<100)  		count++;		//采样
				else			                        	count=0;	
				if(count>100)  //连线3秒处于平衡位置，读取中值
				{	
				Zhongzhi=(int)Angle_Balance;	
				Flag_Zhongzhi=1;
				}
		 }
}


/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(float angle)
{
    u8 temp;
    if(angle<-40||angle>40)
        {	                                                 //===倾角大于40度关闭电机
            temp=1;                                            //===Flag_Stop置1关闭电机
            Moto1=0;
            Moto2=0;		
            EN=1;	
            Flag_Stop=1;
        HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
        HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_2);

        }
    else
    {	
        temp=0;
        EN=0;
        HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
        HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_2);

    }
    return temp;
}


/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
        int Amplitude_H=5000, Amplitude_L=-5000;       
        if(Moto1<Amplitude_L)  Moto1=Amplitude_L;	
		if(Moto1>Amplitude_H)  Moto1=Amplitude_H;	
        if(Moto2<Amplitude_L)  Moto2=Amplitude_L;	
		if(Moto2>Amplitude_H)  Moto2=Amplitude_H;	
}

/**************************************************************************
函数功能：赋值给PWM寄存器,并且判断转向
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
      if(moto1>0)		   Right_Direction=1;
			else 	       Right_Direction=0;
	  if(moto2>0)		   Left_Direction=0;
			else 	       Left_Direction=1;
    

	    Final_Moto1=Linear_Conversion(moto1);  //线性化
    	Final_Moto2=Linear_Conversion(moto2);
}

/**************************************************************************
函数功能：对控制输出的PWM线性化,便于给系统寄存器赋值
入口参数：PWM
返回  值：线性化后的PWM
**************************************************************************/
float Linear_Conversion(int moto)
{ 
	 float temp;
     float Linear_Moto;
    
        temp=36000000.0/(PRESCALER+1)/13000.0*5000.0/myabs(moto);
	 if(temp>65535.0) Linear_Moto=65535.0;
	 else if(temp<0.0) Linear_Moto=0.0;
	 else Linear_Moto=temp;
	 return Linear_Moto;
}	


/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
long long myabs(int a)
{ 		   
	  long long temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

