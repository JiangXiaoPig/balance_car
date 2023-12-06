#include "exti.h"

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;



void EXTI9_5_IRQHandler(void)
{
 // ���ȼ���Ƿ���GPIO_PIN_5�������ж�
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET) 
    {   
          // ����жϱ�־
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);

        Flag_Stop=0; 
        Get_Angle();   
        
        Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                     //===ƽ�����	

        Velocity_Pwm=velocity(Moto1,Moto2);                                   //===�ٶȻ�PI����	 �ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��    
        Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                              //===�������ֵ������PWM
        Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                              //===�������ֵ������PWM
        Xianfu_Pwm();

            printf("Pitch: %d ,%d \n\r", Moto1,Moto2);
               
        if(Turn_Off(Angle_Balance)==0)                                //===����������쳣
        {

            Set_Pwm(Moto1,Moto2); 


            led_on;
        }

        else
	     Set_Pwm(0,0);           
         Get_Zhongzhi();		 
      

   	}       	    
}


