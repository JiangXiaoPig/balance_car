#include "exti.h"

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;



int EXTI9_5_IRQHandler(void)
{
 // ���ȼ���Ƿ���GPIO_PIN_5�������ж�
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET) 
    {   
          // ����жϱ�־
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
        
        
        HAL_GPIO_WritePin(bsq_led_GPIO_Port, bsq_led_Pin, GPIO_PIN_RESET);

       Flag_Stop=0;
        
        Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                     //===ƽ�����	
//        Velocity_Pwm=velocity(Moto1,Moto2);                                   //===�ٶȻ�PI����	 �ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��    
      
//        Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                              //===�������ֵ������PWM
//        Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                              //===�������ֵ������PWM
       Moto1=Balance_Pwm;
        Moto2=Balance_Pwm;
        if(Turn_Off(Angle_Balance)==0)                                //===����������쳣
         MotorControl(Moto1,Moto2); 

        else
        MotorControl(0,0);  
        Get_Zhongzhi();

   	}     
//        printf("%d, %d \n\r",Balance_Pwm,Velocity_Pwm);    

//    printf("data:%d,%d,%f\n\r",Moto1,Moto2,Angle_Balance,Gyro_Balance);    
//     printf("data:%d,%fd\n\r",Angle_Balance,Gyro_Balance);    
     return 0;	  
}


