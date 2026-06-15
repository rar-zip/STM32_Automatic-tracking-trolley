#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "robot.h"
#include "Key.h"



uint8_t i;

int main(void)
{
	robot_Init();    
	Key_Init();      
	while (1)
	{
		if(Key_GetNum() == 1)
		{
		 makerobo_run(80,10000); 
		 makerobo_Left(70,6000);
		 makerobo_brake(500);//ֹͣ0.5S
   	}
	}
}
