#include "stm32f4xx.h"
#include "system_timetick.h"
#include "DRIVER.h"
#include "fuzzyController.h"



extern double SP, distance, Output;
extern double error;
extern uint8_t Data_Recieve[BUFF_SIZE];
int k = 0, h = 0;
double out_fuzzy = 0;

void controller_simple(double e);

int main(void)
	{
		init_main();
		SysTick_Config(SystemCoreClock/200);
		SP = 50.0;
	while(1)
	{
		if((Data_Recieve[1]=='+') || (Data_Recieve[1]=='-'))
		//if(1)
		{
				if (tick_flag)
			{
				tick_flag = 0;
	//			if(h<50)
	//				{
	//					Deg2Pulse((uint32_t)out_fuzzy);
	//					h++;
	//				}
	//			else
	//				{
	//					out_fuzzy = controller(error, 0.005);
	//					h=0;
	//				}
				if (k<100)
				{
					PID_SpeedController(0.005,0.45,0.000001,0.000000005,1000, SP);
					k++;
				}
				else
				{
					k = 0;
					Output = distance;
				};
				out_fuzzy = controller(error, 0.005);
				Deg2Pulse((uint32_t)out_fuzzy);
				
	//			controller_simple(error);
			}
		}
	}
//		while(1)
//			{
//				PWM0_Set_Freq(1000);
//				PWM0_Set_Duty(-1000);
//				}
		
}
	
void controller_simple(double e)
{
	uint32_t result;
	if(e>229)
		result = 72;
	else if (e>138)
		result = 86;
	else if (e>47)
		result = 101;
	else if (e>-44)
		result = 115;
	else if (e>-135)
		result = 131;
	else if (e>-226)
		result = 146;
	else
		result = 162;
	Deg2Pulse(result);
}
