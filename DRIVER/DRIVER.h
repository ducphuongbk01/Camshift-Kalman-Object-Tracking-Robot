/*****THIET LAP DRIVER CHO PROJECT*****/
/*****KHAI BAO PROTOTYPE CUA CAC HAM*****/

#include "stm32f4xx.h"
#include "system_timetick.h"

#define		BUFF_SIZE			6


//Ham khoi tao chuc nang
void init_main(void);

//Ham set tan so PWM
void PWM0_Set_Freq(uint32_t freq);
//void PWM2_Set_Freq(uint32_t freq);
//void PWM3_Set_Freq(uint32_t freq);

//Ham set duty cycle
void PWM0_Set_Duty(int32_t d);
//void PWM2_Set_Duty(int32_t d);
//void PWM3_Set_Duty(int32_t d);

void Deg2Pulse(uint32_t b);

//Ham doc vi tri encoder
int32_t Encoder_GetPosition(void);
//int32_t Encoder1_GetPosition(void);

//Ham gui du lieu thong qua DMA UART
//void SendData (double ydat, double ydo,double udk);

//Ham chuyen ma sang ASCII (NEU CAN)
double chuyenma (uint8_t*code, uint8_t num_b);

//Ham DELAY us
void delay_us(uint16_t period);

//Ham DELAY ms
void delay_01ms(uint16_t period);

//PID Controller
void PID_SpeedController(double Ts,double dkp,double dki,double dkd,int32_t freq, double setPoint);

//Ngat nhan UART4
//void UART4_IRQHandler(void);

