/*****THIET LAP DRIVER CHO PROJECT*****/
/*****DINH NGHIA CAC HAM O FILE.C*****/

/*
PWM0: TIM12 --> PB14		SERVO DIEU TOC
PWM1: TIM9-->PE5        SERVO DIEU HUONG
PWM2: TIM2 --> PA15			DU TRU
PWM3: TIM4 --> PD12			DU TRU

UART4 --> PA0; PA1
UART5 --> PC12; PD2			DU TRU

ENCODER0 TIM1 --> PE9; PE11		PHONG HO
ENCODER1 TIM8 --> PC6; PC7		PHONG HO

Output: PB15(DIR)			; PD10(ENA/B)

Ultra Sonic Sensor:
Timer update output --> TIM5
Timer measure time --> TIM3 --> PC9 (Echo)
Output: Trigger: PB9

MG966R 
f = 50Hz

*/

// Them cac thu vien can thiet
#include "stm32f4xx.h"                  // Device header
#include "system_timetick.h"
#include "DRIVER.h"
#include <stdio.h>
#include <math.h>

#define		BUFF_SIZE	6

//Khoi tao cac ham con 

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

//PID Controller
void PID_SpeedController(double Ts,double dkp,double dki,double dkd,int32_t freq, double setPoint);

//Tao noi chua du lieu can gui
//static uint8_t txbuff[22];

//Tao noi chua du lieu can nhan
uint8_t Data_Recieve[BUFF_SIZE];
int i = 0;

//Khai bao cac bien can su dung

int check = 0;
int16_t time = 0;
double distance = 0;
uint32_t a;

// Khai bao cac bien cho bo PID
double Kp, Ki, Kd;
double SP = 0;
double Output = 0;
double ek , ek_1 = 0, ek_2 = 0,u=0, u_1 = 0;
double pre_Pulse = 0,Pulse;
int32_t UDK;

//Bien sai so goc quay
double error = 0.0;




/*****Cau hinh ham khoi tao chuc nang******/
void init_main(void)
	{
//=================================================================================================================================================================	
		//Khoi tao cac bien cau hinh
		GPIO_InitTypeDef 					GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
		TIM_OCInitTypeDef 				TIM_OCInitStructure;
		USART_InitTypeDef 				USART_InitStructure;   
		DMA_InitTypeDef  					DMA_InitStructure;
		NVIC_InitTypeDef  				NVIC_InitStructure;
		TIM_ICInitTypeDef  				TIM_ICInitStructure;
		
		
//==================================================================================================================================================================	
		/********PWM_0********/
	
		//Enable xung clock cho TIMER
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE); 					//Su dung TIMER 12 de tao xung CLOCK
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 					//Su dung chan PB14 de lam ngo ra xung PWM
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 					//Su dung chan PD10 de lam ngo ra ENA
		
		//Cau hinh TIMER12 voi chuc nang PWM
		//B1: Cau hinh chan ngo ra PWM
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 											//Cau hinh chan 14
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;										//Do la mode TIMER nen ta chon mode cho chan là AF
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;							//Toc do 100MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									//Cau hinh ngo ra dang Push - Pull
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;										//Cau hinh ngo vao dang dien tro keo len
		GPIO_Init(GPIOB, &GPIO_InitStructure); 													//Luu cac thiet lap tren cho Port B
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12);				//Cau hình AF cho chan PB14 thuc hien chuc nang cua TIMER 12
		
		//B2: Cau hinh cho cac chan quy dinh CHIEU QUAY và ENABLE cho dong co
		//Chan PB15 lam chan QUY DINH CHIEU QUAY
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;										//Chan PB15 ta su dung la mode Output
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
		
		//Chan PD10 lam chan ENA
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;											//Cau hinh tuong tu cho chan PD10 (Khong can phai cau hinh lai, chi can set chan can cau hinh)
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		
		//B3: Cau hinh cho TIMER12
		TIM_TimeBaseStructure.TIM_Period = 1000-1;											//Thiet lap bo chia tan so: 2Mhz/1000 = 2kHz
		TIM_TimeBaseStructure.TIM_Prescaler = 42 - 1;										// 		84MHz/42= 2MHz						
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);
		
		//B4: Cau hinh cho TIMER12 PWM
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;								//Chon Mode PWM1
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//Cho phep xuat xung
		TIM_OCInitStructure.TIM_Pulse = 0;															//Khoi tao gia tri ban dau cho "Duty Cycle"	
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;				//Chon muc uu tien CAO
		TIM_OC1Init(TIM12, &TIM_OCInitStructure);												//Cau hinh PWM tren cho TIMER 12
		TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);							//Cho phep nap lai gia tri khi xong 1 chu ky 

		TIM_Cmd(TIM12, ENABLE);																					//Cho phep TIMER 12 chay
//=================================================================================================================================================================	
		
	
//=================================================================================================================================================================
		/********PWM_1********/
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 											
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;										
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;							
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;										
		GPIO_Init(GPIOE, &GPIO_InitStructure); 													
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
		
		TIM_TimeBaseStructure.TIM_Period = 1000-1;											//Thiet lap bo chia tan so: 2Mhz/1000 = 2kHz
		TIM_TimeBaseStructure.TIM_Prescaler = 3360;										// 		42Mh/21= 2MHz						
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
		
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;								
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		
		TIM_OCInitStructure.TIM_Pulse = 0;																
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;				
		TIM_OC1Init(TIM9, &TIM_OCInitStructure);												
		TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);							 

		TIM_Cmd(TIM9, ENABLE);
	
//==================================================================================================================================================================



//==================================================================================================================================================================
//	/******PWM_2*******/
//  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//		
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; 											
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;										
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;							
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;										
//		GPIO_Init(GPIOA, &GPIO_InitStructure); 													
//		GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
//		
//		TIM_TimeBaseStructure.TIM_Period = 1000-1;											//Thiet lap bo chia tan so: 2Mhz/1000 = 2kHz
//		TIM_TimeBaseStructure.TIM_Prescaler = 42 - 1;										// 		84MHz/42= 2MHz						
//		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//		
//		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;								
//		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		
//		TIM_OCInitStructure.TIM_Pulse = 0;																
//		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;				
//		TIM_OC1Init(TIM2, &TIM_OCInitStructure);												
//		TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);							 

//		TIM_Cmd(TIM2, ENABLE);

//==================================================================================================================================================================


//==================================================================================================================================================================
//	/******PWM_3*******/
//  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
//		
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 											
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;										
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;							
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;									
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;										
//		GPIO_Init(GPIOD, &GPIO_InitStructure); 													
//		GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
//		
//		TIM_TimeBaseStructure.TIM_Period = 1000-1;											//Thiet lap bo chia tan so: 2Mhz/1000 = 2kHz
//		TIM_TimeBaseStructure.TIM_Prescaler = 42 - 1;										// 		84MHz/42= 2MHz						
//		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
//		
//		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;								
//		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		
//		TIM_OCInitStructure.TIM_Pulse = 0;																
//		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;				
//		TIM_OC1Init(TIM4, &TIM_OCInitStructure);												
//		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);							 

//		TIM_Cmd(TIM4, ENABLE);
		
//==================================================================================================================================================================


//==================================================================================================================================================================		
		/******ENCODER_0******/
		//Su dung chuc nang ENCODER cua TIMER 1
		//Enable xung Clock cho TIMER 1 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);						//Su dung chuc nang ENCODER cua TIMER 1 				
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);						//Su dung chan PE9 & PE11 de lam no nhan tin hieu tu ENCODER
		
		//Cau hinh 2 chan Port
		//Su dung 2 kenh cua TIMER 1: channel 1 & 2 cho 2 kenh cua ENCODER A- B
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_11;				//Cau hinh cho ca 2 chan PE9 & PE11
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;										//Mode TIMER la mode AF
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);					//Cau hình AF cho chan PE9 thuc hien chuc nang cua TIMER 1
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);				//Cau hình AF cho chan PE11 thuc hien chuc nang cua TIMER 1
		
		//Cau hinh chuc nang ENCODER cho TIMER
		TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
		
		//Khoi tao gia tri ban dau va cho phep TIMER chay
		TIM1->CNT = 0;																									//Khoi tao gia tri ban dau cho bo dem TIMER
		TIM_Cmd(TIM1, ENABLE);																					//Cho phep TIMER chay
//=================================================================================================================================================================	


//==================================================================================================================================================================		
//   	/******ENCODER_1******/
//		//Su dung chuc nang ENCODER cua TIMER 1
//		//Enable xung Clock cho TIMER 1 
//  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);						//Su dung chuc nang ENCODER cua TIMER 1 				
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);						//Su dung chan PE9 & PE11 de lam no nhan tin hieu tu ENCODER
//		
//		//Cau hinh 2 chan Port
//		//Su dung 2 kenh cua TIMER 1: channel 1 & 2 cho 2 kenh cua ENCODER A- B
//		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;				//Cau hinh cho ca 2 chan PE9 & PE11
//		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;										//Mode TIMER la mode AF
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//		GPIO_Init(GPIOC, &GPIO_InitStructure);
//		
//		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM1);					//Cau hình AF cho chan PE9 thuc hien chuc nang cua TIMER 1
//		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM1);				//Cau hình AF cho chan PE11 thuc hien chuc nang cua TIMER 1
//		
//		//Cau hinh chuc nang ENCODER cho TIMER
//		TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//		
//		//Khoi tao gia tri ban dau va cho phep TIMER chay
//		TIM8->CNT = 0;																									//Khoi tao gia tri ban dau cho bo dem TIMER
//		TIM_Cmd(TIM8, ENABLE);																					//Cho phep TIMER chay
		
//=================================================================================================================================================================	
	
	
	
//=================================================================================================================================================================	
		/********USART0********/
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		/* Enable UART clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		/* Enable DMA1 clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

		/* GPIO Configuration for UART4 Tx */
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* GPIO Configuration for USART Rx */
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		
		/* Connect UART4 pins to AF2 */  
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); 
       
		/* USARTx configured as follow:
			- BaudRate = 115200 baud  
			- Word Length = 8 Bits
			- One Stop Bit
			- No parity
			- Hardware flow control disabled (RTS and CTS signals)
			- Receive and transmit enabled
		*/
		USART_InitStructure.USART_BaudRate = 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		USART_Init(UART4, &USART_InitStructure);
	
		/* Enable USART */
		USART_Cmd(UART4, ENABLE);
	
		/* Enable UART4 DMA */
		//USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);	 
		USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);	 
	
	
		/* DMA1 Stream4 Channel4 for UART4 Tx configuration */			
//		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
//		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR; 																	// dia chi nguon
//		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txbuff; 																				//dia chi dich
//		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; 																					//chon mode DMA: memory to peripheral
//		//DMA_InitStructure.DMA_BufferSize = BUFF_SIZE; 																									//Co du lieu = 5
//		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 																	//dia chi ngoai vi khong tang
//		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 																					//dia chi bo nho tang
//		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;														//data size ngoai vi 8 bit
//		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;																		//data size memory 8 bit
//		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 																										// mode normal: tang dia chi o nho lien tuc de chep  // mode circular: tang de khi dung tran thì reset lai dia chi dau 
//		DMA_InitStructure.DMA_Priority = DMA_Priority_High; 																							// Uu tien DMA
//		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; 																						//tat fifo        
//		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 																//tat fifo
//		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
//		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//		DMA_Init(DMA1_Stream4, &DMA_InitStructure);
//		//DMA_Cmd(DMA1_Stream4, ENABLE);
		
		/* DMA1 Stream2 Channel4 for USART4 Rx configuration */			
		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Data_Recieve;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream2, &DMA_InitStructure);
		DMA_Cmd(DMA1_Stream2, ENABLE);
		
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
		
		/* Enable DMA Interrupt to the highest priority */
		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* Transfer complete interrupt mask */
		DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
//=================================================================================================================================================================

		
		
//=================================================================================================================================================================	
//		/********USART1********/
//		/* Enable GPIO clock */
//  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//		/* Enable UART clock */
//		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
//		/* Enable DMA1 clock */
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

//			/* GPIO Configuration for UART4 Tx */
//		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
//		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_Init(GPIOC, &GPIO_InitStructure);

//		/* GPIO Configuration for USART Rx */
//		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
//		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_Init(GPIOD, &GPIO_InitStructure);
//		
//			/* Connect UART5 pins to AF2 */  
//		GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
//		GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5); 

//       
//		/* USARTx configured as follow:
//			- BaudRate = 115200 baud  
//			- Word Length = 8 Bits
//			- One Stop Bit
//			- No parity
//			- Hardware flow control disabled (RTS and CTS signals)
//			- Receive and transmit enabled
//		*/
//		USART_InitStructure.USART_BaudRate = 115200;
//		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//		USART_InitStructure.USART_StopBits = USART_StopBits_1;
//		USART_InitStructure.USART_Parity = USART_Parity_No;
//		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

//		USART_Init(UART5, &USART_InitStructure);
//	
//		/* Enable USART */
//		USART_Cmd(UART5, ENABLE);
//	
//		/* Enable UART4 DMA */
//		USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE); 
//	
//	
//		/* DMA1 Stream4 Channel4 for UART4 Tx configuration */			
//		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
//		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART5->DR; 																	// dia chi nguon
//		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txbuff; 																				//dia chi dich
//		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; 																					//chon mode DMA: memory to peripheral
//		//DMA_InitStructure.DMA_BufferSize = BUFF_SIZE; 																									//Co du lieu = 5
//		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 																	//dia chi ngoai vi khong tang
//		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 																					//dia chi bo nho tang
//		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;														//data size ngoai vi 8 bit
//		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;																		//data size memory 8 bit
//		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 																										// mode normal: tang dia chi o nho lien tuc de chep  // mode circular: tang de khi dung tran thì reset lai dia chi dau 
//		DMA_InitStructure.DMA_Priority = DMA_Priority_High; 																							// Uu tien DMA
//		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; 																						//tat fifo        
//		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 																//tat fifo
//		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
//		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//		DMA_Init(DMA1_Stream7, &DMA_InitStructure);
//		//DMA_Cmd(DMA1_Stream4, ENABLE);
//		
//		/* DMA1 Stream2 Channel4 for USART5 Rx configuration */			
////		DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
////		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART5->DR;
////		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxbuff;
////		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
////		DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
////		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
////		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
////		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
////		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
////		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
////		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
////		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
////		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
////		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
////		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
////		DMA_Init(DMA1_Stream7, &DMA_InitStructure);
////		DMA_Cmd(DMA1_Stream7, ENABLE);
//		

//		/*Clear Flag for the first recieve*/
//		USART_ClearFlag(UART5,USART_IT_RXNE);
//		/* Transfer complete interrupt mask */
//		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	
//		/* Enable DMA Interrupt to the highest priority */
//		NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);

//==================================================================================================================================================================


//==================================================================================================================================================================

/*****************ULTRA SONIC SENSOR********************************/
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
		
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF;											
		TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;											
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
			
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
		/* Enable the TIM1 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;

		TIM_ICInit(TIM3, &TIM_ICInitStructure);
		
		/* TIM enable counter */
		TIM_Cmd(TIM3, ENABLE);

		/* Enable the CC2 Interrupt Request */
		TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
		
		//TIM5
		
		TIM_TimeBaseStructure.TIM_Period = 50000-1;											
		TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;											
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
			
		TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		/* Enable the TIM1 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		TIM_Cmd(TIM5,ENABLE);
		
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

//==================================================================================================================================================================


//==================================================================================================================================================================
//------------------------------------------   THE END OF INIT MAIN ---------------------------------------------------------------------------
}
	





//=================================================================================================================================================================
	
	
	
//=================================================================================================================================================================
	/********Tao cac ham DELAY can thiet***********/
void delay_us(uint16_t period)
		{

			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			TIM6->PSC = 83;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 1MHz
			TIM6->ARR = period-1;
			TIM6->CNT = 0;
			TIM6->EGR = 1;		// update registers;
	
			TIM6->SR  = 0;		// clear overflow flag
			TIM6->CR1 = 1;		// enable Timer6

			while (!TIM6->SR);
    
			TIM6->CR1 = 0;		// stop Timer6
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
		}
//=================================================================================================================================================================
		
		
//=================================================================================================================================================================
void delay_01ms(uint16_t period)
		{

			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
			TIM6->PSC = 8399;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 10KHz
			TIM6->ARR = period-1;
			TIM6->CNT = 0;
			TIM6->EGR = 1;		// update registers;

			TIM6->SR  = 0;		// clear overflow flag
			TIM6->CR1 = 1;		// enable Timer6
	
			while (!TIM6->SR);
    
			TIM6->CR1 = 0;		// stop Timer6
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
		}
//=================================================================================================================================================================
	
		
//=================================================================================================================================================================
	/***********Tao ham gui cac du lieu thu duoc***********/
//void SendData (double ydat, double ydo,double udk)
//		{
//			/*chuyenma(ydat);
//				txbuff[i++]=' ';
//				chuyenma(ydo);
//				txbuff[i++]=' ';
//				chuyenma(udk);
//				txbuff[i++]='\r';
//				txbuff[i++]='\n';*/
//				int Lengh_tx;
//				strcpy((char*)txbuff,"");																									//emty the array
//				sprintf(txbuff,"%.4f %.4f %.4f \r\n",ydat,ydo,udk);
//				Lengh_tx = strlen(txbuff);
//				DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4); 															// Phai xoa sau khi doc xong moi truyen tiep duoc
//				DMA1_Stream4->NDTR = Lengh_tx;																						// moi lan truyen thi co du lieu giam xuong 1 nen phai nap lai
//				DMA_Cmd(DMA1_Stream4, ENABLE); 																						// bat dau truyen
//		}
//=================================================================================================================================================================

			
//=================================================================================================================================================================
double chuyenma (uint8_t*code, uint8_t num_b)
{
	double z = 0;
	for(uint8_t j = 0; j<num_b; j++)
	{
		z+=(double)(code[j]-0x30)*pow(10.0,(double)(num_b - 1 - j));
	}
	return z;
}
//=================================================================================================================================================================
			
			
//=================================================================================================================================================================
			
//Ngat nhan Data
void DMA1_Stream2_IRQHandler(void)
{
  /* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
	if (Data_Recieve[0] == 0x02 && Data_Recieve[5] == 0x03)
	{
		if (Data_Recieve[1] == '+')
			error = chuyenma(&Data_Recieve[2],3);
		else if (Data_Recieve[1] == '-')
			error = -(chuyenma(&Data_Recieve[2],3));
		else
			error = error;
	}
	
	DMA_Cmd(DMA1_Stream2, ENABLE);
}
//=================================================================================================================================================================
		
		
//=================================================================================================================================================================
			
////Ngat nhan Data
//void UART5_IRQHandler(void)
//	 {
//			char k;
//			if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
//				{
//					/* Read one byte from the receive data register */
//					k = (USART_ReceiveData(UART5) & 0x7F);
//					if(k=='a' && i==0)
//						{
//							Data_Recieve[i++]=k;
//						}
//					else if(Data_Recieve[0]=='a'&&i>0)
//						{
//							Data_Recieve[i++]=k;
//						}
//					if(Data_Recieve[BUFF_SIZE-1] == 'b' && i==BUFF_SIZE)
//						{
//							i=0;
//						}
//				} 
//		}
//=================================================================================================================================================================
	
		
			
//=================================================================================================================================================================
			/******Ham doc ENCODER*******/
int32_t Encoder_GetPosition(void)
		{
			static int32_t 	p = 0, p_pre = 0, dp = 0;
			static int32_t 	pulse_cur = 0.0;
					
			p = (int32_t)TIM1->CNT;
			dp = p - p_pre;
			if (dp > 32768)
				 dp -= 65536;
			else if (dp < -32768)
				 dp += 65536;
			p_pre = p;
			pulse_cur += dp;
					
			return pulse_cur;
		}
//=================================================================================================================================================================
		
		

//=================================================================================================================================================================
//			/******Ham doc ENCODER1*******/
//int32_t Encoder1_GetPosition(void)
//		{
//			static int32_t 	p = 0, p_pre = 0, dp = 0;
//			static int32_t 	pulse_cur = 0.0;
//					
//			p = (int32_t)TIM8->CNT;
//			dp = p - p_pre;
//			if (dp > 32768)
//				 dp -= 65536;
//			else if (dp < -32768)
//				 dp += 65536;
//			p_pre = p;
//			pulse_cur += dp;
//					
//			return pulse_cur;
//		}
//=================================================================================================================================================================

		

//=================================================================================================================================================================
/*******Ham SET Freq********/
void PWM0_Set_Freq(uint32_t freq)
		{
			uint32_t period;
						
			period = 84000000/(freq*(TIM12->PSC+1))-1;	
			if (period > 0xffff)
					period = 0xffff;
						
			TIM12->ARR = period;
			
		}
		
//=================================================================================================================================================================
		
		
//=================================================================================================================================================================
					
/******Ham SET Duty cycle********/
void PWM0_Set_Duty(int32_t d)
		{
			if (d<-1000)
				 d = -1000;
			else if (d>1000)
				 d = 1000;
			if (d == 0){
					GPIO_ResetBits(GPIOD,GPIO_Pin_10);  	// disable pwm
					}	
			else if (d > 0)	{
					GPIO_SetBits(GPIOD,GPIO_Pin_10);	// enabe pwm
					GPIO_ResetBits(GPIOB,GPIO_Pin_15); 	// dir = 1
					}
			else 	{
					GPIO_SetBits(GPIOD,GPIO_Pin_10);	// enabe pwm
					GPIO_SetBits(GPIOB,GPIO_Pin_15); 		// dir = 0
					d = d + 1000;
						}
			d = (TIM12->ARR+1)*d/1000;
			TIM12->CCR1 = (uint32_t)d;
		}
//==================================================================================================================================================================
				
		
//==================================================================================================================================================================
		
		
//==================================================================================================================================================================
		
		

////==================================================================================================================================================================
//void PWM2_Set_Freq(uint32_t freq)
//{
//	uint32_t period;
//						
//			period = 84000000/(freq*(TIM2->PSC+1))-1;	
//			if (period > 0xffff)
//					period = 0xffff;
//						
//			TIM2->ARR = period;
//}
		
////==================================================================================================================================================================
//		
//		
////==================================================================================================================================================================
//void PWM2_Set_Duty(int32_t d)
//{
//	if (d<-1000)
//				 d = -1000;
//			else if (d>1000)
//				 d = 1000;
//			if (d == 0){
//					GPIO_ResetBits(GPIOD,GPIO_Pin_10);  	// disable pwm
//					}	
//			else if (d > 0)	{
//					GPIO_SetBits(GPIOD,GPIO_Pin_10);	// enabe pwm
//					GPIO_ResetBits(GPIOB,GPIO_Pin_15); 	// dir = 1
//					}
//			else 	{
//					GPIO_SetBits(GPIOD,GPIO_Pin_10);	// enabe pwm
//					GPIO_SetBits(GPIOB,GPIO_Pin_15); 		// dir = 0
//					d = -d;
//						}
//			d = (int32_t)(TIM2->ARR+1)*d/1000;
//			TIM2->CCR1 = (uint32_t)d;
//}
////===================================================================================================================================================================



////==================================================================================================================================================================
//void PWM3_Set_Freq(uint32_t freq)
//{
//	uint32_t period;
//						
//			period = 84000000/(freq*(TIM4->PSC+1))-1;	
//			if (period > 0xffff)
//					period = 0xffff;
//						
//			TIM4->ARR = period;
//}
		
////==================================================================================================================================================================
//		
//		
////==================================================================================================================================================================
//void PWM3_Set_Duty(int32_t d)
//{
//	if (d<-1000)
//				 d = -1000;
//			else if (d>1000)
//				 d = 1000;
//			if (d == 0){
//					GPIO_ResetBits(GPIOD,GPIO_Pin_10);  	// disable pwm
//					}	
//			else if (d > 0)	{
//					GPIO_SetBits(GPIOD,GPIO_Pin_10);	// enabe pwm
//					GPIO_ResetBits(GPIOB,GPIO_Pin_15); 	// dir = 1
//					}
//			else 	{
//					GPIO_SetBits(GPIOD,GPIO_Pin_10);	// enabe pwm
//					GPIO_SetBits(GPIOB,GPIO_Pin_15); 		// dir = 0
//					d = -d;
//						}
//			d = (int32_t)(TIM4->ARR+1)*d/1000;
//			TIM4->CCR1 = (uint32_t)d;
//}
////===================================================================================================================================================================


//=================================================================================================================================================================
			
			
//=================================================================================================================================================================

void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM5,TIM_IT_Update))
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);
		GPIO_SetBits(GPIOB,GPIO_Pin_9);
		delay_us(10);
		GPIO_ResetBits(GPIOB,GPIO_Pin_9);
	}
}


//=================================================================================================================================================================
			
			
//=================================================================================================================================================================

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3,TIM_IT_CC4))
	{
		if(check==1)
		{
			check = 0;
			time = TIM_GetCapture4(TIM3);
			distance = (double)time*343.2/10000/2;
		}
		else
			check = 1;
		TIM3->CNT = 0;
		TIM3->CCR2 =0;
	};
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
}

//=================================================================================================================================================================
			
			
//=================================================================================================================================================================

void PID_SpeedController(double Ts, double dkp,double dki,double dkd, int32_t freq, double setPoint)
	{
		double P_Part, I_Part, D_Part;
		//Output = (double)Encoder_GetPosition()*360.0/912.0;
		//Pulse = (double)Encoder_GetPosition();
		//Output = (Pulse - pre_Pulse)*60/912.0/Ts;
		ek = setPoint - Output;
		P_Part = dkp * ek;
		I_Part = 0.5*dki*Ts*(ek + ek_1);
		D_Part = dkd*(ek - 2* ek_1 + ek_2)/Ts;
		u = P_Part + I_Part + D_Part;
		if (u>12)
			u = 12;
		else if (u<-12)
			u = -12;
		else 
			u = u;
		u_1 = u;;
		ek_2 = ek_1;
		ek_1 = ek;
		UDK =(int32_t)(u*freq * 0.95 / 12);
		PWM0_Set_Freq((uint32_t)freq);
		PWM0_Set_Duty(UDK);
		pre_Pulse = Pulse;
	}
	
void Deg2Pulse(uint32_t b)
{
	a = 0.54*b+20;
	TIM9->CCR1 = a;
}	
