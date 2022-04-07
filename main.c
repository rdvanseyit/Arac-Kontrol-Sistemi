/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"

#include "math.h"

//#define axle 0.630434782 // vehicle chassis values

#define axle 0.4957264957264957

#define PI 3.14159265 // pi coefficient

#define D 116

#define L 234

#define Rw 29

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
		.name = "myTask02",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
		.name = "myTask03",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
		.name = "myTask04",
		.stack_size = 256 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef pTxHeader[7];
uint32_t pTxMailbox;
CAN_RxHeaderTypeDef pRxHeader[2];
CAN_FilterTypeDef sFilterConfig;
uint8_t can1_messages[8]={0,0,0,0,0,0,0,0}; //left motor controller CAN BUS messages
uint8_t can2_messages[8]={0,0,0,0,0,0,0,0}; //right motor controller CAN BUS messages
uint8_t left_message1[8]={0,0,0,0,0,0,0,0}; //left motor controller CAN BUS message1
uint8_t left_message2[8]={0,0,0,0,0,0,0,0}; //left motor controller CAN BUS message2
uint8_t right_message1[8]={0,0,0,0,0,0,0,0}; //right motor controller CAN BUS message1
uint8_t right_message2[8] = {0,0,0,0,0,0,0,0}; //right motor controller CAN BUS message2
uint8_t main_board_message1[8] = {0,0,0,0,0,0,0,0};//main board datas
uint8_t main_board_message2[8] = {0,0,0,0,0,0,0,0};//main board datas
uint8_t main_board_message3[8] = {0,0,0,0,0,0,0,0};//main board datas
uint8_t left_ERR_LSB[8] = {0,0,0,0,0,0,0,0};//Left motor controller Error code LSB
uint8_t left_ERR_MSB[8] = {0,0,0,0,0,0,0,0};//Left motor controller Error code MSB
uint8_t right_ERR_LSB[8] = {0,0,0,0,0,0,0,0};//Right motor controller Error code LSB
uint8_t right_ERR_MSB[8] = {0,0,0,0,0,0,0,0};//Right motor controller Error code MSB
uint8_t shifting1[8] = {0,0,0,0,0,0,0,0};//Error code shifting process
uint8_t shifting2[8] = {0,0,0,0,0,0,0,0};//Error code shifting process
uint8_t shifting3[8] = {0,0,0,0,0,0,0,0};//Error code shifting process
uint8_t shifting4[8] = {0,0,0,0,0,0,0,0};//Error code shifting process
uint8_t shifting5;//Error code shifting process
uint8_t shifting6;//Error code shifting process
uint8_t geri1;
uint8_t geri2;
uint8_t geriE;
uint8_t differantial_datas[8] = {0,0,0,0,0,0,0,0};//Electronic differantial datas
uint8_t left_angle = 0; // steering wheel left angle
uint8_t right_angle = 0; // steering wheel right angle
uint16_t left_motor_speed = 0; // left motor speed 
uint16_t right_motor_speed = 0; // right motor speed
uint16_t left_RPM = 0; // left motor RPM
uint16_t right_RPM = 0; // right motor RPM
uint16_t left_temp = 0; // left motor controller temperature
uint16_t right_temp = 0; // right motor controller temperature
uint16_t vehicle_speed = 0; // vehicle speed
float left_current = 0.0; // left motor controller instantaneous current
float right_current = 0.0; // right motor controller instantaneous current
float batt_voltagel = 0.0; // battery voltage information sent by the left motor controller
float batt_voltager = 0.0; // battery voltage information sent by the right motor controller
float current = 0.0; // vehicle instantaneous current
float hucre[24];
uint8_t fault = 0; // fault error in speed display 
uint8_t MB = 0; // main board error in the speed display
uint8_t BC = 0; // balance card error in the speed display
uint8_t CC = 0; // current card error in the speed display
char tx[40]; // display Transmit variable

uint16_t adc_value[2] = {0,0}; // adc_value[0]: steering wheel ADC value, adc_value[1]: pedal ADC

float left; //
float right; //
float left_out; // PWM output for left motor controller 
float right_out; // PWM output for right motor controller
float coefficient; // coefficent calculation for differantial 
float R;// turning radius
float Rr1, Rr2;//
float Vr1, Vr2;//
float Wr1, Wr2;//
float ohm;

float ort;
float ort_sonuc;
float katsayi = 0.0;
float akimort;
float akimortsonuc;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t map(float In, float Inmin, float Inmax, float Outmin, float Outmax)
{
	return (In - Inmin) * (Outmax - Outmin) / (Inmax - Inmin) + Outmin ;
}
void CAN_BUS()
{
	/* CAN BUS messages receiving side */
	/* Left controller messages */
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &pRxHeader[0], can1_messages);
	if(pRxHeader[0].ExtId == 0x0CF11E05)
	{
		for(int i = 0; i < 8; i++)
		{ 	 
			left_message1[i] = can1_messages[i];
		}
	}
	else if(pRxHeader[0].ExtId == 0x0CF11F05)
	{
		for(int i = 0; i < 8; i++)
		{ 	 
			left_message2[i] = can1_messages[i];
		}
	}
	/* Right controller messages */
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &pRxHeader[1], can2_messages);
	if(pRxHeader[1].ExtId == 0x0CF11E05)
	{
		for(int i = 0; i < 8; i++)
		{
			right_message1[i] = can2_messages[i];
		}
	}
	if(pRxHeader[1].ExtId == 0x0CF11F05)
	{
		for(int i = 0; i < 8; i++)
		{
			right_message2[i] = can2_messages[i];
		}	
	}
	/* Main board data messages*/
	if(pRxHeader[1].ExtId == 0x0CF31F05)
	{
		for(int i = 0; i < 8; i++)
		{ 	 
			main_board_message1[i] = can2_messages[i];
			hucre[i] = (main_board_message1[i]) / 20.0;// 0-8 cell voltage 
		}
	}
	else if(pRxHeader[1].ExtId == 0x0CF51F05)
	{
		for(int i = 0; i < 8; i++)
		{ 	 
			main_board_message2[i] = can2_messages[i];
			hucre[i + 8] = (main_board_message2[i]) / 20.0;// 8-16 cell voltage
		}
	}
	else if(pRxHeader[1].ExtId == 0x0CF71F05)
	{
		for(int i = 0; i < 8; i++)
		{ 	 
			main_board_message3[i] = can2_messages[i];
		}
		hucre[16] =	(main_board_message3[0]) / 20.0;//cell 16 voltage
		hucre[17] =	(main_board_message3[1]) / 20.0;//cell 17 voltage
		for(int i = 18; i < 24; i++)
		{
			hucre[i] =	(main_board_message3[i - 16]);//hucre[18]:max.battary temp,hucre[19]:battary voltage
		}
	}
	/* converting CAN BUS messages to vehicle data */
	left_RPM = (((left_message1[1] * 256) + left_message1[0]) / 4); 	
	right_RPM = (((right_message1[1] * 256) + right_message1[0]) / 4); 		
	left_current = (((left_message1[3] * 256 )+ left_message1[2]) / 10);
	right_current = (((right_message1[3] * 256 )+ right_message1[2]) / 10);
	current = (left_current + right_current )* 1.73;
	batt_voltagel = (((left_message1[5] *256) + left_message1[4]) / 10);
	batt_voltager = (((right_message1[5] *256) + right_message1[4]) / 10);
	left_temp = (left_message2[1] - 40) * 10;	
	right_temp = (right_message2[1] - 40) * 10;
	left_motor_speed = left_RPM * 0.10933;		
	right_motor_speed = right_RPM * 0.10933;
	/* Motor controllers error codes */
	for(int i = 0; i < 8; i++)
	{
		shifting1[i] = left_message1[6] << (7 - i);
		left_ERR_LSB[i] = shifting1[i] >> 7;
		shifting2[i] = left_message1[7] << (7 - i);
		left_ERR_MSB[i] = shifting2[i] >> 7;
		shifting3[i] = right_message1[6] << (7 - i);
		right_ERR_LSB[i] = shifting3[i] >> 7;
		shifting4[i] = right_message1[7] << (7 - i);
		right_ERR_MSB[i] = shifting4[i] >> 7;
	}	
	shifting5 = left_message2[5] << 3;
	geri1 = shifting5 >> 7;
	shifting6 = right_message2[5] << 3;
	geri2 = shifting6 >> 7; 
	if(geri1 || geri2)
		geriE = 1;
	else 
		geriE = 0;
	/* CAN BUS transmitted messages */
	HAL_CAN_AddTxMessage(&hcan1,&pTxHeader[0], differantial_datas, &pTxMailbox);
	HAL_CAN_AddTxMessage(&hcan1,&pTxHeader[1], left_ERR_LSB, &pTxMailbox);
	HAL_CAN_AddTxMessage(&hcan1,&pTxHeader[2], left_ERR_MSB, &pTxMailbox);
	HAL_CAN_AddTxMessage(&hcan2,&pTxHeader[3], right_ERR_LSB, &pTxMailbox);
	HAL_CAN_AddTxMessage(&hcan2,&pTxHeader[4], right_ERR_MSB, &pTxMailbox);
}
void ekranyaz()
{
	HAL_UART_Transmit(&huart2,(uint8_t*)tx,sprintf(tx,"n6.val=%dÿÿÿ", right_motor_speed),200);
	HAL_UART_Transmit(&huart2,(uint8_t*)tx,sprintf(tx,"x0.val=%dÿÿÿ", left_temp),200);//left_temp
	HAL_UART_Transmit(&huart2,(uint8_t*)tx,sprintf(tx,"x1.val=%dÿÿÿ", right_temp),200);//right_temp
	HAL_UART_Transmit(&huart2,(uint8_t*)tx,sprintf(tx,"n5.val=%dÿÿÿ", fault),100);
	HAL_UART_Transmit(&huart2,(uint8_t*)tx,sprintf(tx,"n0.val=%dÿÿÿ", MB),100);
	HAL_UART_Transmit(&huart2,(uint8_t*)tx,sprintf(tx,"n1.val=%dÿÿÿ", BC),100);
	HAL_UART_Transmit(&huart2,(uint8_t*)tx,sprintf(tx,"n2.val=%dÿÿÿ", CC),100);  
	HAL_UART_Transmit(&huart2,(uint8_t*)tx,sprintf(tx,"n7.val=%dÿÿÿ", geriE),100); 
}
void fault_control()
{
	//fault control algorithm
	if(HAL_GPIO_ReadPin(GPIOD, MB_Pin) == 0)
		MB = 1;
	else
		MB = 0;
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == 0)
		BC = 1;	
	else
		BC = 0;
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 0)
		CC = 1;
	else
		CC = 0;
	for(int i = 0; i < 8; i++)
	{
		if(left_ERR_LSB[i] == 1)
			fault = 1;
		else if(left_ERR_MSB[i] == 1)
			fault = 1;
		else if(right_ERR_LSB[i] == 1)
			fault = 1;
		else if(right_ERR_MSB[i] == 1)
			fault = 1;
		else if(hucre[18] > 55)
			fault = 1;
		else
			fault = 0;
	}
}
void electronic_differential()
{
	for(int i = 0; i < 250001; i++)
	{
		if(i > 6000)
		{
			ort = ort + adc_value[0];
		}
		if(i == 244000)
		{
			ort_sonuc = ort/244000;
		}
		if(i >= 250000)
		{
				if(ort_sonuc < 3110)
				{		
					left_angle = map(ort_sonuc, 3110, 2675, 0 ,28); 
					right_angle = map(left_angle,0, 28, 0, 27);
					coefficient = axle * tan(left_angle * PI / 180) + 1;
					left = adc_value[1] / coefficient;		
					right = adc_value[1];
					left_out = map(left,0,4095,0,87);	
					right_out = map(right,0,4095,0,87);				
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, left_out);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, right_out);	
					differantial_datas[0] = map(left_angle, 0, 28, 27, 1);
					differantial_datas[1] = map(left, 0, 4095, 0, 100);
					differantial_datas[2] = map(right, 0, 4095, 0, 100);
					differantial_datas[4]	= left_angle;
					differantial_datas[5]	= right_angle;			
					/*
					R = (L / (tan(left_angle * PI / 180))) + D / 2;		
					Rr1 = (R + (D / 2));
					Rr2 = (R - (D / 2));				
					Vr1 = left_motor_speed * Rw;		
					Vr2 = right_motor_speed * Rw;
					ohm = Vr1 / Rr1;
					*/
				}
				else if(ort_sonuc > 3132)
				{	
					right_angle = map(ort_sonuc, 3132, 3557, 0, 27); 
					left_angle = map(right_angle,0, 28, 0, 27);			
					coefficient = axle * tan(right_angle * PI / 180) + 1;	
					left = adc_value[1];		
					right = adc_value[1] / coefficient;	
					left_out = map(left,0,4095,0,87);	
					right_out = map(right,0,4095,0,87);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, left_out);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, right_out);
					differantial_datas[0] = map(right_angle, 0, 27, 29, 55);
					differantial_datas[1] = map(left, 0, 4095, 0, 100);
					differantial_datas[2] = map(right, 0, 4095, 0, 100);		
					differantial_datas[4]	= left_angle;
					differantial_datas[5]	= right_angle;	
          /*					
					R = (L / (tan(right_angle * PI / 180))) + D / 2;	
					Rr1 = (R + (D / 2));
					Rr2 = (R - (D / 2));
					Vr1 = left_motor_speed * Rw;		
					Vr2 = right_motor_speed * Rw;
					ohm = Vr1 / Rr1;
          */					
				}
				else if(ort_sonuc <= 3132 && ort_sonuc >= 3110)
				{
					left_angle = 0;
					right_angle = 0;
					left = right = adc_value[1];
					left_out = map(left,0,4095,0,87);	
					right_out = map(right,0,4095,0,87);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, left_out);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, right_out);
					differantial_datas[0] = 28;
					differantial_datas[1] = map(left, 0, 4095, 0, 100);
					differantial_datas[2] = map(right, 0, 4095, 0, 100);
					differantial_datas[4]	= left_angle;
					differantial_datas[5]	= right_angle;
          /*					
					R = 0 ;	
					Rr1 = (R + (D / 2));
					Rr2 = (R - (D / 2));
					*/
				}
				ort=0;
			
			/*
			else if((hucre[18] > 55 && hucre[18] < 65))
			{
				if(ort_sonuc < 3110)
				{		
					left_angle = map(ort_sonuc, 3110, 2675, 0 ,28); 
					right_angle = map(left_angle,0, 28, 0, 27);
					coefficient = axle * tan(left_angle * PI / 180) + 1;
					left = adc_value[1] / coefficient;		
					right = adc_value[1];
						left_out = map(left,0,4095,0,87)/((hucre[18])/40);	
						right_out = map(right,0,4095,0,87)/((hucre[18])/40);
						differantial_datas[1] = map(left, 0, 4095, 0, 100)/((hucre[18])/40);
						differantial_datas[2] = map(right, 0, 4095, 0, 100)/((hucre[18])/40);											 
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, left_out);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, right_out);	
					differantial_datas[0] = map(left_angle, 0, 28, 27, 1);
					differantial_datas[4]	= left_angle;
					differantial_datas[5]	= right_angle;			
				}
				else if(ort_sonuc > 3132)
				{	
					right_angle = map(ort_sonuc, 3132, 3557, 0, 27); 
					left_angle = map(right_angle,0, 28, 0, 27);			
					coefficient = axle * tan(right_angle * PI / 180) + 1;	
					left = adc_value[1];		
					right = adc_value[1] / coefficient;	
						left_out = map(left,0,4095,0,87)/((hucre[18])/40);	
						right_out = map(right,0,4095,0,87)/((hucre[18])/40);
					  differantial_datas[1] = map(left, 0, 4095, 0, 100)/((hucre[18])/40);
					  differantial_datas[2] = map(right, 0, 4095, 0, 100)/((hucre[18])/40);							
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, left_out);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, right_out);
					differantial_datas[0] = map(right_angle, 0, 27, 29, 55);
					differantial_datas[4]	= left_angle;
					differantial_datas[5]	= right_angle;					
				}
				else if(ort_sonuc <= 3132 && ort_sonuc >= 3110)
				{
					left_angle = 0;
					right_angle = 0;
					left = right = adc_value[1];
						left_out = map(left,0,4095,0,87)/((hucre[18])/40);	
						right_out = map(right,0,4095,0,87)/((hucre[18])/40);
						differantial_datas[1] = map(left, 0, 4095, 0, 100)/((hucre[18])/40);
						differantial_datas[2] = map(right, 0, 4095, 0, 100)/((hucre[18])/40);						
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, left_out);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, right_out);
					differantial_datas[0] = 28;
					differantial_datas[4]	= left_angle;
					differantial_datas[5]	= right_angle;			
				}
				ort=0;
			}
			else if(hucre[18] >= 65)
			{
				left_out = 0;	
				right_out = 0;
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, left_out);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, right_out);
			}
			*/
		}		
	}
	differantial_datas[3] = map(adc_value[1], 100, 4095, 0, 100);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_ADC1_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	/* CAN BUS transmitted messages ID's */

	pTxHeader[0].DLC = 8;
	pTxHeader[0].IDE = CAN_ID_EXT ;
	pTxHeader[0].RTR = CAN_RTR_DATA;
	pTxHeader[0].ExtId = 0x0CF13E05;
	pTxHeader[1].DLC = 8;
	pTxHeader[1].IDE = CAN_ID_EXT ;
	pTxHeader[1].RTR = CAN_RTR_DATA;
	pTxHeader[1].ExtId = 0x0CF15E05;
	pTxHeader[2].DLC = 8;
	pTxHeader[2].IDE = CAN_ID_EXT ;
	pTxHeader[2].RTR = CAN_RTR_DATA;
	pTxHeader[2].ExtId = 0x0CF17E05;
	pTxHeader[3].DLC = 8;
	pTxHeader[3].IDE = CAN_ID_EXT ;
	pTxHeader[3].RTR = CAN_RTR_DATA;
	pTxHeader[3].ExtId = 0x0CF19E05;	
	pTxHeader[4].DLC = 8;
	pTxHeader[4].IDE = CAN_ID_EXT ;
	pTxHeader[4].RTR = CAN_RTR_DATA;
	pTxHeader[4].ExtId = 0x0CF1BE05;
	/*	pTxHeader[5].DLC = 8;
		pTxHeader[5].IDE = CAN_ID_EXT ;
		pTxHeader[5].RTR = CAN_RTR_DATA;
		pTxHeader[5].ExtId = 0x0CF1DE05;	
		pTxHeader[6].DLC = 8;
		pTxHeader[6].IDE = CAN_ID_EXT ;
		pTxHeader[6].RTR = CAN_RTR_DATA;
		pTxHeader[6].ExtId = 0x0CF1FE05;
	 */		
	/* CAN BUS 1 receiving messages filters*/
	HAL_CAN_Start(&hcan1);
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	/* CAN BUS 2 receiving messages filters*/
	HAL_CAN_Start(&hcan2);
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Left motor controller PWM start function
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // Right motor controller PWM start function
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, 2); // ADC start

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of myTask02 */
	myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

	/* creation of myTask03 */
	myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

	/* creation of myTask04 */
	myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 12;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void)
{

	/* USER CODE BEGIN CAN2_Init 0 */

	/* USER CODE END CAN2_Init 0 */

	/* USER CODE BEGIN CAN2_Init 1 */

	/* USER CODE END CAN2_Init 1 */
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 12;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = DISABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CAN2_Init 2 */

	/* USER CODE END CAN2_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 83;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 99;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : MB_Pin PD3 */
	GPIO_InitStruct.Pin = MB_Pin|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : BC_Pin CC_Pin */
	GPIO_InitStruct.Pin = BC_Pin|CC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		CAN_BUS();
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
	/* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	for(;;)
	{
		electronic_differential();
		osDelay(1);
	}
	/* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
	/* USER CODE BEGIN StartTask03 */
	/* Infinite loop */
	for(;;)
	{

		ekranyaz();		
		osDelay(1);
	}
	/* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the myTask04 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
	/* USER CODE BEGIN StartTask04 */
	/* Infinite loop */
	for(;;)
	{

		fault_control();
		osDelay(1);
	}
	/* USER CODE END StartTask04 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
