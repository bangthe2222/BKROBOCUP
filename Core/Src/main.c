/* USER CODE BEGIN Header */
/**
  ****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ****************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ****************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "PS2.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
 TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
int check = 0;
int leftMotorSpeed;
int rightMotorSpeed;
int count = 0;
uint8_t LFSensor[5];
uint8_t Obsen[2];
uint16_t InitSpeedLeft = 500;
uint16_t InitSpeedRight = 500;
int error;
int previous_error = 0;
int chuyen_mode = 0;
int time_delay = 20;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SpeedControl(int leftMotorSpeed, int rightMotorSpeed);
void Tien(){
	HAL_GPIO_WritePin(GPIOB, Tren_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Tren_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_2, GPIO_PIN_SET);
	//////////////////////
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_2, GPIO_PIN_SET);
}
void Lui(){
HAL_GPIO_WritePin(GPIOB, Tren_Trai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Tren_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_2, GPIO_PIN_RESET);
	//////////////////////
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_2, GPIO_PIN_RESET);
}
void Trai(){
	HAL_GPIO_WritePin(GPIOB, Tren_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Tren_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_2, GPIO_PIN_RESET);
	//////////////////////
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_2, GPIO_PIN_RESET);
}
void Phai(){
	HAL_GPIO_WritePin(GPIOB, Tren_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Tren_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_2, GPIO_PIN_SET);
	//////////////////////
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_2, GPIO_PIN_SET);
}
void Trai_Tai_Cho(){
	HAL_GPIO_WritePin(GPIOB, Tren_Trai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Tren_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_2, GPIO_PIN_SET);
	//////////////////////
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_2, GPIO_PIN_SET);
}
void Phai_Tai_Cho(){
	HAL_GPIO_WritePin(GPIOB, Tren_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Tren_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_2, GPIO_PIN_RESET);
	//////////////////////
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_2, GPIO_PIN_RESET);
}
void Dung(){
	HAL_GPIO_WritePin(GPIOB, Tren_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Tren_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Tren_Phai_2, GPIO_PIN_RESET);
	//////////////////////
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Trai_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Duoi_Phai_2, GPIO_PIN_RESET);	
}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void read_sen()
{
			LFSensor[0] = HAL_GPIO_ReadPin(GPIOB,	GPIO_PIN_12);
		LFSensor[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		LFSensor[2] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
		LFSensor[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
		LFSensor[4] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
}
void read_line_sensor()
{	
		read_sen();
		if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) 
		{
				error = previous_error;
		}
	
		if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 )) error = 4; // Thang trai vua
	
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) error = 3; // Thang trai nhe
	
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) error = 4; // Thang trai vua
	
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = 0; // Thang full speed
	
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) error = 0; // Thang full speed
		
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = 0; // Thang full speed
	
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) error = 2; // Trai tai cho full speed
		
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) error = 2; // Trai tai cho full speed
	
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = -4; // Thang phai vua
	
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = -3; // Thang phai nhe
			
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = -4; // Thang phai vua
			
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = -2; // Phai tai cho full speed
			
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) error = -2; // Phai tai cho full speed
			
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) 
			{
					check = 1;
					error = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}
			
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 )) 
			{
					check = 1;
					error = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}
			
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) 
			{
					check = 1;
					error = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}			
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) 
			{
					check = 1;
					error = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}
			
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 )) 
			{
					check = 1;
					error = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}
			
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) 
			{
					check = 1;
					error = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}
			
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) 
			{
					check = 1;
					error = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}
			
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) 
			{
					check = 1;
					error = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}
			
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 )) 
			{
					check = 1;
					error = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}
			
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 )) 
			{
					check = 1;
					error = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}
			
			else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 )) 
			{
					check = 1;
					error = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			}
			
			else if((LFSensor[0]== 1)&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )){
					if (check == 0){
						error = 0;
					}else if (check ==1){
						error = 10;
						check =0;
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					}
			}
			else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )){
					check = 1;
					error = 0;
			}
			if (count>1000){
					check = 0;
					count = 0;
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			}
}

void calculateSpeed()
{

	if(error==0){
			Tien();
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			SpeedControl(InitSpeedLeft-100, InitSpeedRight-100);
			HAL_Delay(time_delay);
	}else if(error==-4){
			Phai_Tai_Cho();
			//Tien();
			SpeedControl(InitSpeedLeft - 100, InitSpeedRight - 100);
			HAL_Delay(time_delay);
	}else if(error==4){
			Trai_Tai_Cho();
			//Tien();
			SpeedControl(InitSpeedLeft - 100 , InitSpeedRight - 100);
			HAL_Delay(time_delay);
//	}else if(error==-2){
//			Phai_Tai_Cho();
//			//Tien();
//			SpeedControl(InitSpeedLeft - 100 , InitSpeedRight - 100);
//			HAL_Delay(time_delay);
//	}else if(error==2){
//			Trai_Tai_Cho();
//			//Tien();
//			SpeedControl(InitSpeedLeft - 100, InitSpeedRight - 100);
//			HAL_Delay(time_delay);
	}else if(error==-3){
			Phai_Tai_Cho();
			//Tien();
			SpeedControl(InitSpeedLeft - 100 , InitSpeedRight - 100);
			HAL_Delay(time_delay);
	}else if(error==3){
			Trai_Tai_Cho();
			//Tien();
			SpeedControl(InitSpeedLeft - 100, InitSpeedRight - 100);
			HAL_Delay(time_delay);
	}else if(error==-4){
			Phai_Tai_Cho();
			//Tien();
			SpeedControl(InitSpeedLeft - 100, InitSpeedRight - 100);


		HAL_Delay(time_delay);
	}else if(error==4){
			Trai_Tai_Cho();
			//Tien();
			SpeedControl(InitSpeedLeft - 100, InitSpeedRight - 100);
			HAL_Delay(time_delay);
	}else if(error==2){	
			Dung();
			//Lui();
			//HAL_Delay(100);
			Dung();
			do{
				read_line_sensor();
				Trai_Tai_Cho();
				SpeedControl(InitSpeedLeft-100, InitSpeedRight-100);	
				HAL_Delay(200);
			}while(error!=0);
	}else if(error==-2){
			Dung();
			//Lui();
			//HAL_Delay(100);
			Dung();
			do{
				read_line_sensor();
				Phai_Tai_Cho();
				SpeedControl(InitSpeedLeft-100, InitSpeedRight-100);	
				HAL_Delay(200);
			}while(error!=0);
	} else if (error == 10){
			Dung();
			HAL_Delay(100);
			Phai_Tai_Cho();
			SpeedControl(400, 400);
			HAL_Delay(500);
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}
		previous_error = error;
}

void SpeedControl(int leftMotorSpeed, int rightMotorSpeed)
{

	TIM4->CCR1 = rightMotorSpeed;
	TIM4->CCR2 = leftMotorSpeed;
	
	TIM4->CCR3 = rightMotorSpeed;
	TIM4->CCR4 = leftMotorSpeed;
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	
 float servo1 = 5;
 float servo2 = 25;
 float servo3 = 10;
 float servo4 = 25;
 TIM2->CCR1 = servo1;
 TIM2->CCR2 = servo2;
 TIM2->CCR3 = servo3;
 TIM2->CCR4 = servo4;
  /* USER CODE END 2 */
		
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	PS2Buttons PS2;
	PS2_Init(&htim3, &PS2);	
  while(1)
  {
		TIM4->CCR1 = 500;
		TIM4->CCR2 = 500;
		TIM4->CCR3 = 500;
		TIM4->CCR4 = 500;
		
		PS2_Update();

		
		if (PS2.UP) {
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			Tien();
		} else if (PS2.DOWN) {
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			Lui();
		}else if (PS2.LEFT) {
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			Trai();
		}else if (PS2.RIGHT) {
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			Phai();
		} else if (PS2.L1) {
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			Trai_Tai_Cho();
		} else if (PS2.L2) {
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			Phai_Tai_Cho();
		} else if (PS2.L3) {
			chuyen_mode = 1;
		} else if(PS2.CIRCLE) {
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
					servo1 += 0.4;
					if(servo1 >= 12.5) servo1 = 12.5;
					TIM2->CCR1 = servo1;
					HAL_Delay(10);
			} else if(PS2.SQUARE) {
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					servo1 -= 0.4;
					if(servo1 <= 5) servo1 = 5;
					TIM2->CCR1 = servo1;
					HAL_Delay(10);
			} else if(PS2.R1) {
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
					servo2 += 0.3;
					if(servo2 >= 25) servo2 = 25;
					TIM2->CCR2 = servo2;
					HAL_Delay(40);
			} else if(PS2.R2) {
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					servo2 -= 0.3;
					if(servo2 <= 18) servo2 = 18;
					TIM2->CCR2 = servo2;
					HAL_Delay(40);
			}  else if(PS2.TRIANGLE) {
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
					servo3 -= 0.2;
					servo4 -= 0.1;
					if(servo3 <= 2.5) servo3 = 2.5;
					if(servo4 <= 20) servo4 = 20;  
					TIM2->CCR3 = servo3;
					TIM2->CCR4 = servo4;
					HAL_Delay(10);
			} else if(PS2.CROSS) {
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					servo3 += 0.1;
					servo4 += 0.2;
					if(servo3 >= 10) servo3 = 10;
					if(servo4 >= 25) servo4 = 25;
					TIM2->CCR3 = servo3;
					TIM2->CCR4 = servo4;
					HAL_Delay(10);
			} else {
			Dung();
		}
		
		if(chuyen_mode == 1){
			Dung();
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			while(1){
				int a = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
				int b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
				if (a == 0) {
						Dung();
						TIM4->CCR1 = 400;
						TIM4->CCR2 = 400;
						TIM4->CCR3 = 400;
						TIM4->CCR4 = 400;
						Phai_Tai_Cho();
						HAL_Delay(500);
						Tien();
						HAL_Delay(500);
						Trai_Tai_Cho();
						HAL_Delay(500);
						Tien();
						HAL_Delay(500);
						Trai_Tai_Cho();
						HAL_Delay(500);
						Tien();
						HAL_Delay(500);
						Phai_Tai_Cho();
						HAL_Delay(500);
				}
				if (b == 0) {
						Dung();
						TIM4->CCR1 = 400;
						TIM4->CCR2 = 400;
						TIM4->CCR3 = 400;
						TIM4->CCR4 = 400;
						Trai_Tai_Cho();
						HAL_Delay(500);
						Tien();
						HAL_Delay(500);
						Phai_Tai_Cho();
						HAL_Delay(500);
						Tien();
						HAL_Delay(500);
						Phai_Tai_Cho();
						HAL_Delay(500);
						Tien();
						HAL_Delay(500);
						Trai_Tai_Cho();
						HAL_Delay(500);
				}
				Dung();
				read_line_sensor();
				calculateSpeed();	
				if (check==1){
					count+=1;
				}
				HAL_Delay(2);
			}

		}	
		HAL_Delay(2);		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SS_Pin|SCK_Pin|MOSI_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : SS_Pin SCK_Pin MOSI_Pin */
  GPIO_InitStruct.Pin = SS_Pin|SCK_Pin|MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MISO_Pin PA8 */
  GPIO_InitStruct.Pin = MISO_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA11 PA12
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
