/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "my_1602.h"
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#define ARR_CNT 7
#define CMD_SIZE 50
#define GETWEATHER 60//1분에 한번 GETWEATHER 송신
#define GETTIME 60 // 1분에 한번 GETTIME 송신
#define LCDTEXTSUN 5
#define LCDTEXTTIME 2
#define LCDTEXTOTHER 1
#define LCDTEMP_WEATH (LCDTEXTSUN + LCDTEXTTIME + LCDTEXTTIME + LCDTEXTOTHER + LCDTEXTOTHER)
#define LCDTEMP_WEATH_VALUE (LCDTEMP_WEATH + 2)

#define GETSUNFLAG  (1 << 0)
#define WEATHERFLAG (1 << 1)
#define GETTIMEFLAG (1 << 2)

#define TEMPFLAG1  	(1 << 3)
#define TEMPFLAG2 	(1 << 4)
#define TEMPFLAG3 	(1 << 5)
#define TEMPFLAG4  	(1 << 6)
#define TEMPFLAG5	(1 << 7)

#define ADDFLAG(b,a)   (b |= (a))   // 플래그 set
#define CLEARFLAG(b,a) (b &= ~(a))  // 플래그 clear
#define CHECKFLAG(b,a) (b & (a))    // 플래그 확인

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx2char;
volatile unsigned char rx2Flag = 0;
volatile char rx2Data[50];
volatile unsigned char btFlag = 0;
uint8_t btchar;
char btData[50];
volatile uint16_t weatherCounter = 0;
volatile uint16_t timeCounter =0;
uint8_t sunSentToday = 0; // 0: GETSUN 안보냄, 1: 이미 보냄
volatile uint8_t sunsetTime = -1;
volatile uint8_t sunsetMin = -1;
volatile uint8_t sunriseTime = -1;
volatile uint8_t sunriseMin = -1;
volatile uint8_t flag = 0;
RTC_TimeTypeDef sTime;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void bluetooth_Event();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// --------------------------- Custom Char Start
uint8_t rainChar[8] = {
	0b00100,
	0b00100,
	0b01010,
	0b01010,
	0b10001,
	0b10001,
	0b01110,
	0b00000
};

uint8_t snowChar[8] = {
  0b00000,
  0b00100,
  0b10101,
  0b01110,
  0b10101,
  0b00100,
  0b00000,
  0b00000
};

uint8_t cloudChar[8] = {
  0b00000,
  0b00000,
  0b00110,
  0b11111,
  0b11111,
  0b01110,
  0b00000,
  0b00000
};

uint8_t fogChar[8] = {
  0b00000,
  0b11111,
  0b00000,
  0b11111,
  0b00000,
  0b11111,
  0b00000,
  0b00000
};

uint8_t sunChar[8] = {
  0b00100,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b10101,
  0b00100,
  0b00000
};
uint8_t moonChar[8] = {
  0b00110,
  0b01100,
  0b01100,
  0b01100,
  0b01100,
  0b00110,
  0b00000,
  0b00000
};
// ---------------------------- Custom Char End
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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx2char,1);
  HAL_UART_Receive_IT(&huart6, &btchar,1);
  printf("start main2()\r\n");
  HAL_TIM_Base_Start_IT(&htim1);
  // -------------------------------------------LCD INIT START
  HAL_Delay(1000); // imsi
  lcd_init(&hi2c1);
  HAL_Delay(1000);
  lcd_create_char(0, sunChar);
  lcd_create_char(1, cloudChar);
  lcd_create_char(2, fogChar);
  lcd_create_char(3, rainChar);
  lcd_create_char(4, snowChar);
  lcd_create_char(5, moonChar);

  lcd_set_cursor(0,0);
  lcd_send_string("Rise:");
  lcd_set_cursor(1, 0);
  lcd_send_string("set :");


  ADDFLAG(flag, WEATHERFLAG);
  ADDFLAG(flag, GETSUNFLAG);

  char sendBuf[CMD_SIZE]={0};
  sprintf(sendBuf,"[LDH_SUN]GETTIME\n");
  HAL_UART_Transmit(&huart6, (uint8_t *)sendBuf, strlen(sendBuf), 0xFFFF);
  HAL_Delay(100);
  // -------------------------------------------LCD INIT END

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 /*
	 for(int i =0 ; i < 6; i++)
	  {
		  lcd_set_cursor(0, 0);
		  lcd_replace_char(0,0,1,i);
		  HAL_Delay(1000);

	  }
	  */
	  if(rx2Flag)
	  {
			printf("recv2 : %s\r\n",rx2Data);
			rx2Flag =0;
	//	    HAL_UART_Transmit(&huart6, (uint8_t *)buf, strlen(buf), 0xFFFF);
	  }
	  if(btFlag)
	  {
//		printf("bt : %s\r\n",btData);
			btFlag =0;
			bluetooth_Event();
	  }


	  if(CHECKFLAG(flag, GETSUNFLAG)){
		  sprintf(sendBuf,"[LDH_SUN]GETSUN\n");
		  HAL_UART_Transmit(&huart6, (uint8_t *)sendBuf, strlen(sendBuf), 0xFFFF);
		  CLEARFLAG(flag, GETSUNFLAG);
		  HAL_Delay(100);
	  }
	  else if(CHECKFLAG(flag, GETTIMEFLAG)){
		  sprintf(sendBuf,"[LDH_SUN]GETTIME\n");
		  HAL_UART_Transmit(&huart6, (uint8_t *)sendBuf, strlen(sendBuf), 0xFFFF);
		  HAL_Delay(100);
		  CLEARFLAG(flag, GETTIMEFLAG);
	  }
	  else if(CHECKFLAG(flag, WEATHERFLAG)){
		  sprintf(sendBuf,"[LDH_SUN]GETWEATHER\n");
		  HAL_UART_Transmit(&huart6, (uint8_t *)sendBuf, strlen(sendBuf), 0xFFFF);
		  HAL_Delay(100);
		  CLEARFLAG(flag, WEATHERFLAG);
	  }


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void bluetooth_Event()
{

  int i=0;
  char * pToken;
  char * pArray[ARR_CNT]={0};
  char recvBuf[CMD_SIZE]={0};
  char sendBuf[CMD_SIZE]={0};
  strcpy(recvBuf,btData);

  printf("%s\r\n",btData);

  pToken = strtok(recvBuf,"[@]");
  while(pToken != NULL)
  {
    pArray[i] =  pToken;
    if(++i >= ARR_CNT)
      break;
    pToken = strtok(NULL,"[@]");
  }
//  printf("pArray[0] : %s\r\n",pArray[0]);
//  printf("pArray[1] : %s\r\n",pArray[1]);
//  printf("pArray[2] : %s\r\n",pArray[2]);
  if(!strcmp(pArray[1],"GETWEATHER"))
  {
		  lcd_set_cursor(0, LCDTEMP_WEATH);
		  lcd_send_string("T:");
		  lcd_set_cursor(0,LCDTEMP_WEATH_VALUE);
		  lcd_send_string(pArray[3]);

		  lcd_set_cursor(1, LCDTEMP_WEATH);
		  lcd_send_string("W:");

		  lcd_set_cursor(1,LCDTEMP_WEATH_VALUE);

		  RTC_DateTypeDef sDate;
		  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		  if(((sunriseTime >= sTime.Hours) && (sunriseMin > sTime.Minutes)) ||
		     ((sunsetTime <= sTime.Hours)  && (sunsetMin < sTime.Minutes))  ){

			  lcd_replace_char(1,LCDTEMP_WEATH_VALUE , LCDTEMP_WEATH_VALUE+1, 5);
		  }
		  else
			  lcd_replace_char(1,LCDTEMP_WEATH_VALUE , LCDTEMP_WEATH_VALUE+1, (uint8_t)atoi(pArray[2]));

		  //printf("firstgetsun %d %d %d",sTime.Minutes, sTime.Hours,atoi(pArray[2]));

  }
  if(!strcmp(pArray[1],"GETSUN"))
  {
	  for(int i = 0; i < 2;i++){

		  lcd_set_cursor(i, LCDTEXTSUN);
		  lcd_send_string(pArray[2+2*i]);
		  lcd_set_cursor(i,LCDTEXTSUN+LCDTEXTTIME);
		  lcd_send_string(":");
		  lcd_set_cursor(i,LCDTEXTSUN+LCDTEXTTIME+LCDTEXTOTHER);
		  lcd_send_string(pArray[3+2*i]);
	  }

	  sunriseTime = atoi(pArray[2]);
	  sunriseMin = atoi(pArray[3]);
	  sunsetTime = atoi(pArray[4]);
	  sunsetMin = atoi(pArray[5]);
  }
  if(!strcmp(pArray[1],"GETTIME"))
  {
	  sTime.Hours = atoi(pArray[2]);
	  sTime.Minutes = atoi(pArray[3]);
	  sTime.Seconds = atoi(pArray[4]);
	  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  //printf("current hours : %d\n", sTime.Hours);
  }


}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART6 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
    	static int i=0;
    	rx2Data[i] = rx2char;
    	if((rx2Data[i] == '\r')||(btData[i] == '\n'))
    	{
    		rx2Data[i] = '\0';
    		rx2Flag = 1;
    		i = 0;
    	}
    	else
    	{
    		i++;
    	}
    	HAL_UART_Receive_IT(&huart2, &rx2char,1);
    }
    if(huart->Instance == USART6)
    {
    	static int i=0;
    	btData[i] = btchar;
    	if((btData[i] == '\n') || btData[i] == '\r')
    	{
    		btData[i] = '\0';
    		btFlag = 1;
    		i = 0;
    	}
    	else
    	{
    		i++;
    	}
    	HAL_UART_Receive_IT(&huart6, &btchar,1);
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
        weatherCounter++;
        if(weatherCounter >= GETWEATHER) // 600초 = 10분
        {
            weatherCounter = 0;
            ADDFLAG(flag, WEATHERFLAG);
        	timeCounter =0;
        	ADDFLAG(flag, GETTIMEFLAG);
        }

        if(timeCounter >= GETTIME)
        {

        }

        if(sTime.Hours == 0 && sTime.Minutes == 0 && sunSentToday == 0)
        {
        	ADDFLAG(flag, GETSUNFLAG);
        	sunSentToday = 1;
        }
        if(sTime.Hours != 0 || sTime.Minutes != 0)
        {
        	sunSentToday = 0;
        }
    }
}
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
#ifdef USE_FULL_ASSERT
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
