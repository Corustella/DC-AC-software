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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../my_drivers/include/float_tools.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SQRT2 1.414213f
#define DMA_len 5
#define MAIN_DUTY_STEP 10.0f
#define MAIN_DUTY_MIN 100.0f
#define MAIN_DUTY_MAX 950.0f
#define MAIN_DEADTIME_PERCENT 2.5f
#define MAIN_TOTAL_ON_PERCENT 95.0f
#define VOLTAGE_INPUT_MIN 18U
#define VOLTAGE_INPUT_MAX 24U
#define VOLTAGE_OUTPUT_MIN 30U
#define VOLTAGE_OUTPUT_MAX 36U
#define VOLTAGE_CONTROL_SAMPLE_HZ 1000.0f
#define VOLTAGE_CONTROL_KP 3.0f
#define VOLTAGE_CONTROL_KI_PER_SAMPLE 0.020f
#define VOLTAGE_CONTROL_FILTER_ALPHA 0.10f
#define VOLTAGE_CONTROL_SOFTSTART_V_PER_S 2.0f
#define VOLTAGE_CONTROL_DUTY_MARGIN 100.0f
#define VOLTAGE_CONTROL_ABSOLUTE_MAX 650.0f
#define CURRENT_SENSOR_V_PER_A 1.0f
#define OVERCURRENT_TRIP_A 2.50f
#define OVERCURRENT_RESET_A 2.20f
#define OVERCURRENT_DEBOUNCE_SAMPLES 3U
#define OVERVOLTAGE_MARGIN_V 2.0f
#define OVERVOLTAGE_ABSOLUTE_V 38.0f
#define OVERVOLTAGE_DEBOUNCE_SAMPLES 5U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void Display();
void KeyToControl(int key);
void doADC();
void U_PID();
void mppt();
void *model_func_pointer[4] = {doADC,U_PID};
void paremeter_group_add();
//void paremeter_group_sub();
void paremeter_choose_add();
//void paremeter_choose_sub();
void paremeter_display();
void MainDutyAdjust(float duty_step);
void VoltageSettingEnter(void);
void VoltageSettingHandleKey(int key);
void VoltageSettingDisplay(void);
void VoltageSettingApply(void);
void VoltageControlStep(void);
void VoltageControlFault(uint8_t fault_code);
int KeyCodeToDigit(int key);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int mode_flag = 0,mppt_flag=0,mppt_try_flag=0,rs_flag=0;
volatile int main_duty_manual_flag = 0;
volatile int voltage_control_active = 0;
volatile uint8_t voltage_control_fault = 0;
int voltage_setting_page = 0, voltage_setting_value = 0;
int voltage_setting_digits = 0, voltage_setting_error = 0;
int voltage_setting_previous_mode = 0, voltage_setting_previous_manual = 0;
int voltage_setting_previous_control = 0;
float voltage_setting_vin = 18.0f;
float voltage_control_duty_min = MAIN_DUTY_MIN;
float voltage_control_duty_max = VOLTAGE_CONTROL_ABSOLUTE_MAX;
float voltage_control_integral = 0.0f;
float voltage_control_soft_setpoint = VOLTAGE_OUTPUT_MIN;
uint16_t overcurrent_count = 0, overvoltage_count = 0;
uint8_t adc_filter_initialized = 0;
int paremeter_group_flag = 0, paremeter_choose_flag = 0, paremeter_flag = 0;
int test;
int key_value=0,key_value0=0;
int time_flag=0,flag=0;
float vol0,vol1,vol2,vol3,vol4,vref;
volatile uint32_t DMA_data[DMA_len];
float Uo_set=30.0f,Uin_set=25.0f;
volatile float Uin,Iin,Uo,Us,Iout,Uo_instant,Iout_instant;
float Rs=10.5f,Rs1,Rs2,Rs3,Uin1,Iin1,Uin2,Iin2,Uin3,Iin3;
float tem_err1,tem_err2,uo_b=0.0,us_b=0.0,bias1=0.3f,bias2=0.1f;
float u1_duty=800.0,u2_duty=650.0;
float u1_p=12,u1_i=0.001;
float u2_p=12,u2_i=0.001;
//4组*3 可调参数
float *paremeter[12] = {&uo_b, &u1_p, &u1_i,
						&us_b,&u2_p, &u2_i,
						&bias1,&bias2,&bias2,
						&u1_duty,&u2_duty,&u2_duty};
PID_CONTROLER_F u1_loop,u2_loop;
//float test;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI3_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  OLED_RST_Set();//拉高屏幕复位引脚
  OLED_Init();
  OLED_Clear();
//  OLED_ShowString(0, 0, "Test");
//  OLED_ShowString(0, 2, "sin_of_key");
//  test = arm_sin_f32(0.5);
  PID_init_F(&u1_loop, u1_duty, u1_p, u1_i, 0, 950, 100);
  PID_init_F(&u2_loop, u2_duty, u2_p, u2_i, 0, 950, 100);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);

  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)DMA_data,DMA_len);//启动ADC
  HAL_TIM_Base_Start(&htim3);//启动定时器

  HAL_Delay(200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  	HAL_Delay(150);
		key_value0=key_value;
		key_value = getKeyValue();
		if (key_value != key_value0)
		{
		  KeyToControl(key_value);
		}
		Display();
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
  RCC_OscInitStruct.PLL.PLLN = 160;
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

/* USER CODE BEGIN 4 */
void doADC()//mode0
{
	float uo_sample, iout_sample;
//    if (time_flag < 1000)
//    {
//      time_flag++;
//      return;
//    }
//    time_flag = 0;

	vol0=DMA_data[0];//i
	vol1=DMA_data[1];//uo
	vol2=DMA_data[2];//ui
	vol3=DMA_data[3];//us
	vol4=DMA_data[4];

	if(vol4<100.0f)
		return;
	vref=1.21f*(4095.0f/vol4);

	Iin=((vol0/4095.0f)*vref)*0.9105f+0.0112f;
//	Uo=((vol1/4095.0f)*vref)*16.147f+0.034f+uo_b;
	uo_sample=((vol1/4095.0f)*vref)*20.5f;//Preserve the calibrated Uo scale.
	Uin=((vol2/4095.0f)*vref)*16.185f-0.012f;
	Us=((vol3/4095.0f)*vref)*20.93f+us_b;

	uo_sample=(Us>52.0f)?(uo_sample+bias1):((Us>48)?(uo_sample+bias2):uo_sample);
	iout_sample=((vol0/4095.0f)*vref)/CURRENT_SENSOR_V_PER_A;
	Uo_instant = uo_sample;
	Iout_instant = iout_sample;
	if(!adc_filter_initialized)
	{
		Uo = uo_sample;
		Iout = iout_sample;
		adc_filter_initialized = 1;
	}
	else
	{
		Uo += VOLTAGE_CONTROL_FILTER_ALPHA*(uo_sample-Uo);
		Iout += VOLTAGE_CONTROL_FILTER_ALPHA*(iout_sample-Iout);
	}
//	if(flag<2)
//	{
//		if(flag==0)
//		{
//			Uin1=Uin;
//			Iin1=Iin;
//			if(Uin1>29&&Uin1<31)
//			{
//				flag=1;
//				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)(u1_duty+100));//boost
//			}
//		}
//		if(flag==1)
//		{
//			Uin2=Uin;
//			Iin2=Iin;
//		}
//		if(fabs(Uin1-Uin2)>2)
//		{
//			Rs=fabs((Uin1-Uin2)/(Iin2-Iin1));
//			flag=2;
//		}
//	}

}

void U_PID()//mode1
{
	doADC();

	tem_err2=Uo-Uo_set;
	PID_step_F(&u2_loop, tem_err2);
	u2_duty=u2_loop.out;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint16_t)(u2_duty));//双向dcdc

	if(main_duty_manual_flag==0)
	{
		tem_err1=Us*0.5f-Uin;
		if(rs_flag==0)
			PID_step_F(&u1_loop, tem_err1);
		else
			PID_step_F(&u1_loop, Uin_set-Uin);
		u1_duty=u1_loop.out;
	}
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)(u1_duty));//boost

	PID_CHANGE_PARAM_F(&u1_loop,u1_p,u1_i,0);
	PID_CHANGE_PARAM_F(&u2_loop,u2_p,u2_i,0);
}

void mppt()
{
    if(mppt_try_flag==0)
    {
      mppt_try_flag=1;
    }
    else if(mppt_try_flag==1)
	{
		Uin1=Uin;
		Iin1=Iin;
		Uin_set=Uin+1;
		mppt_try_flag=2;
	}
    else if(mppt_try_flag==2)
    {
    	Uin2=Uin;
        Iin2=Iin;
        Rs1=fabs((Uin2-Uin1)/(Iin2-Iin1));
        Uin_set=Uin-2;
        mppt_try_flag=3;
    }
    else if(mppt_try_flag==3)
    {
    	Uin3=Uin;
    	Iin3=Iin;
        Rs2=fabs((Uin3-Uin1)/(Iin3-Iin1));
        Rs3=fabs((Uin3-Uin2)/(Iin3-Iin2));
      	Rs=(Rs1+Rs2+Rs3)/3.0f;
        Uin_set=(Uin+Iin*Rs)/2.0f;
        mppt_flag=0;
        mppt_try_flag=0;
    }
}

void Display()//x 0~127 一个6格
{
	float li_duty_percent = u1_duty/10.0f-MAIN_DEADTIME_PERCENT;
	float hi_duty_percent = MAIN_TOTAL_ON_PERCENT-li_duty_percent;

	if(voltage_setting_page!=0)
	{
		VoltageSettingDisplay();
		return;
	}

	OLED_ShowString(0, -1, "mode");
	OLED_ShowNum(30, -1, mode_flag, 1, 16);

	/* LI=TIM1_CH1, HI=TIM1_CH1N; both values include the configured dead time. */
	OLED_ShowString(0, 0, "L");
	OLED_ShowFloat(12, 0, li_duty_percent, 2, 1, 12);
	OLED_ShowString(48, 0, "H");
	OLED_ShowFloat(60, 0, hi_duty_percent, 2, 1, 12);
	OLED_ShowString(96, 0, "S");
	OLED_ShowNum(108, 0, (uint32_t)MAIN_TOTAL_ON_PERCENT, 2, 12);

	OLED_ShowString(0, 1, "UIi");
	OLED_ShowFloat(24, 1, Uin, 2, 3, 12);
	OLED_ShowFloat(60, 1, Iin, 2, 3, 12);

	OLED_ShowString(0, 2, "Uo");
	OLED_ShowFloat(24, 2, Uo, 2, 4, 12);

	OLED_ShowString(0, 3, "Rs");
	OLED_ShowFloat(24, 3, Rs, 2, 2, 12);
	OLED_ShowNum(80, 3, rs_flag, 1, 12);

	OLED_ShowString(0, 4, "Us");
	OLED_ShowFloat(24, 4, Us, 2, 4, 12);

	OLED_ShowString(0, 5, "Main");
	OLED_ShowFloat(30, 5, u1_duty/10.0f, 2, 1, 12);
	OLED_ShowChar(60, 5, '%');
	if(voltage_control_active)
		OLED_ShowString(72, 5, "VC");
	else
		OLED_ShowString(72, 5, "  ");

	OLED_ShowNum(96, 5, mppt_flag, 1, 12);
	OLED_ShowNum(112, 5, mppt_try_flag, 1, 12);

	if(voltage_control_active)
	{
		OLED_ShowString(0, 6, "Set");
		OLED_ShowNum(24, 6, (uint32_t)Uo_set, 2, 12);
		OLED_ShowString(48, 6, "In");
		OLED_ShowNum(66, 6, (uint32_t)Uin_set, 2, 12);
		OLED_ShowString(90, 6, "Io");
		OLED_ShowFloat(108, 6, Iout, 1, 1, 12);
	}
	else if(voltage_control_fault)
	{
		OLED_ShowString(0, 6, "FAULT");
		OLED_ShowNum(42, 6, voltage_control_fault, 1, 12);
	}
	else
		paremeter_display();

}

void MainDutyAdjust(float duty_step)
{
	voltage_control_active = 0;
	main_duty_manual_flag = 1;
	u1_duty += duty_step;
	u1_duty = (u1_duty > MAIN_DUTY_MAX) ? MAIN_DUTY_MAX :
			  ((u1_duty < MAIN_DUTY_MIN) ? MAIN_DUTY_MIN : u1_duty);

	/* Keep the PI state aligned so returning to automatic control is smooth. */
	u1_loop.out = u1_duty;
	u1_loop.out_last = u1_duty;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)(u1_duty));
}

int KeyCodeToDigit(int key)
{
	switch(key)
	{
	case 2:  return 1;
	case 1:  return 2;
	case 3:  return 3;
	case 6:  return 4;
	case 5:  return 5;
	case 7:  return 6;
	case 10: return 7;
	case 9:  return 8;
	case 11: return 9;
	case 13: return 0;
	default: return -1;
	}
}

void VoltageSettingEnter(void)
{
	voltage_setting_previous_mode = mode_flag;
	voltage_setting_previous_manual = main_duty_manual_flag;
	voltage_setting_previous_control = voltage_control_active;
	voltage_control_active = 0;
	main_duty_manual_flag = 1;
	voltage_setting_page = 1;
	voltage_setting_value = 0;
	voltage_setting_digits = 0;
	voltage_setting_error = 0;
	OLED_Clear();
}

void VoltageSettingApply(void)
{
	float duty_percent = (1.0f-Uin_set/Uo_set)*100.0f+MAIN_DEADTIME_PERCENT;

	u1_duty = duty_percent*10.0f;
	u1_duty = (u1_duty > MAIN_DUTY_MAX) ? MAIN_DUTY_MAX :
			  ((u1_duty < MAIN_DUTY_MIN) ? MAIN_DUTY_MIN : u1_duty);
	voltage_control_duty_min = MAIN_DUTY_MIN;
	voltage_control_duty_max = u1_duty+VOLTAGE_CONTROL_DUTY_MARGIN;
	voltage_control_duty_max = (voltage_control_duty_max>VOLTAGE_CONTROL_ABSOLUTE_MAX) ?
						   VOLTAGE_CONTROL_ABSOLUTE_MAX : voltage_control_duty_max;
	voltage_control_integral = 0.0f;
	voltage_control_soft_setpoint = Uo;
	if(voltage_control_soft_setpoint<Uin_set)
		voltage_control_soft_setpoint = Uin_set;
	if(voltage_control_soft_setpoint>Uo_set)
		voltage_control_soft_setpoint = Uo_set;
	voltage_control_fault = 0;
	overcurrent_count = 0;
	overvoltage_count = 0;
	__HAL_TIM_MOE_ENABLE(&htim1);
	/* Start at minimum duty; feed-forward follows the ramped reference. */
	u1_duty = MAIN_DUTY_MIN;
	u1_loop.out = u1_duty;
	u1_loop.out_last = u1_duty;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)u1_duty);

	/* Use the new voltage controller without changing the legacy MODE1 loops. */
	mode_flag = 0;
	main_duty_manual_flag = 1;
	voltage_control_active = 1;
	voltage_setting_page = 0;
	OLED_Clear();
}

void VoltageSettingHandleKey(int key)
{
	int digit;

	if(key==16) /* D: cancel */
	{
		mode_flag = voltage_setting_previous_mode;
		main_duty_manual_flag = voltage_setting_previous_manual;
		voltage_control_active = voltage_setting_previous_control;
		voltage_setting_page = 0;
		OLED_Clear();
		return;
	}

	if(key==14) /* *: backspace */
	{
		if(voltage_setting_digits>0)
		{
			voltage_setting_value /= 10;
			voltage_setting_digits--;
		}
		voltage_setting_error = 0;
		return;
	}

	if(key==15) /* #: confirm */
	{
		if(voltage_setting_digits==0)
		{
			voltage_setting_error = 1;
			return;
		}

		if(voltage_setting_page==1)
		{
			if(voltage_setting_value<(int)VOLTAGE_INPUT_MIN ||
			   voltage_setting_value>(int)VOLTAGE_INPUT_MAX)
			{
				voltage_setting_error = 1;
				return;
			}
			voltage_setting_vin = (float)voltage_setting_value;
			voltage_setting_page = 2;
			voltage_setting_value = 0;
			voltage_setting_digits = 0;
			voltage_setting_error = 0;
			OLED_Clear();
			return;
		}

		if(voltage_setting_value<(int)VOLTAGE_OUTPUT_MIN ||
		   voltage_setting_value>(int)VOLTAGE_OUTPUT_MAX ||
		   (float)voltage_setting_value<=voltage_setting_vin)
		{
			voltage_setting_error = 1;
			return;
		}

		Uin_set = voltage_setting_vin;
		Uo_set = (float)voltage_setting_value;
		VoltageSettingApply();
		return;
	}

	digit = KeyCodeToDigit(key);
	if(digit>=0 && voltage_setting_digits<2)
	{
		voltage_setting_value = voltage_setting_value*10+digit;
		voltage_setting_digits++;
		voltage_setting_error = 0;
	}
}

void VoltageSettingDisplay(void)
{
	OLED_ShowString(0, 0, "DC-DC SET");
	if(voltage_setting_page==1)
	{
		OLED_ShowString(0, 1, "VIN 18-24V");
		OLED_ShowString(0, 3, "VIN:");
	}
	else
	{
		OLED_ShowString(0, 1, "VIN:");
		OLED_ShowNum(30, 1, (uint32_t)voltage_setting_vin, 2, 12);
		OLED_ShowString(0, 2, "OUT 30-36V");
		OLED_ShowString(0, 3, "OUT:");
	}

	if(voltage_setting_digits==0)
		OLED_ShowString(36, 3, "--");
	else
		OLED_ShowNum(36, 3, (uint32_t)voltage_setting_value,
					 voltage_setting_digits, 16);

	if(voltage_setting_error)
		OLED_ShowString(0, 5, "RANGE ERROR");
	else
		OLED_ShowString(0, 5, "           ");
	OLED_ShowString(0, 6, "#OK *DEL DEXIT");
}

void VoltageControlStep(void)
{
	float error, feedforward, proportional, unsaturated, next_integral;
	float overvoltage_limit = Uo_set+OVERVOLTAGE_MARGIN_V;

	if(!voltage_control_active || voltage_setting_page!=0)
		return;

	/* INA282 gain 50 V/V and 20 mOhm shunt give approximately 1 V/A. */
	if(Iout_instant>=OVERCURRENT_TRIP_A)
		overcurrent_count++;
	else if(Iout_instant<=OVERCURRENT_RESET_A)
		overcurrent_count = 0;
	if(overcurrent_count>=OVERCURRENT_DEBOUNCE_SAMPLES)
	{
		VoltageControlFault(1);
		return;
	}

	if(overvoltage_limit>OVERVOLTAGE_ABSOLUTE_V)
		overvoltage_limit = OVERVOLTAGE_ABSOLUTE_V;
	if(Uo_instant>=overvoltage_limit)
		overvoltage_count++;
	else
		overvoltage_count = 0;
	if(overvoltage_count>=OVERVOLTAGE_DEBOUNCE_SAMPLES)
	{
		VoltageControlFault(2);
		return;
	}

	if(voltage_control_soft_setpoint<Uo_set)
	{
		voltage_control_soft_setpoint += VOLTAGE_CONTROL_SOFTSTART_V_PER_S/
									 VOLTAGE_CONTROL_SAMPLE_HZ;
		if(voltage_control_soft_setpoint>Uo_set)
			voltage_control_soft_setpoint = Uo_set;
	}

	feedforward = (1.0f-Uin_set/voltage_control_soft_setpoint)*1000.0f+
				  MAIN_DEADTIME_PERCENT*10.0f;
	error = voltage_control_soft_setpoint-Uo;
	proportional = VOLTAGE_CONTROL_KP*error;
	next_integral = voltage_control_integral+VOLTAGE_CONTROL_KI_PER_SAMPLE*error;
	unsaturated = feedforward+proportional+next_integral;

	/* Conditional integration prevents wind-up at either duty limit. */
	if((unsaturated<voltage_control_duty_max && unsaturated>voltage_control_duty_min) ||
	   (unsaturated>=voltage_control_duty_max && error<0.0f) ||
	   (unsaturated<=voltage_control_duty_min && error>0.0f))
		voltage_control_integral = next_integral;

	u1_duty = feedforward+proportional+voltage_control_integral;
	u1_duty = (u1_duty>voltage_control_duty_max) ? voltage_control_duty_max :
			  ((u1_duty<voltage_control_duty_min) ? voltage_control_duty_min : u1_duty);
	u1_loop.out = u1_duty;
	u1_loop.out_last = u1_duty;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)u1_duty);
}

void VoltageControlFault(uint8_t fault_code)
{
	voltage_control_fault = fault_code;
	voltage_control_active = 0;
	main_duty_manual_flag = 1;
	__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

void KeyToControl(int key)
{
	if(voltage_setting_page!=0)
	{
		VoltageSettingHandleKey(key);
		return;
	}

/* Physical layout (key code): 1(2) 2(1) 3(3) A(4)
 *                             4(6) 5(5) 6(7) B(8)
 *                             7(10)8(9) 9(11)C(12)
 *                             *(14)0(13)#(15)D(16) */
  switch (key)
  {
  case 2://physical 1: MODE0
	  mode_flag = 0;
	  main_duty_manual_flag = 0;
	  voltage_control_active = 0;
	  test=1;
	  u1_duty=800;
	  u2_duty=650;
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)(u1_duty));
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint16_t)(u2_duty));
    break;
  case 6://physical 4: MODE1
	  test=2;
	  main_duty_manual_flag = 0;
	  voltage_control_active = 0;
	  mode_flag = 1;
    break;
  case 10://physical 7
	  rs_flag=1-rs_flag;
	  test=3;
    break;
  case 14://physical *: reserved
	  test=4;
    break;
  case 1://physical 2
	  *paremeter[paremeter_flag] += 50;
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)(u1_duty));
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint16_t)(u2_duty));
	  test=5;
    break;
  case 5: //physical 5
	  *paremeter[paremeter_flag] -= 50;
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)(u1_duty));
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint16_t)(u2_duty));
	  test=6;
	break;
  case 9: //physical 8
	  test=7;
      *paremeter[paremeter_flag] += 10;
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)(u1_duty));
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint16_t)(u2_duty));
    break;
  case 13://physical 0
	  test=8;
	  *paremeter[paremeter_flag] -= 10;
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)(u1_duty));
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint16_t)(u2_duty));
    break;
  case 3://physical 3: enter voltage setting page
	  test=9;
	  VoltageSettingEnter();
    break;
  case 7://physical 6
	  test=10;
	  *paremeter[paremeter_flag] -= 1;
    break;
  case 11://physical 9
	  test=11;
	  *paremeter[paremeter_flag] += 0.1;
    break;
  case 15://physical #
	  test=12;
	  *paremeter[paremeter_flag] -= 0.1;
    break;
  case 4://physical A: main duty +1%
	  test=13;
	  MainDutyAdjust(MAIN_DUTY_STEP);
    break;
  case 8://physical B: main duty -1%
	  test=14;
	  MainDutyAdjust(-MAIN_DUTY_STEP);
    break;
  case 12://physical C
	  test=15;
	  paremeter_group_add();
    break;
  case 16://physical D
	  test=16;Rs-=0.1;
	  paremeter_choose_add();
    break;
  default:
    break;
  }
}

void paremeter_group_add()
{
  paremeter_group_flag = (paremeter_group_flag + 1) % PAREMETER_GROUP_COUNT;
  paremeter_choose_flag = 0;
  paremeter_flag = paremeter_group_flag * PAREMETER_CHOOSE_COUNT + paremeter_choose_flag;
  OLED_ShowString(0, 6, "              ");
}

void paremeter_choose_add()
{
  paremeter_choose_flag = (paremeter_choose_flag + 1) % PAREMETER_CHOOSE_COUNT;
  paremeter_flag = paremeter_group_flag * PAREMETER_CHOOSE_COUNT + paremeter_choose_flag;
  OLED_ShowString(0, 6, "              ");
}

//{&Uo_set, &spwm_p,&spwm_i,
//&fre, &pwm_duty,&pwm_duty};
void paremeter_display()
{
  switch (paremeter_flag)
  {
  case 0:
	OLED_ShowString(0, 6, "uo_b");
	OLED_ShowFloat(36, 6, *paremeter[0], 4, 4, 12);
    break;
  case 1:
	OLED_ShowString(0, 6, "u1_p");
	OLED_ShowFloat(36, 6, *paremeter[1], 4, 4, 12);
    break;
  case 2:
	OLED_ShowString(0, 6, "u1_i");
	OLED_ShowFloat(36, 6, *paremeter[2], 4, 4, 12);
    break;
  case 3:
	OLED_ShowString(0, 6, "us_b");
	OLED_ShowFloat(36, 6, *paremeter[3], 4, 4, 12);
    break;
  case 4:
	OLED_ShowString(0, 6, "u2_p");
	OLED_ShowFloat(36, 6, *paremeter[4], 4, 4, 12);
    break;
  case 5:
	OLED_ShowString(0, 6, "u2_i");
	OLED_ShowFloat(36, 6, *paremeter[5], 4, 4, 12);
    break;
  case 6:
	OLED_ShowString(0, 6, "bias1");
	OLED_ShowFloat(36, 6, *paremeter[6], 4, 4, 12);
    break;
  case 7:
	OLED_ShowString(0, 6, "bias2");
	OLED_ShowFloat(36, 6, *paremeter[7], 4, 4, 12);
    break;
  case 8:
	OLED_ShowString(0, 6, "bias2");
	OLED_ShowFloat(36, 6, *paremeter[8], 4, 4, 12);
    break;
  case 9:
	OLED_ShowString(0, 6, "duty1");
	OLED_ShowFloat(36, 6, *paremeter[9], 4, 4, 12);
    break;
  case 10:
	OLED_ShowString(0, 6, "duty2");
	OLED_ShowFloat(36, 6, *paremeter[10], 4, 4, 12);
    break;
  case 11:
	OLED_ShowString(0, 6, "duty2");
	OLED_ShowFloat(36, 6, *paremeter[11], 4, 4, 12);
    break;
  default:
    break;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 涓柇鏈嶅姟鍑芥暟鑷繁缂栧啓
{
  if (htim->Instance == TIM2)
  {
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	if(!voltage_control_active)
	{
	  void *p = model_func_pointer[mode_flag];
	  (*(unsigned int (*)(void))p)();
	}
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
  }

  if (htim->Instance == TIM4&&mppt_flag)
  {
	  mppt();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance==ADC1)
	{
		doADC();
		VoltageControlStep();
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
