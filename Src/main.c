
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_iwdg.h"
#include "MPU6050.h"
#include "math.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//IWDG_HandleTypeDef hiwdg;
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
//static void MX_IWDG_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*
Constants 
*/
#define M_PI	3.16159265358979323846
#define T_SEC	122880000
#define SCS_SignalPeriod 0x2FF

#define SCS_BaseVoltage	0x935//0x9FC // V = 2.06
#define SCS_InitialVoltage	0xD00//0xCEE//((0xcc5)0xC1E) // V = 2.66
#define SCS_UpperlVoltage	0xFFF // V = 3.27
#define SCS_VoltageRange	0x5DD // V = 1.21
#define SCS_VoltageAmplitude	0x2F4 // V = 0.60

/*
Controls
*/
#define SCS_Sensor_isEnable	0x1
#define SCS_Joystick_isEnable	0x2
#define SCS_Settings_isEnable	0x3
#define SCS_DisableAll	0x4

/*
Sensitivity
*/
#define SCS_MaximumAngleSensitivity_Low	40
#define SCS_MaximumAngleSensitivity_Normal	30
#define SCS_MaximumAngleSensitivity_Hight	20

#define SCS_SaveAngleSensitivity	5

/*
Sensitivity settings
*/
#define SCS_GyroscopeValueForSensitivityStep_Y	30

/*
Interrupt
*/
#define SCS_AccelerometerLimit	1.2
#define SCS_ADCVoltageDelta	0.3 //0.3

struct GyroscopeValueXYZ
{
	float xGValue;
	float yGValue;
	float zGValue;
};

struct AngleOfRotationXYZAxis
{
	float xAngle;
	float yAngle;
	float zAngle;
};

struct AngleOfRotationXYZAxis xyzStartAngles = {0,0,0};
int maximumAngleSensitivity = 30;
int controlFlag = SCS_Sensor_isEnable;


/*
	Управление с помощью датчика
*/
void sensorControl();
/*
	Получение угла наклона датчика
	@bref вторичный катет - проекция на ось, угол между которой и вектором ускрения(g) является искомым; 
				первичные катеты - проекции на остальные две оси.
	@param secondaryCatheter вторичный катет, который учавствует в поиске угла наклона межд
	@param primaryCatheter_1 первичный катет, который учавствует в поиске гипотенузы, 
													 которая является вторичним катетом в последующем поиске угла наклона
	@param primaryCatheter_2 первичный катет, который учавствует в поиске гипотенузы, 
													 которая является вторичним катетом в последующем поиске угла наклона
	@return угол наклона
*/
float angleFromProjections(float secondaryCatheter, float primaryCatheter_1, float primaryCatheter_2);
/*
	Получить текущие углы наклона датика с начальным смещением
	@return AngleOfRotationXYZAxis текущие углы наклона с начальным смещением
*/
struct AngleOfRotationXYZAxis obtainCurrentAngleOfRotationWithStartAngle();
/*
	Передача сигнала через ЦАП для движения в сторону
	@param dacValue значение для вывода
*/
void sidewaysDACWithDACValue(int dacValue);
/*
	Передача сигнала через ЦАП для движения прямо
	@param dacValue значение для вывода
*/
void straightDACWithDACValue(int dacValue);
/*
	Передача данных в ЦАП для движение вперед
	@param currentAngel текущий угол в градусах
*/
void straightMotionDACWithCurrentAngel(float currentAngel);
/*
	Передача данных в ЦАП для движение в сторону
	@param currentAngel текущий угол в градусах
*/
void sidewaysMotionDACWithCurrentAngel(float currentAngel);
/*
	Конвертация градусов текущего угла в даные для ЦАП
	@param currentAngel текущий угол в градусах
	@return значение для ЦАП, нормированное на полную шкалу ЦАП 3,3В (0xFFF)
*/
int convertAngelToDACValue(float currentAngel);
/*
	Получить текущие углы наклона датика
	@return AngleOfRotationXYZAxis текущие углы наклона
*/
struct AngleOfRotationXYZAxis obtainCurrentAngleOfRotation();
/*
	Получить углы начального смещения датика
	@return AngleOfRotationXYZAxis начальные углы наклона
*/
struct AngleOfRotationXYZAxis createStartAngleOfRotation();
/*
	Получить текущие значение напряжения в вольтах с джойстика для движения вперед
	@return текущие напряжение в вольтах
*/
float straightMotionVoltageADC();
/*
	Получить текущие значение напряжения в вольтах с джойстика для движения в сторону
	@return текущие напряжение в вольтах
*/
float sidewaysMotionVoltageADC();

/*
	Запустить таймер
*/
void startTimer();
/*
	Остановить таймер
*/
void stopTimer();
/*
	Поизвести звуковые сигналы заданное количество раз
	@param numberOfSignals заданное количество сигналов 
*/
void signalsWithNumberOfSignals(int numberOfSignals);
/*
	Осуществляет синхронную задержку
*/
void synchronousDelay();
/*
	Вывод звкука
	@param signalPeriod период сигнала
*/
void speakerWithSignalPeriod(float signalPeriod);
/*
	Получить текущие проекции ускорения(a) на оси
	@return GyroscopeValueXYZ текущие проекции ускорения(a)
*/
struct GyroscopeValueXYZ obtainCurrentGyroscopeValue();
/*
	Конвертация градусов текущего угла в период сигнала для динамика
	@param currentAngel текущий угол в градусах
	@return период сигнала
*/
float convertCurrentXAngelToSignalPeriod(float currentXAngel);
/*
Управление с помощью джойстика
*/
void joystickControl();
/*
	Настройки чувствительности датчика
*/
void sensitivitySetting();
/*
	Отслеижвание выхода текущего значения угла за допустимые границы
	@param currentAngel_X текущее значение угла
	@param currentAngel_Y текущее значение угла	
*/
void emergencyTrackingWithCurrentAngels(float currentAngel_X, float currentAngel_Z);


float (*straightMotionVoltageADCPointer)() = &straightMotionVoltageADC;
float (*sidewaysMotionVoltageADCPointer)() = &sidewaysMotionVoltageADC;

void (*signalsWithNumberOfSignalsPointer)(int) = signalsWithNumberOfSignals;

#define mpu9265Address	MPU6050_DEFAULT_ADDRESS
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
//	MX_IWDG_Init();
  /* USER CODE BEGIN 2 */



//	for(uint8_t i=0; i<255; i++)
//	{
//		if(HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 10) != HAL_OK)
//		{
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
//			break;
//		}
//	}
	while (HAL_I2C_IsDeviceReady(&hi2c1, mpu9265Address, 5, 10) != HAL_OK) __asm__ ("NOP");
	Initialize();
	xyzStartAngles = createStartAngleOfRotation();
	HAL_ADC_Start(&hadc1);
	HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
//	HAL_TIM_Base_Start_IT(&htim2);
//	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while((2 > straightMotionVoltageADC()) 
				|| (2 > sidewaysMotionVoltageADC())){}
	for (int i = 0; i < 100000; i++){}
	straightDACWithDACValue(hadc1.Instance->DR);
	sidewaysDACWithDACValue(hadc1.Instance->JDR1);
//	straightDACWithDACValue(SCS_InitialVoltage);
//	sidewaysDACWithDACValue(SCS_InitialVoltage);
	
//	for (int i = 0; i < 100000; i++){}
//	for (int i = 0; i < 100000; i++){}
	HAL_Delay(2000);
	xyzStartAngles = createStartAngleOfRotation();
	HAL_TIM_Base_Start_IT(&htim2);
	signalsWithNumberOfSignals(1);
  while (1)
  {
//		HAL_IWDG_Refresh(&hiwdg);
		if (SCS_Joystick_isEnable == controlFlag)
		{
//			synchronousDelay();
			signalsWithNumberOfSignals(2);
//			HAL_TIM_Base_Sta_IT(&htim2);
			HAL_TIM_Base_Stop_IT(&htim2);
			joystickControl();
		}
		if (SCS_Sensor_isEnable == controlFlag)
		{
//			synchronousDelay();
			HAL_TIM_Base_Start_IT(&htim2);
			signalsWithNumberOfSignals(3);
			sensorControl();
		}
		if (SCS_DisableAll == controlFlag)
		{
			straightDACWithDACValue(SCS_InitialVoltage);
			sidewaysDACWithDACValue(SCS_InitialVoltage);
			startTimer();
			speakerWithSignalPeriod(SCS_SignalPeriod);
			while (SCS_DisableAll == controlFlag);
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

void joystickControl()
{
	while(SCS_Joystick_isEnable == controlFlag)
	{
//		HAL_IWDG_Refresh(&hiwdg);
		if (obtainCurrentGyroscopeValue().yGValue > SCS_AccelerometerLimit)
		{
			controlFlag = SCS_Sensor_isEnable;
		}
		straightDACWithDACValue(hadc1.Instance->DR);
		sidewaysDACWithDACValue(hadc1.Instance->JDR1);
	}
}

void sensorControl()
{
	struct AngleOfRotationXYZAxis xyzCurrentAnglesWithStartAngles = {0,0,0};
	while(SCS_Sensor_isEnable == controlFlag)
	{		
//		HAL_IWDG_Refresh(&hiwdg);
		if (obtainCurrentGyroscopeValue().yGValue > SCS_AccelerometerLimit)
		{
//			synchronousDelay();
			signalsWithNumberOfSignals(4);
			
			controlFlag = SCS_Settings_isEnable;
			sensitivitySetting();
		}
		xyzCurrentAnglesWithStartAngles = obtainCurrentAngleOfRotationWithStartAngle();
		straightMotionDACWithCurrentAngel(xyzCurrentAnglesWithStartAngles.xAngle);
		sidewaysMotionDACWithCurrentAngel(xyzCurrentAnglesWithStartAngles.zAngle);
		emergencyTrackingWithCurrentAngels(xyzCurrentAnglesWithStartAngles.xAngle, xyzCurrentAnglesWithStartAngles.zAngle);
//		enableTimerInterruptIfNeeded(htim2);
	}
}

//void sensorControl()
//{
//	struct AngleOfRotationXYZAxis xyzCurrentAnglesWithStartAngles = {0,0,0};
//	xyzCurrentAnglesWithStartAngles = obtainCurrentAngleOfRotationWithStartAngle();
//	straightMotionDACWithCurrentAngel(xyzCurrentAnglesWithStartAngles.xAngle);
//	sidewaysMotionDACWithCurrentAngel(xyzCurrentAnglesWithStartAngles.zAngle);
//}

void sensitivitySetting()
{
	startTimer();
	while(SCS_Settings_isEnable == controlFlag)
	{
		float xAngle = obtainCurrentAngleOfRotationWithStartAngle().xAngle;
		speakerWithSignalPeriod(convertCurrentXAngelToSignalPeriod(xAngle));
		if (xAngle > SCS_GyroscopeValueForSensitivityStep_Y)
		{
			maximumAngleSensitivity = SCS_MaximumAngleSensitivity_Hight;
		}
		else if (xAngle > - SCS_GyroscopeValueForSensitivityStep_Y)
		{
			maximumAngleSensitivity = SCS_MaximumAngleSensitivity_Normal;
		}
		else 
		{
			maximumAngleSensitivity = SCS_MaximumAngleSensitivity_Low;
		}
		
		if (obtainCurrentGyroscopeValue().yGValue > SCS_AccelerometerLimit)
		{
			stopTimer();
			signalsWithNumberOfSignals(5);
			
			controlFlag = SCS_Sensor_isEnable;
		}
	}
}

void emergencyTrackingWithCurrentAngels(float currentAngel_X, float currentAngel_Z)
{
	float maximumCurrentAngel = fmax(fabsf(currentAngel_X), fabsf(currentAngel_Z));
	if (maximumCurrentAngel > maximumAngleSensitivity * 1.5)
	{
		signalsWithNumberOfSignals(1);
		TIM3->SR = 0x0;
		HAL_TIM_Base_Start_IT(&htim3);
	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim3);
		htim3.Instance->CNT = 0;
	}
}

float convertCurrentXAngelToSignalPeriod(float currentXAngel)
{
		int fullScale = 0x2FFF;
		float signalPeriod = fullScale / 4 * (3 - currentXAngel / SCS_MaximumAngleSensitivity_Low);

		if (signalPeriod >= fullScale)
		{
			return fullScale;
		}
		if (signalPeriod <= fullScale / 2)
		{
			return fullScale / 2;
		}
		return signalPeriod;
}

void signalsWithNumberOfSignals(int numberOfSignals)
{
	for(int currentSignalNumber = 0; currentSignalNumber < numberOfSignals; ++currentSignalNumber)
	{
		startTimer();
		speakerWithSignalPeriod(SCS_SignalPeriod);
		synchronousDelay();
		
		stopTimer();
		synchronousDelay();
	}
}

struct GyroscopeValueXYZ obtainCurrentGyroscopeValue()
{
	MPU6050_GYROResult gyroscopeXYZValue = {0,0,0,0};
	HAL_StatusTypeDef status = HAL_ERROR;

	status = GetRawGyro(&gyroscopeXYZValue);
//	while(HAL_BUSY == status){};
	struct GyroscopeValueXYZ gValueXYZ = {0,0,0};
	gValueXYZ.xGValue = gyroscopeXYZValue.Gyroscope_X / 16384.0;
	gValueXYZ.yGValue = gyroscopeXYZValue.Gyroscope_Y / 16384.0;
	gValueXYZ.zGValue = gyroscopeXYZValue.Gyroscope_Z / 16384.0;

	return gValueXYZ;
}

struct AngleOfRotationXYZAxis createStartAngleOfRotation()
{
	return obtainCurrentAngleOfRotation();
}

struct AngleOfRotationXYZAxis obtainCurrentAngleOfRotationWithStartAngle()
{	
	struct AngleOfRotationXYZAxis xyzAngles = {0,0,0};
	
	xyzAngles.xAngle = obtainCurrentAngleOfRotation().xAngle - xyzStartAngles.xAngle;
	xyzAngles.yAngle = obtainCurrentAngleOfRotation().yAngle - xyzStartAngles.yAngle;
	xyzAngles.zAngle = obtainCurrentAngleOfRotation().zAngle - xyzStartAngles.zAngle;
	
	return xyzAngles;
}

struct AngleOfRotationXYZAxis obtainCurrentAngleOfRotation()
{
	MPU6050_ACCResult accelerometrXYZValue = {0,0,0,0};
	HAL_StatusTypeDef status = HAL_ERROR;

	status = GetRawAcc(&accelerometrXYZValue);
//	while(HAL_BUSY == status){};
	
	float ax = (accelerometrXYZValue.Accelerometr_X / 16384.00); //- 0.09326169;
	float ay = (accelerometrXYZValue.Accelerometr_Y / 16384.0);
	float az = (accelerometrXYZValue.Accelerometr_Z / 16384.0); //- 0.01269531;
	
	struct AngleOfRotationXYZAxis xyzAngles = {0,0,0};
	xyzAngles.xAngle = angleFromProjections(ax, ay, az);
	xyzAngles.yAngle = angleFromProjections(ay, az, ax);
	xyzAngles.zAngle = angleFromProjections(az, ax, ay);
	
	return xyzAngles;
}

//float currentADCValueWithHandleType(ADC_HandleTypeDef handleTypeADC)
//{
//	HAL_ADC_Start(&handleTypeADC);
//	float currentADCValue = 0.0;
//	if (HAL_ADC_PollForConversion(&handleTypeADC, 100) == HAL_OK)
//	{
//		currentADCValue = (float)HAL_ADC_GetValue(&handleTypeADC);
//	}
//	HAL_ADC_Stop(&handleTypeADC);
//	return currentADCValue;
//}

void straightMotionDACWithCurrentAngel(float currentAngel)
{
	int dacValue = convertAngelToDACValue(currentAngel);
	straightDACWithDACValue(dacValue);
}

void sidewaysMotionDACWithCurrentAngel(float currentAngel)
{
	int dacValue = convertAngelToDACValue(currentAngel);
	sidewaysDACWithDACValue(dacValue);
}

void straightDACWithDACValue(int dacValue)
{
	DAC->DHR12R1 = dacValue;
	DAC->SWTRIGR|=1;
}

void sidewaysDACWithDACValue(int dacValue)
{
	DAC->DHR12R2= dacValue;
	DAC->SWTRIGR|=2;
}

float angleFromProjections(float secondaryCatheter, float primaryCatheter_1, float primaryCatheter_2)
{
	float hypotenuse = sqrt(pow(primaryCatheter_1, 2) + pow(primaryCatheter_2, 2));
	return (atan2(secondaryCatheter, hypotenuse) * 180 / M_PI) - 90;
}

int convertAngelToDACValue(float currentAngel)
{	
	if (currentAngel >= maximumAngleSensitivity)
	{
		return SCS_BaseVoltage;
	}
	else if (currentAngel <= - maximumAngleSensitivity)
	{
		return SCS_UpperlVoltage;
	}
	else
	{
		return SCS_InitialVoltage;
	}
}

float straightMotionVoltageADC()
{
	float currentADCValue = hadc1.Instance->DR;
	return (currentADCValue) * 3.3 / 4096;
}

float sidewaysMotionVoltageADC()
{
	float currentADCValue = hadc1.Instance->JDR1;
	return (currentADCValue) * 3.3 / 4096;
}

void startTimer()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void stopTimer()
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

void synchronousDelay()
{
	for(int i = 0; i < T_SEC / 10; ++i){}
}

void speakerWithSignalPeriod(float signalPeriod)
{
		TIM1->ARR = signalPeriod;
		TIM1->CCR1 = signalPeriod / 2;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

//static void MX_IWDG_Init(void)
//{

//  hiwdg.Instance = IWDG;
//  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
//  hiwdg.Init.Reload = 512;
//  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_480CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
//static void MX_TIM1_Init(void)
//{

//  TIM_MasterConfigTypeDef sMasterConfig;
//  TIM_OC_InitTypeDef sConfigOC;
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 0xF;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 0x2FFF;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = 0;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//  HAL_TIM_MspPostInit(&htim1);

//}

static void MX_TIM1_Init(void)
{
	
	__HAL_RCC_TIM1_CLK_ENABLE();
	HAL_TIM_PWM_MspInit(&htim1);
	htim1.Instance = TIM1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Prescaler = 0xF;
	htim1.Init.Period = 0xFFF;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim1);
	TIM_OC_InitTypeDef pwm;
	pwm.OCMode = TIM_OCMODE_PWM2;
	pwm.Pulse = 0;
	pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
	pwm.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_1);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 250;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0x4FF;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
