
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
//#pragma push
//#pragma O0

#include "main.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdbool.h"


/*
Constants 
*/
#define M_PI	3.16159265358979323846
#define T_SEC	12288000
#define V_VCC	2.5

#define SCS_BaseVoltage	0x83E // V = 1.7
#define SCS_InitialVoltage	0xC1E // V = 2.5
#define SCS_UpperlVoltage	0xFFF // V = 3.3
#define SCS_VoltageAmplitude	0x7C1 // V = 1.6

#define SCS_SignalPeriod 0x5FF


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
#define SCS_AccelerometerLimit	0.8
#define SCS_ADCVoltageDelta	0.5   //  (V_VCC / 2 - SCS_ADCVoltageDelta; V_VCC / 2 + SCS_ADCVoltageDelta)


ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

static TIM_HandleTypeDef htim1 = {TIM1};
static TIM_HandleTypeDef htim2 = {TIM2};
static TIM_HandleTypeDef htim3 = {TIM3};

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


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/*
	Запретить прерывание, если оно разрешено для таймера
	@param htim необходимый таймер
*/
void disableTimerInterruptIfNeeded(TIM_HandleTypeDef htim);
/*
	Разрешить прерывание, если оно запрещено для таймера
	@param htim необходимый таймер
*/
void enableTimerInterruptIfNeeded(TIM_HandleTypeDef htim);
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
	Подготовка I2C у MPU-6050
*/
void prepareI2C();
/*
	Получение данных с MPU-6050
	@param address адрес, по которому следует прочитать данные 
	@return прочитанное значение
*/
int readI2C(int address);


/*
	Получить проекцию ускорение(а) на ось Х
	@return проекция ускорение(а)
*/
float obtainGyroscopeValueX();
/*
	Получить проекцию ускорение(а) на ось Y
	@return проекция ускорение(а)
*/
float obtainGyroscopeValueY();
/*
	Получить проекцию ускорение(а) на ось Z
	@return проекция ускорение(а)
*/
float obtainGyroscopeValueZ();


/*
	Получить проекцию ускорение(g) на ось Х
	@return проекция ускорение(g)
*/
float obtainAccelerationXAxis();
/*
	Получить проекцию ускорение(g) на ось Y
	@return проекция ускорение(g)
*/
float obtainAccelerationYAxis();
/*
	Получить проекцию ускорение(g) на ось Z
	@return проекция ускорение(g)
*/
float obtainAccelerationZAxis();


/*
	Отслеижвание выхода текущего значения угла за допустимые границы
	@param currentAngel_X текущее значение угла
	@param currentAngel_Y текущее значение угла	
*/
void emergencyTrackingWithCurrentAngels(float currentAngel_X, float currentAngel_Z);
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
	Получить текущие значение напряжения от АЦП в вольтах
	@param handleTypeADC структура, которая содержит информацию о конфигурации для указанного АЦП
	@return напряжение в вольтах
*/
float currentVoltageWithHandleType(ADC_HandleTypeDef handleTypeADC);
/*
	Получить текущие значение от АЦП
	@param handleTypeADC структура, которая содержит информацию о конфигурации для указанного АЦП
	@return текущие значение
*/
float currentADCValueWithHandleType(ADC_HandleTypeDef handleTypeADC);
/*
	Конвертация градусов текущего угла в период сигнала для динамика
	@param currentAngel текущий угол в градусах
	@return период сигнала
*/
float convertCurrentXAngelToSignalPeriod(float currentXAngel);
/*
	Вывод звкука
	@param signalPeriod период сигнала
*/
void speakerWithSignalPeriod(float signalPeriod);


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
	Получить текущие углы наклона датика с начальным смещением
	@return AngleOfRotationXYZAxis текущие углы наклона с начальным смещением
*/
struct AngleOfRotationXYZAxis obtainCurrentAngleOfRotationWithStartAngle();
/*
	Получить текущие проекции ускорения(a) на оси
	@return GyroscopeValueXYZ текущие проекции ускорения(a)
*/
struct GyroscopeValueXYZ obtainCurrentGyroscopeValue();
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
Управление с помощью джойстика
*/
void joystickControl();
/*
	Управление с помощью датчика
*/
void sensorControl();
/*
	Настройки чувствительности датчика
*/
void sensitivitySetting();


struct AngleOfRotationXYZAxis xyzStartAngles = {sizeof(float),
																								sizeof(float),
																								sizeof(float)};
int maximumAngleSensitivity = SCS_MaximumAngleSensitivity_Normal;
int controlFlag = SCS_Sensor_isEnable;

																								
void TIM3_IRQHandler()
{
	HAL_TIM_IRQHandler(&htim3);
}

void TIM2_IRQHandler() 
{
	HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (TIM3 == htim->Instance && 0x1 == htim->Instance->CR1)
	{
		controlFlag = SCS_DisableAll;
		disableTimerInterruptIfNeeded(htim3);
	}
	else if (TIM2 == htim->Instance)
	{
		float sumOfVoltages = straightMotionVoltageADC();
		sumOfVoltages += sidewaysMotionVoltageADC();
		if (2 * V_VCC - SCS_ADCVoltageDelta > sumOfVoltages
				|| 2 * V_VCC + SCS_ADCVoltageDelta < sumOfVoltages)
		{
			controlFlag = SCS_Joystick_isEnable;
			synchronousDelay();
			signalsWithNumberOfSignals(1);
			disableTimerInterruptIfNeeded(htim2);
		}
	}
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_DAC_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	
	straightDACWithDACValue(SCS_InitialVoltage);
	sidewaysDACWithDACValue(SCS_InitialVoltage);
	
	synchronousDelay();
	
	prepareI2C();
	
	xyzStartAngles = createStartAngleOfRotation();
	while (1)
  {
		if (SCS_Joystick_isEnable == controlFlag)
		{
			synchronousDelay();
			signalsWithNumberOfSignals(2);
			joystickControl();
		}
		if (SCS_Sensor_isEnable == controlFlag)
		{
			synchronousDelay();
			signalsWithNumberOfSignals(3);
			sensorControl();
		}
		if (SCS_DisableAll == controlFlag)
		{
			straightMotionDACWithCurrentAngel(xyzStartAngles.xAngle);
			sidewaysMotionDACWithCurrentAngel(xyzStartAngles.zAngle);
			startTimer();
			speakerWithSignalPeriod(SCS_SignalPeriod);
			while (SCS_DisableAll == controlFlag);
		}
  }
}

void joystickControl()
{
	while(SCS_Joystick_isEnable == controlFlag)
	{
		if (obtainCurrentGyroscopeValue().yGValue > SCS_AccelerometerLimit)
		{
			controlFlag = SCS_Sensor_isEnable;
		}
		straightDACWithDACValue(currentADCValueWithHandleType(hadc1));
		sidewaysDACWithDACValue(currentADCValueWithHandleType(hadc2));
	}
}

void disableTimerInterruptIfNeeded(TIM_HandleTypeDef htim)
{
	if (((TIM2 == htim.Instance && SCS_Joystick_isEnable == controlFlag)
				|| TIM3 == htim.Instance) && 0x1 == htim.Instance->DIER)
	{
		HAL_TIM_Base_Stop_IT(&htim);
		if (TIM3 == htim.Instance)
		{
			htim.Instance->CNT = 0;
		}
	}
}

void enableTimerInterruptIfNeeded(TIM_HandleTypeDef htim)
{
	if (((TIM2 == htim.Instance && SCS_Sensor_isEnable == controlFlag)
				|| TIM3 == htim.Instance)&& 0x0 == htim.Instance->DIER)
	{
		HAL_TIM_Base_Start_IT(&htim);
		if (TIM3 == htim.Instance)
		{
			HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
		}
	}
}

void sensorControl()
{
	struct AngleOfRotationXYZAxis xyzCurrentAnglesWithStartAngles = {0,0,0};
	while(SCS_Sensor_isEnable == controlFlag)
	{		
		if (obtainCurrentGyroscopeValue().yGValue > SCS_AccelerometerLimit)
		{
			synchronousDelay();
			signalsWithNumberOfSignals(4);
			
			controlFlag = SCS_Settings_isEnable;
			sensitivitySetting();
		}
		xyzCurrentAnglesWithStartAngles = obtainCurrentAngleOfRotationWithStartAngle();
		emergencyTrackingWithCurrentAngels(xyzCurrentAnglesWithStartAngles.xAngle, xyzCurrentAnglesWithStartAngles.zAngle);
		straightMotionDACWithCurrentAngel(xyzCurrentAnglesWithStartAngles.xAngle);
		sidewaysMotionDACWithCurrentAngel(xyzCurrentAnglesWithStartAngles.zAngle);
		enableTimerInterruptIfNeeded(htim2);
	}
}

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

float straightMotionVoltageADC()
{
	return currentVoltageWithHandleType(hadc1);
}

float sidewaysMotionVoltageADC()
{
	return currentVoltageWithHandleType(hadc2);
}

float currentVoltageWithHandleType(ADC_HandleTypeDef handleTypeADC)
{
	float currentADCValue = currentADCValueWithHandleType(handleTypeADC);
	return (currentADCValue) * 3.3 / 4096;
}

float currentADCValueWithHandleType(ADC_HandleTypeDef handleTypeADC)
{
	HAL_ADC_Start(&handleTypeADC);
	HAL_ADC_PollForConversion(&handleTypeADC, 100);
	float currentADCValue = (float)HAL_ADC_GetValue(&handleTypeADC);
	HAL_ADC_Stop(&handleTypeADC);
	return currentADCValue;
}

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

void emergencyTrackingWithCurrentAngels(float currentAngel_X, float currentAngel_Z)
{
	float maximumCurrentAngel = fmax(fabsf(currentAngel_X), fabsf(currentAngel_Z));
	if (maximumCurrentAngel > maximumAngleSensitivity * 1.25)
	{
		signalsWithNumberOfSignals(1);
		enableTimerInterruptIfNeeded(htim3);
	}
	else
	{
		disableTimerInterruptIfNeeded(htim3);
	}
}

int convertAngelToDACValue(float currentAngel)
{	
	if (currentAngel > maximumAngleSensitivity)
	{
		return SCS_UpperlVoltage;
	}
	else if (currentAngel < - maximumAngleSensitivity)
	{
		return SCS_BaseVoltage;
	}
	else if (currentAngel < SCS_SaveAngleSensitivity 
						&& currentAngel > - SCS_SaveAngleSensitivity)
	{
		return SCS_InitialVoltage;
	}
	
	int saveAngle = 0;
	if (currentAngel > 0)
	{
		saveAngle = SCS_SaveAngleSensitivity;
	}
	else
	{
		saveAngle = - SCS_SaveAngleSensitivity;
	}
	return SCS_BaseVoltage + SCS_InitialVoltage / 2 * (1 + ((currentAngel - saveAngle) / (maximumAngleSensitivity - SCS_SaveAngleSensitivity)));
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

struct GyroscopeValueXYZ obtainCurrentGyroscopeValue()
{
	struct GyroscopeValueXYZ gValueXYZ = {0,0,0};
	gValueXYZ.xGValue = obtainGyroscopeValueX();
	gValueXYZ.yGValue = obtainGyroscopeValueY();
	gValueXYZ.zGValue = obtainGyroscopeValueZ();
	
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
	float ax = obtainAccelerationXAxis();
	float ay = obtainAccelerationYAxis();
	float az = obtainAccelerationZAxis();

	struct AngleOfRotationXYZAxis xyzAngles = {0,0,0};
	xyzAngles.xAngle = angleFromProjections(ax, ay, az);
	xyzAngles.yAngle = angleFromProjections(ay, az, ax);
	xyzAngles.zAngle = angleFromProjections(az, ax, ay);
	
	return xyzAngles;
}

float angleFromProjections(float secondaryCatheter, float primaryCatheter_1, float primaryCatheter_2)
{
	float hypotenuse = sqrt(pow(primaryCatheter_1,2) + pow(primaryCatheter_2,2));
	return (atan2(secondaryCatheter, hypotenuse) * 180 / M_PI) - 90;
}


#pragma mark - Work with sensor 

float obtainGyroscopeValueX()
{
	int16_t gx =(readI2C(0x43)<<8);
	gx|=readI2C(0x44);
	return (float)gx / 32768;
}

float obtainGyroscopeValueY()
{
	int16_t gy =(readI2C(0x45)<<8);
	gy|=readI2C(0x46);
	return (float)gy / 32768;
}

float obtainGyroscopeValueZ()
{
	int16_t gz =(readI2C(0x47)<<8);
	gz|=readI2C(0x48);
	return (float)gz / 32768;
}

float obtainAccelerationXAxis()
{
	int16_t ax =(readI2C(0x3B)<<8);
	ax|=readI2C(0x3C);
	return (float)ax / 32768;
}

float obtainAccelerationYAxis()
{
	int16_t ay =(readI2C(0x3D)<<8);
	ay|=readI2C(0x3E);
	return (float)ay / 32768;
}

float obtainAccelerationZAxis()
{
	int16_t az =(readI2C(0x3F)<<8);
	az|=readI2C(0x40);
	return (float)az / 32768;
}

int readI2C(int address)
{
	I2C1->CR1 = 0xC;
	I2C1->OAR1 = (0x4000);
	I2C1->CCR = 0x3C;
	I2C1->TRISE = 0xD;
	I2C1->CR1 = I2C_CR1_PE;
//	for(int i=0; i<0xFFF; i++){}		
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB)) {}
	(void) I2C1->SR1;
	I2C1->DR = 0xD0;
	while (!(I2C1->SR1 & I2C_SR1_ADDR)){}
	(void) I2C1->SR1;
	(void) I2C1->SR2;
	I2C1->DR = address;		
	while (!(I2C1->SR1 & I2C_SR1_BTF)){}
		
//---------------------------------//
		
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB)){}
	(void) I2C1->SR1;
	I2C1->DR = 0xD1;
	while (!(I2C1->SR1 & I2C_SR1_ADDR)){}
	(void) I2C1->SR1;
	(void) I2C1->SR2;
	
//---------------------------------//
		
	while (!(I2C1->SR1 & I2C_SR1_RXNE)){}
	volatile int data_i2c = I2C1->DR;
	I2C1->CR1 |= I2C_CR1_ACK;
	(void) I2C1->SR1;
	(void) I2C1->SR2;
	I2C1->CR1 |= I2C_CR1_STOP;
	(void) I2C1->SR1;
	(void) I2C1->SR2;
	data_i2c = I2C1->DR;		
	I2C1->CR1 |= I2C_CR1_STOP;
	while(I2C1->SR1 & I2C_SR1_STOPF){}
	while((I2C1->SR2)&&(0x2)==0x2){}
	I2C1->CR1 |= I2C_CR1_SWRST;
//	for(int i=0; i<0xFF; i++){}
	return data_i2c;
}

void prepareI2C()
{
//---------------------------------//

		I2C1->CR1 |= I2C_CR1_PE;
		I2C1->CR1 |= I2C_CR1_START;
		while (!(I2C1->SR1 & I2C_SR1_SB)){}
		(void) I2C1->SR1;
		I2C1->DR = 0xD0;
		while (!(I2C1->SR1 & I2C_SR1_ADDR)){}
		(void) I2C1->SR1;
		(void) I2C1->SR2;
		I2C1->DR = 0x6B;
		while (!(I2C1->SR1 & I2C_SR1_BTF)){}
		I2C1->DR = 0x00;
		while (!(I2C1->SR1 & I2C_SR1_BTF)){}
		I2C1->CR1 |= I2C_CR1_STOP;

//---------------------------------//

		I2C1->CR1 |= I2C_CR1_START;
		while (!(I2C1->SR1 & I2C_SR1_SB)){}
		(void) I2C1->SR1;
		I2C1->DR = 0xD0;
		while (!(I2C1->SR1 & I2C_SR1_ADDR)){}
		(void) I2C1->SR1;
		(void) I2C1->SR2;
		I2C1->DR = 0x38;
		while (!(I2C1->SR1 & I2C_SR1_BTF)){}
		I2C1->DR = 0x01;
		while (!(I2C1->SR1 & I2C_SR1_BTF)){}
		I2C1->CR1 |= I2C_CR1_STOP;

//---------------------------------//
		
		I2C1->CR1 |= I2C_CR1_START;
		while (!(I2C1->SR1 & I2C_SR1_SB)){}
		(void) I2C1->SR1;
		I2C1->DR = 0xD0;
		while (!(I2C1->SR1 & I2C_SR1_ADDR)){}
		(void) I2C1->SR1;
		(void) I2C1->SR2;
		I2C1->DR = 0x1A;
		while (!(I2C1->SR1 & I2C_SR1_BTF)){}
		I2C1->DR = 0x04;
		while (!(I2C1->SR1 & I2C_SR1_BTF)){}
		I2C1->CR1 |= I2C_CR1_STOP;

//---------------------------------//

		I2C1->CR1 |= I2C_CR1_START;
		while (!(I2C1->SR1 & I2C_SR1_SB)){}
		(void) I2C1->SR1;
		I2C1->DR = 0xD0;
		while (!(I2C1->SR1 & I2C_SR1_ADDR)){}
		(void) I2C1->SR1;
		(void) I2C1->SR2;
		I2C1->DR = 0x19;
		while (!(I2C1->SR1 & I2C_SR1_BTF)){}
		I2C1->DR = 0x00;
		while (!(I2C1->SR1 & I2C_SR1_BTF)){}
		I2C1->CR1 |= I2C_CR1_STOP;
		for(int i=0; i<0xFF; i++){}
}

//#pragma pop

/* Конфигурация тактирования */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* Конфигурация ADC1 */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* Конфигурация ADC2 */
static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* Конфигурация DAC */
static void MX_DAC_Init(void)
{
  DAC_ChannelConfTypeDef sConfig;
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	DAC->CR|=(1|1<<2|(7<<3)|5<<16|7<<19);
}

/* Конфигурация I2C1 */
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

/* Конфигурация TIM1 */
static void MX_TIM1_Init(void)
{
	__HAL_RCC_TIM1_CLK_ENABLE();
	HAL_TIM_PWM_MspInit(&htim1);
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Prescaler = 0xF;
	htim1.Init.Period = 0x2FFF;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&htim1);
	TIM_OC_InitTypeDef pwm;
	pwm.OCMode = TIM_OCMODE_PWM2;
	pwm.Pulse = 0;
	pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
	pwm.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_1);
}

/* Конфигурация TIM2 */
static void MX_TIM2_Init(void)
{
	__HAL_RCC_TIM2_CLK_ENABLE();
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Prescaler = 250;
	htim2.Init.Period = 0xFFFF;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	NVIC_EnableIRQ(TIM2_IRQn);
}

/* Конфигурация TIM3 */
static void MX_TIM3_Init(void)
{
	__HAL_RCC_TIM3_CLK_ENABLE();
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Prescaler = 0x4FF;
	htim3.Init.Period = 0xFFFF;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim3);
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	NVIC_EnableIRQ(TIM3_IRQn);
}

/* Конфигурация пинов */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void _Error_Handler(char *file, int line)
{
  while(1)
  {
		
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
