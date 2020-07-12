#include <stdint.h>
#include <stdio.h>

#include "state.h"
#include "calibration.h"
#include "lsm6ds3_tools.h"
#include "lsm303c_tools.h"
#include "MPU9255.h"

#define MOTOR_PORT			GPIOA
#define MOTOR_FORWARD_PIN	GPIO_PIN_3
#define MOTOR_REVERSE_PIN	GPIO_PIN_2

int MOTOR_360 = 4000;

#if (LSM6DS3)
int MOTOR_STEPS = 60;
#elif (LSM303C)
int MOTOR_STEPS = 100;
#else
int STOP_TIME = 60;
#endif

int PWM_MAX = 270;
int PWM_MIN = 30;
int PWM_STEPS = 12;

#if (LSM6DS3)
int STOP_TIME = 400;
#elif (LSM303C)
int STOP_TIME = 0;
#else
int STOP_TIME = 400;
#endif

#if (LSM6DS3)
int MEAS_TIME = 20;
#elif (LSM303C)
int MEAS_TIME = 5;
#else
int MEAS_TIME = 20;
#endif

#if (LSM6DS3)
int INIT_TIME = 1000;
#elif (LSM303C)
int INIT_TIME = 150;
#else
int INIT_TIME = 1000;
#endif

void SetPWMValue(TIM_HandleTypeDef *htim, uint16_t value)
{
//    TIM_OC_InitTypeDef sConfigOC;
//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = value;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);

	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, value);
}

void RotateMotor(int forward, int time)
{
	if (forward == 1)
	{
		HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_REVERSE_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_FORWARD_PIN, GPIO_PIN_SET);
		HAL_Delay(time);
		HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_FORWARD_PIN, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_FORWARD_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_REVERSE_PIN, GPIO_PIN_SET);
		HAL_Delay(time);
		HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_REVERSE_PIN, GPIO_PIN_RESET);
	}
}

void IMU_Init(I2C_HandleTypeDef *hi2c)
{
	HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_FORWARD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_REVERSE_PIN, GPIO_PIN_RESET);

	if(MPU9255)
	{
		//	MPU9255 init
		uint8_t mpu9255_initError = 0;
		mpu9255_initError = mpu9255_init(hi2c);
		if (mpu9255_initError != HAL_OK)
		{
			HAL_Delay(100);
			mpu9255_initError = mpu9255_init(hi2c);
		}

		printf("mpu: %d\n", mpu9255_initError);
		state_system.MPU_state = mpu9255_initError;
	}

	else
	{
		if (LSM6DS3)
		{
			//	LSM6DS3 init
			int error = lsm6ds3_platform_init();
			printf("lsm6ds3: %d\n", error);
			state_system.MPU_state = error;
		}
		if (LSM303C)
		{
			//	LSM303C init
			int error = lsm303c_platform_init();
			printf("lsm303c: %d\n", error);
			state_system.NRF_state = error;
		}
	}

}

uint8_t CalibrationData_Send(UART_HandleTypeDef *huart)
{
	uint8_t error = 0;

	state_msg_t msg_state;
	msg_state.descr1 = 0x0A;
	msg_state.descr2 = 0x0A;

	msg_state.time = (float)HAL_GetTick() / 1000;

	msg_state.state = state_system.MPU_state;

	for (int i = 0; i < 3; i++)
	{
		msg_state.accel[i] = stateIMU_rsc.accel[i];
		msg_state.gyro[i] = stateIMU_rsc.gyro[i];
		msg_state.magn[i] = stateIMU_rsc.magn[i];
	}

	error |= HAL_UART_Transmit(huart, (uint8_t*)(&msg_state), sizeof(msg_state), 10);

	return error;
}

int CalibrationData_ObtainPoint(void)
{
	int error = 0;

	//	Arrays
	float accel[3] = {0, 0, 0};
	float gyro[3] = {0, 0, 0};
	float magn[3] = {0, 0, 0};

	////////////////////////////////////////////////////
	/////////////////	GET IMU DATA  //////////////////
	////////////////////////////////////////////////////

#if (MPU9255)
	int16_t accelData[3] = {0, 0, 0};
	int16_t gyroData[3] = {0, 0, 0};
	int16_t magnData[3] = {0, 0, 0};

	PROCESS_ERROR(mpu9255_readIMU(accelData, gyroData));
	PROCESS_ERROR(mpu9255_readCompass(magnData));

	//	Recalc data to floats
	mpu9255_recalcAccel(accelData, accel);
	mpu9255_recalcGyro(gyroData, gyro);
	mpu9255_recalcMagn(magnData, magn);

#elif (LSM6DS3)
	error = lsm6ds3_get_xl_data_g(accel);
	error |= lsm6ds3_get_g_data_rps(gyro);
	if (error)
	{
		state_system.MPU_state = error;
		goto end;
	}

#elif (LSM303C)
	error = lsm303c_get_m_data_mG(magn);
	if (error)
	{
		state_system.NRF_state = error;
		goto end;
	}
#endif

	float _time = (float)HAL_GetTick() / 1000;
	state_system.time = _time;
	//	пересчитываем их и записываем в структуры
	for (int k = 0; k < 3; k++)
	{
		stateIMU_rsc.accel[k] = accel[k];
		gyro[k] -= state_zero.gyro_staticShift[k];
		stateIMU_rsc.gyro[k] = gyro[k];
		stateIMU_rsc.magn[k] = magn[k];
	}
end:
	return error;
}

void Calibration_PerformCycle(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim)
{
	int value = PWM_MIN;
	int PWM_STEP = (PWM_MAX - PWM_MIN) / PWM_STEPS;
	int MOTOR_TIME = MOTOR_360 / MOTOR_STEPS;

	SetPWMValue(htim, value);

	HAL_Delay(INIT_TIME);

	CalibrationData_ObtainPoint();
	CalibrationData_Send(huart);

	for (int i = 1; i < PWM_STEPS + 1; i++)
	{
		for (int k = value; k < value + PWM_STEP + 1; k++)
		{
			SetPWMValue(htim, k);
			HAL_Delay(MEAS_TIME);
		}

		value += PWM_STEP;
		if (value > PWM_MAX) value = PWM_MIN;

		HAL_Delay(STOP_TIME);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		CalibrationData_ObtainPoint();
		CalibrationData_Send(huart);
	}

	RotateMotor(1, MOTOR_TIME);
}

void Calibration(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim)
{
	for (int i = 0; i < MOTOR_STEPS; i++)
		Calibration_PerformCycle(huart, htim);

	RotateMotor(-1, MOTOR_360);
}

