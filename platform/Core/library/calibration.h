#ifndef LIBRARY_CALIBRATION_H_
#define LIBRARY_CALIBRATION_H_

typedef struct
{
	uint8_t descr1;
	uint8_t descr2;
	float time;

	uint8_t state;

	float accel[3];
	float gyro[3];
	float magn[3];

}__attribute__((packed, aligned(1))) state_msg_t;

void IMU_Init(I2C_HandleTypeDef *hi2c);
uint8_t CalibrationData_Send(UART_HandleTypeDef *huart);
int CalibrationData_ObtainPoint(void);
void Calibration_PerformCycle(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);

#endif /* LIBRARY_CALIBRATION_H_ */
