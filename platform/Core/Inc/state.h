#ifndef STATE_H_
#define STATE_H_

#include <stdint.h>
#include <stdbool.h>

#include <stm32f1xx.h>

#include "lsm6ds3_reg.h"
#include "lis3mdl_reg.h"

#define MPU9255			0
#define LSM6DS3			0
#define LSM303C			1

// if error set value and go to end
#define PROCESS_ERROR(x) if (0 != (error = (x))) { goto end; }


//	IMU data in RSC (related system of coordinates)
typedef struct {
	float accel[3];
	float gyro[3];
	float magn[3];
} stateIMU_rsc_t;


//	orientation and position of device in ISC (inertial system of coordinates)
typedef struct {
	//	IMU data
	float accel[3];
	float magn[3];

	//	position
	float velocities[3];
	float coordinates[3];

	//	orientation
	float quaternion[4];
} stateIMU_isc_t;

//	system parameters
typedef struct {
	uint8_t MPU_state;		//	state of IMU module
	uint8_t NRF_state;		//	state of NRF24L01

	float time;				//	current time
	float magnASA[3];		//	magnitometer shift
} state_system_t;


typedef struct {
	//	zero params; this fields should be filled when device started it`s work
	double zero_pressure;
	float zero_quaternion[4];
	float zero_GPS[2];
	float gyro_staticShift[3];
	float accel_staticShift[3];
} state_zero_t;


extern lsm6ds3_ctx_t lsm6ds3_dev_ctx;
extern lis3mdl_ctx_t lsm303c_dev_ctx;

// Global structures.
extern stateIMU_rsc_t 		stateIMU_rsc;
extern stateIMU_isc_t 		stateIMU_isc;
extern state_system_t 		state_system;
extern state_zero_t			state_zero;

extern stateIMU_isc_t		stateIMU_isc_prev;
extern state_system_t		state_system_prev;


#endif /* STATE_H_ */
