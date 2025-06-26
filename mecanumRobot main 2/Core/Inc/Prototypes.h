/*
 * Prototypes.h
 *
 *  Created on: Apr 27, 2025
 *      Author: talha
 */

#ifndef INC_PROTOTYPES_H_
#define INC_PROTOTYPES_H_
//MEMS
int16_t findMedian(int16_t arr[], int size);
void filterAccelerometer(int16_t rawData[3], int16_t filteredData[3]);
void calibrateSensors(void);

//Commands
void ProcessCommand(uint8_t command, uint8_t targetRobot);
void SendResponse(uint8_t command, const char *data);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

//MotorsandEncoders
void MecanumInit(void);
void PreciseWheelPositionInit();
void MoveToPosition(float x, float y, float theta, float speed);
void SetMecanumWheelSpeeds(float vx, float vy, float omega);
void SetMotorSpeed(uint8_t motor, int16_t speed);
void WheelPIDControllersInit(float flKp, float flKi, float flKd, float frKp,
		float frKi, float frKd, float rlKp, float rlKi, float rlKd, float rrKp,
		float rrKi, float rrKd);
void RotateToAngle(float targetAngle, float speed);
void UpdatePositionSimple(void);
void UpdateEncoderCounts();
void ResetEncoders(void);
int16_t GetEncoderDelta(TIM_HandleTypeDef *htim, uint16_t *lastCount);
void UpdateEncoderCounts();
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

//ServoandVL53l0x
int16_t calibrateOffset(void);
uint16_t calibrateCrossTalk(void);
void setCrossTalkCompensation(uint16_t crosstalkValue);
void UpdateSensorData(void);
// Matrix operation function prototypes

#endif /* INC_PROTOTYPES_H_ */
