/*
 * DefinesandVariables.h
 *
 *  Created on: Apr 26, 2025
 *      Author: talha
 */

#ifndef INC_DEFINESANDVARIABLES_H_
#define INC_DEFINESANDVARIABLES_H_
#include <stdbool.h>
#include <VL53L0X.h>
/* Robot ID - set unique for each robot (0-3) */
#define ROBOT_ID 2 // nilayların araç için 2  // CHANGE THIS FOR EACH ROBOT!

// I2C addresses
#define SLAVE_1_ADDR 0x10  // Robot 1
#define SLAVE_2_ADDR 0x20  // Robot 2
#define SLAVE_3_ADDR 0x30  // Robot 3

// Simple movement command
typedef struct {
	float x;
	float y;
	float theta;
	float speed;
	uint8_t command;    // 1=move, 0=stop
} movement_command_t;

// Global variables
movement_command_t received_move;
uint8_t new_movement = 0;
uint32_t last_move_time = 0;

// MEMS
int16_t accelOffsetX = -300, accelOffsetY = -199.53, accelOffsetZ = 14740.77;
float norm;
//int16_t accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroBiasX = -162.63, gyroBiasY = -1085.16, gyroBiasZ = 330.0;
float orientationZ = 0;
float velocityX = 0.0f, velocityY = 0.0f;
float positionX = 0, positionY = 0;
uint8_t calibrationDone = 0;
int16_t accelData[3];
float gyroData[3];
int16_t tempAccelX = 0, tempAccelY = 0, tempAccelZ = 0;
float accelGX = 0, accelGY = 0, accelGZ = 0;
uint32_t start_time = 0, end_time = 0;

// Median Filter
#define MEDIAN_SIZE 15
// Buffers for X and Y axes
int16_t accelBufferX[MEDIAN_SIZE];
int16_t accelBufferY[MEDIAN_SIZE];
uint8_t bufferIndex = 0;
bool bufferFilled = false;

typedef struct {
	float kp;          // Proportional gain
	float ki;          // Integral gain
	float kd;          // Derivative gain
	float setpoint;    // Target position
	float lastError;   // Previous error for derivative calculation
	float integral;    // Accumulated error for integral calculation
	float maxOutput;   // Maximum output limit
	float minOutput;   // Minimum output limit
	float integralMax; // Maximum integral windup limit
	float integralMin; // Minimum integral windup limit
} WheelPIDController;

/* PID Controllers for each wheel */
WheelPIDController flPID = { 0 }; // Front Left Wheel
WheelPIDController frPID = { 0 }; // Front Right Wheel
WheelPIDController rlPID = { 0 }; // Rear Left Wheel
WheelPIDController rrPID = { 0 }; // Rear Right Wheel

/* Speed control variables */
float targetSpeed;          // Target linear movement speed in m/s
float currentSpeed;         // Current actual speed in m/s
float lastMovementTime;     // Time of last movement update

/* USER CODE END PTD */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI 3.14159265358979323846

// Front Left Motor
#define WHEEL_RADIUS     0.04f  // meters
#define WHEEL_BASE       0.222f   // meters (distance between left and right wheels)
#define TRACK_WIDTH      0.224f   // meters (distance between front and back wheels)
#define MAX_PWM          1000   // Maximum PWM value
#define ANGLE_THRESHOLD 0.09f
#define POSITION_THRESHOLD 0.09f // Position accuracy threshold in meters

#define ENCODER_PPR       12    // Pulses per revolution (before gearing)
#define GEAR_RATIO        155    // Motor gear ratio (84:1) -> 110rpm / Motor Gear ratio (150:1) -> 60rpm
#define TICKS_PER_REV     (ENCODER_PPR * 4 * GEAR_RATIO) // Total ticks per wheel revolution (4x for quadrature)

#define RPM_UPDATE_PERIOD 100   // RPM update interval in milliseconds
#define SECONDS_PER_MINUTE 60.0f

/* Timer period for position update (in milliseconds) */
#define UPDATE_PERIOD    2

// Front Left Motor
#define FR_IN1_GPIO_Port motor1_in1_GPIO_Port
#define FR_IN1_Pin motor1_in1_Pin
#define FR_IN2_GPIO_Port motor1_in2_GPIO_Port
#define FR_IN2_Pin motor1_in2_Pin

// Front Right Motor
#define FL_IN1_GPIO_Port motor2_in1_GPIO_Port
#define FL_IN1_Pin motor2_in2_Pin
#define FL_IN2_GPIO_Port motor2_in2_GPIO_Port
#define FL_IN2_Pin motor2_in1_Pin

// Rear Left Motor
#define RR_IN1_GPIO_Port motor3_in1_GPIO_Port
#define RR_IN1_Pin motor3_in1_Pin
#define RR_IN2_GPIO_Port motor3_in2_GPIO_Port
#define RR_IN2_Pin motor3_in2_Pin

// Rear Right Motor
#define RL_IN1_GPIO_Port motor4_in1_GPIO_Port
#define RL_IN1_Pin motor4_in2_Pin
#define RL_IN2_GPIO_Port motor4_in2_GPIO_Port
#define RL_IN2_Pin motor4_in1_Pin

/* PWM Timer Channels for Motors */
#define FR_PWM_TIMER &htim1
#define FR_PWM_CHANNEL TIM_CHANNEL_1
#define FL_PWM_TIMER &htim1
#define FL_PWM_CHANNEL TIM_CHANNEL_2
#define RR_PWM_TIMER &htim1
#define RR_PWM_CHANNEL TIM_CHANNEL_3
#define RL_PWM_TIMER &htim1
#define RL_PWM_CHANNEL TIM_CHANNEL_4

/* Encoder Timer Configurations */
#define FR_ENC_TIMER &htim2
#define FL_ENC_TIMER &htim3
#define RR_ENC_TIMER &htim4
#define RL_ENC_TIMER &htim5

float currentX = 0.0f;     // Current X position in meters
float currentY = 0.0f;     // Current Y position in meters
float currentTheta = 0.0f; // Current orientation in radians

/* Target position and speed */
float targetX = 0.0f;
float targetY = 0.0f;
float targetTheta = 0.0f;
float targetSpeed = 0.0f;
uint8_t isMoving = 0;
int32_t flCount = 0, frCount = 0, rlCount = 0, rrCount = 0; // Current encoder counts
int32_t prevFLCount = 0, prevFRCount = 0, prevRLCount = 0, prevRRCount = 0; // Previous encoder counts
volatile int32_t flTotalCount = 0, frTotalCount = 0, rlTotalCount = 0,
		rrTotalCount = 0;

float flPosition_mm = 0.0f;  // Front Left wheel position in mm
float frPosition_mm = 0.0f;  // Front Right wheel position in mm
float rlPosition_mm = 0.0f;  // Rear Left wheel position in mm
float rrPosition_mm = 0.0f;  // Rear Right wheel position in mm

volatile uint32_t encoderPosition[4] = { 0 }; // Stores current encoder positions for each wheel
volatile uint8_t wheelMoved = 0;
int16_t currentPWM;
// New variables for RPM calculations
float flRPM = 0.0f;  // Front Left wheel RPM
float frRPM = 0.0f;  // Front Right wheel RPM
float rlRPM = 0.0f;  // Rear Left wheel RPM
float rrRPM = 0.0f;  // Rear Right wheel RPM

int32_t prevFLCountRPM = 0; // Previous encoder counts for RPM calculation
int32_t prevFRCountRPM = 0;
int32_t prevRLCountRPM = 0;
int32_t prevRRCountRPM = 0;

uint32_t lastRpmUpdateTime = 0; // Time of last RPM update
float rpmValue;

// Servo and Vl53l0x

#define SERVO_MIN_COMPARE 35  // 1 ms pulse for 0 degrees
#define SERVO_MAX_COMPARE 130  // 2 ms pulse for 180 degrees
#define SERVO_SWEEP_DELAY 10  // Delay between servo steps (ms)

uint16_t distance;
uint8_t rx_buffer[4];
int32_t lednumber;
int32_t servoAngle = 0;  // Current servo angle
uint8_t servoDirection = 1;  // 1 = increasing angle, 0 = decreasing angle
int32_t pulse_width;

statInfo_t_VL53L0X distanceStr;
int16_t offsetValue = 0;
uint16_t crossTalkValue = 0;

int motors_check = 0;
int encoders_check = 0;
int accelero_check = 0;
int gyro_check = 0;

#endif /* INC_DEFINESANDVARIABLES_H_ */
