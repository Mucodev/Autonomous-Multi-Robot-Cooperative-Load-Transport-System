/*
 * MotorsandEncoders.h - CORRECTED VERSION
 *
 * This version fixes the major issues:
 * 1. Uses wheel velocities for odometry instead of PID outputs
 * 2. Implements correct mecanum kinematics
 * 3. Properly separates odometry from control
 * 4. Uses consistent units throughout
 */

#ifndef INC_MOTORSANDENCODERS_H_
#define INC_MOTORSANDENCODERS_H_

#include <DefinesandVariables.h>
#include <Prototypes.h>
#include <MEMS.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim10;

// Previous counter values for delta calculation
int32_t prevFlCounter = 0;
int32_t prevFrCounter = 0;
int32_t prevRlCounter = 0;
int32_t prevRrCounter = 0;

// Timer periods (auto-reload values) for center-aligned mode
uint32_t flTimerPeriod = 0;
uint32_t frTimerPeriod = 0;
uint32_t rlTimerPeriod = 0;
uint32_t rrTimerPeriod = 0;

// Previous wheel positions for delta calculation
static float prevFlPosition = 0.0f;
static float prevFrPosition = 0.0f;
static float prevRlPosition = 0.0f;
static float prevRrPosition = 0.0f;

// Add timing variables for proper velocity calculation
static uint32_t lastUpdateTime = 0;

// Add pose PID controllers for Method 2
static WheelPIDController xPID, yPID, thetaPID;
static uint8_t useWheelPositionPID = 0; // 0 = use pose PID, 1 = use wheel PID

// Forward declarations to fix compilation order
void WheelPIDInit(WheelPIDController *pid, float kp, float ki, float kd,
		float maxOutput, float minOutput);
float WheelPIDCompute(WheelPIDController *pid, float processVariable,
		float deltaTime);

// Wrapper function for pose PID computation
float PositionPIDCompute(WheelPIDController *pid, float error,
		float deltaTime) {
	// For position PID, we directly use error instead of (setpoint - processVariable)
	// Store current error as a negative setpoint
	pid->setpoint = 0;
	return WheelPIDCompute(pid, error, deltaTime);
}

// Function to initialize pose PID controllers
void PosePIDControllersInit(float xKp, float xKi, float xKd, float yKp,
		float yKi, float yKd, float thetaKp, float thetaKi, float thetaKd) {
	// Initialize with appropriate max outputs for velocities (m/s and rad/s)
	WheelPIDInit(&xPID, xKp, xKi, xKd, 1.0f, -1.0f);      // Max 1 m/s
	WheelPIDInit(&yPID, yKp, yKi, yKd, 1.0f, -1.0f);      // Max 1 m/s
	WheelPIDInit(&thetaPID, thetaKp, thetaKi, thetaKd, 2.0f, -2.0f); // Max 2 rad/s
}

void MecanumInit(void) {
	/* Start all PWM channels */
	HAL_TIM_PWM_Start(FL_PWM_TIMER, FL_PWM_CHANNEL);
	HAL_TIM_PWM_Start(FR_PWM_TIMER, FR_PWM_CHANNEL);
	HAL_TIM_PWM_Start(RL_PWM_TIMER, RL_PWM_CHANNEL);
	HAL_TIM_PWM_Start(RR_PWM_TIMER, RR_PWM_CHANNEL);
	HAL_GPIO_WritePin(stdby1_GPIO_Port, stdby1_Pin, 1);
	HAL_GPIO_WritePin(stdby2_GPIO_Port, stdby2_Pin, 1);
	motors_check = 1;

	// Store timer periods (auto-reload values) for center-aligned mode
	flTimerPeriod = __HAL_TIM_GET_AUTORELOAD(FL_ENC_TIMER);
	frTimerPeriod = __HAL_TIM_GET_AUTORELOAD(FR_ENC_TIMER);
	rlTimerPeriod = __HAL_TIM_GET_AUTORELOAD(RL_ENC_TIMER);
	rrTimerPeriod = __HAL_TIM_GET_AUTORELOAD(RR_ENC_TIMER);

	// Initialize counters to middle value for bidirectional counting
	__HAL_TIM_SET_COUNTER(FL_ENC_TIMER, 65535 / 2);
	__HAL_TIM_SET_COUNTER(FR_ENC_TIMER, 4294967295 / 2);
	__HAL_TIM_SET_COUNTER(RL_ENC_TIMER, 4294967295 / 2);
	__HAL_TIM_SET_COUNTER(RR_ENC_TIMER, 65535 / 2);

	// Initialize previous counter values
	prevFlCounter = 0;
	prevFrCounter = 0;
	prevRlCounter = 0;
	prevRrCounter = 0;

	// Start encoder timers
	HAL_TIM_Encoder_Start(FL_ENC_TIMER, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(FR_ENC_TIMER, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(RL_ENC_TIMER, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(RR_ENC_TIMER, TIM_CHANNEL_ALL);
	encoders_check = 1;
	HAL_TIM_Base_Start_IT(&htim10);
	BSP_ACCELERO_Init();
	accelero_check = 1;
	BSP_GYRO_Init();
	gyro_check = 1;

	/* Set all motors to stop initially */
	SetMotorSpeed(0, 0); // Front Left
	SetMotorSpeed(1, 0); // Front Right
	SetMotorSpeed(2, 0); // Rear Left
	SetMotorSpeed(3, 0); // Rear Right

	// Initialize timing
	lastUpdateTime = HAL_GetTick();
}

void PreciseWheelPositionInit() {
	// Method 1: PID gains for individual wheel position control
	WheelPIDControllersInit(2.0f, 0.0f, 0.008f,  // Front Left (Kp, Ki, Kd)
			2.0f, 0.0f, 0.008f,  // Front Right
			2.0f, 0.0f, 0.008f,  // Rear Left
			2.0f, 0.0f, 0.008f   // Rear Right
			);

	// Method 2: PID gains for robot pose control (x, y, theta)
	// These values need tuning for your specific robot
	PosePIDControllersInit(1.0f, 0.05f, 0.5f,    // X position (was 0.8f for Kp)
			1.0f, 0.05f, 0.5f,    // Y position (was 0.8f for Kp)
			2.0f, 0.05f, 1.0f     // Theta (was 2.0f for Kp)
			);

	// Select control method (0 = pose PID, 1 = wheel PID)
	useWheelPositionPID = 0; // Default to pose PID
}

void WheelPIDInit(WheelPIDController *pid, float kp, float ki, float kd,
		float maxOutput, float minOutput) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->setpoint = 0.0f;
	pid->lastError = 0.0f;
	pid->integral = 0.0f;
	pid->maxOutput = maxOutput;
	pid->minOutput = minOutput;
	pid->integralMax = maxOutput * 0.5f;  // Limit integral to 50% of max output
	pid->integralMin = minOutput * 0.5f;
}

float WheelPIDCompute(WheelPIDController *pid, float processVariable,
		float deltaTime) {
	float error = pid->setpoint - processVariable;
	float p = pid->kp * error;

	pid->integral += error * deltaTime;
	pid->integral = fmaxf(pid->integralMin,
			fminf(pid->integralMax, pid->integral));
	float i = pid->ki * pid->integral;

	float derivative =
			(deltaTime > 0) ?
					pid->kd * ((error - pid->lastError) / deltaTime) : 0.0f;

	float output = p + i + derivative;
	output = fmaxf(pid->minOutput, fminf(pid->maxOutput, output));

	pid->lastError = error;
	return output;
}

void WheelPIDSetSetpoint(WheelPIDController *pid, float setpoint) {
	pid->setpoint = setpoint;
	pid->integral = 0.0f;
	pid->lastError = 0.0f;
}

uint8_t WheelPIDAtSetpoint(WheelPIDController *pid, float tolerance) {
	return fabsf(pid->setpoint - pid->lastError) <= tolerance;
}

void WheelPIDControllersInit(float flKp, float flKi, float flKd, float frKp,
		float frKi, float frKd, float rlKp, float rlKi, float rlKd, float rrKp,
		float rrKi, float rrKd) {
	WheelPIDInit(&flPID, flKp, flKi, flKd, MAX_PWM, -MAX_PWM);
	WheelPIDInit(&frPID, frKp, frKi, frKd, MAX_PWM, -MAX_PWM);
	WheelPIDInit(&rlPID, rlKp, rlKi, rlKd, MAX_PWM, -MAX_PWM);
	WheelPIDInit(&rrPID, rrKp, rrKi, rrKd, MAX_PWM, -MAX_PWM);
}

void MoveToPosition(float x, float y, float theta, float speed) {
	// Check if this is a pure rotation command
	if (fabsf(x - currentX) < 0.01f && fabsf(y - currentY) < 0.01f) {
		RotateToAngle(theta, speed);
		return;
	}

	targetX = x;
	targetY = y;
	targetTheta = theta;
	targetSpeed = speed;
	isMoving = 1;

	/* Reset encoders before moving */
	ResetEncoders();

	/* Initial position update */
	UpdatePositionSimple();
}

void RotateToAngle(float targetAngle, float speed) {
	ResetEncoders();

	targetX = currentX;
	targetY = currentY;
	targetTheta = targetAngle;
	targetSpeed = speed;
	isMoving = 1;

	float dtheta = targetTheta - currentTheta;

	// Normalize angle difference to [-π, π]
	if (isfinite(dtheta)) {
		dtheta = fmodf(dtheta, 2.0f * PI);
		if (dtheta > PI)
			dtheta -= 2.0f * PI;
		if (dtheta < -PI)
			dtheta += 2.0f * PI;
	} else {
		dtheta = 0;
	}

	float rotationSpeed = speed;
	float omega = (dtheta > 0) ? rotationSpeed : -rotationSpeed;

	SetMecanumWheelSpeeds(0.0f, 0.0f, omega);
}

uint32_t encoderCounts[4];
float encpos[4];
float prevencpos[4] = { 0 };
float encvel[4];
uint32_t prevtime = 0;
uint32_t delta_time;
int32_t signedCount[4];

void UpdateEncoderCounts(void) {
	uint32_t time = HAL_GetTick();
	uint32_t delta_tick = time - prevtime;
	if (delta_tick == 0)
		delta_tick = 1;

	encoderCounts[0] = __HAL_TIM_GET_COUNTER(&htim2);
	encoderCounts[1] = __HAL_TIM_GET_COUNTER(&htim3);
	encoderCounts[2] = __HAL_TIM_GET_COUNTER(&htim4);
	encoderCounts[3] = __HAL_TIM_GET_COUNTER(&htim5);

	signedCount[0] = (int32_t) encoderCounts[0] - 2147483648;
	signedCount[1] = (int32_t) encoderCounts[1] - 32768;
	signedCount[2] = (int32_t) encoderCounts[2] - 32768;
	signedCount[3] = (int32_t) encoderCounts[3] - 2147483648;

	int32_t frDelta = signedCount[0] - prevFrCounter;
	int32_t flDelta = signedCount[1] - prevFlCounter;
	int32_t rrDelta = signedCount[2] - prevRrCounter;
	int32_t rlDelta = signedCount[3] - prevRlCounter;

	prevFrCounter = signedCount[0];
	prevFlCounter = signedCount[1];
	prevRrCounter = signedCount[2];
	prevRlCounter = signedCount[3];

	// Update total counts // nilayların araç için
	flTotalCount += flDelta; // fldelta
	frTotalCount += frDelta; // frdelta
	rlTotalCount += rlDelta; // rldelta
	rrTotalCount += rrDelta; // rrdelta

	// Copy to global variables
	flCount = flTotalCount;
	frCount = frTotalCount;
	rlCount = rlTotalCount;
	rrCount = rrTotalCount;
}

/**
 * CORRECTED UpdatePositionSimple function
 * This version properly calculates odometry using wheel velocities
 * and applies correct mecanum kinematics
 */
void UpdatePositionSimple(void) {
//	if (!isMoving) {
//		return;
//	}

	// Get current time and calculate delta
	uint32_t currentTime = HAL_GetTick();
	float deltaTimeMs = (float) (currentTime - lastUpdateTime);
	if (deltaTimeMs < 1.0f)
		return; // Skip if time delta too small

	float deltaTimeSec = deltaTimeMs / 1000.0f;
	lastUpdateTime = currentTime;

	UpdateEncoderCounts();
//	UpdateSensorData();

	// Calculate individual wheel positions in meters
	float flPosition = (float) (flTotalCount)
			* (2 * PI * WHEEL_RADIUS)/ TICKS_PER_REV;
	float frPosition = (float) (frTotalCount)
			* (2 * PI * WHEEL_RADIUS)/ TICKS_PER_REV;
	float rlPosition = (float) (rlTotalCount)
			* (2 * PI * WHEEL_RADIUS)/ TICKS_PER_REV;
	float rrPosition = (float) (rrTotalCount)
			* (2 * PI * WHEEL_RADIUS)/ TICKS_PER_REV;

	// Calculate position deltas since last update
	float flPositionDelta = flPosition - prevFlPosition;
	float frPositionDelta = frPosition - prevFrPosition;
	float rlPositionDelta = rlPosition - prevRlPosition;
	float rrPositionDelta = rrPosition - prevRrPosition;

	// Calculate wheel velocities (m/s)
	float flVelocity = flPositionDelta / deltaTimeSec;
	float frVelocity = frPositionDelta / deltaTimeSec;
	float rlVelocity = rlPositionDelta / deltaTimeSec;
	float rrVelocity = rrPositionDelta / deltaTimeSec;

	// Update mm positions for tracking
	flPosition_mm += flPositionDelta * 1000.0f;
	frPosition_mm += frPositionDelta * 1000.0f;
	rlPosition_mm += rlPositionDelta * 1000.0f;
	rrPosition_mm += rrPositionDelta * 1000.0f;

	// Save current positions for next iteration
	prevFlPosition = flPosition;
	prevFrPosition = frPosition;
	prevRlPosition = rlPosition;
	prevRrPosition = rrPosition;

	// CORRECTED: Apply proper mecanum wheel forward kinematics using velocities
	// For standard 45-degree roller mecanum wheels:
	float L = (WHEEL_BASE + TRACK_WIDTH) / 2.0f;

	// Robot frame velocities from wheel velocities
	float vx_robot = (flVelocity + frVelocity + rlVelocity + rrVelocity) / 4.0f;
	float vy_robot = (-flVelocity + frVelocity + rlVelocity - rrVelocity)
			/ 4.0f;
	float omega_robot = (-flVelocity + frVelocity - rlVelocity + rrVelocity)
			/ (4.0f * L);

	// Transform velocities from robot frame to global frame
	float cos_theta = cosf(currentTheta);
	float sin_theta = sinf(currentTheta);

	float vx_global = vx_robot * cos_theta - vy_robot * sin_theta;
	float vy_global = vx_robot * sin_theta + vy_robot * cos_theta;

	// Integrate velocities to update position
	currentX += vx_global * deltaTimeSec; // positif olacak
	currentY += vy_global * deltaTimeSec;
	currentTheta += omega_robot * deltaTimeSec;

	// Normalize theta to [0, 2π]
	while (currentTheta >= 2.0f * PI)
		currentTheta -= 2.0f * PI;
	while (currentTheta < 0)
		currentTheta += 2.0f * PI;

	// CONTROL SECTION - PID-based position control
	// Calculate errors in global frame
	float dx = targetX - currentX;
	float dy = targetY - currentY;
	float dtheta = targetTheta - currentTheta;

	// Normalize angle difference to [-π, π]
	if (isfinite(dtheta)) {
		dtheta = fmodf(dtheta, 2.0f * PI);
		if (dtheta > PI)
			dtheta -= 2.0f * PI;
		if (dtheta < -PI)
			dtheta += 2.0f * PI;
	} else {
		dtheta = 0;
	}

	float distance = sqrtf(dx * dx + dy * dy);
	float angleError = fabsf(dtheta);

	// Check if target is reached
	if (distance < POSITION_THRESHOLD && angleError < ANGLE_THRESHOLD) {
		// Stop all motors completely
		SetMecanumWheelSpeeds(0, 0, 0);
		SetMotorSpeed(0, 0);  // Front Left
		SetMotorSpeed(1, 0);  // Front Right
		SetMotorSpeed(2, 0);  // Rear Left
		SetMotorSpeed(3, 0);  // Rear Right

		// Reset PID controllers to prevent integral windup
		flPID.integral = 0;
		frPID.integral = 0;
		rlPID.integral = 0;
		rrPID.integral = 0;
		xPID.integral = 0;
		yPID.integral = 0;
		thetaPID.integral = 0;

		isMoving = 0;
		return;
	}

	// METHOD 1: PID on wheel positions (if you want individual wheel control)
	// Calculate target wheel positions for the desired motion
	if (useWheelPositionPID) {
		// Calculate required wheel rotations to reach target
		float path_length = distance;
		float rotation_angle = dtheta;

		// Inverse kinematics for wheel position targets
		float L = (WHEEL_BASE + TRACK_WIDTH) / 2.0f;
		float wheel_circumference = 2 * PI * WHEEL_RADIUS;

		// Calculate target positions for each wheel
		float fl_target = prevFlPosition
				+ (path_length - rotation_angle * L) / wheel_circumference * 2
						* PI;
		float fr_target = prevFrPosition
				+ (path_length + rotation_angle * L) / wheel_circumference * 2
						* PI;
		float rl_target = prevRlPosition
				+ (path_length - rotation_angle * L) / wheel_circumference * 2
						* PI;
		float rr_target = prevRrPosition
				+ (path_length + rotation_angle * L) / wheel_circumference * 2
						* PI;

		// Set PID targets
		WheelPIDSetSetpoint(&flPID, fl_target);
		WheelPIDSetSetpoint(&frPID, fr_target);
		WheelPIDSetSetpoint(&rlPID, rl_target);
		WheelPIDSetSetpoint(&rrPID, rr_target);

		// Compute PID outputs
		float fl_output = WheelPIDCompute(&flPID, flPosition, deltaTimeSec);
		float fr_output = WheelPIDCompute(&frPID, frPosition, deltaTimeSec);
		float rl_output = WheelPIDCompute(&rlPID, rlPosition, deltaTimeSec);
		float rr_output = WheelPIDCompute(&rrPID, rrPosition, deltaTimeSec);

		// Set motor speeds directly from PID outputs
		SetMotorSpeed(0, (int16_t) fl_output);
		SetMotorSpeed(1, (int16_t) fr_output);
		SetMotorSpeed(2, (int16_t) rl_output);
		SetMotorSpeed(3, (int16_t) rr_output);
	}
	// METHOD 2: PID on robot pose (x, y, theta)
	else {
		// Transform errors to robot frame for better control
		float dx_robot = dx * cos_theta + dy * sin_theta;
		float dy_robot = -dx * sin_theta + dy * cos_theta;

		// Use separate PID controllers for x, y, and theta
		// You'll need to add these PID controllers to your structure
		float vx_cmd = PositionPIDCompute(&xPID, -dx_robot, deltaTimeSec);
		float vy_cmd = PositionPIDCompute(&yPID, -dy_robot, deltaTimeSec);
		float omega_cmd = PositionPIDCompute(&thetaPID, -dtheta, deltaTimeSec);

		// Limit velocities to maximum
		float max_linear_vel = targetSpeed;
		float max_angular_vel = 1.0f; // rad/s

		// Limit linear velocity
		float linear_vel = sqrtf(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
		if (linear_vel > max_linear_vel) {
			vx_cmd = vx_cmd / linear_vel * max_linear_vel;
			vy_cmd = vy_cmd / linear_vel * max_linear_vel;
		}

		// Limit angular velocity
		if (fabsf(omega_cmd) > max_angular_vel) {
			omega_cmd = (omega_cmd > 0) ? max_angular_vel : -max_angular_vel;
		}

		// Add deadband to prevent motor stall
		const float deadband = 0.05f;  // 5% of max speed
		if (fabsf(vx_cmd) < deadband)
			vx_cmd = 0;
		if (fabsf(vy_cmd) < deadband)
			vy_cmd = 0;
		if (fabsf(omega_cmd) < deadband)
			omega_cmd = 0;

		// Apply wheel speeds
		SetMecanumWheelSpeeds(vx_cmd, vy_cmd, omega_cmd);
	}
}

void SetWheelPositionTargets(float flTarget, float frTarget, float rlTarget,
		float rrTarget) {
	WheelPIDSetSetpoint(&flPID, flTarget);
	WheelPIDSetSetpoint(&frPID, frTarget);
	WheelPIDSetSetpoint(&rlPID, rlTarget);
	WheelPIDSetSetpoint(&rrPID, rrTarget);
}

void SetMecanumWheelSpeeds(float vx, float vy, float omega) {
	/* Mecanum wheel inverse kinematics */
	float L = (WHEEL_BASE + TRACK_WIDTH) / 2.0f;

	float frontLeft = vx - vy - omega * L;
	float frontRight = vx + vy + omega * L;
	float rearLeft = vx + vy - omega * L;
	float rearRight = vx - vy + omega * L;

	/* Find the maximum wheel speed */
	float maxWheelSpeed = fabsf(frontLeft);
	if (fabsf(frontRight) > maxWheelSpeed)
		maxWheelSpeed = fabsf(frontRight);
	if (fabsf(rearLeft) > maxWheelSpeed)
		maxWheelSpeed = fabsf(rearLeft);
	if (fabsf(rearRight) > maxWheelSpeed)
		maxWheelSpeed = fabsf(rearRight);

	/* Normalize wheel speeds if any exceeds the maximum allowed */
	if (maxWheelSpeed > 1.0f) {
		frontLeft /= maxWheelSpeed;
		frontRight /= maxWheelSpeed;
		rearLeft /= maxWheelSpeed;
		rearRight /= maxWheelSpeed;
	}

	/* Convert to motor speeds (scaled to MAX_PWM) */
	int16_t frontLeftPWM = (int16_t) (frontLeft * MAX_PWM);
	int16_t frontRightPWM = (int16_t) (frontRight * MAX_PWM);
	int16_t rearLeftPWM = (int16_t) (rearLeft * MAX_PWM);
	int16_t rearRightPWM = (int16_t) (rearRight * MAX_PWM);

	/* Set motor speeds */
	SetMotorSpeed(0, frontLeftPWM);  // Front Left
	SetMotorSpeed(1, frontRightPWM); // Front Right
	SetMotorSpeed(2, rearLeftPWM);   // Rear Left
	SetMotorSpeed(3, rearRightPWM);  // Rear Right
}

void SetMotorSpeed(uint8_t motor, int16_t speed) {
	GPIO_TypeDef *in1Port;
	GPIO_TypeDef *in2Port;
	uint16_t in1Pin;
	uint16_t in2Pin;
	TIM_HandleTypeDef *pwmTimer;
	uint32_t pwmChannel;

	/* Limit the speed to MAX_PWM */
	if (speed > MAX_PWM)
		speed = MAX_PWM;
	if (speed < -MAX_PWM)
		speed = -MAX_PWM;

	// Add deadband to prevent motor stall
	const int16_t MOTOR_DEADBAND = 50;  // Adjust based on your motors
	if (abs(speed) < MOTOR_DEADBAND) {
		speed = 0;
	}

	uint16_t pwmValue = (uint16_t) abs(speed);

	/* Determine pins and timer based on motor number */
	switch (motor) {
	case 0: // Front Left
		in1Port = FL_IN1_GPIO_Port;
		in1Pin = FL_IN1_Pin;
		in2Port = FL_IN2_GPIO_Port;
		in2Pin = FL_IN2_Pin;
		pwmTimer = FL_PWM_TIMER;
		pwmChannel = FL_PWM_CHANNEL;
		break;
	case 1: // Front Right
		in1Port = FR_IN2_GPIO_Port; // in2
		in1Pin = FR_IN2_Pin; //in2
		in2Port = FR_IN1_GPIO_Port; //in1
		in2Pin = FR_IN1_Pin; // in1
		pwmTimer = FR_PWM_TIMER;
		pwmChannel = FR_PWM_CHANNEL;
		break;
	case 2: // Rear Left
		in1Port = RL_IN1_GPIO_Port;
		in1Pin = RL_IN1_Pin;
		in2Port = RL_IN2_GPIO_Port;
		in2Pin = RL_IN2_Pin;
		pwmTimer = RL_PWM_TIMER;
		pwmChannel = RL_PWM_CHANNEL;
		break;
	case 3: // Rear Right
		in1Port = RR_IN1_GPIO_Port;
		in1Pin = RR_IN1_Pin;
		in2Port = RR_IN2_GPIO_Port;
		in2Pin = RR_IN2_Pin;
		pwmTimer = RR_PWM_TIMER;
		pwmChannel = RR_PWM_CHANNEL;
		break;
	default:
		return; // Invalid motor number
	}

	/* Set direction pins based on speed direction */
	if (speed > 0) {
		// Forward direction
		HAL_GPIO_WritePin(in1Port, in1Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(in2Port, in2Pin, GPIO_PIN_RESET);
	} else if (speed < 0) {
		// Backward direction
		HAL_GPIO_WritePin(in1Port, in1Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(in2Port, in2Pin, GPIO_PIN_SET);
	} else {
		// Stop - set both pins low for brake mode
		HAL_GPIO_WritePin(in1Port, in1Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(in2Port, in2Pin, GPIO_PIN_RESET);
	}

	/* Set PWM value */
	__HAL_TIM_SET_COMPARE(pwmTimer, pwmChannel, pwmValue);
}

void ResetEncoders(void) {
	// Set all encoder hardware counters to midpoint
	__HAL_TIM_SET_COUNTER(FL_ENC_TIMER, 32768);
	__HAL_TIM_SET_COUNTER(FR_ENC_TIMER, 2147483648);
	__HAL_TIM_SET_COUNTER(RL_ENC_TIMER, 2147483648);
	__HAL_TIM_SET_COUNTER(RR_ENC_TIMER, 32768);

	// Reset all software counters
	flTotalCount = 0;
	frTotalCount = 0;
	rlTotalCount = 0;
	rrTotalCount = 0;

	flCount = 0;
	frCount = 0;
	rlCount = 0;
	rrCount = 0;

	prevFLCount = 0;
	prevFRCount = 0;
	prevRLCount = 0;
	prevRRCount = 0;

	// Reset positions
	flPosition_mm = 0.0f;
	frPosition_mm = 0.0f;
	rlPosition_mm = 0.0f;
	rrPosition_mm = 0.0f;

	// Reset previous positions for velocity calculation
	prevFlPosition = 0.0f;
	prevFrPosition = 0.0f;
	prevRlPosition = 0.0f;
	prevRrPosition = 0.0f;

	// Reset timing
	lastUpdateTime = HAL_GetTick();
}

void SetGlobalPosition(float x, float y, float theta) {
	currentX = x;
	currentY = y;
	currentTheta = theta;
}

int16_t GetEncoderDelta(TIM_HandleTypeDef *htim, uint16_t *lastCount) {
	uint16_t currentCount = (uint16_t) __HAL_TIM_GET_COUNTER(htim);
	int16_t delta;

	if (currentCount >= *lastCount) {
		if (currentCount - *lastCount > 32768) {
			delta = -(int16_t) ((65536 - currentCount) + *lastCount);
		} else {
			delta = (int16_t) (currentCount - *lastCount);
		}
	} else {
		if (*lastCount - currentCount > 32768) {
			delta = (int16_t) ((65536 - *lastCount) + currentCount);
		} else {
			delta = -(int16_t) (*lastCount - currentCount);
		}
	}

	*lastCount = currentCount;
	return delta;
}

void TestMotorDirections() {
	ResetEncoders();

	// Drive all motors forward at the same fixed speed
	SetMotorSpeed(0, 500);  // Front right
	HAL_Delay(1000);
	SetMotorSpeed(0, 0);  // Front right
	HAL_Delay(1000);
	SetMotorSpeed(1, 500);  // Front Left
	HAL_Delay(1000);
	SetMotorSpeed(1, 0);  // Front Left
	HAL_Delay(1000);
	SetMotorSpeed(2, 500);  // Front Left
	HAL_Delay(1000);
	SetMotorSpeed(2, 0);  // Front Left
	HAL_Delay(1000);
	SetMotorSpeed(3, 500);  // Front Left
	HAL_Delay(1000);
	SetMotorSpeed(3, 0);  // Front Left
	HAL_Delay(1000);

	HAL_Delay(1000);  // Run for 1 second

	// Stop all motors
	SetMotorSpeed(0, 0);
	SetMotorSpeed(1, 0);
	SetMotorSpeed(2, 0);
	SetMotorSpeed(3, 0);

	UpdateEncoderCounts();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM10) {
		UpdatePositionSimple();
		wheelMoved = 0;
	}
}

#endif /* INC_MOTORSANDENCODERS_H_ */
