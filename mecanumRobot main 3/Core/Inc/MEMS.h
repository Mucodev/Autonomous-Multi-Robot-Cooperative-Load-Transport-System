/*
 * MEMS.h
 *
 *  Created on: Apr 26, 2025
 *      Author: talha
 */

#ifndef INC_MEMS_H_
#define INC_MEMS_H_
#include <DefinesandVariables.h>

// Function to find the median value in an array
int16_t findMedian(int16_t arr[], int size) {
	// Make a copy of the array for sorting
	int16_t temp[size];
	for (int i = 0; i < size; i++) {
		temp[i] = arr[i];
	}

	// Insertion sort (faster than bubble sort for small arrays)
	for (int i = 1; i < size; i++) {
		int16_t key = temp[i];
		int j = i - 1;

		while (j >= 0 && temp[j] > key) {
			temp[j + 1] = temp[j];
			j--;
		}
		temp[j + 1] = key;
	}

	// Return the middle element (median)
	return temp[size / 2];
}

// Filter function that handles both X and Y accelerometer data
void filterAccelerometer(int16_t rawData[3], int16_t filteredData[3]) {
	// Add raw values to circular buffers
	accelBufferX[bufferIndex] = rawData[0];
	accelBufferY[bufferIndex] = rawData[1];

	// Update buffer index
	bufferIndex = (bufferIndex + 1) % MEDIAN_SIZE;
	if (bufferIndex == 0) {
		bufferFilled = true;
	}

	// Apply median filtering if buffer is filled
	if (bufferFilled) {
		filteredData[0] = findMedian(accelBufferX, MEDIAN_SIZE);
		filteredData[1] = findMedian(accelBufferY, MEDIAN_SIZE);
	} else {
		// Not enough samples yet, use raw values
		filteredData[0] = rawData[0];
		filteredData[1] = rawData[1];
	}

	// Copy the Z-axis value directly (or filter it too if needed)
	filteredData[2] = rawData[2];

	// Additional spike rejection for extreme values (like ±3264)
	for (int i = 0; i < 3; i++) {
		if (abs(filteredData[i]) > 2000) {
			// Replace extreme values with 0 or the previous value
			filteredData[i] = 0;
		}
	}
}

/* Get calibrated accelerometer data -----------------------------------------*/
void calibrateSensors(void) {
	/* Calibrate accelerometer */
	int16_t accelData[3];
	int32_t sumX = 0, sumY = 0, sumZ = 0;
	const int accelSamples = 1000;
	uint8_t usbBuffer[64];

	// Collect accelerometer samples
	for (int i = 0; i < accelSamples; i++) {
		BSP_ACCELERO_GetXYZ(accelData);
		sumX += accelData[0];
		sumY += accelData[1];
		sumZ += accelData[2];
		uint8_t len = sprintf((char*) usbBuffer, "ACC %d\r\n", i);
		CDC_Transmit_FS(usbBuffer, len);
		HAL_Delay(10);
	}

	// Calculate average offsets
	accelOffsetX = sumX / accelSamples;
	accelOffsetY = sumY / accelSamples;
	// For Z, subtract expected 1g (but we'll adapt based on the value range)
	accelOffsetZ = sumZ / accelSamples;
	if (abs(accelOffsetZ) > 1000) {
		// If value is large, we're probably not in 16g mode, assume 2g mode (~1000 counts per g)
		accelOffsetZ -= 1000; // Subtract expected 1g in Z for 2g mode
	} else {
		// We're in 16g mode
		accelOffsetZ -= 83; // Subtract expected 1g in Z for 16g mode
	}

	/* Calibrate gyroscope */
	float gyroData[3];
	float gyrSumX = 0, gyrSumY = 0, gyrSumZ = 0;
	const int gyroSamples = 1000;

	// Collect gyroscope samples
	for (int i = 0; i < gyroSamples; i++) {
		BSP_GYRO_GetXYZ(gyroData);
		gyrSumX += gyroData[0];
		gyrSumY += gyroData[1];
		gyrSumZ += gyroData[2];
		uint8_t len = sprintf((char*) usbBuffer, "GYRO %d\r\n", i);
		CDC_Transmit_FS(usbBuffer, len);
		HAL_Delay(5);
	}

	// Calculate average bias
	gyroBiasX = gyrSumX / gyroSamples;
	gyroBiasY = gyrSumY / gyroSamples;
	gyroBiasZ = gyrSumZ / gyroSamples;

}

void UpdateSensorData(void) {
    // Read sensor data
    BSP_ACCELERO_GetXYZ(accelData);
    BSP_GYRO_GetXYZ(gyroData);

    // Time management - calculate delta time
    start_time = HAL_GetTick();
    uint32_t delta = (start_time - end_time);
    if (delta > 200 || end_time == 0)
        delta = 20; // Cap delta and handle first loop
    float deltaSeconds = delta / 1000.0f;

    // Process gyro data - correct bias and scale
    gyroData[0] -= gyroBiasX;
    gyroData[1] -= gyroBiasY;
    gyroData[2] -= gyroBiasZ;

    gyroData[0] /= 700;
    gyroData[1] /= 700;
    gyroData[2] /= 700 * (90.0f / 129.0f * 2.0f);

    // Update orientation from gyro
    orientationZ += gyroData[2] * deltaSeconds;

    // Normalize orientation to [0, 2π]
    while (orientationZ >= 2.0f * PI)
        orientationZ -= 2.0f * PI;
    while (orientationZ < 0)
        orientationZ += 2.0f * PI;

    // Process accelerometer data with direct scaling for ±16g range
    // Sensitivity factor: 0.11772 m/s²/LSB
    const float ACC_SCALE_FACTOR = 0.11772f;

    // Apply scaling and offset correction
    if (fabs(accelData[0]) <= 15500) {
        accelGX = (accelData[0] - accelOffsetX) * ACC_SCALE_FACTOR;
        tempAccelX = accelGX;
    } else {
        accelGX = tempAccelX;
    }

    if (fabs(accelData[1]) <= 15500) {
        accelGY = (accelData[1] - accelOffsetY) * ACC_SCALE_FACTOR;
        tempAccelY = accelGY;
    } else {
        accelGY = tempAccelY;
    }

    if (fabs(accelData[2]) <= 15500) {
        accelGZ = (accelData[2] - accelOffsetZ) * ACC_SCALE_FACTOR;
        tempAccelZ = accelGZ;
    } else {
        accelGZ = tempAccelZ;
    }

    // Apply deadband filter to remove noise when acceleration is very small
    const float ACCEL_DEADBAND = 0.3f; // m/s² - adjust based on your noise level
    if (fabsf(accelGX) < ACCEL_DEADBAND) accelGX = 0.0f;
    if (fabsf(accelGY) < ACCEL_DEADBAND) accelGY = 0.0f;

    // Apply high-pass filter to acceleration to remove bias
    static float filtered_accel_x = 0.0f;
    static float filtered_accel_y = 0.0f;
    static float prev_accel_x = 0.0f;
    static float prev_accel_y = 0.0f;

    float alpha = 0.9f; // Increased filtering (was 0.8f)
    filtered_accel_x = alpha * (filtered_accel_x + accelGX - prev_accel_x);
    filtered_accel_y = alpha * (filtered_accel_y + accelGY - prev_accel_y);

    prev_accel_x = accelGX;
    prev_accel_y = accelGY;

    // Update with filtered values
    accelGX = filtered_accel_x;
    accelGY = filtered_accel_y;

    // Detect if we're likely stationary (very low acceleration)
    const float STATIONARY_THRESHOLD = 0.2f; // m/s² - increased from 0.1f
    bool is_stationary = (fabsf(accelGX) < STATIONARY_THRESHOLD &&
                          fabsf(accelGY) < STATIONARY_THRESHOLD);

    // Integration with stronger drift correction
    // Update velocity with filtered acceleration
    velocityX += accelGX * deltaSeconds;
    velocityY += accelGY * deltaSeconds;

    // Apply stronger velocity decay/damping to prevent drift
    float velocity_decay = 0.94f; // Increased decay (was 0.98f)
    velocityX *= velocity_decay;
    velocityY *= velocity_decay;

    // Reset velocity when likely stationary to prevent drift
    if (is_stationary) {
        // More aggressive reset when stationary
        velocityX *= 0.6f; // Was 0.8f
        velocityY *= 0.6f; // Was 0.8f

        // If velocity is very small when stationary, zero it out completely
        if (fabsf(velocityX) < 0.05f) velocityX = 0.0f;
        if (fabsf(velocityY) < 0.05f) velocityY = 0.0f;
    }

    // Add a scaling factor to better match real-world distances
    // This will need calibration based on your specific setup
    const float DISTANCE_SCALE = 0.2f; // Adjust to match your 0.5m real movement

    // Update position with scaled velocity
    positionX += velocityX * deltaSeconds * DISTANCE_SCALE;
    positionY += velocityY * deltaSeconds * DISTANCE_SCALE;

    // Report data
    uint8_t usbBuffer[128];
    uint8_t len = sprintf((char*)usbBuffer,
            "Accel[%.2f,%.2f] Vel[%.2f,%.2f] Pos[%.2f,%.2f] Orient: %.2f %s\r\n",
            accelGX, accelGY,
            velocityX, velocityY,
            positionX, positionY,
            orientationZ * 180.0f / PI,  // Convert to degrees for display
            is_stationary ? "[STATIONARY]" : "");
    CDC_Transmit_FS(usbBuffer, len);

    end_time = start_time;
}

#endif /* INC_MEMS_H_ */
