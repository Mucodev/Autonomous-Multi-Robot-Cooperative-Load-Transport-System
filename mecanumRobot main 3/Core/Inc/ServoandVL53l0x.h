/*
 * ServoandVL53l0x.h
 *
 *  Created on: Apr 27, 2025
 *      Author: talha
 */

#ifndef INC_SERVOANDVL53L0X_H_
#define INC_SERVOANDVL53L0X_H_
#include <DefinesandVariables.h>
int16_t calibrateOffset(void) {
    // Place the sensor at a known distance from a white target (e.g., 100mm)
    // The distance should be between 50-300mm for best results
    const uint16_t knownDistance = 100; // mm
    const uint8_t numMeasurements = 50; // More measurements for better accuracy

    uint32_t sumMeasurements = 0;
    statInfo_t_VL53L0X stats;

    // Take multiple measurements and average them
    for (uint8_t i = 0; i < numMeasurements; i++) {
        uint16_t measurement = readRangeSingleMillimeters(&stats);

        // Skip invalid measurements
        if (measurement != 8190 && stats.rangeStatus == 0) {
            sumMeasurements += measurement;
        } else {
            i--; // Retry the measurement
            HAL_Delay(10);
            continue;
        }

        HAL_Delay(30); // Short delay between measurements
    }

    uint16_t avgMeasurement = sumMeasurements / numMeasurements;
    int16_t offset = knownDistance - avgMeasurement;

    return offset;
}

uint16_t calibrateCrossTalk(void) {
    // Place the sensor at a known distance from a white target
    const uint16_t calibrationDistance = 400; // mm
    const uint8_t numMeasurements = 20;

    uint32_t sumSignalRates = 0;
    uint32_t sumRanges = 0;
    uint8_t validMeasurements = 0;

    for (uint8_t i = 0; i < numMeasurements; i++) {
        statInfo_t_VL53L0X stats;
        uint16_t range = readRangeSingleMillimeters(&stats);

        if (stats.rangeStatus == 0 && range != 8190) {
            sumSignalRates += stats.signalCnt;
            sumRanges += range;
            validMeasurements++;
        }

        HAL_Delay(30);
    }

    if (validMeasurements == 0) {
        return 0; // Error case
    }

    uint16_t avgSignalRate = sumSignalRates / validMeasurements;
    uint16_t avgRange = sumRanges / validMeasurements;

    // Calculate crosstalk based on the difference between measured and actual distance
    float rangeDelta = (float)(calibrationDistance - avgRange) / calibrationDistance;

    // Scale the signal rate by the range error factor
    // This is a more sophisticated approach that accounts for the actual measured distance
    uint16_t crossTalkValue = (uint16_t)(avgSignalRate * rangeDelta * 0.85); // 85% scaling factor

    return crossTalkValue;
}

void setCrossTalkCompensation(uint16_t crosstalkValue) {
    // Convert from fixed point 9.7 format (as used in the library) to 7.9 format
    uint16_t xTalkCompMegaCps = crosstalkValue << 2;

    // Write to the device register
    writeReg16Bit(CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, xTalkCompMegaCps);
}

/* Example Usage
#include "main.h"
#include "VL53L0X.h"
#include "stdio.h"
#include "string.h"

// Function declarations
int16_t calibrateOffset(void);
uint16_t calibrateCrossTalk(void);
void setCrossTalkCompensation(uint16_t crosstalkValue);

// Initialize buffer
char msgBuffer[100];
statInfo_t_VL53L0X distanceStr;
int16_t offsetValue = 0;
uint16_t crossTalkValue = 0;
bool performCalibration = true;

// Initialize sensor
if(!initVL53L0X(1, &hi2c1)) {
    sprintf(msgBuffer, "Sensor init failed!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), 100);
    Error_Handler();
}

// Configure the sensor for high accuracy
setSignalRateLimit(200);
setVcselPulsePeriod(VcselPeriodPreRange, 10);
setVcselPulsePeriod(VcselPeriodFinalRange, 14);
setMeasurementTimingBudget(300 * 1000UL);

// Perform calibration if needed
if(performCalibration) {
    // Offset calibration
    sprintf(msgBuffer, "Place sensor at 100mm from white target\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), 100);
    HAL_Delay(5000);

    offsetValue = calibrateOffset();
    sprintf(msgBuffer, "Offset calibration done: %d mm\r\n", offsetValue);
    HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), 100);

    // Crosstalk calibration
    sprintf(msgBuffer, "Place sensor at 400mm from white target\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), 100);
    HAL_Delay(5000);

    crossTalkValue = calibrateCrossTalk();
    sprintf(msgBuffer, "Crosstalk calibration done: %u\r\n", crossTalkValue);
    HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), 100);

    // Apply crosstalk calibration
    setCrossTalkCompensation(crossTalkValue);
}
while (1) {
    // Read distance
    uint16_t rawDistance = readRangeSingleMillimeters(&distanceStr);

    // Apply offset calibration
    uint16_t calibratedDistance = rawDistance;
    if(rawDistance != 8190) { // Not a timeout
        calibratedDistance += offsetValue;
    }

    // Display results
    sprintf(msgBuffer, "Distance: %u mm (raw: %u mm)\r\n",
            calibratedDistance, rawDistance);
    HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), 100);

    HAL_Delay(100);

}

// Implementation of calibration functions

int16_t calibrateOffset(void) {
    const uint16_t knownDistance = 100; // mm
    const uint8_t numMeasurements = 20;

    uint32_t sumMeasurements = 0;
    uint8_t validMeasurements = 0;

    for (uint8_t i = 0; i < numMeasurements; i++) {
        statInfo_t_VL53L0X stats;
        uint16_t measurement = readRangeSingleMillimeters(&stats);

        if (measurement != 8190 && stats.rangeStatus == 0) {
            sumMeasurements += measurement;
            validMeasurements++;
        }

        HAL_Delay(30);
    }

    if (validMeasurements == 0) {
        return 0; // Error case
    }

    uint16_t avgMeasurement = sumMeasurements / validMeasurements;
    return knownDistance - avgMeasurement;
}

uint16_t calibrateCrossTalk(void) {
    const uint8_t numMeasurements = 20;

    uint32_t sumSignalRates = 0;
    uint8_t validMeasurements = 0;

    for (uint8_t i = 0; i < numMeasurements; i++) {
        statInfo_t_VL53L0X stats;
        readRangeSingleMillimeters(&stats);

        if (stats.rangeStatus == 0) {
            sumSignalRates += stats.signalCnt;
            validMeasurements++;
        }

        HAL_Delay(30);
    }

    if (validMeasurements == 0) {
        return 0; // Error case
    }

    uint16_t avgSignalRate = sumSignalRates / validMeasurements;

    // The signal rate is in fixed point 9.7 format
    // We need to calculate a percentage of this for crosstalk
    // Typically between 5-15% depending on cover glass
    return (avgSignalRate * 10) / 100; // 10% of the signal rate
}

void setCrossTalkCompensation(uint16_t crosstalkValue) {
    // The register expects value in 7.9 format, while our calculation is in 9.7
    uint16_t xTalkCompensation = crosstalkValue << 2;
    writeReg16Bit(CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, xTalkCompensation);
} */
#endif /* INC_SERVOANDVL53L0X_H_ */
