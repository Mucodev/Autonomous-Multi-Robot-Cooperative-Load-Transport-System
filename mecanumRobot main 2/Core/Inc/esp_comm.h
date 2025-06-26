/* esp_comm.h
 * STM32 ESP8266 Communication Library Header
 * Integrated with Mecanum Robot Control System
 */

#ifndef ESP_COMM_H
#define ESP_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
// Don't include DefinesandVariables.h here to avoid multiple definitions
// #include "DefinesandVariables.h"
// #include "Prototypes.h"

/* Forward declarations of external variables from your robot system ---------*/
extern float currentX, currentY, currentTheta;
extern float targetX, targetY, targetTheta;
extern uint8_t isMoving;
extern int motors_check;  // Changed to int to match DefinesandVariables.h

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint8_t isRunning;
    uint8_t isESPControlled;    // Flag to indicate ESP control mode
    int16_t speed_vx;           // Linear velocity X (mm/s)
    int16_t speed_vy;           // Linear velocity Y (mm/s)
    int16_t speed_omega;        // Angular velocity (mrad/s)
    float target_x;             // Target position X (m)
    float target_y;             // Target position Y (m)
    float target_theta;         // Target angle (rad)
    uint32_t lastCommandTime;
    uint8_t movementMode;       // 0=velocity, 1=position
} ESPRobotState_t;

typedef enum {
    ESP_CMD_UNKNOWN = 0,
    ESP_CMD_START,
    ESP_CMD_STOP,
    ESP_CMD_MOVE_VELOCITY,      // Move with velocity
    ESP_CMD_MOVE_POSITION,      // Move to position
    ESP_CMD_EMERGENCY_STOP,
    ESP_CMD_GET_STATUS,
    ESP_CMD_RESET_POSITION,     // Reset odometry
    ESP_CMD_SET_POSITION        // Set current position
} ESPCommand_t;

/* Exported constants --------------------------------------------------------*/
#define ESP_COMM_BUFFER_SIZE    150
#define ESP_COMM_TIMEOUT_MS     1000

/* Robot ID Configuration - CHANGE THIS FOR EACH ROBOT */
#ifndef ROBOT_ID
#define ROBOT_ID 1  // 1=Master, 2=Slave Robot 2, 3=Slave Robot 3
#endif

/* UART Configuration */
#ifndef ESP_UART_HANDLE
#define ESP_UART_HANDLE huart2  // UART2 for ESP8266 communication at 115200
#endif

/* Remove debug UART - only using UART2 for ESP communication */
/* Exported macro ------------------------------------------------------------*/
#define ESP_COMM_DEBUG_ENABLE   0  // Disabled - no separate debug UART

/* Exported variables --------------------------------------------------------*/
extern UART_HandleTypeDef ESP_UART_HANDLE;
/* Velocity scaling factors */
#define VEL_SCALE_FACTOR 1000.0f    // Convert m/s to mm/s
#define OMEGA_SCALE_FACTOR 1000.0f  // Convert rad/s to mrad/s

extern ESPRobotState_t esp_robot_state;

/* Exported function prototypes ----------------------------------------------*/

/* Initialization Functions */
void ESPComm_Init(void);
void ESPComm_SetRobotID(uint8_t id);

/* Main Loop Function */
void ESPComm_Process(void);

/* Command Processing Functions */
void ESPComm_ProcessCommand(char* command);
ESPCommand_t ESPComm_ParseCommand(char* command, uint8_t* robotId, float* params);

/* Communication Functions */
void ESPComm_SendResponse(char* response);
void ESPComm_SendStatus(void);
void ESPComm_SendReady(void);
void ESPComm_SendPosition(void);

/* Robot State Functions */
uint8_t ESPComm_IsRobotRunning(void);
void ESPComm_SetRobotRunning(uint8_t running);
ESPRobotState_t* ESPComm_GetRobotState(void);
void ESPComm_UpdateMovement(int16_t vx, int16_t vy, int16_t omega);
void ESPComm_SetMovementMode(uint8_t mode);  // 0=velocity, 1=position

/* Robot Control Integration Functions */
void ESPComm_StartRobot(void);
void ESPComm_StopRobot(void);
void ESPComm_MoveVelocity(float vx, float vy, float omega);
void ESPComm_MoveToPosition(float x, float y, float theta, float speed);
void ESPComm_EmergencyStop(void);
void ESPComm_ResetPosition(void);
void ESPComm_SetCurrentPosition(float x, float y, float theta);

/* Forward declarations of robot control functions */
void MecanumInit(void);
void SetMecanumWheelSpeeds(float vx, float vy, float omega);
void SetMotorSpeed(uint8_t motor, int16_t speed);
void MoveToPosition(float x, float y, float theta, float speed);
void ResetEncoders(void);
void SetGlobalPosition(float x, float y, float theta);

/* Callback Functions - Implement these in your main code if needed */
void ESPComm_OnRobotStart(void);
void ESPComm_OnRobotStop(void);
void ESPComm_OnRobotMove(int16_t vx, int16_t vy, int16_t omega);
void ESPComm_OnEmergencyStop(void);
void ESPComm_OnPositionReset(void);

/* Utility Functions */
void ESPComm_Debug(char* message);
uint32_t ESPComm_GetTick(void);

/* UART Callback - Call this from HAL_UART_RxCpltCallback */
void ESPComm_UART_RxCallback(UART_HandleTypeDef *huart);

/* Conversion utilities */
float ESPComm_ConvertVelocity(int16_t vel_scaled);   // Convert scaled velocity to m/s
int16_t ESPComm_ScaleVelocity(float vel_ms);        // Convert m/s to scaled velocity
float ESPComm_ConvertOmega(int16_t omega_scaled);    // Convert scaled omega to rad/s
int16_t ESPComm_ScaleOmega(float omega_rad);        // Convert rad/s to scaled omega

#ifdef __cplusplus
}
#endif

#endif /* ESP_COMM_H */
