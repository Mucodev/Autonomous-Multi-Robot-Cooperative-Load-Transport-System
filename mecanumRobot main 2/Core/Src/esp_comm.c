/* esp_comm.c
 * STM32 ESP8266 Communication Library Implementation
 * Integrated with Mecanum Robot Control System
 */

#include "esp_comm.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t uart_rx_buffer[ESP_COMM_BUFFER_SIZE];
static uint8_t uart_rx_index = 0;
static uint8_t command_ready = 0;
static uint8_t current_robot_id = ROBOT_ID;

/* Public variables ----------------------------------------------------------*/
ESPRobotState_t esp_robot_state = {0};

/* Private function prototypes -----------------------------------------------*/
static void ESP_ResetBuffer(void);
static void ESP_ParseMovementParams(char* command, float* params);
static void ESP_ParsePositionParams(char* command, float* params);

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize ESP8266 communication
 */
void ESPComm_Init(void)
{
    // Initialize ESP robot state
    esp_robot_state.isRunning = 0;
    esp_robot_state.isESPControlled = 0;
    esp_robot_state.speed_vx = 0;
    esp_robot_state.speed_vy = 0;
    esp_robot_state.speed_omega = 0;
    esp_robot_state.target_x = 0.0f;
    esp_robot_state.target_y = 0.0f;
    esp_robot_state.target_theta = 0.0f;
    esp_robot_state.lastCommandTime = HAL_GetTick();
    esp_robot_state.movementMode = 0; // Default to velocity mode

    // Reset UART buffer
    ESP_ResetBuffer();

    // Start UART reception
    HAL_UART_Receive_IT(&ESP_UART_HANDLE, &uart_rx_buffer[0], 1);

    // Send ready message
    ESPComm_SendReady();

    ESPComm_Debug("ESP Communication Initialized for Mecanum Robot\n");
}

/**
 * @brief Set robot ID
 * @param id Robot ID (1-3)
 */
void ESPComm_SetRobotID(uint8_t id)
{
    if (id >= 1 && id <= 3) {
        current_robot_id = id;
    }
}

/**
 * @brief Main processing function - call this in main loop
 */
void ESPComm_Process(void)
{
    if (command_ready) {
        ESPComm_ProcessCommand((char*)uart_rx_buffer);
        ESP_ResetBuffer();
        command_ready = 0;
    }

    // Update last command time
    esp_robot_state.lastCommandTime = HAL_GetTick();

    // Monitor robot state from main control system
    esp_robot_state.isRunning = isMoving;
}

/**
 * @brief Process received command
 * @param command Command string
 */
void ESPComm_ProcessCommand(char* command)
{
    char response[100];
    uint8_t robotId;
    float params[6] = {0}; // vx, vy, omega, x, y, theta

    ESPCommand_t cmd = ESPComm_ParseCommand(command, &robotId, params);

    // Only process commands for this robot
    if (robotId != current_robot_id && robotId != 0) {
        return; // Not for this robot
    }

    switch (cmd) {
        case ESP_CMD_START:
            ESPComm_StartRobot();
            sprintf(response, "ROBOT_%d_STARTED\n", current_robot_id);
            ESPComm_SendResponse(response);
            break;

        case ESP_CMD_STOP:
            ESPComm_StopRobot();
            sprintf(response, "ROBOT_%d_STOPPED\n", current_robot_id);
            ESPComm_SendResponse(response);
            break;

        case ESP_CMD_MOVE_VELOCITY:
            ESPComm_MoveVelocity(params[0], params[1], params[2]);
            sprintf(response, "ROBOT_%d_MOVING_VEL\n", current_robot_id);
            ESPComm_SendResponse(response);
            break;

        case ESP_CMD_MOVE_POSITION:
            ESPComm_MoveToPosition(params[0], params[1], params[2], params[3]);
            sprintf(response, "ROBOT_%d_MOVING_POS\n", current_robot_id);
            ESPComm_SendResponse(response);
            break;

        case ESP_CMD_EMERGENCY_STOP:
            ESPComm_EmergencyStop();
            sprintf(response, "ROBOT_%d_EMERGENCY_STOPPED\n", current_robot_id);
            ESPComm_SendResponse(response);
            break;

        case ESP_CMD_GET_STATUS:
            ESPComm_SendStatus();
            break;

        case ESP_CMD_RESET_POSITION:
            ESPComm_ResetPosition();
            sprintf(response, "ROBOT_%d_POSITION_RESET\n", current_robot_id);
            ESPComm_SendResponse(response);
            break;

        case ESP_CMD_SET_POSITION:
            ESPComm_SetCurrentPosition(params[0], params[1], params[2]);
            sprintf(response, "ROBOT_%d_POSITION_SET\n", current_robot_id);
            ESPComm_SendResponse(response);
            break;

        default:
            ESPComm_SendResponse("ERROR_UNKNOWN_COMMAND\n");
            break;
    }
}

/**
 * @brief Parse command string
 * @param command Command string
 * @param robotId Pointer to store robot ID
 * @param params Pointer to store parameters [vx, vy, omega, x, y, theta]
 * @return Command type
 */
ESPCommand_t ESPComm_ParseCommand(char* command, uint8_t* robotId, float* params)
{
    *robotId = 0;

    if (strncmp(command, "START_ROBOT_", 12) == 0) {
        *robotId = command[12] - '0';
        return ESP_CMD_START;
    }
    else if (strncmp(command, "STOP_ROBOT_", 11) == 0) {
        *robotId = command[11] - '0';
        return ESP_CMD_STOP;
    }
    else if (strncmp(command, "MOVE_VEL_", 9) == 0) {
        *robotId = command[9] - '0';
        ESP_ParseMovementParams(command, params);
        return ESP_CMD_MOVE_VELOCITY;
    }
    else if (strncmp(command, "MOVE_POS_", 9) == 0) {
        *robotId = command[9] - '0';
        ESP_ParsePositionParams(command, params);
        return ESP_CMD_MOVE_POSITION;
    }
    else if (strncmp(command, "EMERGENCY_STOP_", 15) == 0) {
        *robotId = command[15] - '0';
        return ESP_CMD_EMERGENCY_STOP;
    }
    else if (strncmp(command, "GET_STATUS", 10) == 0) {
        *robotId = current_robot_id;
        return ESP_CMD_GET_STATUS;
    }
    else if (strncmp(command, "RESET_POS_", 10) == 0) {
        *robotId = command[10] - '0';
        return ESP_CMD_RESET_POSITION;
    }
    else if (strncmp(command, "SET_POS_", 8) == 0) {
        *robotId = command[8] - '0';
        ESP_ParsePositionParams(command, params);
        return ESP_CMD_SET_POSITION;
    }

    return ESP_CMD_UNKNOWN;
}

/**
 * @brief Send response to ESP8266
 * @param response Response string
 */
void ESPComm_SendResponse(char* response)
{
    HAL_UART_Transmit(&ESP_UART_HANDLE, (uint8_t*)response, strlen(response), ESP_COMM_TIMEOUT_MS);

    // No separate debug UART - only ESP communication via UART2
}

/**
 * @brief Send robot status
 */
void ESPComm_SendStatus(void)
{
    char status[150];
    sprintf(status, "ROBOT_%d_STATUS:%d_%d_%d_%d_%.3f_%.3f_%.3f\n",
            current_robot_id,
            esp_robot_state.isRunning,
            esp_robot_state.speed_vx,
            esp_robot_state.speed_vy,
            esp_robot_state.speed_omega,
            currentX,
            currentY,
            currentTheta);
    ESPComm_SendResponse(status);
}

/**
 * @brief Send position data
 */
void ESPComm_SendPosition(void)
{
    char pos[100];
    sprintf(pos, "ROBOT_%d_POS:%.3f_%.3f_%.3f\n",
            current_robot_id,
            currentX,
            currentY,
            currentTheta);
    ESPComm_SendResponse(pos);
}

/**
 * @brief Send ready message
 */
void ESPComm_SendReady(void)
{
    char ready[50];
    sprintf(ready, "STM32_%d_MECANUM_READY\n", current_robot_id);
    ESPComm_SendResponse(ready);
}

/**
 * @brief Start robot using existing robot control system
 */
void ESPComm_StartRobot(void)
{
    esp_robot_state.isRunning = 1;
    esp_robot_state.isESPControlled = 1;

    // Enable robot systems if not already enabled
    if (!motors_check) {
        MecanumInit();  // Initialize if not already done
    }

    // Call callback
    ESPComm_OnRobotStart();
}

/**
 * @brief Stop robot using existing robot control system
 */
void ESPComm_StopRobot(void)
{
    esp_robot_state.isRunning = 0;
    esp_robot_state.isESPControlled = 0;

    // Stop all motors using existing system
    SetMecanumWheelSpeeds(0.0f, 0.0f, 0.0f);

    // Force stop flag
    isMoving = 0;

    // Call callback
    ESPComm_OnRobotStop();
}

/**
 * @brief Move robot with velocity commands
 * @param vx Linear velocity X (m/s)
 * @param vy Linear velocity Y (m/s)
 * @param omega Angular velocity (rad/s)
 */
void ESPComm_MoveVelocity(float vx, float vy, float omega)
{
    if (!esp_robot_state.isRunning) {
        return;
    }

    // Update ESP state
    esp_robot_state.speed_vx = ESPComm_ScaleVelocity(vx);
    esp_robot_state.speed_vy = ESPComm_ScaleVelocity(vy);
    esp_robot_state.speed_omega = ESPComm_ScaleOmega(omega);
    esp_robot_state.movementMode = 0; // Velocity mode

    // Use existing mecanum control system
    SetMecanumWheelSpeeds(vx, vy, omega);

    // Override movement flag to allow manual velocity control
    isMoving = 1;

    // Call callback
    ESPComm_OnRobotMove(esp_robot_state.speed_vx, esp_robot_state.speed_vy, esp_robot_state.speed_omega);
}

/**
 * @brief Move robot to position
 * @param x Target X position (m)
 * @param y Target Y position (m)
 * @param theta Target angle (rad)
 * @param speed Maximum speed (m/s)
 */
void ESPComm_MoveToPosition(float x, float y, float theta, float speed)
{
    if (!esp_robot_state.isRunning) {
        return;
    }

    // Update ESP state
    esp_robot_state.target_x = x;
    esp_robot_state.target_y = y;
    esp_robot_state.target_theta = theta;
    esp_robot_state.movementMode = 1; // Position mode

    // Use existing position control system
    MoveToPosition(x, y, theta, speed);

    char debug_msg[100];
    sprintf(debug_msg, "Moving to: %.3f, %.3f, %.3f at %.3f m/s\n", x, y, theta, speed);
    ESPComm_Debug(debug_msg);
}

/**
 * @brief Emergency stop
 */
void ESPComm_EmergencyStop(void)
{
    esp_robot_state.isRunning = 0;
    esp_robot_state.isESPControlled = 0;
    esp_robot_state.speed_vx = 0;
    esp_robot_state.speed_vy = 0;
    esp_robot_state.speed_omega = 0;

    // Emergency stop all motors
    SetMotorSpeed(0, 0);  // Front Left
    SetMotorSpeed(1, 0);  // Front Right
    SetMotorSpeed(2, 0);  // Rear Left
    SetMotorSpeed(3, 0);  // Rear Right

    // Force stop all movement
    isMoving = 0;

    // Call callback
    ESPComm_OnEmergencyStop();
}

/**
 * @brief Reset robot position/odometry
 */
void ESPComm_ResetPosition(void)
{
    ResetEncoders();
    SetGlobalPosition(0.0f, 0.0f, 0.0f);

    esp_robot_state.target_x = 0.0f;
    esp_robot_state.target_y = 0.0f;
    esp_robot_state.target_theta = 0.0f;

    ESPComm_OnPositionReset();
}

/**
 * @brief Set current position
 * @param x Current X position (m)
 * @param y Current Y position (m)
 * @param theta Current angle (rad)
 */
void ESPComm_SetCurrentPosition(float x, float y, float theta)
{
    SetGlobalPosition(x, y, theta);

    char debug_msg[100];
    sprintf(debug_msg, "Position set to: %.3f, %.3f, %.3f\n", x, y, theta);
    ESPComm_Debug(debug_msg);
}

/**
 * @brief Check if robot is running
 * @return 1 if running, 0 if stopped
 */
uint8_t ESPComm_IsRobotRunning(void)
{
    return esp_robot_state.isRunning;
}

/**
 * @brief Set robot running state
 * @param running 1 to start, 0 to stop
 */
void ESPComm_SetRobotRunning(uint8_t running)
{
    esp_robot_state.isRunning = running;
}

/**
 * @brief Get robot state pointer
 * @return Pointer to robot state
 */
ESPRobotState_t* ESPComm_GetRobotState(void)
{
    return &esp_robot_state;
}

/**
 * @brief Send debug message (disabled - no separate debug UART)
 * @param message Debug message
 */
void ESPComm_Debug(char* message)
{
    // Debug disabled - only using UART2 for ESP communication
    // If you need debug, send via ESP UART or use SWV/ITM
}

/**
 * @brief Get system tick
 * @return Current tick count
 */
uint32_t ESPComm_GetTick(void)
{
    return HAL_GetTick();
}

/**
 * @brief UART receive callback - call from HAL_UART_RxCpltCallback
 * @param huart UART handle
 */
void ESPComm_UART_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == ESP_UART_HANDLE.Instance) {
        if (uart_rx_buffer[uart_rx_index] == '\n' || uart_rx_buffer[uart_rx_index] == '\r') {
            uart_rx_buffer[uart_rx_index] = '\0'; // Null terminate
            command_ready = 1;
        } else {
            uart_rx_index++;
            if (uart_rx_index >= ESP_COMM_BUFFER_SIZE - 1) {
                ESP_ResetBuffer(); // Reset if buffer overflow
            }
        }

        // Continue receiving
        HAL_UART_Receive_IT(&ESP_UART_HANDLE, &uart_rx_buffer[uart_rx_index], 1);
    }
}

/**
 * @brief Convert scaled velocity to m/s
 * @param vel_scaled Scaled velocity
 * @return Velocity in m/s
 */
float ESPComm_ConvertVelocity(int16_t vel_scaled)
{
    return (float)vel_scaled / VEL_SCALE_FACTOR;
}

/**
 * @brief Convert m/s to scaled velocity
 * @param vel_ms Velocity in m/s
 * @return Scaled velocity
 */
int16_t ESPComm_ScaleVelocity(float vel_ms)
{
    return (int16_t)(vel_ms * VEL_SCALE_FACTOR);
}

/**
 * @brief Convert scaled omega to rad/s
 * @param omega_scaled Scaled angular velocity
 * @return Angular velocity in rad/s
 */
float ESPComm_ConvertOmega(int16_t omega_scaled)
{
    return (float)omega_scaled / OMEGA_SCALE_FACTOR;
}

/**
 * @brief Convert rad/s to scaled omega
 * @param omega_rad Angular velocity in rad/s
 * @return Scaled angular velocity
 */
int16_t ESPComm_ScaleOmega(float omega_rad)
{
    return (int16_t)(omega_rad * OMEGA_SCALE_FACTOR);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Reset UART buffer
 */
static void ESP_ResetBuffer(void)
{
    uart_rx_index = 0;
    memset(uart_rx_buffer, 0, ESP_COMM_BUFFER_SIZE);
}

/**
 * @brief Parse movement parameters from velocity command
 * @param command Command string
 * @param params Array to store [vx, vy, omega]
 */
static void ESP_ParseMovementParams(char* command, float* params)
{
    // Parse MOVE_VEL_X_vx_vy_omega format
    char* token = strtok(command, "_");
    token = strtok(NULL, "_"); // Skip "MOVE"
    token = strtok(NULL, "_"); // Skip "VEL"
    token = strtok(NULL, "_"); // Skip robot ID

    if (token != NULL) params[0] = atof(token); // vx
    token = strtok(NULL, "_");
    if (token != NULL) params[1] = atof(token); // vy
    token = strtok(NULL, "_");
    if (token != NULL) params[2] = atof(token); // omega
}

/**
 * @brief Parse position parameters from position command
 * @param command Command string
 * @param params Array to store [x, y, theta, speed] or [x, y, theta]
 */
static void ESP_ParsePositionParams(char* command, float* params)
{
    // Parse MOVE_POS_X_x_y_theta_speed or SET_POS_X_x_y_theta format
    char* token = strtok(command, "_");
    token = strtok(NULL, "_"); // Skip first part
    token = strtok(NULL, "_"); // Skip second part
    token = strtok(NULL, "_"); // Skip robot ID

    if (token != NULL) params[0] = atof(token); // x
    token = strtok(NULL, "_");
    if (token != NULL) params[1] = atof(token); // y
    token = strtok(NULL, "_");
    if (token != NULL) params[2] = atof(token); // theta
    token = strtok(NULL, "_");
    if (token != NULL) params[3] = atof(token); // speed (for MOVE_POS)
}

/* Weak callback functions - Override these in your main code ---------------*/

/**
 * @brief Robot start callback - Override this function
 */
__weak void ESPComm_OnRobotStart(void)
{
    /* USER CODE: Add your robot start code here */
}

/**
 * @brief Robot stop callback - Override this function
 */
__weak void ESPComm_OnRobotStop(void)
{
    /* USER CODE: Add your robot stop code here */
}

/**
 * @brief Robot move callback - Override this function
 * @param vx Forward/backward velocity (scaled)
 * @param vy Left/right velocity (scaled)
 * @param omega Rotational velocity (scaled)
 */
__weak void ESPComm_OnRobotMove(int16_t vx, int16_t vy, int16_t omega)
{
    /* USER CODE: Add your robot movement code here */
}

/**
 * @brief Emergency stop callback - Override this function
 */
__weak void ESPComm_OnEmergencyStop(void)
{
    /* USER CODE: Add your emergency stop code here */
}

/**
 * @brief Position reset callback - Override this function
 */
__weak void ESPComm_OnPositionReset(void)
{
    /* USER CODE: Add your position reset code here */
}
