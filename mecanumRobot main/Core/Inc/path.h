/*
 * Sine-based S-Path Implementation using UpdatePositionSimple
 * This works with your existing position control system
 */

#include "MotorsandEncoders.h"
#include <math.h>

// Path state for continuous following
typedef struct {
    float pathLength;
    float pathAmplitude;
    float pathSpeed;
    float currentPathPosition;  // Current position along the path (0 to pathLength)
    uint8_t pathActive;
} SinePathState;

static SinePathState sPath = {0};

// Generate target point on sine path
void GetSinePathTarget(float pathPosition, float length, float amplitude,
                      float *targetX, float *targetY, float *targetTheta) {
    // Ensure we're within path bounds
    float t = pathPosition / length;
    t = fmaxf(0.0f, fminf(1.0f, t));

    // Calculate position on sine curve
    *targetX = pathPosition;
    *targetY = amplitude * sinf(2.0f * PI * t);

    // Calculate heading (tangent to curve)
    float dy_dx = amplitude * (2.0f * PI / length) * cosf(2.0f * PI * t);
    *targetTheta = atan2f(dy_dx, 1.0f);
}

// Main sine path following function
void FollowSinePath(float length, float amplitude, float speed) {
    // Initialize path parameters
    sPath.pathLength = length;
    sPath.pathAmplitude = amplitude;
    sPath.pathSpeed = speed;
    sPath.currentPathPosition = 0.0f;
    sPath.pathActive = 1;

    // Reset position to start
    SetGlobalPosition(0, 0, 0);

    // Follow the path
    uint32_t lastUpdateTime = HAL_GetTick();
    float lookAheadDistance = 0.3f;  // Look 30cm ahead

    while (sPath.pathActive && sPath.currentPathPosition < length) {
        uint32_t currentTime = HAL_GetTick();
        float dt = (currentTime - lastUpdateTime) / 1000.0f;

        if (dt < 0.02f) {  // Run at 50Hz
            HAL_Delay(5);
            continue;
        }
        lastUpdateTime = currentTime;

        // Update current position along path based on robot's actual X position
        // This keeps the path synchronized with actual robot position
        sPath.currentPathPosition = currentX;

        // Calculate lookahead position
        float lookAheadPosition = sPath.currentPathPosition + lookAheadDistance;
        lookAheadPosition = fminf(lookAheadPosition, length);

        // Get target point ahead on the path
        float pathTargetX, pathTargetY, pathTargetTheta;
        GetSinePathTarget(lookAheadPosition, length, amplitude,
                         &pathTargetX, &pathTargetY, &pathTargetTheta);

        // Set targets for UpdatePositionSimple
        targetX = pathTargetX;
        targetY = pathTargetY;
        targetTheta = pathTargetTheta;
        targetSpeed = speed;
        isMoving = 1;

        // UpdatePositionSimple will be called by timer interrupt automatically
        // and will handle the PID control to reach the target

        // Check if we're near the end
        float distanceToEnd = length - currentX;
        if (distanceToEnd < 0.05f) {  // Within 5cm of end
            // Set final target
            GetSinePathTarget(length, length, amplitude,
                             &targetX, &targetY, &targetTheta);

            // Wait for final position
            while (isMoving) {
                HAL_Delay(10);
            }

            sPath.pathActive = 0;
            break;
        }

        // Optional: Add speed adjustment based on curvature
        float curvature = fabsf(amplitude * (2.0f * PI / length) *
                               (2.0f * PI / length) * sinf(2.0f * PI * lookAheadPosition / length));
        float speedFactor = 1.0f / (1.0f + 2.0f * curvature);  // Slow down on tight curves
        targetSpeed = speed;
    }

    // Ensure we stop at the end
    SetMecanumWheelSpeeds(0, 0, 0);
    isMoving = 0;
}

// Alternative implementation with more control points
void FollowSinePathSmooth(float length, float amplitude, float speed) {
    const float lookAhead = 0.2f + 0.3f * speed;  // Dynamic lookahead
    const float updateRate = 0.02f;  // 50Hz update

    // Start at beginning
    SetGlobalPosition(0, 0, 0);
    uint32_t lastTime = HAL_GetTick();

    // Continuous following
    float pathProgress = 0.0f;

    while (pathProgress < 1.0f) {
        // Timing
        uint32_t currentTime = HAL_GetTick();
        float dt = (currentTime - lastTime) / 1000.0f;
        if (dt < updateRate) {
            HAL_Delay(1);
            continue;
        }
        lastTime = currentTime;

        // Update path progress based on current X position
        pathProgress = currentX / length;

        // Look ahead on the path
        float lookAheadProgress = pathProgress + (lookAhead / length);
        lookAheadProgress = fminf(lookAheadProgress, 1.0f);

        // Calculate target position
        float lookX = length * lookAheadProgress;
        float lookY = amplitude * sinf(2.0f * PI * lookAheadProgress);

        // Calculate target heading (tangent at lookahead point)
        float dy_dx = amplitude * (2.0f * PI / length) *
                     cosf(2.0f * PI * lookAheadProgress);
        float lookTheta = atan2f(dy_dx, 1.0f);

        // Set targets
        targetX = lookX;
        targetY = lookY;
        targetTheta = lookTheta;
        targetSpeed = speed;
        isMoving = 1;

        // Check if near end
        if (currentX > length - 0.1f) {
            // Final approach
            targetX = length;
            targetY = 0.0f;  // Sine returns to 0 at t=1
            targetTheta = 0.0f;

            // Wait for completion
            while (isMoving && (fabsf(targetX - currentX) > 0.02f)) {
                HAL_Delay(10);
            }
            break;
        }
    }

    // Stop
    SetMecanumWheelSpeeds(0, 0, 0);
    isMoving = 0;
}

// Test different S-path configurations
void TestSinePaths(void) {
    printf("Testing sine-based S-paths\n");

    // Test 1: Standard S-path
    printf("Test 1: Standard S-path (2m, 0.3m amplitude)\n");
    FollowSinePath(2.0f, 0.3f, 0.7f);
    HAL_Delay(2000);

    // Test 2: Tight S-path
    printf("Test 2: Tight S-path (1.5m, 0.5m amplitude)\n");
    SetGlobalPosition(0, 0, 0);
    FollowSinePath(1.5f, 0.5f, 0.7f);
    HAL_Delay(2000);

    // Test 3: Long gentle S-path
    printf("Test 3: Long gentle S-path (3m, 0.2m amplitude)\n");
    SetGlobalPosition(0, 0, 0);
    FollowSinePathSmooth(3.0f, 0.2f, 0.7f);
    HAL_Delay(2000);

    // Test 4: Fast S-path
    printf("Test 4: Fast S-path (2m, 0.3m amplitude, high speed)\n");
    SetGlobalPosition(0, 0, 0);
    FollowSinePathSmooth(2.0f, 0.3f, 0.7f);
}

// Simple usage in main
void RunSPath(void) {
    // Basic S-path
    FollowSinePath(2.0f, 0.4f, 0.3f);  // 2m long, 0.4m amplitude, 0.3 m/s

    // Or smooth version with dynamic lookahead
    // FollowSinePathSmooth(2.0f, 0.4f, 0.4f);
}
