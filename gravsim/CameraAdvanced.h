#include <raylib.h>
#include <raymath.h>
#include <algorithm>
#include "raymathDouble.h"
#pragma once
#define CAMERA_MOVE_SPEED                           5.4f       // Units per second
#define CAMERA_ROTATION_SPEED                       0.03f
#define CAMERA_PAN_SPEED                            2.0f

// Camera mouse movement sensitivity
#define CAMERA_MOUSE_MOVE_SENSITIVITY               0.003f

// Camera orbital speed in CAMERA_ORBITAL mode
#define CAMERA_ORBITAL_SPEED                        0.5f       // Radians per second

//----------------------------------------------------------------------------------
// Types and Structures Definition
//----------------------------------------------------------------------------------
//...

//----------------------------------------------------------------------------------
// Global Variables Definition
//----------------------------------------------------------------------------------
//...

//----------------------------------------------------------------------------------
// Module Internal Functions Declaration
//----------------------------------------------------------------------------------
//...

//----------------------------------------------------------------------------------
// Module Functions Definition
//----------------------------------------------------------------------------------
// Returns the cameras forward vector (normalized)
static Vector3 GetCameraForward(Camera* camera)
{
    return Vector3Normalize(Vector3Subtract(camera->target, camera->position));
}

// Returns the cameras up vector (normalized)
// Note: The up vector might not be perpendicular to the forward vector
static Vector3 GetCameraUp(Camera* camera)
{
    return Vector3Normalize(camera->up);
}

// Returns the cameras right vector (normalized)
static Vector3 GetCameraRight(Camera* camera)
{
    Vector3 forward = GetCameraForward(camera);
    Vector3 up = GetCameraUp(camera);

    return Vector3Normalize(Vector3CrossProduct(forward, up));
}

// Moves the camera in its forward direction
static void CameraMoveForward(Camera* camera, double distance, bool moveInWorldPlane, Vector3d& worldCamPos)
{
    Vector3 forward = GetCameraForward(camera);

    if (moveInWorldPlane)
    {
        // Project vector onto world plane (the plane defined by the up vector)
        if (fabsf(camera->up.z) > 0.7071f) forward.z = 0;
        else if (fabsf(camera->up.x) > 0.7071f) forward.x = 0;
        else forward.y = 0;

        forward = Vector3Normalize(forward);
    }

    // Scale by distance
    forward = Vector3Scale(forward, distance);

    // Move position and target
    worldCamPos += Vector3Add(camera->position, forward);
}

// Moves the camera in its up direction
static void CameraMoveUp(Camera* camera, double distance, Vector3d& worldCamPos)
{
    Vector3 up = GetCameraUp(camera);

    // Scale by distance
    up = Vector3Scale(up, distance);

    // Move position and target
    worldCamPos += Vector3Add(camera->position, up);
}

// Moves the camera target in its current right direction
static void CameraMoveRight(Camera* camera, double distance, bool moveInWorldPlane, Vector3d& worldCamPos)
{
    Vector3 right = GetCameraRight(camera);

    if (moveInWorldPlane)
    {
        // Project vector onto world plane (the plane defined by the up vector)
        if (fabsf(camera->up.z) > 0.7071f) right.z = 0;
        else if (fabsf(camera->up.x) > 0.7071f) right.x = 0;
        else right.y = 0;

        right = Vector3Normalize(right);
    }

    // Scale by distance
    right = Vector3Scale(right, distance);

    // Move position and target
    worldCamPos += Vector3Add(camera->position, right);
}

// Moves the camera position closer/farther to/from the camera target

// Rotates the camera around its up vector
// Yaw is "looking left and right"
// If rotateAroundTarget is false, the camera rotates around its position
// Note: angle must be provided in radians
static void CameraYaw(Camera* camera, float angle, bool rotateAroundTarget)
{
    // Rotation axis
    Vector3 up = GetCameraUp(camera);

    // View vector
    Vector3 targetPosition = Vector3Subtract(camera->target, camera->position);

    // Rotate view vector around up axis
    targetPosition = Vector3RotateByAxisAngle(targetPosition, up, angle);

    if (rotateAroundTarget)
    {
        // Move position relative to target
        camera->position = Vector3Subtract(camera->target, targetPosition);
    }
    else // rotate around camera.position
    {
        // Move target relative to position
        camera->target = Vector3Add(camera->position, targetPosition);
    }
}

// Rotates the camera around its right vector, pitch is "looking up and down"
//  - lockView prevents camera overrotation (aka "somersaults")
//  - rotateAroundTarget defines if rotation is around target or around its position
//  - rotateUp rotates the up direction as well (typically only useful in CAMERA_FREE)
// NOTE: [angle] must be provided in radians
static void CameraPitch(Camera* camera, float angle, bool lockView, bool rotateAroundTarget, bool rotateUp)
{
    // Up direction
    Vector3 up = GetCameraUp(camera);

    // View vector
    Vector3 targetPosition = Vector3Subtract(camera->target, camera->position);

    if (lockView)
    {
        // In these camera modes, clamp the Pitch angle
        // to allow only viewing straight up or down

        // Clamp view up
        float maxAngleUp = Vector3Angle(up, targetPosition);
        maxAngleUp -= 0.001f; // astatic void numerical errors
        if (angle > maxAngleUp) angle = maxAngleUp;

        // Clamp view down
        float maxAngleDown = Vector3Angle(Vector3Negate(up), targetPosition);
        maxAngleDown *= -1.0f; // downwards angle is negative
        maxAngleDown += 0.001f; // astatic void numerical errors
        if (angle < maxAngleDown) angle = maxAngleDown;
    }

    // Rotation axis
    Vector3 right = GetCameraRight(camera);

    // Rotate view vector around right axis
    targetPosition = Vector3RotateByAxisAngle(targetPosition, right, angle);

    if (rotateAroundTarget)
    {
        // Move position relative to target
        camera->position = Vector3Subtract(camera->target, targetPosition);
    }
    else // Rotate around camera.position
    {
        // Move target relative to position
        camera->target = Vector3Add(camera->position, targetPosition);
    }

    if (rotateUp)
    {
        // Rotate up direction around right axis
        camera->up = Vector3RotateByAxisAngle(camera->up, right, angle);
    }
}
void UpdateCameraCustom(Camera* camera, float* camSpeed, Vector3d& worldCamPos)
{
    Vector2 mousePositionDelta = GetMouseDelta();
    // Camera speeds based on frame time
    float cameraMoveSpeed = *camSpeed * GetFrameTime();
    float cameraRotationSpeed = CAMERA_ROTATION_SPEED * GetFrameTime();
    CameraYaw(camera, -mousePositionDelta.x * CAMERA_MOUSE_MOVE_SENSITIVITY, false);
    CameraPitch(camera, -mousePositionDelta.y * CAMERA_MOUSE_MOVE_SENSITIVITY, true, false, false);
    // Camera rotation
    if (IsKeyDown(KEY_DOWN)) CameraPitch(camera, -cameraRotationSpeed, true, false, false);
    if (IsKeyDown(KEY_UP)) CameraPitch(camera, cameraRotationSpeed, true, false, false);
    if (IsKeyDown(KEY_RIGHT)) CameraYaw(camera, -cameraRotationSpeed, false);
    if (IsKeyDown(KEY_LEFT)) CameraYaw(camera, cameraRotationSpeed, false);

    // Camera movement
    // Camera pan (for CAMERA_FREE)

    // Keyboard support
    if (IsKeyDown(KEY_W)) CameraMoveForward(camera, cameraMoveSpeed, false, worldCamPos);
    if (IsKeyDown(KEY_A)) CameraMoveRight(camera, -cameraMoveSpeed, false, worldCamPos);
    if (IsKeyDown(KEY_S)) CameraMoveForward(camera, -cameraMoveSpeed, false, worldCamPos);
    if (IsKeyDown(KEY_D)) CameraMoveRight(camera, cameraMoveSpeed, false, worldCamPos);

    if (IsKeyDown(KEY_SPACE)) CameraMoveUp(camera, cameraMoveSpeed, worldCamPos);
    if (IsKeyDown(KEY_LEFT_CONTROL)) CameraMoveUp(camera, -cameraMoveSpeed, worldCamPos);
    if (std::abs(GetMouseWheelMove()) > 0.2f) *camSpeed *= 1 + std::clamp(GetMouseWheelMove() * 0.05f, -0.05f, 0.05f);
}
Vector3 offset;
void UpdateOrbitalCamera(Camera& camera, Vector3d& worldCamPos, const Vector3d& targetPos, float& trackDistance, bool lock)
{
    double wheel = GetMouseWheelMove();
    if (wheel != 0.0) trackDistance *= (1.0 - wheel * 0.1);

    if (lock)
    {
        Vector2 mouseDelta = GetMouseDelta();
        CameraYaw(&camera, -mouseDelta.x * CAMERA_MOUSE_MOVE_SENSITIVITY, false);
        CameraPitch(&camera, -mouseDelta.y * CAMERA_MOUSE_MOVE_SENSITIVITY, true, false, false);
    }

    Vector3d forward = Vector3dNormalize({ camera.target.x, camera.target.y, camera.target.z });
    worldCamPos = Vector3dSubtract(targetPos, Vector3dScale(forward, trackDistance));
}
