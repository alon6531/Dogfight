#include "MPCController.h"
#include "raymath.h"
#include <algorithm>

Vector3 MPCController::CalculateSteering(Vector3 pos, Vector3 vel, const std::vector<Vector3> &path) const {
    if (path.size() < 2) return (Vector3Length(vel) > 0.1f) ? Vector3Normalize(vel) : Vector3{0, 0, 1};

    Vector3 bestControl = Vector3Zero();
    float minCost = 1e20f;

    Vector3 currentForward = (Vector3Length(vel) > 0.1f) ? Vector3Normalize(vel) : Vector3{0, 0, 1};
    float currentSpeed = Vector3Length(vel);


    const float MAX_TURN_RATE = 1.2f;
    const int NUM_SAMPLES = 25;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        Vector3 testControlDir;

        if (i == 0) {
            testControlDir = currentForward;
        } else {

            float maxAngle = MAX_TURN_RATE * m_params.dt * 5.0f;
            float angleScale = (i % 2 == 0) ? 0.5f : 1.0f;

            float phi = i * (360.0f / (float)NUM_SAMPLES) * DEG2RAD;
            float theta = maxAngle * angleScale;


            Vector3 rawOffset = { sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), cosf(theta) };
            testControlDir = Vector3Normalize(Vector3RotateByQuaternion(rawOffset, QuaternionFromVector3ToVector3({0,0,1}, currentForward)));
        }

        float totalCost = 0;
        Vector3 simPos = pos;
        Vector3 simVel = vel;
        Vector3 simForward = currentForward;


        for (int h = 0; h < m_params.horizon; h++) {

            simForward = Vector3Normalize(Vector3RotateTowards(simForward, testControlDir, MAX_TURN_RATE * m_params.dt));


            simVel = Vector3Scale(simForward, currentSpeed);
            simPos = Vector3Add(simPos, Vector3Scale(simVel, m_params.dt));


            int lookAhead = std::min((int)path.size() - 1, h + 1);
            float distSq = Vector3DistanceSqr(simPos, path[lookAhead]);


            totalCost += distSq * m_params.weightPos;


            Vector3 pathDir = Vector3Normalize(Vector3Subtract(path[lookAhead], simPos));
            float alignment = 1.0f - Vector3DotProduct(simForward, pathDir);
            totalCost += alignment * 100.0f;
        }


        float angularChange = 1.0f - Vector3DotProduct(testControlDir, currentForward);
        totalCost += (angularChange * angularChange) * 800.0f;

        if (totalCost < minCost) {
            minCost = totalCost;
            bestControl = testControlDir;
        }
    }

    return bestControl;
}
Vector3 MPCController::Vector3RotateTowards(Vector3 current, Vector3 target, float maxAngle) {

    float angle = Vector3Angle(current, target);


    if (angle <= maxAngle) return target;


    Vector3 axis = Vector3CrossProduct(current, target);


    if (Vector3LengthSqr(axis) < 0.0001f) {

        axis = { 0, 1, 0 };
    } else {
        axis = Vector3Normalize(axis);
    }


    Quaternion rotation = QuaternionFromAxisAngle(axis, maxAngle);


    return Vector3RotateByQuaternion(current, rotation);
}