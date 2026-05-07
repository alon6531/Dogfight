#include "MPCController.h"
#include "raymath.h"
#include "../../../World/GraphBuilder.h"
#include "../../../Collision/GJK.h"
#include "../../Plane/Plane.h"

Vector3 MPCController::CalculateBestSteer(Vector3 pos, Vector3 vel, Vector3 forward, Vector3 target,
                                          const std::vector<Obstacle>& obstacles, const std::shared_ptr<Plane> &enemy) {

    const int NUM_SAMPLES = 15; // Number of hypothetical paths to check
    Vector3 bestSteer = forward;
    float minCost = std::numeric_limits<float>::max();

    // Sample different directions (up, right, etc.)
    for (int i = 0; i < NUM_SAMPLES; i++) {
        // Create a reactive jitter vector (controlled random deviation from current forward)
        Vector3 jitter = {
            (float)GetRandomValue(-100, 100) / 100.0f,
            (float)GetRandomValue(-100, 100) / 100.0f,
            (float)GetRandomValue(-100, 100) / 100.0f
        };

        // Hypothetical direction: blending current forward with slight jitter
        Vector3 candidateDir = Vector3Normalize(Vector3Add(forward, Vector3Scale(jitter, 0.5f)));

        // Calculate the "cost" for this specific path
        float currentCost = SimulateAndGetCost(pos, vel, candidateDir, target, obstacles, enemy);

        // Keep the direction that results in the lowest cost
        if (currentCost < minCost) {
            minCost = currentCost;
            bestSteer = candidateDir;
        }
    }

    return bestSteer;
}

float MPCController::SimulateAndGetCost(Vector3 pos, Vector3 vel, Vector3 steerDir, Vector3 target,
                                        const std::vector<Obstacle>& obstacles, const std::shared_ptr<Plane>& enemy) const {
    float totalCost = 0.0f;
    Vector3 currentPos = pos;
    float speed = Vector3Length(vel);

    // Predict future positions over the horizon
    for (int i = 1; i <= m_horizon; i++) {
        // 1. Predict future position at step 'i'
        currentPos = Vector3Add(currentPos, Vector3Scale(steerDir, speed * m_predictionDt));

        // 2. Distance Penalty: We want to be as close to the target as possible at the end of the horizon
        if (i == m_horizon) {
            totalCost += Vector3Distance(currentPos, target) * 1.0f;
        }

        // 3. Collision Check: Use GJK to see if we hit the enemy at this future position
        if (enemy) {
            if (GJK::CheckCollisionAt(currentPos, steerDir, *enemy)) {
                totalCost += 10000.0f; // Massive penalty for crashing
                // Optimization: We could 'break' here since the path is already invalid
            }
        }

        // 4. Ground Avoidance: Penalty for being too low
        if (currentPos.y < 20.0f) {
            totalCost += 500.0f;
        }
    }

    return totalCost;
}
