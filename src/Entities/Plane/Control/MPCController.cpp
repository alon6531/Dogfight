#include "MPCController.h"
#include "raymath.h"
#include "../../../World/GraphBuilder.h"
#include "../../../Collision/GJK.h"
#include "../../Plane/Plane.h"




Vector3 MPCController::CalculateBestSteer(Vector3 pos, Vector3 vel, Vector3 forward, Vector3 target,
                                          const std::vector<Obstacle>& obstacles, const std::shared_ptr<Plane> &enemy) {

    Vector3 bestSteer = forward;
    float minCost = std::numeric_limits<float>::max();

    for (int i = 0; i < MPC_NUM_SAMPLES; i++) {
        Vector3 jitter = {
            (float)GetRandomValue(-MPC_JITTER_RANGE, MPC_JITTER_RANGE) / MPC_JITTER_SCALE,
            (float)GetRandomValue(-MPC_JITTER_RANGE, MPC_JITTER_RANGE) / MPC_JITTER_SCALE,
            (float)GetRandomValue(-MPC_JITTER_RANGE, MPC_JITTER_RANGE) / MPC_JITTER_SCALE
        };

        Vector3 candidateDir = Vector3Normalize(Vector3Add(forward, Vector3Scale(jitter, MPC_JITTER_BLEND)));

        float currentCost = SimulateAndGetCost(pos, vel, candidateDir, target, obstacles, enemy);

        if (currentCost < minCost) {
            minCost = currentCost;
            bestSteer = candidateDir;
        }
    }

    return bestSteer;
}

float MPCController::SimulateAndGetCost(Vector3 pos, Vector3 vel, Vector3 steerDir, Vector3 target,
                                        const std::vector<Obstacle>& obstacles, const std::shared_ptr<Plane>& enemy) const {
    float totalCost  = 0.0f;
    Vector3 currentPos = pos;
    float speed = Vector3Length(vel);

    for (int i = 1; i <= m_horizon; i++) {
        currentPos = Vector3Add(currentPos, Vector3Scale(steerDir, speed * m_predictionDt));

        if (i == m_horizon) {
            totalCost += Vector3Distance(currentPos, target) * MPC_DISTANCE_WEIGHT;
        }

        if (enemy) {
            if (GJK::CheckCollisionAt(currentPos, steerDir, *enemy)) {
                totalCost += MPC_COLLISION_PENALTY;
            }
        }

        if (currentPos.y < MPC_GROUND_MIN_HEIGHT) {
            totalCost += MPC_GROUND_PENALTY;
        }
    }

    return totalCost;
}