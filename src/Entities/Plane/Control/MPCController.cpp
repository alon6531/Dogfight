#include "MPCController.h"
#include "raymath.h"
#include "../../../World/GraphBuilder.h"
#include "../../../Collision/GJK.h" // וודא שהקלאס החדש זמין
#include "../../Plane/Plane.h" // לצורך הגישה לנתוני האויב

Vector3 MPCController::CalculateBestSteer(Vector3 pos, Vector3 vel, Vector3 forward, Vector3 target,
                                          const std::vector<Obstacle>& obstacles, const std::shared_ptr<Plane> &enemy) {
    Vector3 bestSteer = forward;
    float minCost = __FLT_MAX__;

    const int steps = 20;
    const float stepDt = 0.2f;
    float speed = Vector3Length(vel);

    for (float yaw = -1.0f; yaw <= 1.0f; yaw += 0.4f) {
        for (float pitch = -0.5f; pitch <= 0.5f; pitch += 0.5f) {

            Vector3 simPos = pos;
            Vector3 simForward = forward;

            Vector3 candidateTargetDir = Vector3RotateByAxisAngle(forward, {0, 1, 0}, yaw * 0.8f);
            candidateTargetDir = Vector3RotateByAxisAngle(candidateTargetDir, Vector3CrossProduct(forward, {0, 1, 0}), pitch * 0.4f);

            bool collided = false;

            for (int i = 0; i < steps; ++i) {
                simForward = Vector3Normalize(Vector3Lerp(simForward, candidateTargetDir, 2.0f * stepDt));
                simPos = Vector3Add(simPos, Vector3Scale(simForward, speed * stepDt));

                // 1. בדיקת התנגשות מול האויב (GJK) - ללא לופ!
                if (enemy) {
                    float distToEnemy = Vector3Distance(simPos, enemy->GetPosition());
                    // Broad Phase: בדיקת רדיוס גסה לחיסכון בביצועים
                    if (distToEnemy < 250.0f) {
                        // Narrow Phase: GJK מדויק במיקום החזוי
                        if (GJK::CheckCollisionAt(simPos, simForward, *enemy)) {
                            collided = true;
                            break;
                        }
                    }
                }

                // 2. בדיקת מכשולים סטטיים (הרים/מבנים)
                for (const auto& obs : obstacles) {
                    if (Vector3Distance(simPos, obs.pos) < obs.radius + 60.0f) {
                        collided = true;
                        break;
                    }
                }
                if (collided) break;
            }

            float cost = CalculateCost(simPos, simForward, target, collided);

            // קנס על שינוי היגוי למניעת רעידות
            float steeringChange = 1.0f - Vector3DotProduct(candidateTargetDir, forward);
            cost += steeringChange * 150.0f;

            if (cost < minCost) {
                minCost = cost;
                bestSteer = candidateTargetDir;
            }
        }
    }
    return bestSteer;
}

float MPCController::CalculateCost(Vector3 finalPos, Vector3 finalForward, Vector3 target, bool collided) {
    if (collided) return 5000000.0f; // עלות גבוהה מאוד להתנגשות

    float distToTarget = Vector3Distance(finalPos, target);

    Vector3 dirToTarget = Vector3Normalize(Vector3Subtract(target, finalPos));
    float alignment = 1.0f - Vector3DotProduct(finalForward, dirToTarget);

    // קנס גובה (מניעת התרסקות בקרקע)
    float altitudePenalty = (finalPos.y < 120.0f) ? (120.0f - finalPos.y) * 20.0f : 0.0f;

    return distToTarget + (alignment * 900.0f) + altitudePenalty;
}