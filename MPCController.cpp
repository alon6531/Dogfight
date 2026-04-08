#include "MPCController.h"
#include "raymath.h"
#include <algorithm>

Vector3 MPCController::CalculateSteering(Vector3 pos, Vector3 vel, const std::vector<Vector3> &path) const {
    if (path.size() < 2) return (Vector3Length(vel) > 0.1f) ? Vector3Normalize(vel) : Vector3{0, 0, 1};

    Vector3 bestControl = Vector3Zero();
    float minCost = 1e20f;
    Vector3 currentDir = (Vector3Length(vel) > 0.1f)
                             ? Vector3Normalize(vel)
                             : Vector3Normalize(Vector3Subtract(path[1], pos));

    for (int i = 0; i < 32; i++) {
        Vector3 testDir;
        if (i == 0) {
            testDir = currentDir;
        } else {
            // סטיות קלות כדי לאפשר למטוס לתקן מסלול בעדינות
            float angle = i * (360.0f / 31.0f) * DEG2RAD;
            float pitch = (i % 3 - 1) * 0.25f;

            testDir = {
                currentDir.x + cosf(angle) * 0.4f, // הגדלנו מעט את טווח החיפוש
                currentDir.y + pitch,
                currentDir.z + sinf(angle) * 0.4f
            };
            testDir = Vector3Normalize(testDir);
        }

        Vector3 simPos = pos;
        Vector3 simVel = vel;
        float totalCost = 0;

        for (int h = 0; h < m_params.horizon; h++) {
            simVel = Vector3Add(simVel, Vector3Scale(testDir, m_params.maxForce * m_params.dt));
            simPos = Vector3Add(simPos, Vector3Scale(simVel, m_params.dt));

            int targetIdx = std::min(2 + (h / 3), (int) path.size() - 1);
            float distToPath = Vector3Distance(simPos, path[targetIdx]);

            totalCost += (distToPath * distToPath) * m_params.weightPos;
        }

        // קנס על "זגזוג" - מעדיף את הכיוון הנוכחי
        float alignment = 1.0f - Vector3DotProduct(testDir, currentDir);
        totalCost += alignment * 100.0f;

        // --- התיקון לקטיעת הלולאה וצורת ה-L ---
        // המטוס יעדיף לחתוך את פינות הרשת אם זה מפנה אותו ישירות לאויב
        Vector3 finalTargetDir = Vector3Normalize(Vector3Subtract(path.back(), pos));
        float finalAlignment = 1.0f - Vector3DotProduct(testDir, finalTargetDir);
        totalCost += finalAlignment * 80.0f; // ככל שהערך גבוה, הוא יחתוך יותר את הפינה

        if (totalCost < minCost) {
            minCost = totalCost;
            bestControl = testDir;
        }
    }
    return bestControl;
}
