#include "MPCController.h"
#include "raymath.h"
#include <algorithm>

Vector3 MPCController::CalculateSteering(Vector3 pos, Vector3 vel, const std::vector<Vector3> &path) const {
    if (path.size() < 2) return (Vector3Length(vel) > 0.1f) ? Vector3Normalize(vel) : Vector3{0, 0, 1};

    Vector3 bestControl = Vector3Zero();
    float minCost = 1e20f;

    Vector3 currentForward = (Vector3Length(vel) > 0.1f) ? Vector3Normalize(vel) : Vector3{0, 0, 1};
    float currentSpeed = Vector3Length(vel);

    // דגימה חכמה יותר: במקום פיזור אקראי, נדגום קשתות פנייה אפשריות
    // מטוס קרב מוגבל בזווית הצידוד שלו (Turn Rate)
    for (int i = 0; i < 45; i++) {
        Vector3 testDir;

        if (i == 0) {
            testDir = currentForward; // כיוון ישר
        } else {
            // יצירת וקטורים במעגל סביב כיוון הטיסה הנוכחי
            // הגבלה של הסטייה מהכיוון הנוכחי כדי לדמות רדיוס פנייה ריאליסטי
            float spread = 0.25f; // ככל שזה קטן יותר, המטוס יעשה פניות רחבות יותר
            float angle = i * (360.0f / 44.0f) * DEG2RAD;

            Vector3 offset = { cosf(angle) * spread, sinf(angle) * spread, 1.0f };
            // סיבוב ה-offset שיהיה יחסי לכיוון הטיסה (Local to World)
            testDir = Vector3Normalize(Vector3RotateByQuaternion(offset, QuaternionFromVector3ToVector3({0,0,1}, currentForward)));
        }

        float totalCost = 0;
        Vector3 simPos = pos;
        Vector3 simVel = vel;

        for (int h = 0; h < m_params.horizon; h++) {
            // דינמיקה ריאליסטית: הכוח פועל בניצב למהירות (Lift) או כדחף (Thrust)
            // כאן נשתמש במודל פשוט שבו testDir הוא הכיוון אליו המטוס שואף
            Vector3 force = Vector3Scale(testDir, m_params.maxForce);

            // אינטגרציה של המהירות (Euler)
            simVel = Vector3Add(simVel, Vector3Scale(force, m_params.dt));

            // הגבלת מהירות מקסימלית (Drag)
            if (Vector3Length(simVel) > 200.0f) simVel = Vector3Scale(Vector3Normalize(simVel), 200.0f);

            simPos = Vector3Add(simPos, Vector3Scale(simVel, m_params.dt));

            // בחירת נקודת מטרה דינמית על הנתיב
            // ככל שאנחנו רחוקים יותר בסימולציה, נסתכל על נקודה רחוקה יותר בנתיב
            int pathStep = std::min((int)path.size() - 1, h + 2);
            float distToPath = Vector3Distance(simPos, path[pathStep]);

            // עלות המרחק מהנתיב
            totalCost += (distToPath * distToPath) * m_params.weightPos;

            // עלות הסטייה מהכיוון הרצוי (כדי לשמור על טיסה חלקה)
            Vector3 desiredDir = Vector3Normalize(Vector3Subtract(path[pathStep], simPos));
            totalCost += (1.0f - Vector3DotProduct(Vector3Normalize(simVel), desiredDir)) * 50.0f;
        }

        // קנס על שינוי כיוון חריף (Angular Acceleration Penalty)
        float angularChange = 1.0f - Vector3DotProduct(testDir, currentForward);
        totalCost += (angularChange * angularChange) * 500.0f;

        if (totalCost < minCost) {
            minCost = totalCost;
            bestControl = testDir;
        }
    }
    return bestControl;
}
