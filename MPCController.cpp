#include "MPCController.h"
#include "raymath.h"
#include <algorithm>

Vector3 MPCController::CalculateSteering(Vector3 pos, Vector3 vel, const std::vector<Vector3> &path) const {
    if (path.size() < 2) return (Vector3Length(vel) > 0.1f) ? Vector3Normalize(vel) : Vector3{0, 0, 1};

    Vector3 bestControl = Vector3Zero();
    float minCost = 1e20f;

    Vector3 currentForward = (Vector3Length(vel) > 0.1f) ? Vector3Normalize(vel) : Vector3{0, 0, 1};
    float currentSpeed = Vector3Length(vel);

    // הגדרת מגבלת פנייה ריאליסטית בתוך ה-MPC
    // ככל שהערך הזה נמוך יותר, ה-MPC יבין שהוא חייב לעשות סיבובים רחבים יותר
    const float MAX_TURN_RATE = 1.2f;
    const int NUM_SAMPLES = 25;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        Vector3 testControlDir;

        if (i == 0) {
            testControlDir = currentForward; // טיסה ישר
        } else {
            // דגימת כיוונים במעגל, אבל עם הגבלה לזווית שהמטוס באמת יכול להגיע אליה
            // אנחנו בודקים פקודות היגוי בתוך "קונוס" היכולת של המטוס
            float maxAngle = MAX_TURN_RATE * m_params.dt * 5.0f; // בדיקה של פוטנציאל פנייה קצר טווח
            float angleScale = (i % 2 == 0) ? 0.5f : 1.0f; // דגימה רב-שכבתית (קרוב ורחוק)

            float phi = i * (360.0f / (float)NUM_SAMPLES) * DEG2RAD;
            float theta = maxAngle * angleScale;

            // יצירת וקטור דגימה סביב ציר ה-Z והמרתו לכיוון הטיסה
            Vector3 rawOffset = { sinf(theta) * cosf(phi), sinf(theta) * sinf(phi), cosf(theta) };
            testControlDir = Vector3Normalize(Vector3RotateByQuaternion(rawOffset, QuaternionFromVector3ToVector3({0,0,1}, currentForward)));
        }

        float totalCost = 0;
        Vector3 simPos = pos;
        Vector3 simVel = vel;
        Vector3 simForward = currentForward;

        // --- הרצת הסימולציה קדימה (The Horizon) ---
        for (int h = 0; h < m_params.horizon; h++) {
            // כפיית רדיוס פנייה בתוך הסימולציה:
            // המטוס לא יכול לפנות ישר ל-testControlDir, הוא "מסתובב אליו" לאט
            simForward = Vector3Normalize(Vector3RotateTowards(simForward, testControlDir, MAX_TURN_RATE * m_params.dt));

            // עדכון המהירות והמיקום לפי הכיוון החדש
            simVel = Vector3Scale(simForward, currentSpeed);
            simPos = Vector3Add(simPos, Vector3Scale(simVel, m_params.dt));

            // בחירת נקודת יעד דינמית בנתיב (מסתכלים רחוק יותר ככל שהאופק מתקדם)
            int lookAhead = std::min((int)path.size() - 1, h + 1);
            float distSq = Vector3DistanceSqr(simPos, path[lookAhead]);

            // 1. עלות מרחק מהנתיב (משקל גבוה לשמירה על המסלול)
            totalCost += distSq * m_params.weightPos;

            // 2. עלות זוויתית (כמה הטיסה "מיושרת" עם הנתיב)
            Vector3 pathDir = Vector3Normalize(Vector3Subtract(path[lookAhead], simPos));
            float alignment = 1.0f - Vector3DotProduct(simForward, pathDir);
            totalCost += alignment * 100.0f;
        }

        // 3. קנס על פניות חדות מדי (כדי להעדף נתיבים ישרים ומהירים)
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
    // חישוב הזווית בין שני הוקטורים (ברדיאנים)
    float angle = Vector3Angle(current, target);

    // אם הזווית קטנה מהשינוי המקסימלי המותר, פשוט נחזיר את היעד
    if (angle <= maxAngle) return target;

    // חישוב ציר הסיבוב (ניצב לשני הוקטורים)
    Vector3 axis = Vector3CrossProduct(current, target);

    // אם הוקטורים הפוכים לגמרי (180 מעלות), ה-Cross Product יהיה אפס
    if (Vector3LengthSqr(axis) < 0.0001f) {
        // במקרה כזה נבחר ציר שרירותי (למשל למעלה)
        axis = { 0, 1, 0 };
    } else {
        axis = Vector3Normalize(axis);
    }

    // יצירת קוונטרניון לסיבוב הוקטור ב-maxAngle מעלות סביב הציר שמצאנו
    Quaternion rotation = QuaternionFromAxisAngle(axis, maxAngle);

    // סיבוב הוקטור הנוכחי
    return Vector3RotateByQuaternion(current, rotation);
}