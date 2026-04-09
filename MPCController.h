#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

#include "raylib.h"
#include <vector>

struct MPCParameters {
    int horizon = 10;        // כמה צעדים קדימה לחזות
    float dt = 0.1f;         // הפרש זמן בין צעדי החיזוי
    float maxForce = 50.0f;  // עוצמת היגוי מקסימלית (G-Limit)
    float weightPos = 1.0f;  // חשיבות הדיוק במסלול
    float weightSmooth = 0.5f; // חשיבות החלקת הפנייה
};

class MPCController {
public:
    MPCController(MPCParameters params) : m_params(params) {}

    // מחשב את וקטור ההיגוי האופטימלי
    Vector3 CalculateSteering(Vector3 pos, Vector3 vel, const std::vector<Vector3>& path) const;
    static Vector3 Vector3RotateTowards(Vector3 current, Vector3 target, float maxAngle);


private:

    MPCParameters m_params;
};

#endif