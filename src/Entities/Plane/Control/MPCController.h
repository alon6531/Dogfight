#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

#include <memory>

#include "raylib.h"
#include <vector>



#include "raylib.h"
#include <vector>

class Plane;

struct MPCResult {
    Vector3 bestSteer;
    float cost;
};

class MPCController {
public:
    MPCController(int horizon = 10, float dt = 0.1f)
        : m_horizon(horizon), m_predictionDt(dt) {}

    // מחשב את כיוון ההיגוי האופטימלי
    Vector3 CalculateBestSteer(Vector3 pos, Vector3 vel, Vector3 forward, Vector3 target, const std::vector<struct Obstacle> &obstacles, const std::shared_ptr<Plane> &enem);

private:
    int m_horizon;          // כמה צעדים קדימה לנבא
    float m_predictionDt;   // הפרש הזמן בין צעדי הניבוי

    // פונקציית העלות - ככל שהציון נמוך יותר, המסלול טוב יותר
    float CalculateCost(Vector3 finalPos, Vector3 finalForward, Vector3 target, bool collided);
};


#endif