#include "FSM.h"
#include "Plane.h"
#include "raymath.h"
#include "GraphBuilder.h"
#include "external/glfw/src/internal.h"



FSM::FSM(NavigationGraph& graph) : m_graph(graph) {
    InitializeTable();
}

void FSM::InitializeTable() {
    // טבלת חוקים כללית לכל מטוס
    transitionTable[{AIState::IDLE, AIEvent::OBJECTIVE_ACTIVE}] = AIState::PATROL;
    transitionTable[{AIState::PATROL, AIEvent::REACHED_OBJECTIVE}] = AIState::PURSUIT;
    transitionTable[{AIState::PURSUIT, AIEvent::ENEMY_LOST}] = AIState::IDLE;
    transitionTable[{AIState::PURSUIT, AIEvent::DANGER_DETECTED}] = AIState::EVADE;
    transitionTable[{AIState::EVADE, AIEvent::ENEMY_SPOTTED}] = AIState::PURSUIT;
}

bool FSM::Update(Plane& actor, Plane& opponent, const Vector3& targetPos, float deltaTime) {
    AIEvent currentEvent;
    bool eventOccurred = false;

    float myStrength = CalculateTacticalStrength(actor, opponent);
    float opponentStrength = CalculateTacticalStrength(opponent, actor);
    constexpr float ENGAGE_THRESHOLD = 0.6f;

    if (actor.GetCurrentState() == AIState::IDLE) {
        currentEvent = AIEvent::OBJECTIVE_ACTIVE;
        actor.SetDestinationViaAStar(m_graph.GetClosestNode(targetPos));
        eventOccurred = true;
    }
    else if (actor.GetCurrentState() == AIState::PATROL && Vector3Distance(actor.GetPosition(), targetPos) < 40.0f) {
        currentEvent = AIEvent::REACHED_OBJECTIVE;


        opponent.SetCurrentState(AIState::EVADE);
        opponent.SetCurrentEvent(AIEvent::DANGER_DETECTED);
        eventOccurred = true;
    }
    else if (myStrength > opponentStrength && myStrength > ENGAGE_THRESHOLD) {
        if (actor.GetCurrentState() != AIState::PURSUIT) {
            actor.SetCurrentState(AIState::PURSUIT);
            std::cout << "FSM: Actor dominates (Strength: " << myStrength << "). Engaging!" << std::endl;
        }
        // במקביל, האויב נכנס למגננה (אם הוא לא שם כבר)
        if (opponent.GetCurrentState() != AIState::EVADE) {
            opponent.SetCurrentState(AIState::EVADE);
        }
    }
    else if (opponentStrength > myStrength && opponentStrength > ENGAGE_THRESHOLD) {
        if (actor.GetCurrentState() != AIState::EVADE) {
            actor.SetCurrentState(AIState::EVADE);
            std::cout << "FSM: Opponent dominates (Strength: " << opponentStrength << "). Evading!" << std::endl;
        }
        // האויב הופך לרודף
        if (opponent.GetCurrentState() != AIState::PURSUIT) {
            opponent.SetCurrentState(AIState::PURSUIT);
        }
    }
    // תנאי ג': מצב התחלתי
    else if (actor.GetCurrentState() == AIState::IDLE) {
        actor.SetCurrentState(AIState::PATROL);
    }





    const float LOCK_RANGE = 250.0f;
    const float LOCK_ANGLE = 50.0f;
    const float REQUIRED_LOCK_TIME = 3.0f;

    bool actorIsUnderCrosshair = IsTargetLocked(opponent, actor, LOCK_RANGE, LOCK_ANGLE);
    bool opponentIsUnderCrosshair = IsTargetLocked(actor, opponent, LOCK_RANGE, LOCK_ANGLE);

    actor.SetIsLocked(false);
    opponent.SetIsLocked(false);

    actor.UpdateLockTimer(deltaTime, actorIsUnderCrosshair);
    opponent.UpdateLockTimer(deltaTime, opponentIsUnderCrosshair);





    if (actor.GetLockTimer() >= REQUIRED_LOCK_TIME) {
        actor.SetIsLocked(true);
        std::cout << "GAME OVER: Opponent locked you for 3 seconds!" << std::endl;
        return true;
    }

    if (opponent.GetLockTimer() >= REQUIRED_LOCK_TIME) {
        opponent.SetIsLocked(true);
        std::cout << "VICTORY: You locked the enemy for 3 seconds!" << std::endl;
        return true;
    }



    if (eventOccurred) {
        AIState stateInPlane = actor.GetCurrentState();
        auto key = std::make_pair(stateInPlane, currentEvent);

        if (transitionTable.count(key)) {
            AIState nextState = transitionTable[key];
            if (nextState != stateInPlane) {
                actor.SetCurrentState(nextState);
                std::cout << "Plane [" << &actor << "] switched to: " << GetCurrentStateName(nextState) << std::endl;
            }
        }
    }
    return false;
}

bool FSM::IsTargetLocked(const Plane& attacker, const Plane& target, float maxDist, float lockAngleDeg) {
    Vector3 attackerPos = attacker.GetPosition();
    Vector3 targetPos = target.GetPosition();

    // 1. בדיקת מרחק
    float dist = Vector3Distance(attackerPos, targetPos);
    if (dist > maxDist) return false;

    // 2. בדיקת זווית (Lock Cone)
    Vector3 toTarget = Vector3Normalize(Vector3Subtract(targetPos, attackerPos));

    // שימוש בוקטור המהירות ככיוון החרטום (Forward)
    Vector3 forward = (Vector3LengthSqr(attacker.GetVelocity()) > 0.1f)
                      ? Vector3Normalize(attacker.GetVelocity())
                      : Vector3{0, 0, 1};

    // חישוב הקוסינוס של הזווית בין החרטום למטרה
    float dotProduct = Vector3DotProduct(forward, toTarget);

    // המרת זווית הפתיחה לקוסינוס (חצי מזווית הקונוס הכוללת)
    float minDot = cosf((lockAngleDeg * 0.5f) * DEG2RAD);

    // אם ה-Dot Product גדול מה-minDot, המטרה בתוך הקונוס
    return dotProduct >= minDot;
}

float FSM::CalculateTacticalStrength(const Plane& owner, const Plane& target) {
    // 1. חישוב וקטור מבט (Facing)
    Vector3 toTarget = Vector3Normalize(Vector3Subtract(target.GetPosition(), owner.GetPosition()));
    Vector3 forward = (Vector3LengthSqr(owner.GetVelocity()) > 0.1f)
                      ? Vector3Normalize(owner.GetVelocity())
                      : Vector3{0, 0, 1};

    float lookDot = Vector3DotProduct(forward, toTarget); // טווח: 1.0 (מסתכל עליו) עד -1.0 (גב אליו)

    // ננרמל את ה-lookDot שיהיה רק חיובי (0 אם הוא לא מסתכל עליו בכלל)
    float facingScore = fmaxf(0.0f, lookDot);

    // 2. חישוב יתרון גובה (Altitude Advantage)
    float heightDiff = owner.GetPosition().y - target.GetPosition().y;

    // ננרמל את הגובה: יתרון של 100 יחידות ומעלה נחשב "יתרון מקסימלי" (1.0)
    // גובה נמוך מהאויב יוריד את הציון
    float altitudeScore = Clamp(heightDiff / 100.0f, -1.0f, 1.0f);

    // 3. שילוב לציון סופי (חוזק טקטי)
    // נוסחה: (דיוק המבט * משקל) + (יתרון גובה * משקל)
    // אנחנו נותנים 70% חשיבות למבט ו-30% לגובה
    float totalStrength = (facingScore * 0.7f) + (altitudeScore * 0.3f);

    return totalStrength;
}

std::string FSM::GetCurrentStateName(AIState state) {
    switch (state) {
        case AIState::IDLE:            return "IDLE";
        case AIState::PATROL:          return "PATROL";
        case AIState::PURSUIT:         return "PURSUIT";
        case AIState::COLLISION_AVOID: return "COLLISION_AVOID";
        case AIState::EVADE:           return "EVADE";
        default:                       return "UNKNOWN";
    }
}