#include "EvasionState.h"
#include "../../Plane.h"
#include "raymath.h"
#include <cfloat>
#include "imgui.h"

EvasionState::EvasionState(Plane& self, NavigationGraph& navGraph, std::shared_ptr<Plane> enemy)
    : AIState(self, navGraph), m_enemy(std::move(enemy)) {
    m_dStarLite = std::make_unique<DStarLite>(p_graph);
}

EvasionState::~EvasionState() = default;

int EvasionState::PickEscapeNode(Vector3 selfPos, Vector3 enemyPos) const {
    const float MAP_CEILING = p_graph.GetArenaSize().y;
    const float MAP_LIMIT_X = p_graph.GetArenaSize().x / 2.0f;
    const float MAP_LIMIT_Z = p_graph.GetArenaSize().z / 2.0f;

    const float W_ALIGNMENT = 200.0f;
    const float W_DISTANCE = 20.0f;
    const float W_PENALTY = 5000.0f;

    // משתנה הרעש - ככל שהוא גבוה יותר, ה-AI יתעלם מהלוגיקה ופשוט "ישתולל"
    // אפשר להוסיף אותו ל-Header כ-m_noiseScale
    const float noiseScale = 1600.0f;

    const auto& nodes = p_graph.GetNodes();
    int totalNodes = (int)nodes.size();
    int bestIdx = -1;
    float bestScore = -FLT_MAX;
    Vector3 forward = p_self.GetForward();

    for (int attempt = 0; attempt < m_candidateSamples; ++attempt) {
        int idx = GetRandomValue(0, totalNodes - 1);
        if (nodes[idx].neighbors.empty()) continue;

        Vector3 nPos = nodes[idx].position;

        // 1. מרחק מהאויב (פסילה בסיסית)
        float distEnemy = Vector3Distance(nPos, enemyPos);
        if (distEnemy < m_minEscapeDist * 0.4f) continue;

        // 2. כיווניות (Alignment)
        Vector3 dirToNode = Vector3Normalize(Vector3Subtract(nPos, selfPos));
        float dot = Vector3DotProduct(forward, dirToNode);
        float alignmentScore = (1.0f - fabsf(dot - 0.5f)) * W_ALIGNMENT;

        // 3. עונש גבולות (חובה כדי שלא יצא מהמפה)
        float penalty = 0.0f;
        if (fabsf(nPos.x) > (MAP_LIMIT_X - 150.0f) ||
            fabsf(nPos.z) > (MAP_LIMIT_Z - 150.0f) ||
            nPos.y > (MAP_CEILING - 100.0f) ||
            nPos.y < 50.0f) {
            penalty = W_PENALTY;
        }

        // 4. מרחק
        float distScore = sqrtf(distEnemy) * W_DISTANCE;

        // 5. רעש רנדומלי חזק (The Chaos Factor)
        // אנחנו מגרילים מספר בטווח רחב מאוד כדי ש"ינצח" את שאר הציונים
        float chaos = (float)GetRandomValue(0, (int)noiseScale);

        // שקלול סופי: רעש משמעותי + לוגיקה מינימלית
        float score = alignmentScore + distScore + chaos - penalty;

        if (score > bestScore) {
            bestScore = score;
            bestIdx = idx;
        }
    }

    return (bestIdx != -1) ? bestIdx : GetRandomValue(0, totalNodes - 1);
}

AIStateType EvasionState::Update(float deltaTime) {
    m_stateTime += deltaTime;
    Vector3 selfPos = p_self.GetPosition();
    Vector3 enemyPos = m_enemy->GetPosition();

    if (ShouldReturnToPursuit(selfPos, enemyPos)) {
        ResetState();
        return AIStateType::PURSUIT;
    }

    UpdateEscapeTarget(selfPos, enemyPos);
    ReplanPathIfNeeded(selfPos);
    NavigatePath(selfPos, deltaTime, enemyPos);

    return AIStateType::EVASION;
}

bool EvasionState::ShouldReturnToPursuit(Vector3 selfPos, Vector3 enemyPos) const {
    float distSq = Vector3DistanceSqr(selfPos, enemyPos);
    bool hasHeightAdvantage = (selfPos.y - enemyPos.y) > (m_highGroundAdvantage + 20.0f);
    bool hasSafeDistance = distSq > (m_safeDistSq * 1.44f);

    return (m_stateTime > 2.0f && (hasHeightAdvantage || hasSafeDistance));
}

void EvasionState::ResetState() {
    p_path.clear();
    m_escapeNodeIdx = -1;
}

void EvasionState::UpdateEscapeTarget(Vector3 selfPos, Vector3 enemyPos) {
    bool needNewNode = (m_escapeNodeIdx == -1);

    if (m_escapeNodeIdx != -1) {
        Vector3 escapePos = p_graph.GetNodes()[m_escapeNodeIdx].position;
        if (Vector3DistanceSqr(selfPos, escapePos) < m_escapeNodeReachedSq) {
            needNewNode = true;
        }
    }

    if (needNewNode) {
        m_escapeNodeIdx = PickEscapeNode(selfPos, enemyPos);
        p_path.clear();
        m_pathTimer = m_replanInterval;
    }
}

void EvasionState::ReplanPathIfNeeded(Vector3 selfPos) {
    m_pathTimer += GetFrameTime();
    if ((m_pathTimer >= m_replanInterval || p_path.empty()) && m_escapeNodeIdx != -1) {
        m_pathTimer = 0.0f;
        Vector3 escapePos = p_graph.GetNodes()[m_escapeNodeIdx].position;
        auto pathPoints = m_dStarLite->PlanPath(selfPos, escapePos);
        float pathCost = m_dStarLite->GetLastPathCost();

        if (!pathPoints.empty() && pathCost < m_maxPathCost) {
            p_path.clear();
            int nodesToCopy = fmin((int)pathPoints.size(), m_maxEvasionNodes);
            for (int i = 0; i < nodesToCopy; i++) {
                p_path.push_back(pathPoints[i]);
            }
            m_currentDir = escapePos;
        } else {
            m_escapeNodeIdx = -1;
        }
    }
}

void EvasionState::NavigatePath(Vector3 selfPos, float deltaTime, Vector3 enemyPos) {
    if (!p_path.empty()) {
        Vector3 nextTarget = p_path.front();
        if (Vector3DistanceSqr(selfPos, nextTarget) < m_waypointRadiusSq) {
            p_path.pop_front();
            if (!p_path.empty()) nextTarget = p_path.front();
        }
        p_self.SteerTowards(nextTarget, deltaTime);
        m_currentDir = nextTarget;
    } else {
        ExecuteEmergencyClimb(selfPos, enemyPos, deltaTime);
    }
}

void EvasionState::ExecuteEmergencyClimb(Vector3 selfPos, Vector3 enemyPos, float deltaTime) {
    Vector3 awayDir = Vector3Normalize(Vector3Subtract(selfPos, enemyPos));
    Vector3 climbDir = Vector3Normalize({ awayDir.x * 0.6f, 1.0f, awayDir.z * 0.6f });
    Vector3 emergencyTarget = Vector3Add(selfPos, Vector3Scale(climbDir, 600.0f));
    p_self.SteerTowards(emergencyTarget, deltaTime);
    m_currentDir = emergencyTarget;
}

Vector3 EvasionState::GetCurrentTargetFromAI() {
    return m_currentDir;
}

void EvasionState::DrawDebugUI() {
    float padding = 20.0f;
    float windowWidth = 300.0f;

    ImGui::SetNextWindowPos({(float)GetScreenWidth() - windowWidth - padding, padding}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({windowWidth, 550}, ImGuiCond_Always);

    if (ImGui::Begin("AI Evasion Tuning", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)) {
        ImGui::TextColored({0, 255, 0, 255}, "STATE: EVASION");
        ImGui::Separator();

        ImGui::Text("Pathfinding");
        ImGui::SliderFloat("Replan Interval", &m_replanInterval, 0.05f, 2.0f);
        ImGui::SliderFloat("Max Path Cost", &m_maxPathCost, 500.0f, 15000.0f);

        ImGui::Separator();
        ImGui::Text("Scoring Weights");
        ImGui::DragFloat("Altitude Weight", &m_altitudeWeight, 0.1f, 0.0f, 20.0f);
        ImGui::DragFloat("Distance Weight", &m_distanceWeight, 0.1f, 0.0f, 20.0f);

        ImGui::Separator();
        ImGui::Text("Distances");
        ImGui::SliderFloat("Min Escape Dist", &m_minEscapeDist, 100.0f, 2000.0f);
        ImGui::SliderFloat("Safe Distance Sq", &m_safeDistSq, 10000.0f, 1000000.0f);

        ImGui::Separator();
        ImGui::Text("Path Constraints");
        ImGui::SliderInt("Max Path Nodes", &m_maxEvasionNodes, 1, 50);

        if (ImGui::Button("Force Path Reset", { -1, 30 })) {
            ResetState();
        }
        ImGui::End();
    }
}