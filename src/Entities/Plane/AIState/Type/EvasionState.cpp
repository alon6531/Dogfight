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

    const auto& nodes = p_graph.GetNodes();
    int totalNodes = (int)nodes.size();
    int bestIdx    = -1;
    float bestScore = -FLT_MAX;
    Vector3 forward = p_self.GetForward();

    for (int attempt = 0; attempt < m_candidateSamples; ++attempt) {
        int idx = GetRandomValue(0, totalNodes - 1);

        Vector3 nPos      = nodes[idx].position;
        float distEnemy   = Vector3Distance(nPos, enemyPos);

        Vector3 dirToNode    = Vector3Normalize(Vector3Subtract(nPos, selfPos));
        float dot            = Vector3DotProduct(forward, dirToNode);
        float alignmentScore = (1.0f - fabsf(dot - ESCAPE_ALIGNMENT_TARGET_DOT)) * ESCAPE_ALIGNMENT_SCALE;

        bool outX    = fabsf(nPos.x) > (MAP_LIMIT_X - ESCAPE_BOUNDARY_MARGIN_XZ);
        bool outZ    = fabsf(nPos.z) > (MAP_LIMIT_Z - ESCAPE_BOUNDARY_MARGIN_XZ);
        bool outY    = nPos.y > (MAP_CEILING - ESCAPE_BOUNDARY_MARGIN_Y_TOP) || nPos.y < ESCAPE_BOUNDARY_MIN_Y;
        float penalty = (outX || outZ || outY) * ESCAPE_BOUNDARY_PENALTY;

        float distScore = sqrtf(distEnemy) * ESCAPE_DIST_SCORE_SCALE;
        float chaos     = (float)GetRandomValue(0, ESCAPE_CHAOS_MAX);

        float score = alignmentScore + distScore + chaos - penalty;

        if (score > bestScore) {
            bestScore = score;
            bestIdx   = idx;
        }
    }

    return (bestIdx != -1) ? bestIdx : GetRandomValue(0, totalNodes - 1);
}

AIStateType EvasionState::Update(float deltaTime) {
    m_stateTime += deltaTime;
    Vector3 selfPos  = p_self.GetPosition();
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

void EvasionState::UpdateEscapeTarget(Vector3 selfPos, Vector3 enemyPos) {
    bool hasNoTarget   = (m_escapeNodeIdx == -1);
    bool targetReached = !hasNoTarget &&
                         (Vector3DistanceSqr(selfPos, p_graph.GetNodes()[m_escapeNodeIdx].position) < m_escapeNodeReachedSq);

    if (hasNoTarget || targetReached) {
        m_escapeNodeIdx = PickEscapeNode(selfPos, enemyPos);
        p_path.clear();
        m_pathTimer = m_replanInterval;
    }
}

void EvasionState::NavigatePath(Vector3 selfPos, float deltaTime, Vector3 enemyPos) {
    Vector3 nextTarget = p_path.front();

    if (Vector3DistanceSqr(selfPos, nextTarget) < m_waypointRadiusSq) {
        p_path.pop_front();
        nextTarget = p_path.empty() ? nextTarget : p_path.front();
    }

    p_self.SteerTowards(nextTarget, deltaTime);
    m_currentDir = nextTarget;
}

bool EvasionState::ShouldReturnToPursuit(Vector3 selfPos, Vector3 enemyPos) const {
    bool hasHeightAdvantage = (selfPos.y - enemyPos.y) > (m_highGroundAdvantage + PURSUIT_HEIGHT_ADVANTAGE_BONUS);
    bool hasSafeDistance = Vector3DistanceSqr(selfPos, enemyPos) > (m_enemy->Get_lockDistanceSq() * PURSUIT_SAFE_DIST_SQ_MULT);

    return (m_stateTime > PURSUIT_MIN_STATE_TIME && (hasHeightAdvantage || hasSafeDistance));
}

void EvasionState::ResetState() {
    p_path.clear();
    m_escapeNodeIdx = -1;
}

void EvasionState::ReplanPathIfNeeded(Vector3 selfPos) {
    m_pathTimer += GetFrameTime();
    if ((m_pathTimer >= m_replanInterval || p_path.empty()) && m_escapeNodeIdx != -1) {
        m_pathTimer = 0.0f;
        Vector3 escapePos = p_graph.GetNodes()[m_escapeNodeIdx].position;
        auto pathPoints   = m_dStarLite->PlanPath(selfPos, escapePos);
        float pathCost    = m_dStarLite->GetLastPathCost();

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

Vector3 EvasionState::GetCurrentTargetFromAI() {
    return m_currentDir;
}

void EvasionState::DrawDebugUI() {
    ImGui::SetNextWindowPos( { (float)GetScreenWidth() - DEBUG_WIN_WIDTH - DEBUG_WIN_PADDING, DEBUG_WIN_PADDING }, ImGuiCond_Always);
    ImGui::SetNextWindowSize({ DEBUG_WIN_WIDTH, DEBUG_WIN_HEIGHT }, ImGuiCond_Always);

    if (ImGui::Begin("AI Evasion Tuning", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)) {
        ImGui::TextColored({ DEBUG_STATE_COLOR_R, DEBUG_STATE_COLOR_G, DEBUG_STATE_COLOR_B, DEBUG_STATE_COLOR_A }, "STATE: EVASION");
        ImGui::Separator();

        ImGui::Text("Pathfinding");
        ImGui::SliderFloat("Replan Interval", &m_replanInterval, DEBUG_REPLAN_MIN,    DEBUG_REPLAN_MAX);
        ImGui::SliderFloat("Max Path Cost",   &m_maxPathCost,    DEBUG_PATH_COST_MIN, DEBUG_PATH_COST_MAX);

        ImGui::Separator();
        ImGui::Text("Scoring Weights");
        ImGui::DragFloat("Altitude Weight", &m_altitudeWeight, DEBUG_WEIGHT_STEP, DEBUG_WEIGHT_MIN, DEBUG_WEIGHT_MAX);
        ImGui::DragFloat("Distance Weight", &m_distanceWeight, DEBUG_WEIGHT_STEP, DEBUG_WEIGHT_MIN, DEBUG_WEIGHT_MAX);

        ImGui::Separator();
        ImGui::Text("Distances");
        ImGui::SliderFloat("Min Escape Dist",  &m_minEscapeDist, DEBUG_ESCAPE_DIST_MIN,  DEBUG_ESCAPE_DIST_MAX);
        ImGui::SliderFloat("Safe Distance Sq", &m_enemy->Get_lockDistanceSq(),    DEBUG_SAFE_DIST_SQ_MIN, DEBUG_SAFE_DIST_SQ_MAX);

        ImGui::Separator();
        ImGui::Text("Path Constraints");
        ImGui::SliderInt("Max Path Nodes", &m_maxEvasionNodes, DEBUG_PATH_NODES_MIN, DEBUG_PATH_NODES_MAX);

        if (ImGui::Button("Force Path Reset", { -1, DEBUG_RESET_BTN_HEIGHT })) {
            ResetState();
        }
        ImGui::End();
    }
}