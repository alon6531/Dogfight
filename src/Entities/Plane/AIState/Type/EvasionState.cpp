// EvasionState.cpp
#include "EvasionState.h"
#include "../../Plane.h"
#include "raymath.h"
#include <cfloat>

#include "imgui.h"


EvasionState::EvasionState(Plane& self, NavigationGraph& navGraph,
                           std::shared_ptr<Plane> enemy)
    : AIState(self, navGraph), m_enemy(std::move(enemy))
{
    m_dStarLite = std::make_unique<DStarLite>(p_graph);
}

EvasionState::~EvasionState() = default;

int EvasionState::PickEscapeNode(Vector3 selfPos, Vector3 enemyPos) const {
    const auto& nodes = p_graph.GetNodes();
    int   totalNodes  = (int)nodes.size();
    int   bestIdx     = -1;
    float bestScore   = -FLT_MAX;

    for (int attempt = 0; attempt < m_candidateSamples; ++attempt) {
        int idx = GetRandomValue(0, totalNodes - 1);
        if (nodes[idx].neighbors.empty()) continue;

        Vector3 nPos    = nodes[idx].position;
        float distEnemy = Vector3Distance(nPos, enemyPos);
        if (distEnemy < m_minEscapeDist) continue;

        float altGain = nPos.y - selfPos.y;
        float score   = altGain * m_altitudeWeight + distEnemy * m_distanceWeight;

        if (score > bestScore) {
            bestScore = score;
            bestIdx   = idx;
        }
    }

    if (bestIdx == -1)
        bestIdx = p_graph.GetRandomNodeFarFrom(enemyPos, m_minEscapeDist * 0.5f);

    return bestIdx;
}

AIStateType EvasionState::Update(float deltaTime) {
    Vector3 selfPos  = p_self.GetPosition();
    Vector3 enemyPos = m_enemy->GetPosition();

    float dx     = selfPos.x - enemyPos.x;
    float dy     = selfPos.y - enemyPos.y;
    float dz     = selfPos.z - enemyPos.z;
    float distSq = dx*dx + dy*dy + dz*dz;

    // ── 1. Transition back to PURSUIT ───────────────────────────────────────
    bool hasHeightAdvantage = dy > m_highGroundAdvantage;
    bool hasSafeDistance    = distSq > m_safeDistSq;

    if (hasHeightAdvantage || hasSafeDistance) {
        p_path.clear();
        m_escapeNodeIdx = -1;  // reset so next evasion picks fresh
        return AIStateType::PURSUIT;
    }

    // ── 2. Check if we reached the current escape node ──────────────────────
    bool needNewEscapeNode = (m_escapeNodeIdx == -1);

    if (m_escapeNodeIdx != -1) {
        Vector3 escapePos = p_graph.GetNodes()[m_escapeNodeIdx].position;
        float edx = selfPos.x - escapePos.x;
        float edy = selfPos.y - escapePos.y;
        float edz = selfPos.z - escapePos.z;
        if ((edx*edx + edy*edy + edz*edz) < m_escapeNodeReachedSq)
            needNewEscapeNode = true;  // arrived — pick a new destination
    }

    if (needNewEscapeNode) {
        m_escapeNodeIdx = PickEscapeNode(selfPos, enemyPos);
        p_path.clear();  // force immediate replan toward new node
        m_pathTimer = m_replanInterval;
    }

    // ── 3. Periodic path replan toward the SAME escape node ─────────────────
    // This lets DStarLite react to moving obstacles without changing destination
    m_pathTimer += deltaTime;
    if ((m_pathTimer >= m_replanInterval || p_path.empty()) && m_escapeNodeIdx != -1) {
        m_pathTimer = 0.0f;

        Vector3 escapePos = p_graph.GetNodes()[m_escapeNodeIdx].position;
        auto pathPoints   = m_dStarLite->PlanPath(selfPos, escapePos);
        float pathCost    = m_dStarLite->GetLastPathCost();

        bool pathValid = !pathPoints.empty() && pathCost < m_maxPathCost;

        if (pathValid) {
            p_path.clear();
            for (const auto& pt : pathPoints)
                p_path.push_back(pt);
            m_currentDir = escapePos;
        } else {
            // Path is blocked — force a new escape node next frame
            m_escapeNodeIdx = -1;
        }
    }

    // ── 4. Navigate ─────────────────────────────────────────────────────────
    if (!p_path.empty()) {
        Vector3 nextTarget = p_path.front();

        float tdx = nextTarget.x - selfPos.x;
        float tdy = nextTarget.y - selfPos.y;
        float tdz = nextTarget.z - selfPos.z;

        if ((tdx*tdx + tdy*tdy + tdz*tdz) < m_waypointRadiusSq) {
            p_path.pop_front();
            if (!p_path.empty())
                nextTarget = p_path.front();
        }

        p_self.SteerTowards(nextTarget, deltaTime);
        m_currentDir = nextTarget;
    } else {
        // Fallback: climb away from enemy
        Vector3 awayDir        = Vector3Normalize(Vector3Subtract(selfPos, enemyPos));
        Vector3 climbDir       = Vector3Normalize({ awayDir.x * 0.6f, 1.0f, awayDir.z * 0.6f });
        Vector3 emergencyTarget = Vector3Add(selfPos, Vector3Scale(climbDir, 600.0f));
        p_self.SteerTowards(emergencyTarget, deltaTime);
        m_currentDir = emergencyTarget;
    }

    return AIStateType::EVASION;
}

Vector3 EvasionState::GetCurrentTargetFromAI() {
    return m_currentDir;
}

void EvasionState::DrawDebugUI() {
    float padding = 20.0f;
    float windowWidth = 300.0f;



    // שימוש ב-Always יכריח את החלון להיות בצד ימין כל עוד המטוס ב-Evasion
    // המיקום מחושב לפי רוחב המסך פחות הרוחב שתכננו לחלון
    ImGui::SetNextWindowPos(
        {(float)GetScreenWidth() - windowWidth - padding, padding},
        ImGuiCond_Always
    );

    ImGui::SetNextWindowSize({windowWidth, 550}, ImGuiCond_Always);

    // הוספת דגל שימנע מהחלון לזוז בטעות אם תגרור אותו
    ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;

    if (ImGui::Begin("AI Evasion Tuning", nullptr, flags)) {
        ImGui::TextColored({0, 255, 0, 255}, "STATE: EVASION");
        ImGui::Separator();

        ImGui::Text("Pathfinding");
        ImGui::SliderFloat("Replan Interval", &m_replanInterval, 0.05f, 2.0f);
        ImGui::SliderFloat("Max Path Cost", &m_maxPathCost, 500.0f, 15000.0f);

        ImGui::Separator();
        ImGui::Text("Scoring Weights");
        // כאן אתה משנה את המשקל של הגובה - זה מה שימנע ממנו להתרסק
        ImGui::DragFloat("Altitude Weight", &m_altitudeWeight, 0.1f, 0.0f, 20.0f);
        ImGui::DragFloat("Distance Weight", &m_distanceWeight, 0.1f, 0.0f, 20.0f);

        ImGui::Separator();
        ImGui::Text("Distances");
        ImGui::SliderFloat("Min Escape Dist", &m_minEscapeDist, 100.0f, 2000.0f);
        ImGui::SliderFloat("Safe Distance Sq", &m_safeDistSq, 10000.0f, 1000000.0f);

        if (ImGui::Button("Force Path Reset", { -1, 30 })) {
            p_path.clear();
            m_escapeNodeIdx = -1;
        }

        ImGui::End();
    }
}