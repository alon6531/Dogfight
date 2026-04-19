#include "DStarLite.h"
#include <limits>
#include <algorithm>

#include "raymath.h"

// שימוש בקבועים ברורים למניעת Overflow
const float INF = std::numeric_limits<float>::infinity();

DStarLite::DStarLite(NavigationGraph& graph)
    : m_graph(graph), m_startNodeIdx(-1), m_targetNodeIdx(-1), m_kM(0.0f) {
    // אתחול וקטור הצמתים מראש לפי גודל הגרף לביצועים מקסימליים
    m_nodeData.resize(m_graph.GetNodes().size(), {INF, INF, 0});
}

std::pair<float, float> DStarLite::CalculateKey(int s) {
    if (s < 0 || s >= (int)m_nodeData.size()) return {INF, INF};

    float g = m_nodeData[s].g;
    float rhs = m_nodeData[s].rhs;
    float min_g_rhs = std::min(g, rhs);

    // Heuristic: מרחק אווירי למיקום הנוכחי של הרובוט
    float h = Vector3Distance(m_graph.GetNodes()[s].position, m_graph.GetNodes()[m_startNodeIdx].position);

    return { min_g_rhs + h + m_kM, min_g_rhs };
}

void DStarLite::UpdateVertex(int u) {
    if (u < 0 || u >= (int)m_nodeData.size()) return;

    // עדכון RHS (למעט המטרה שהיא תמיד 0)
    if (u != m_targetNodeIdx) {
        m_nodeData[u].rhs = INF;
        for (const auto& edge : m_graph.GetNodes()[u].neighbors) {
            float targetG = m_nodeData[edge.target].g;
            if (targetG != INF) {
                m_nodeData[u].rhs = std::min(m_nodeData[u].rhs, targetG + edge.weight);
            }
        }
    }

    // אם הצומת לא עקבי, נדחוף לתור העדכונים
    if (m_nodeData[u].g != m_nodeData[u].rhs) {
        m_nodeData[u].gen++; // מנגנון גרסאות למניעת כפילויות בתור
        m_openSet.push({ CalculateKey(u), u, m_nodeData[u].gen });
    }
}

void DStarLite::ComputeShortestPath() {
    int safetyCounter = 0;

    while (!m_openSet.empty() && safetyCounter++ < 1000) {
        auto top = m_openSet.top();
        auto startKey = CalculateKey(m_startNodeIdx);

        // תנאי העצירה הקלאסי של D* Lite
        if (!(top.key < startKey || m_nodeData[m_startNodeIdx].rhs != m_nodeData[m_startNodeIdx].g)) {
            break;
        }

        m_openSet.pop();
        if (top.gen != m_nodeData[top.node].gen) continue; // דילוג על איברים ישנים

        int u = top.node;
        auto k_new = CalculateKey(u);

        if (top.key < k_new) {
            m_nodeData[u].gen++;
            m_openSet.push({ k_new, u, m_nodeData[u].gen });
        }
        else if (m_nodeData[u].g > m_nodeData[u].rhs) {
            // צומת Overconsistent
            m_nodeData[u].g = m_nodeData[u].rhs;
            for (const auto& edge : m_graph.GetNodes()[u].neighbors) {
                UpdateVertex(edge.target);
            }
        }
        else {
            // צומת Underconsistent
            m_nodeData[u].g = INF;
            UpdateVertex(u);
            for (const auto& edge : m_graph.GetNodes()[u].neighbors) {
                UpdateVertex(edge.target);
            }
        }
    }
}

std::vector<Vector3> DStarLite::PlanPath(Vector3 startPos, Vector3 targetPos) {
    int newStart = m_graph.GetClosestNode(startPos);
    int newTarget = m_graph.GetClosestNode(targetPos);

    if (newStart == -1 || newTarget == -1) return {};

    // טיפול בתנועת הרובוט (עדכון kM)
    if (m_startNodeIdx != -1 && newStart != m_startNodeIdx) {
        m_kM += Vector3Distance(m_graph.GetNodes()[m_startNodeIdx].position, m_graph.GetNodes()[newStart].position);
    }
    m_startNodeIdx = newStart;

    // אם המטרה השתנתה - איפוס הכרחי
    if (newTarget != m_targetNodeIdx) {
        m_targetNodeIdx = newTarget;
        // איפוס נתונים רק כשמטרה משתנה (אופציונלי, תלוי אם המפה משתנה)
        std::fill(m_nodeData.begin(), m_nodeData.end(), DStarNode{INF, INF, 0});
        m_nodeData[m_targetNodeIdx].rhs = 0.0f;
        UpdateVertex(m_targetNodeIdx);
    }

    ComputeShortestPath();

    // בניית המסלול מההתחלה למטרה
    std::vector<Vector3> path;
    int curr = m_startNodeIdx;
    int stepLimit = 100;

    while (curr != -1 && curr != m_targetNodeIdx && stepLimit-- > 0) {
        path.push_back(m_graph.GetNodes()[curr].position);

        int bestNext = -1;
        float minCost = INF;

        for (const auto& edge : m_graph.GetNodes()[curr].neighbors) {
            float cost = edge.weight + m_nodeData[edge.target].g;
            if (cost < minCost) {
                minCost = cost;
                bestNext = edge.target;
            }
        }
        curr = bestNext;
    }

    if (curr == m_targetNodeIdx) path.push_back(m_graph.GetNodes()[curr].position);
    return path;
}