#include "DStarLite.h"
#include <iostream>
#include <unordered_set>

#include "GraphBuilder.h"
#include "raymath.h"

DStarLite::DStarLite(NavigationGraph& graph) : m_graph(graph), m_startNodeIdx(-1), m_targetNodeIdx(-1), m_kM(0) {}

std::pair<float, float> DStarLite::CalculateKey(int s) {
    // הגנה: אם הצומת לא קיים בגרף
    if (s < 0 || s >= (int)m_graph.nodes().size()) return { 1e9f, 1e9f };

    float h = Vector3Distance(m_graph.nodes()[s].position, m_graph.nodes()[m_startNodeIdx].position);

    // שימוש ב-at() או בבדיקה כדי למנוע יצירת איברים מיותרים
    float g = m_nodes.count(s) ? m_nodes[s].g : 1e9f;
    float rhs = m_nodes.count(s) ? m_nodes[s].rhs : 1e9f;

    float min_g_rhs = std::min(g, rhs);
    return { min_g_rhs + h + m_kM, min_g_rhs };
}

void DStarLite::UpdateVertex(int u) {
    if (u < 0 || u >= (int)m_graph.nodes().size()) return;

    // אתחול צומת אם הוא חדש במפה
    if (m_nodes.count(u) == 0) {
        m_nodes[u] = { 1e9f, 1e9f };
    }

    if (u != m_targetNodeIdx) {
        float minRhs = 1e9f;
        for (auto& edge : m_graph.nodes()[u].neighbors) {
            float gVal = m_nodes.count(edge.target) ? m_nodes[edge.target].g : 1e9f;
            float val = gVal + edge.weight;
            if (val < minRhs) minRhs = val;
        }
        m_nodes[u].rhs = minRhs;
    }

    // ניהול ה-Priority Queue (מניעת כפילויות)
    if (m_nodes[u].g != m_nodes[u].rhs) {
        m_nodes[u].gen++;
        m_openSet.push({ CalculateKey(u), u, m_nodes[u].gen });
    }
}

void DStarLite::ComputeShortestPath() {
    int iterations = 0;
    while (!m_openSet.empty() &&
           (m_openSet.top().key < CalculateKey(m_startNodeIdx) ||
            m_nodes[m_startNodeIdx].rhs != m_nodes[m_startNodeIdx].g))
    {
        if (iterations++ > 5000) break;

        auto entry = m_openSet.top(); m_openSet.pop();
        int u = entry.node;

        // ← lazy deletion: skip stale entries
        if (entry.gen != m_nodes[u].gen) continue;

        auto k_old = entry.key;
        auto k_new = CalculateKey(u);

        if (k_old < k_new) {
            m_nodes[u].gen++;
            m_openSet.push({ k_new, u, m_nodes[u].gen });
        }
        else if (m_nodes[u].g > m_nodes[u].rhs) {
            m_nodes[u].g = m_nodes[u].rhs;
            for (auto& edge : m_graph.nodes()[u].neighbors)
                UpdateVertex(edge.target);
        }
        else {
            m_nodes[u].g = 1e9f;
            UpdateVertex(u);
            for (auto& edge : m_graph.nodes()[u].neighbors)
                UpdateVertex(edge.target);
        }
    }
}

std::vector<Vector3> DStarLite::PlanPath(Vector3 startPos, Vector3 targetPos) {

    int newStart = m_graph.GetClosestNode(startPos);
    int newTarget = m_graph.GetClosestNode(targetPos);

    if (newStart == -1 || newTarget == -1) return {};

    // 2. אתחול ראשוני (פעם אחת בלבד)
    if (m_startNodeIdx != -1) {
        float distToCurrent = Vector3Distance(startPos, m_graph.nodes()[m_startNodeIdx].position);
        float distToNext = Vector3Distance(startPos, m_graph.nodes()[newStart].position);

        // החלף צומת רק אם החדש קרוב יותר משמעותית (לפחות ב-5 יחידות) מהנוכחי
        if (distToNext < distToCurrent - 5.0f) {
            m_kM += Vector3Distance(m_graph.nodes()[m_startNodeIdx].position, m_graph.nodes()[newStart].position);
            m_startNodeIdx = newStart;
        }
    } else {
        m_startNodeIdx = newStart;
    }

    // 3. עדכון תנועת המטוס (מניעת רעידות)
    // אנחנו מעדכנים את נקודת ההתחלה רק אם המטוס באמת עבר צומת
    if (newStart != m_startNodeIdx) {
        float distToNewStart = Vector3Distance(
            startPos,
            m_graph.nodes()[newStart].position
        );
        float distToCurrentStart = Vector3Distance(
            startPos,
            m_graph.nodes()[m_startNodeIdx].position
        );

        // Only switch if we're meaningfully closer to the new node
        if (distToNewStart < distToCurrentStart - 2.0f) {
            m_kM += Vector3Distance(
                m_graph.nodes()[m_startNodeIdx].position,
                m_graph.nodes()[newStart].position
            );
            m_startNodeIdx = newStart;
        }
    }

    // 4. עדכון תנועת האויב (היעד)
    if (newTarget != m_targetNodeIdx) {
        // FIXED: reset the OLD target before switching
        int oldTarget = m_targetNodeIdx;
        m_nodes[oldTarget].rhs = 1e9f;
        m_nodes[oldTarget].g   = 1e9f;
        m_nodes[oldTarget].gen++;
        UpdateVertex(oldTarget);

        m_kM += Vector3Distance(
            m_graph.nodes()[m_targetNodeIdx].position,
            m_graph.nodes()[newTarget].position
        );

        m_targetNodeIdx = newTarget;
        m_nodes[m_targetNodeIdx].rhs = 0.0f;
        m_nodes[m_targetNodeIdx].g   = 1e9f;
        UpdateVertex(m_targetNodeIdx);
    }

    // 5. חישוב מסלול (הגדלנו איטרציות)
    ComputeShortestPath();

    // 6. שחזור מסלול חכם (מניעת לולאה על המקום)
    std::vector<Vector3> path;
    std::unordered_set<int> visited;
    int curr = m_startNodeIdx;

    for (int step = 0; step < 40; step++) {
        path.push_back(m_graph.nodes()[curr].position);
        if (curr == m_targetNodeIdx) break;
        visited.insert(curr);

        int nextNode = -1;
        float minCost = 1e10f;

        for (auto& edge : m_graph.nodes()[curr].neighbors) {
            if (visited.count(edge.target)) continue; // ← skip already visited
            float gVal = m_nodes.count(edge.target) ? m_nodes[edge.target].g : 1e9f;
            float edgeCost = edge.weight + gVal;
            if (edgeCost < minCost) {
                minCost = edgeCost;
                nextNode = edge.target;
            }
        }

        if (nextNode == -1) break;
        curr = nextNode;
    }

    return path;
}