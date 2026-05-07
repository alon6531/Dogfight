#include "DStarLite.h"
#include <algorithm>
#include "raymath.h"

const float INF = 1e9f;

DStarLite::DStarLite(NavigationGraph &graph)
    : m_graph(graph), m_startNodeIdx(-1), m_targetNodeIdx(-1), m_kM(0.0f) {
    m_nodeData.resize(m_graph.GetNodes().size(), {INF, INF, 0});
}

// Calculate priority keys [f, g]
std::pair<float, float> DStarLite::CalculateKey(int s) {
    float min_g_rhs = std::min(m_nodeData[s].g, m_nodeData[s].rhs);
    float h = Vector3Distance(m_graph.GetNodes()[s].position, m_graph.GetNodes()[m_startNodeIdx].position);
    return {min_g_rhs + h + m_kM, min_g_rhs};
}

// Update node consistency and OpenSet status
void DStarLite::UpdateVertex(int u) {
    if (u != m_targetNodeIdx) {
        m_nodeData[u].rhs = INF;
        for (const auto &edge : m_graph.GetNodes()[u].neighbors) {
            if (m_nodeData[edge.target].g != INF) {
                m_nodeData[u].rhs = std::min(m_nodeData[u].rhs, m_nodeData[edge.target].g + edge.weight);
            }
        }
    }

    if (m_nodeData[u].g != m_nodeData[u].rhs) {
        m_nodeData[u].gen++;
        m_openSet.push({CalculateKey(u), u, m_nodeData[u].gen});
    }
}

// Expand nodes until the optimal path to start is found
void DStarLite::ComputeShortestPath() {
    while (!m_openSet.empty() &&
           (m_openSet.top().key < CalculateKey(m_startNodeIdx) ||
            m_nodeData[m_startNodeIdx].rhs != m_nodeData[m_startNodeIdx].g))
    {
        auto top = m_openSet.top();
        m_openSet.pop();

        if (top.gen == m_nodeData[top.node].gen) {
            int u = top.node;
            auto k_new = CalculateKey(u);

            if (top.key < k_new) {
                m_nodeData[u].gen++;
                m_openSet.push({k_new, u, m_nodeData[u].gen});
            }
            else if (m_nodeData[u].g > m_nodeData[u].rhs) {
                m_nodeData[u].g = m_nodeData[u].rhs; // Over consistent
                for (const auto &edge : m_graph.GetNodes()[u].neighbors) UpdateVertex(edge.target);
            }
            else {
                m_nodeData[u].g = INF; // Under consistent
                UpdateVertex(u);
                for (const auto &edge : m_graph.GetNodes()[u].neighbors) UpdateVertex(edge.target);
            }
        }
    }
}

std::vector<Vector3> DStarLite::PlanPath(Vector3 startPos, Vector3 targetPos) {
    int newStart = m_graph.GetClosestNode(startPos);
    int newTarget = m_graph.GetClosestNode(targetPos);


    // Handle agent movement
    if (m_startNodeIdx != -1 && newStart != m_startNodeIdx) {
        m_kM += Vector3Distance(m_graph.GetNodes()[m_startNodeIdx].position, m_graph.GetNodes()[newStart].position);
    }
    m_startNodeIdx = newStart;

    // Handle goal movement
    if (newTarget != m_targetNodeIdx) {
        m_targetNodeIdx = newTarget;
        m_kM = 0;
        std::fill(m_nodeData.begin(), m_nodeData.end(), DStarNode{INF, INF, 0});
        m_nodeData[m_targetNodeIdx].rhs = 0.0f;
        UpdateVertex(m_targetNodeIdx);
    }

    this->ComputeShortestPath();

    // Reconstruct path from current position to goal
    std::vector<Vector3> path;
    int curr = m_startNodeIdx;
    m_lastPathWeight = 0;

    while (curr != m_targetNodeIdx && curr != -1 && path.size() < m_nodeData.size()) {
        path.push_back(m_graph.GetNodes()[curr].position);
        int bestNext = -1;
        float minCost = INF;

        for (const auto &edge : m_graph.GetNodes()[curr].neighbors) {
            float cost = edge.weight + m_nodeData[edge.target].g;
            if (cost < minCost) {
                minCost = cost;
                bestNext = edge.target;
            }
        }

        if (bestNext != -1) {
            for (const auto &edge : m_graph.GetNodes()[curr].neighbors) {
                if (edge.target == bestNext) { m_lastPathWeight += edge.weight; break; }
            }
        }
        curr = (minCost == INF) ? -1 : bestNext;
    }

    if (curr == m_targetNodeIdx) path.push_back(m_graph.GetNodes()[curr].position);
    return path;
}