#include "DStarLite.h"
#include <iostream>
#include <unordered_set>


#include "raymath.h"

DStarLite::DStarLite(NavigationGraph& graph) : m_graph(graph), m_startNodeIdx(-1), m_targetNodeIdx(-1), m_kM(0) {}

std::pair<float, float> DStarLite::CalculateKey(int s) {

    if (s < 0 || s >= (int)m_graph.GetNodes().size()) return { 1e9f, 1e9f };

    float h = Vector3Distance(m_graph.GetNodes()[s].position, m_graph.GetNodes()[m_startNodeIdx].position);

    float g = m_nodes.count(s) ? m_nodes[s].g : 1e9f;
    float rhs = m_nodes.count(s) ? m_nodes[s].rhs : 1e9f;

    float min_g_rhs = std::min(g, rhs);
    return { min_g_rhs + h + m_kM, min_g_rhs };
}

void DStarLite::UpdateVertex(int u) {
    if (u < 0 || u >= (int)m_graph.GetNodes().size()) return;


    if (m_nodes.count(u) == 0) {
        m_nodes[u] = { 1e9f, 1e9f };
    }

    if (u != m_targetNodeIdx) {
        float minRhs = 1e9f;
        for (auto& edge : m_graph.GetNodes()[u].neighbors) {
            float gVal = m_nodes.count(edge.target) ? m_nodes[edge.target].g : 1e9f;
            float val = gVal + edge.weight;
            if (val < minRhs) minRhs = val;
        }
        m_nodes[u].rhs = minRhs;
    }


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
        if (iterations++ > 500) break;

        auto entry = m_openSet.top(); m_openSet.pop();
        int u = entry.node;


        if (entry.gen != m_nodes[u].gen) continue;

        auto k_old = entry.key;
        auto k_new = CalculateKey(u);

        if (k_old < k_new) {
            m_nodes[u].gen++;
            m_openSet.push({ k_new, u, m_nodes[u].gen });
        }
        else if (m_nodes[u].g > m_nodes[u].rhs) {
            m_nodes[u].g = m_nodes[u].rhs;
            for (auto& edge : m_graph.GetNodes()[u].neighbors)
                UpdateVertex(edge.target);
        }
        else {
            m_nodes[u].g = 1e9f;
            UpdateVertex(u);
            for (auto& edge : m_graph.GetNodes()[u].neighbors)
                UpdateVertex(edge.target);
        }
    }
}

std::vector<Vector3> DStarLite::PlanPath(Vector3 startPos, Vector3 targetPos) {
    int newStart = m_graph.GetClosestNode(startPos);
    int newTarget = m_graph.GetClosestNode(targetPos);


    if (newStart == -1 || newTarget == -1) {

        return { startPos, targetPos };
    }

    if (m_targetNodeIdx == -1) {
        m_nodes.clear();
        while(!m_openSet.empty()) m_openSet.pop();
        m_kM = 0;
        m_targetNodeIdx = newTarget;
        m_startNodeIdx = newStart;

        m_nodes[m_targetNodeIdx].rhs = 0;
        m_nodes[m_targetNodeIdx].g = 1e9f;
        UpdateVertex(m_targetNodeIdx);
        ComputeShortestPath();
    }


    if (newStart != m_startNodeIdx) {

        float dist = Vector3Distance(m_graph.GetNodes()[m_startNodeIdx].position, m_graph.GetNodes()[newStart].position);
        if (!std::isnan(dist)) {
            m_kM += dist;
            m_startNodeIdx = newStart;
        }
    }

    if (newTarget != m_targetNodeIdx) {
        int oldTarget = m_targetNodeIdx;
        if (m_nodes.count(oldTarget)) {
            m_nodes[oldTarget].rhs = 1e9f;
            UpdateVertex(oldTarget);
        }

        m_targetNodeIdx = newTarget;
        if (m_nodes.count(m_targetNodeIdx) == 0) m_nodes[m_targetNodeIdx] = { 1e9f, 1e9f };
        m_nodes[m_targetNodeIdx].rhs = 0.0f;
        UpdateVertex(m_targetNodeIdx);

        float tDist = Vector3Distance(m_graph.GetNodes()[oldTarget].position, m_graph.GetNodes()[newTarget].position);
        if (!std::isnan(tDist)) m_kM += tDist;
    }


    ComputeShortestPath();


    std::vector<Vector3> path;
    int curr = m_startNodeIdx;


    if (m_nodes.count(curr) == 0 || m_nodes[curr].g >= 1e6f) {
        AStar shortestPath;
        auto fallbackPath = shortestPath.FindPath(m_graph, m_startNodeIdx, m_targetNodeIdx);
        if (!fallbackPath.empty()) return fallbackPath;
    }


    std::unordered_set<int> visited;
    for (int step = 0; step < 80; step++) {
        if (curr < 0 || curr >= (int)m_graph.GetNodes().size()) break;

        path.push_back(m_graph.GetNodes()[curr].position);
        if (curr == m_targetNodeIdx) break;
        visited.insert(curr);

        int nextNode = -1;
        float minCost = 1e9f;

        for (auto& edge : m_graph.GetNodes()[curr].neighbors) {
            if (visited.count(edge.target)) continue;

            float gVal = m_nodes.count(edge.target) ? m_nodes[edge.target].g : 1e9f;
            float edgeCost = edge.weight + gVal;

            if (edgeCost < minCost) {
                minCost = edgeCost;
                nextNode = edge.target;
            }
        }

        if (nextNode == -1 || nextNode == curr) break;
        curr = nextNode;
    }


    if (path.size() < 2) {
        path.clear();
        path.push_back(startPos);
        path.push_back(targetPos);
    }

    return path;
}