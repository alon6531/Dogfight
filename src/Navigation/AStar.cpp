// src/Navigation/Pathfinding/AStar.cpp
#include "AStar.h"


AStar::AStar(int maxNodesHint) {
    m_searchData.resize(maxNodesHint);
}

void AStar::Reset(int nodesCount) {

    while(!m_openSet.empty()) m_openSet.pop();


    if (m_searchData.size() < (size_t)nodesCount) {
        m_searchData.resize(nodesCount);
    }


    for (int i = 0; i < nodesCount; ++i) {
        m_searchData[i].gScore = INFINITY;
        m_searchData[i].parentIdx = -1;
    }
}

std::vector<Vector3> AStar::FindPath(NavigationGraph& graph, int startIdx, int targetIdx) {
    const auto& nodes = graph.GetNodes();
    if (startIdx < 0 || targetIdx < 0 || startIdx >= nodes.size() || targetIdx >= nodes.size()) return {};
    if (startIdx == targetIdx) return { nodes[startIdx].position };

    Reset(nodes.size());

    m_searchData[startIdx].gScore = 0;
    float hStart = graph.GetHeuristic(startIdx, targetIdx);
    m_openSet.push({ hStart, startIdx });

    while (!m_openSet.empty()) {
        int current = m_openSet.top().nodeIdx;
        float currentF = m_openSet.top().fScore;
        m_openSet.pop();


        if (current == targetIdx) {
            std::vector<Vector3> path;
            int temp = current;
            while (temp != -1) {
                path.push_back(nodes[temp].position);
                temp = m_searchData[temp].parentIdx;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }


        if (currentF > m_searchData[current].gScore + graph.GetHeuristic(current, targetIdx)) continue;

        for (auto& edge : nodes[current].neighbors) {
            float tentative_gScore = m_searchData[current].gScore + edge.weight;

            if (tentative_gScore < m_searchData[edge.target].gScore) {
                m_searchData[edge.target].parentIdx = current;
                m_searchData[edge.target].gScore = tentative_gScore;

                float fScore = tentative_gScore + graph.GetHeuristic(edge.target, targetIdx);
                m_openSet.push({ fScore, edge.target });
            }
        }
    }

    return {};
}