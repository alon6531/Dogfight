//
// Created by User on 04/04/2026.
//

#include "Global.h"
#include <queue>
#include <limits>
#include "Sims/GraphBuilder.h"
bool gCameraMode = false;
bool gIsVictory = false;

void ComputeDijkstra(NavigationGraph& graph, int startNodeIdx, std::vector<float> &outDistances) {

    if (outDistances.size() != graph.nodes().size()) {
        outDistances.resize(graph.nodes().size());
    }


    std::fill(outDistances.begin(), outDistances.end(), std::numeric_limits<float>::max());


    if (startNodeIdx < 0 || startNodeIdx >= (int)graph.nodes().size()) return;

    outDistances[startNodeIdx] = 0.0f;


    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<std::pair<float, int>>> pq;
    pq.push({0.0f, startNodeIdx});

    while (!pq.empty()) {
        float d = pq.top().first;
        int u = pq.top().second;
        pq.pop();


        if (d > outDistances[u]) continue;


        for (const auto& edge : graph.nodes()[u].neighbors) {
            int v = edge.target;
            float weight = edge.weight;


            if (outDistances[u] + weight < outDistances[v]) {
                outDistances[v] = outDistances[u] + weight;
                pq.push({outDistances[v], v});
            }
        }
    }

    // 4. Mark unreachable nodes as -1.0f for ATL heuristic compatibility
    for (float& d : outDistances) {
        if (d == std::numeric_limits<float>::max()) {
            d = -1.0f;
        }
    }
}
