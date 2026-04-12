//
// Created by User on 03/04/2026.
//
#include "../World/GraphBuilder.h"
#include <cfloat>
#include <iostream>
#include "rlgl.h"
#include "raymath.h"
#include <omp.h>
#include <queue>
#include "../World/Map.h"
#include "../Navigation/Dijkstra.h"

int NavigationGraph::IsPointBlocked(Vector3 p, const std::vector<Obstacle> &obstacles) {
    for (const auto& obs : obstacles) {
        float distSq = Vector3DistanceSqr(p, obs.pos);
        if (distSq < (obs.radius * obs.radius)) {
            return 1;
        }
    }
    return 0;
}

bool NavigationGraph::IsPathBlocked(Vector3 start, Vector3 end, const std::vector<Obstacle>& obstacles, const Map* gameMap) {
    float distance = Vector3Distance(start, end);
    int samples = std::max(10, (int)(distance / 1.0f));

    for (int i = 1; i < samples; i++) {
        float t = (float)i / samples;
        Vector3 checkPoint = Vector3Lerp(start, end, t);

        if (IsPointBlocked(checkPoint, obstacles)) return true;
        if (gameMap != nullptr && gameMap->IsBelowGround(checkPoint)) return true;
    }
    return false;
}

void NavigationGraph::BuildNodes(Vector3 arenaSize, float spacing, const Map& gameMap, int& idCounter, std::unordered_map<std::string, int>& posToId) {
    m_nodes.clear();

    auto getPosKey = [](Vector3 p) -> std::string {
        return std::to_string((int)round(p.x)) + "," +
               std::to_string((int)round(p.y)) + "," +
               std::to_string((int)round(p.z));
    };


    float startX = -arenaSize.x / 2.0f;
    float endX = arenaSize.x / 2.0f;
    float startZ = -arenaSize.z / 2.0f;
    float endZ = arenaSize.z / 2.0f;


    float startY = gameMap.GetPosition().y;
    float endY = startY + arenaSize.y;

    for (float x = startX; x <= endX + 0.1f; x += spacing) {
        for (float z = startZ; z <= endZ + 0.1f; z += spacing) {
            for (float y = startY; y <= endY + 0.1f; y += spacing) {

                Vector3 currentPos = { x, y, z };

                if (gameMap.IsBelowGround(currentPos)) continue;

                Node n = { idCounter++, currentPos };
                m_nodes.push_back(n);
                posToId[getPosKey(currentPos)] = n.id;
            }
        }
    }
}

void NavigationGraph::BuildEdges(Vector3 arenaSize, float spacing, const Map& gameMap, const std::vector<Obstacle> &obstacles, std::unordered_map<std::string, int>& posToId) {
    auto getPosKey = [](Vector3 p) -> std::string {
        return std::to_string((int)round(p.x)) + "," +
               std::to_string((int)round(p.y)) + "," +
               std::to_string((int)round(p.z));
    };

#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)m_nodes.size(); i++) {
        Vector3 p = m_nodes[i].position;


        for (float dx = -spacing; dx <= spacing; dx += spacing) {
            for (float dy = -spacing; dy <= spacing; dy += spacing) {
                for (float dz = -spacing; dz <= spacing; dz += spacing) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;

                    Vector3 nPos = { p.x + dx, p.y + dy, p.z + dz };
                    std::string key = getPosKey(nPos);

                    if (posToId.count(key)) {
                        float weight = Vector3Distance(p, nPos);

                        if (IsPathBlocked(p, nPos, {}, &gameMap)) continue;

                        if (IsPathBlocked(p, nPos, obstacles, &gameMap)) {
                            weight *= 100.0f;
                        }

                        m_nodes[i].neighbors.push_back({posToId[key], weight});
                    }
                }
            }
        }
    }
}

void NavigationGraph::BuildGraphFromMap(Vector3 arenaSize, float spacing, const std::vector<Obstacle> &obstacles, const Map& gameMap) {

    m_nodes.clear();
    int idCounter = 0;
    std::unordered_map<std::string, int> posToId;

    BuildNodes(arenaSize, spacing, gameMap, idCounter, posToId);

    BuildEdges(arenaSize, spacing, gameMap, obstacles, posToId);


}



void NavigationGraph::BuildDistanceMatrix() {
    std::vector<int> landmarkIndices;


    landmarkIndices.push_back(0);
    landmarkIndices.push_back(m_nodes.size() - 1);
    landmarkIndices.push_back(m_nodes.size() / 2);

    distanceMatrix.assign(landmarkIndices.size(), std::vector<float>(m_nodes.size(), -1.0f));


    Dijkstra solver;
    for (size_t i = 0; i < landmarkIndices.size(); ++i) {
        solver.Compute(*this, landmarkIndices[i], distanceMatrix[i]);
    }
}

float NavigationGraph::GetHeuristic(int nIdx, int targetIdx) const {
    float maxH = 0.0f;

    // Apply Triangle Inequality: h(n) = max(|dist(n, L) - dist(target, L)|)
    for (size_t i = 0; i < distanceMatrix.size(); ++i) {
        float d_n_L = distanceMatrix[i][nIdx];
        float d_t_L = distanceMatrix[i][targetIdx];

        if (d_n_L != -1.0f && d_t_L != -1.0f) {
            maxH = std::max(maxH, std::abs(d_n_L - d_t_L));
        }
    }

    // Return the tightest admissible heuristic (MAX of ALT and Euclidean)
    float euclidean = Vector3Distance(m_nodes[nIdx].position, m_nodes[targetIdx].position);
    return std::max(maxH, euclidean);
}

// Finds the index of the node closest to a given 3D position
int NavigationGraph::GetClosestNode(Vector3 position) {
    if (m_nodes.empty()) return -1;
    if (isnan(position.x) || isnan(position.y) || isnan(position.z)) return 0;

    int closestIdx = -1;
    float minDist = FLT_MAX;

    for (int i = 0; i < (int)m_nodes.size(); i++) {
        // דלג על צמתים מנותקים
        if (m_nodes[i].neighbors.empty()) continue;

        Vector3 np = m_nodes[i].position;
        float dx = np.x - position.x;
        float dy = np.y - position.y;
        float dz = np.z - position.z;
        float totalDistSq = (dx*dx + dz*dz) + (dy*dy * 15.0f);

        if (totalDistSq < minDist) {
            minDist = totalDistSq;
            closestIdx = i;
        }
    }
    return closestIdx != -1 ? closestIdx : 0;
}

int NavigationGraph::GetRandomNodeFarFrom(Vector3 position, float minDistance) const {
    const auto& nodes = GetNodes();
    if (nodes.empty()) return -1;

    int totalNodes = (int)nodes.size();

    // ננסה לבחור צומת רנדומלי עד 100 פעמים כדי למצוא אחד רחוק מספיק
    // זה הרבה יותר מהיר מלעבור על כל 10,000 הצמתים בכל פעם
    for (int i = 0; i < 100; i++) {
        int randomIndex = GetRandomValue(0, totalNodes - 1);
        float dist = Vector3Distance(nodes[randomIndex].position, position);

        if (dist >= minDistance) {
            return randomIndex;
        }
    }

    // אם אחרי 100 ניסיונות לא מצאנו צומת רחוק (אולי minDistance גדול מדי?)
    // נחזור על כל הצמתים ונחפש את הרחוק ביותר שמצאנו עד כה
    int bestIndex = -1;
    float maxDistFound = -1.0f;

    for (int i = 0; i < 200; i++) { // בדיקה מדגמית נוספת של 200 צמתים
        int idx = GetRandomValue(0, totalNodes - 1);
        float dist = Vector3Distance(nodes[idx].position, position);
        if (dist > maxDistFound) {
            maxDistFound = dist;
            bestIndex = idx;
        }
    }

    return bestIndex;
}


void NavigationGraph::PrepareGPUData() {
    // Clear previous GPU models to free VRAM
    for (auto &model: m_nodeModels) UnloadModel(model);
    m_nodeModels.clear();

    if (m_nodes.empty()) return;

    // Base geometry for a single node
    Mesh sphere = GenMeshSphere(0.2f, 8, 8);
    bool hasIndices = (sphere.indices != nullptr);
    int indexCount = (sphere.triangleCount * 3);

    // BATCHING: Grouping multiple nodes into a single mesh to reduce Draw Calls
    const int nodesPerMesh = 200;

    for (int startIdx = 0; startIdx < (int) m_nodes.size(); startIdx += nodesPerMesh) {
        int endIdx = std::min(startIdx + nodesPerMesh, (int)m_nodes.size());
        int currentChunkSize = endIdx - startIdx;

        // Allocate memory for the merged mesh (Vertex Buffer)
        Mesh chunkMesh = { 0 };
        chunkMesh.vertexCount = sphere.vertexCount * currentChunkSize;
        chunkMesh.triangleCount = sphere.triangleCount * currentChunkSize;
        chunkMesh.vertices = (float *)MemAlloc(chunkMesh.vertexCount * 3 * sizeof(float));
        chunkMesh.normals = (float *)MemAlloc(chunkMesh.vertexCount * 3 * sizeof(float));
        if (hasIndices) chunkMesh.indices = (unsigned short *)MemAlloc(chunkMesh.triangleCount * 3 * sizeof(unsigned short));

        // Fill the buffers by offsetting base sphere vertices to node positions
        for (int i = 0; i < currentChunkSize; i++) {
            int nodeIdx = startIdx + i;
            int vOffset = i * sphere.vertexCount;
            int iOffset = i * indexCount;

            for (int j = 0; j < sphere.vertexCount; j++) {
                // Copy and Translate vertex to its world position
                chunkMesh.vertices[(vOffset + j) * 3 + 0] = sphere.vertices[j * 3 + 0] + m_nodes[nodeIdx].position.x;
                chunkMesh.vertices[(vOffset + j) * 3 + 1] = sphere.vertices[j * 3 + 1] + m_nodes[nodeIdx].position.y;
                chunkMesh.vertices[(vOffset + j) * 3 + 2] = sphere.vertices[j * 3 + 2] + m_nodes[nodeIdx].position.z;

                if (sphere.normals) {
                    chunkMesh.normals[(vOffset + j) * 3 + 0] = sphere.normals[j * 3 + 0];
                    chunkMesh.normals[(vOffset + j) * 3 + 1] = sphere.normals[j * 3 + 1];
                    chunkMesh.normals[(vOffset + j) * 3 + 2] = sphere.normals[j * 3 + 2];
                }
            }

            if (hasIndices) {
                // Shift indices to point to the correct vertices in the merged buffer
                for (int j = 0; j < indexCount; j++) {
                    chunkMesh.indices[iOffset + j] = (unsigned short)(sphere.indices[j] + vOffset);
                }
            }
        }

        // Upload combined data to VRAM and load into a Model
        UploadMesh(&chunkMesh, false);
        Model model = LoadModelFromMesh(chunkMesh);
        model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = RED;
        m_nodeModels.push_back(model);
    }

    UnloadMesh(sphere); // Free the base template mesh from RAM
    m_isModelReady = true;
}

void NavigationGraph::Draw(Vector3 cameraPos, float renderRadius) const {
    if (!m_isModelReady) return;

    float radiusSq = renderRadius * renderRadius;

    // Use raylib's lower-level rendering for immediate line drawing
    rlBegin(RL_LINES);
    rlColor4ub(180, 180, 180, 120);

    for (size_t i = 0; i < m_nodes.size(); i++) {
        // FRUSTUM/DISTANCE CULLING: Skip drawing edges for distant nodes
        if (Vector3DistanceSqr(m_nodes[i].position, cameraPos) > radiusSq) continue;

        for (const auto &edge: m_nodes[i].neighbors) {
            // Avoid drawing the same edge twice (check target > i)
            if (edge.target > (int) i) {
                rlVertex3f(m_nodes[i].position.x, m_nodes[i].position.y, m_nodes[i].position.z);
                rlVertex3f(m_nodes[edge.target].position.x, m_nodes[edge.target].position.y,
                           m_nodes[edge.target].position.z);
            }
        }
    }
    rlEnd();

    // Render the batched node models
    for (const auto &model: m_nodeModels) {
        DrawModel(model, {0, 0, 0}, 1.0f, WHITE);
    }
}
