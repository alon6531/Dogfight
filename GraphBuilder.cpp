//
// Created by User on 03/04/2026.
//
#include "GraphBuilder.h"

#include <map>

#include "rlgl.h"
#include "raymath.h"
#include <omp.h>
#include <string>
#include <unordered_map>
#include "Global.h"


bool NavigationGraph::IsPointBlocked(Vector3 p, const std::vector<Obstacle> &obstacles) {
    for (const auto& obs : obstacles) {
        // Calculate squared distance between point p and obstacle center
        // Using squared distance is faster as it avoids the sqrt() operation
        float distSq = Vector3DistanceSqr(p, obs.pos);

        // If distance is less than radius squared, the point is inside the obstacle
        if (distSq < (obs.radius * obs.radius)) {
            return true;
        }
    }
    return false; // Point is clear
}

void NavigationGraph::BuildGraphFromMap(Vector3 arenaSize, float spacing, const std::vector<Obstacle> &obstacles) {
    m_nodes.clear();
    int idCounter = 0;

    // Help with identify node layer
    auto getPosKey = [](Vector3 p) {
        return std::to_string((int)round(p.x)) + "," +
               std::to_string((int)round(p.y)) + "," +
               std::to_string((int)round(p.z));
    };
    std::unordered_map<std::string, int> posToId;

    // Init nodes
    for (float x = -arenaSize.x / 2; x <= arenaSize.x / 2; x += spacing) {
        for (float y = 0; y <= arenaSize.y; y += spacing) {
            for (float z = -arenaSize.z / 2; z <= arenaSize.z / 2; z += spacing) {
                Vector3 pos = {x, y, z};
                if (!IsPointBlocked(pos, obstacles)) {
                    Node n;
                    n.id = idCounter++;
                    n.position = pos;
                    m_nodes.push_back(n);
                    posToId[getPosKey(pos)] = n.id;
                }
            }
        }
    }

    // Init Edges
    #pragma omp parallel for schedule(dynamic)
    for (auto & m_node : m_nodes) {
        Vector3 p = m_node.position;

        for (float dx = -spacing; dx <= spacing; dx += spacing) {
            for (float dy = -spacing; dy <= spacing; dy += spacing) {
                for (float dz = -spacing; dz <= spacing; dz += spacing) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;

                    Vector3 neighborPos = { p.x + dx, p.y + dy, p.z + dz };
                    std::string key = getPosKey(neighborPos);

                    // אם קיים צומת במיקום השכן הזה
                    if (posToId.count(key)) {
                        int neighborId = posToId[key];
                        float dist = Vector3Distance(p, neighborPos);

                        // אין צורך ב-critical כאן אם אנחנו מוסיפים רק לצד של ה-i הנוכחי
                        // כי כל Thread מטפל ב-i אחר
                        m_node.neighbors.push_back({neighborId, dist});
                    }
                }
            }
        }
    }
}



void NavigationGraph::BuildDistanceMatrix() {
    std::vector<int> landmarkIndices;

    // Select corner nodes as Landmarks for the ALT heuristic
    landmarkIndices.push_back(0);
    landmarkIndices.push_back(m_nodes.size() - 1);
    landmarkIndices.push_back(m_nodes.size()/2);

    distanceMatrix.assign(landmarkIndices.size(), std::vector<float>(m_nodes.size(), -1.0f));

    for (size_t i = 0; i < landmarkIndices.size(); ++i) {
        // calculate shortest paths from each landmark to all nodes
        ComputeDijkstra(*this, landmarkIndices[i], distanceMatrix[i]);
    }
}

float NavigationGraph::GetHeuristic(int nIdx, int targetIdx) {
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

void NavigationGraph::PrepareGPUData() {
    for (auto &model: m_nodeModels) {
        UnloadModel(model);
    }
    m_nodeModels.clear();

    if (m_nodes.empty()) return;


    Mesh sphere = GenMeshSphere(0.2f, 8, 8);


    bool hasIndices = (sphere.indices != nullptr);
    int indexCount = hasIndices ? (sphere.triangleCount * 3) : 0;

    const int nodesPerMesh = 200;

    for (int startIdx = 0; startIdx < (int) m_nodes.size(); startIdx += nodesPerMesh) {
        int endIdx = ((startIdx + nodesPerMesh) < (int) m_nodes.size())
                         ? (startIdx + nodesPerMesh)
                         : (int) m_nodes.size();
        int currentChunkSize = endIdx - startIdx;

        Mesh chunkMesh = {0};
        chunkMesh.vertexCount = sphere.vertexCount * currentChunkSize;
        chunkMesh.triangleCount = sphere.triangleCount * currentChunkSize;


        chunkMesh.vertices = (float *) MemAlloc(chunkMesh.vertexCount * 3 * sizeof(float));
        chunkMesh.normals = (float *) MemAlloc(chunkMesh.vertexCount * 3 * sizeof(float));
        if (hasIndices) {
            chunkMesh.indices = (unsigned short *) MemAlloc(chunkMesh.triangleCount * 3 * sizeof(unsigned short));
        }

        for (int i = 0; i < currentChunkSize; i++) {
            int nodeIdx = startIdx + i;
            int vOffset = i * sphere.vertexCount;
            int tOffset = i * sphere.triangleCount;


            for (int j = 0; j < sphere.vertexCount; j++) {
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
                for (int j = 0; j < indexCount; j++) {
                    chunkMesh.indices[tOffset * 3 + j] = (unsigned short) (sphere.indices[j] + vOffset);
                }
            }
        }


        UploadMesh(&chunkMesh, false);
        Model model = LoadModelFromMesh(chunkMesh);
        model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = RED;
        m_nodeModels.push_back(model);
    }

    UnloadMesh(sphere);
    m_isModelReady = true;
}

void NavigationGraph::Draw(Vector3 cameraPos, float renderRadius) const {
    if (!m_isModelReady) return;

    float radiusSq = renderRadius * renderRadius;

    rlBegin(RL_LINES);
    rlColor4ub(180, 180, 180, 120);

    for (size_t i = 0; i < m_nodes.size(); i++) {
        if (Vector3DistanceSqr(m_nodes[i].position, cameraPos) > radiusSq) {
            continue;
        }

        for (const auto &edge: m_nodes[i].neighbors) {
            if (edge.target > (int) i) {
                rlVertex3f(m_nodes[i].position.x, m_nodes[i].position.y, m_nodes[i].position.z);
                rlVertex3f(m_nodes[edge.target].position.x, m_nodes[edge.target].position.y,
                           m_nodes[edge.target].position.z);
            }
        }
    }
    rlEnd();

    for (const auto &model: m_nodeModels) {
        DrawModel(model, {0, 0, 0}, 1.0f, WHITE);
    }
}
