#include "Map.h"
#include "raymath.h"



void Map::Load(const char* heightmapPath, Vector3 size, const char* texturePath) {
    m_size = size;

    m_position = { -size.x / 2.0f, -size.y * MAP_POSITION_Y_FACTOR, -size.z / 2.0f };

    m_heightmapImage = LoadImage(heightmapPath);

    if (m_heightmapImage.width > MAP_HEIGHTMAP_MAX_SIZE) {
        ImageResize(&m_heightmapImage, MAP_HEIGHTMAP_MAX_SIZE, MAP_HEIGHTMAP_MAX_SIZE);
    }

    Mesh mesh = GenMeshHeightmap(m_heightmapImage, m_size);
    m_model = LoadModelFromMesh(mesh);
    m_texture = LoadTexture(texturePath);
    m_model.materials[0].maps[MATERIAL_MAP_ALBEDO].texture = m_texture;

    m_shader = LoadShader(MAP_FOG_SHADER_VS, MAP_FOG_SHADER_FS);

    m_shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(m_shader, "matModel");
    m_fogDensityLoc = GetShaderLocation(m_shader, "fogDensity");
    int fogColorLoc = GetShaderLocation(m_shader, "fogColor");

    float fogColor[4] = { MAP_FOG_COLOR_R, MAP_FOG_COLOR_G, MAP_FOG_COLOR_B, MAP_FOG_COLOR_A };
    SetShaderValue(m_shader, fogColorLoc, fogColor, SHADER_UNIFORM_VEC4);

    m_model.materials[0].shader = m_shader;
    m_fogDensity = MAP_FOG_DENSITY_DEFAULT;

    m_isLoaded = true;

    m_airportSize = { MAP_AIRPORT_SIZE_X, MAP_AIRPORT_SIZE_Y, MAP_AIRPORT_SIZE_Z };
    Vector3 basePos = { MAP_AIRPORT_BASE_POS_X, 0, MAP_AIRPORT_BASE_POS_Z };
    basePos.y = GetHeightAt(basePos.x, basePos.z) + MAP_AIRPORT_HEIGHT_OFFSET;
    m_airportPos = basePos;

    Mesh airportMesh = GenMeshCube(m_airportSize.x, m_airportSize.y, m_airportSize.z);
    m_airportModel = LoadModelFromMesh(airportMesh);
    m_airportModel.materials[0].maps[MATERIAL_MAP_ALBEDO].color = DARKGRAY;

    m_hasAirport = true;
}

void Map::UpdateFog(Vector3 cameraPos) {
    SetShaderValue(m_shader, m_shader.locs[SHADER_LOC_VECTOR_VIEW], &cameraPos, SHADER_UNIFORM_VEC3);
    SetShaderValue(m_shader, m_fogDensityLoc, &m_fogDensity, SHADER_UNIFORM_FLOAT);
}

bool Map::IsBelowGround(Vector3 position) const {
    float xRel = (position.x - m_position.x) / m_size.x;
    float zRel = (position.z - m_position.z) / m_size.z;

    if (xRel < 0.0f || xRel > 1.0f || zRel < 0.0f || zRel > 1.0f) return false;

    int px = (int)(xRel * (m_heightmapImage.width  - 1));
    int pz = (int)(zRel * (m_heightmapImage.height - 1));

    Color col = GetImageColor(m_heightmapImage, px, pz);
    unsigned char heightVal = col.r;

    float groundHeight = m_position.y + ((float)heightVal / MAP_HEIGHT_NORMALIZE) * m_size.y;

    return position.y < groundHeight + MAP_GROUND_EPSILON;
}

void Map::Draw() const {
    if (m_isLoaded) {
        DrawModel(m_model, m_position, 1.0f, WHITE);

        if (m_hasAirport) {
            DrawModel(m_airportModel, m_airportPos, 1.0f, WHITE);

            DrawCubeWires(
                m_airportPos,
                m_airportSize.x,
                m_airportSize.y + MAP_AIRPORT_WIRE_Y_OFFSET,
                m_airportSize.z,
                LIGHTGRAY
            );
        }
    }
}

Map::~Map() {
    if (m_isLoaded) {
        UnloadModel(m_model);
        UnloadTexture(m_texture);
        UnloadImage(m_heightmapImage);
    }
}

float Map::GetHeightAt(float x, float z) const {
    float xRel = (x - m_position.x) / m_size.x;
    float zRel = (z - m_position.z) / m_size.z;

    float terrainHeight = m_position.y;
    if (xRel >= 0.0f && xRel <= 1.0f && zRel >= 0.0f && zRel <= 1.0f) {
        int px = (int)(xRel * (m_heightmapImage.width  - 1));
        int pz = (int)(zRel * (m_heightmapImage.height - 1));
        Color col = GetImageColor(m_heightmapImage, px, pz);
        terrainHeight = m_position.y + ((float)col.r / MAP_HEIGHT_NORMALIZE) * m_size.y;
    }

    if (m_hasAirport) {
        float halfW = m_airportSize.x / 2.0f;
        float halfD = m_airportSize.z / 2.0f;

        if (x >= (m_airportPos.x - halfW) && x <= (m_airportPos.x + halfW) &&
            z >= (m_airportPos.z - halfD) && z <= (m_airportPos.z + halfD)) {

            float airportTopHeight = m_airportPos.y + (m_airportSize.y / 2.0f);
            return fmaxf(terrainHeight, airportTopHeight);
        }
    }

    return terrainHeight;
}