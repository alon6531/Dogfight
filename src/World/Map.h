#ifndef DOGFIGHT_MAP_H
#define DOGFIGHT_MAP_H

#include "raylib.h"
#include <vector>

// --- Load ---
#define MAP_HEIGHTMAP_MAX_SIZE          512
#define MAP_POSITION_Y_FACTOR           0.7f
#define MAP_FOG_SHADER_VS               "Assets/shaders/fog.vs"
#define MAP_FOG_SHADER_FS               "Assets/shaders/fog.fs"
#define MAP_FOG_COLOR_R                 0.4f
#define MAP_FOG_COLOR_G                 0.75f
#define MAP_FOG_COLOR_B                 1.0f
#define MAP_FOG_COLOR_A                 1.0f
#define MAP_FOG_DENSITY_DEFAULT         0.001f
#define MAP_AIRPORT_SIZE_X              400.0f
#define MAP_AIRPORT_SIZE_Y              2.0f
#define MAP_AIRPORT_SIZE_Z              100.0f
#define MAP_AIRPORT_BASE_POS_X         -600.0f
#define MAP_AIRPORT_BASE_POS_Z         -400.0f
#define MAP_AIRPORT_HEIGHT_OFFSET       25.0f

// --- IsBelowGround ---
#define MAP_GROUND_EPSILON              1.0f
#define MAP_HEIGHT_NORMALIZE            255.0f

// --- Draw ---
#define MAP_AIRPORT_WIRE_Y_OFFSET       0.5f



class Map {
public:
    Map() = default;
    ~Map();


    void Load(const char* heightmapPath, Vector3 size, const char* texturePath);

    void UpdateFog(Vector3 cameraPos);


    void Draw() const;


    [[nodiscard]] Model GetModel() const { return m_model; }
    [[nodiscard]] Vector3 GetSize() const { return m_size; }
    [[nodiscard]] Image GetHeightmapImage() const { return m_heightmapImage; }


    [[nodiscard]] bool IsBelowGround(Vector3 position) const;

    [[nodiscard]] Vector3 GetPosition() const { return m_position; }

    [[nodiscard]] float GetHeightAt(float x, float z) const;


private:
    Model m_model;
    Texture2D m_texture;
    Image m_heightmapImage;
    Vector3 m_size;
    Vector3 m_position;
    bool m_isLoaded = false;

    Shader m_shader;
    int m_fogDensityLoc;
    float m_fogDensity = 0.002f;

    Vector3 m_airportPos;
    Vector3 m_airportSize;
    bool m_hasAirport = false;
    Model m_airportModel;
};

#endif //DOGFIGHT_MAP_H