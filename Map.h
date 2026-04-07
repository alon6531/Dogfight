#ifndef DOGFIGHT_MAP_H
#define DOGFIGHT_MAP_H

#include "raylib.h"
#include <vector>

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




private:
    Model m_model;
    Texture2D m_texture;
    Image m_heightmapImage;
    Vector3 m_size;
    Vector3 m_position;
    bool m_isLoaded = false;

    Shader m_shader;
    int m_fogDensityLoc;
    float m_fogDensity = 0.015f;
};

#endif //DOGFIGHT_MAP_H