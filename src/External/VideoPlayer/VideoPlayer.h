//
// Created by User on 07/05/2026.
//

#ifndef DOGFIGHT_VIDEOPLAYER_H
#define DOGFIGHT_VIDEOPLAYER_H
#include <string>
#include <vector>

#include "raylib.h"
#include  "../pl_mpeg.h"

class VideoPlayer {
public:
    VideoPlayer();
    ~VideoPlayer();

    bool Load(const std::string& fileName);
    void Update(float deltaTime);
    void Draw(int x, int y, int width, int height);

    [[nodiscard]] bool IsFinished() const;
    void Unload();

    static void OnVideoFrame(plm_t* player, plm_frame_t* frame, void* user);
    static void OnAudioFrame(plm_t* player, plm_samples_t* samples, void* user);

private:
    plm_t* m_plm = nullptr;
    Texture2D m_videoFrame = { 0 };
    unsigned char* m_videoRgbBuffer = nullptr;

    AudioStream m_audioStream = { 0 };
    std::vector<float> m_audioQueue;

    int m_videoWidth = 0;
    int m_videoHeight = 0;
    bool m_isLoaded = false;
};


#endif //DOGFIGHT_VIDEOPLAYER_H