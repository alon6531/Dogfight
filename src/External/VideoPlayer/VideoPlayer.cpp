//
// Created by User on 07/05/2026.
//

#include "VideoPlayer.h"
#define PL_MPEG_IMPLEMENTATION
#include "../../External/pl_mpeg.h"
#include <cstdlib>
#include <string>

#include "raylib.h"

VideoPlayer::VideoPlayer() {}

VideoPlayer::~VideoPlayer() { Unload(); }

bool VideoPlayer::Load(const std::string& fileName) {
    m_plm = plm_create_with_filename(fileName.c_str());
    if (!m_plm) return false;

    plm_set_audio_enabled(m_plm, true);
    m_videoWidth = plm_get_width(m_plm);
    m_videoHeight = plm_get_height(m_plm);

    // הכנת הטקסטורה
    Image img = {
        calloc((size_t)m_videoWidth * m_videoHeight * 3, 1),
        m_videoWidth, m_videoHeight, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8
    };
    m_videoFrame = LoadTextureFromImage(img);
    free(img.data);

    // הכנת הסאונד
    int sampleRate = plm_get_samplerate(m_plm);
    m_audioStream = LoadAudioStream(sampleRate > 0 ? sampleRate : 48000, 32, 2);
    PlayAudioStream(m_audioStream);

    // הגדרת ה-Callbacks
    plm_set_video_decode_callback(m_plm, OnVideoFrame, this);
    plm_set_audio_decode_callback(m_plm, OnAudioFrame, this);

    m_isLoaded = true;
    return true;
}

void VideoPlayer::Update(float deltaTime) {
    if (!m_plm) return;

    plm_decode(m_plm, deltaTime);

    // עדכון סאונד (בדיוק כמו בקוד המקורי שלך)
    const int chunkSize = 4096;
    while (IsAudioStreamProcessed(m_audioStream) && m_audioQueue.size() >= chunkSize * 2) {
        UpdateAudioStream(m_audioStream, m_audioQueue.data(), chunkSize);
        m_audioQueue.erase(m_audioQueue.begin(), m_audioQueue.begin() + (chunkSize * 2));
    }

    // עדכון טקסטורה מה-Buffer
    if (m_videoRgbBuffer) {
        UpdateTexture(m_videoFrame, m_videoRgbBuffer);
    }
}

void VideoPlayer::Draw(int x, int y, int width, int height) {
    if (m_isLoaded && m_videoFrame.id > 0) {
        DrawTexturePro(m_videoFrame,
            {0, 0, (float)m_videoWidth, (float)m_videoHeight},
            {(float)x, (float)y, (float)width, (float)height},
            {0,0}, 0, WHITE);
    }
}

void VideoPlayer::Unload() {
    if (m_plm) {
        plm_destroy(m_plm);
        m_plm = nullptr;
    }
    if (m_videoFrame.id > 0) {
        UnloadTexture(m_videoFrame);
        m_videoFrame.id = 0; // חשוב מאוד!
    }
    if (m_audioStream.buffer != nullptr) {
        UnloadAudioStream(m_audioStream);
        m_audioStream.buffer = nullptr; // חשוב מאוד!
    }
    if (m_videoRgbBuffer) {
        free(m_videoRgbBuffer);
        m_videoRgbBuffer = nullptr;
    }

    m_isLoaded = false;
}

bool VideoPlayer::IsFinished() const {
    return !m_plm || plm_has_ended(m_plm);
}

// מימוש ה-Callbacks הסטטיים
void VideoPlayer::OnVideoFrame(plm_t* player, plm_frame_t* frame, void* user) {
    auto* self = (VideoPlayer*)user;
    if (!self->m_videoRgbBuffer) {
        self->m_videoRgbBuffer = (unsigned char*)malloc(frame->width * frame->height * 3);
    }
    plm_frame_to_rgb(frame, self->m_videoRgbBuffer, frame->width * 3);
}

void VideoPlayer::OnAudioFrame(plm_t* player, plm_samples_t* samples, void* user) {
    auto* self = (VideoPlayer*)user;
    self->m_audioQueue.insert(self->m_audioQueue.end(), samples->interleaved, samples->interleaved + (samples->count * 2));
}