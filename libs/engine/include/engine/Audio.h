#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <raylib.h>

namespace engine
{
    enum class Waveform
    {
        SINE,
        SQUARE,
        TRIANGLE,
        NOISE
    };

    struct ToneSettings
    {
        Waveform shape = Waveform::SINE;
        float startFrequency = 440.0f; // Hz
        float endFrequency = 440.0f; // Sweeps from start to end over the tone
        float seconds = 0.15f;
        float volume = 0.5f;
        int sampleRate = 44100;
    };

    // Builds a short sound effect in memory, so a game can make noise without shipping files. Free it with UnloadWave.
    Wave makeTone(const ToneSettings& settings);

    // Owns sounds and music by name. Every call is a safe no-op when no audio device could be opened.
    class AudioManager
    {
    public:
        AudioManager() = default;
        ~AudioManager();

        AudioManager(const AudioManager&) = delete;
        AudioManager& operator=(const AudioManager&) = delete;

        bool init();
        void shutdown();
        bool isReady() const { return mReady; }

        // Each sound gets `voices` copies sharing one buffer, so rapid repeats overlap instead of cutting off
        bool loadSound(const std::string& name, const std::string& path, int voices = 4);
        bool addSound(const std::string& name, const Wave& wave, int voices = 4);
        void playSound(const std::string& name, float volume = 1.0f, float pitch = 1.0f);
        void stopSound(const std::string& name);
        bool hasSound(const std::string& name) const { return mSounds.contains(name); }

        // Music streams from disk instead of loading whole, for long tracks
        bool loadMusic(const std::string& name, const std::string& path);
        void playMusic(const std::string& name, bool loop = true);
        void stopMusic();
        void setMusicVolume(float volume);
        const std::string& getCurrentMusic() const { return mCurrentMusic; }

        void setMasterVolume(float volume);
        float getMasterVolume() const { return mMasterVolume; }

        // Keeps music streaming; Game calls this every frame
        void update();
        void unloadAll();

    private:
        struct SoundSet
        {
            Sound source{};
            std::vector<Sound> aliases;
            size_t next = 0;
        };

        bool addSoundSet(const std::string& name, Sound source, int voices);

        std::unordered_map<std::string, SoundSet> mSounds;
        std::unordered_map<std::string, Music> mMusic;
        std::string mCurrentMusic;
        float mMusicVolume = 1.0f;
        float mMasterVolume = 1.0f;
        bool mReady = false;
    };
}
