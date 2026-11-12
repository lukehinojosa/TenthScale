#include "engine/Audio.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>

namespace engine
{
    Wave makeTone(const ToneSettings& settings)
    {
        Wave wave{};
        int sampleRate = std::max(settings.sampleRate, 1);
        unsigned int frameCount = (unsigned int)std::max(0.0f, settings.seconds * (float)sampleRate);
        if (frameCount == 0)
        {
            return wave;
        }

        auto* pSamples = (std::int16_t*)MemAlloc(frameCount * sizeof(std::int16_t));
        std::minstd_rand random(1234);
        std::uniform_real_distribution<float> noise(-1.0f, 1.0f);
        const float attackFrames = std::min(0.005f * (float)sampleRate, (float)frameCount * 0.5f);
        const float volume = std::clamp(settings.volume, 0.0f, 1.0f);
        double phase = 0.0;

        for (unsigned int i = 0; i < frameCount; i++)
        {
            float t = (float)i / (float)frameCount;
            float frequency = settings.startFrequency + (settings.endFrequency - settings.startFrequency) * t;
            phase += (double)frequency / (double)sampleRate;
            float cycle = (float)(phase - std::floor(phase));

            float value = 0.0f;
            switch (settings.shape)
            {
            case Waveform::SINE:
                value = std::sin(cycle * 2.0f * PI);
                break;
            case Waveform::SQUARE:
                value = cycle < 0.5f ? 1.0f : -1.0f;
                break;
            case Waveform::TRIANGLE:
                value = 4.0f * std::abs(cycle - 0.5f) - 1.0f;
                break;
            case Waveform::NOISE:
                value = noise(random);
                break;
            }

            // A quick fade in and a fade out to the end, so the tone doesn't click
            float envelope = std::min((float)i / std::max(attackFrames, 1.0f), 1.0f) * (1.0f - t);
            pSamples[i] = (std::int16_t)(value * envelope * volume * 32767.0f);
        }

        wave.frameCount = frameCount;
        wave.sampleRate = (unsigned int)sampleRate;
        wave.sampleSize = 16;
        wave.channels = 1;
        wave.data = pSamples;
        return wave;
    }

    AudioManager::~AudioManager()
    {
        shutdown();
    }

    bool AudioManager::init()
    {
        if (mReady)
        {
            return true;
        }
        InitAudioDevice();
        mReady = IsAudioDeviceReady();
        if (mReady)
        {
            SetMasterVolume(mMasterVolume);
        }
        else
        {
            TraceLog(LOG_WARNING, "AUDIO: no audio device, sound is off");
        }
        return mReady;
    }

    void AudioManager::shutdown()
    {
        if (!mReady)
        {
            return;
        }
        unloadAll();
        CloseAudioDevice();
        mReady = false;
    }

    bool AudioManager::addSoundSet(const std::string& name, Sound source, int voices)
    {
        if (source.frameCount == 0)
        {
            return false;
        }
        if (mSounds.contains(name))
        {
            SoundSet& old = mSounds[name];
            for (Sound& alias : old.aliases)
            {
                UnloadSoundAlias(alias);
            }
            UnloadSound(old.source);
            mSounds.erase(name);
        }

        SoundSet set;
        set.source = source;
        for (int i = 1; i < voices; i++)
        {
            set.aliases.push_back(LoadSoundAlias(source));
        }
        mSounds.emplace(name, std::move(set));
        return true;
    }

    bool AudioManager::loadSound(const std::string& name, const std::string& path, int voices)
    {
        if (!mReady)
        {
            return false;
        }
        return addSoundSet(name, LoadSound(path.c_str()), voices);
    }

    bool AudioManager::addSound(const std::string& name, const Wave& wave, int voices)
    {
        if (!mReady || wave.data == nullptr)
        {
            return false;
        }
        return addSoundSet(name, LoadSoundFromWave(wave), voices);
    }

    void AudioManager::playSound(const std::string& name, float volume, float pitch)
    {
        auto it = mSounds.find(name);
        if (!mReady || it == mSounds.end())
        {
            return;
        }

        // Round robin over the source and its aliases
        SoundSet& set = it->second;
        size_t voiceCount = set.aliases.size() + 1;
        Sound& voice = set.next == 0 ? set.source : set.aliases[set.next - 1];
        set.next = (set.next + 1) % voiceCount;

        SetSoundVolume(voice, volume);
        SetSoundPitch(voice, pitch);
        PlaySound(voice);
    }

    void AudioManager::stopSound(const std::string& name)
    {
        auto it = mSounds.find(name);
        if (!mReady || it == mSounds.end())
        {
            return;
        }
        StopSound(it->second.source);
        for (Sound& alias : it->second.aliases)
        {
            StopSound(alias);
        }
    }

    bool AudioManager::loadMusic(const std::string& name, const std::string& path)
    {
        if (!mReady)
        {
            return false;
        }
        Music music = LoadMusicStream(path.c_str());
        if (music.frameCount == 0)
        {
            return false;
        }
        if (mMusic.contains(name))
        {
            if (mCurrentMusic == name)
            {
                stopMusic();
            }
            UnloadMusicStream(mMusic[name]);
        }
        mMusic[name] = music;
        return true;
    }

    void AudioManager::playMusic(const std::string& name, bool loop)
    {
        auto it = mMusic.find(name);
        if (!mReady || it == mMusic.end())
        {
            return;
        }
        stopMusic();
        it->second.looping = loop;
        SetMusicVolume(it->second, mMusicVolume);
        PlayMusicStream(it->second);
        mCurrentMusic = name;
    }

    void AudioManager::stopMusic()
    {
        auto it = mMusic.find(mCurrentMusic);
        if (mReady && it != mMusic.end())
        {
            StopMusicStream(it->second);
        }
        mCurrentMusic.clear();
    }

    void AudioManager::setMusicVolume(float volume)
    {
        mMusicVolume = volume;
        auto it = mMusic.find(mCurrentMusic);
        if (mReady && it != mMusic.end())
        {
            SetMusicVolume(it->second, volume);
        }
    }

    void AudioManager::setMasterVolume(float volume)
    {
        mMasterVolume = std::clamp(volume, 0.0f, 1.0f);
        if (mReady)
        {
            SetMasterVolume(mMasterVolume);
        }
    }

    void AudioManager::update()
    {
        auto it = mMusic.find(mCurrentMusic);
        if (mReady && it != mMusic.end())
        {
            UpdateMusicStream(it->second);
        }
    }

    void AudioManager::unloadAll()
    {
        stopMusic();
        for (auto& [name, set] : mSounds)
        {
            for (Sound& alias : set.aliases)
            {
                UnloadSoundAlias(alias);
            }
            UnloadSound(set.source);
        }
        mSounds.clear();
        for (auto& [name, music] : mMusic)
        {
            UnloadMusicStream(music);
        }
        mMusic.clear();
    }
}
