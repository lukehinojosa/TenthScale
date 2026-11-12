#pragma once

#include <sstream>
#include <string>

#include <yaml-cpp/yaml.h>

namespace core
{
    // Read-only YAML settings. Missing files, keys, or bad values fall back to the defaults you pass,
    // so the program always runs and a partial file only overrides what it mentions.
    class Config
    {
    public:
        // Checks $envVar first, then config/<fileName> in the working directory and a few parents
        static std::string findFile(const std::string& fileName, const char* envVar = nullptr);

        // Returns false (and keeps every value at its default) if the file can't be read
        bool load(const std::string& path);

        // key is dotted: "physics.fixed_hz"
        template<class T>
        T get(const std::string& key, const T& fallback) const
        {
            YAML::Node node = find(key);
            if (!node || !node.IsScalar())
            {
                return fallback;
            }
            try
            {
                return node.as<T>();
            }
            catch (const YAML::Exception&)
            {
                return fallback;
            }
        }

        bool isLoaded() const { return !mLoadedPath.empty(); }
        const std::string& getLoadedPath() const { return mLoadedPath; }

    private:
        YAML::Node find(const std::string& key) const;

        YAML::Node mRoot;
        std::string mLoadedPath;
    };
}
