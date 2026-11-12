#include "core/Config.h"

#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace core
{
    std::string Config::findFile(const std::string& fileName, const char* envVar)
    {
        if (envVar != nullptr)
        {
            if (const char* fromEnv = std::getenv(envVar))
            {
                return fromEnv;
            }
        }

        // Parents too, so it works when run from a build folder or an IDE's working directory
        std::filesystem::path dir = std::filesystem::current_path();
        for (int depth = 0; depth < 4; depth++)
        {
            std::filesystem::path candidate = dir / "config" / fileName;
            std::error_code error;
            if (std::filesystem::exists(candidate, error))
            {
                return candidate.string();
            }
            if (!dir.has_parent_path() || dir.parent_path() == dir)
            {
                break;
            }
            dir = dir.parent_path();
        }
        return "config/" + fileName;
    }

    bool Config::load(const std::string& path)
    {
        try
        {
            mRoot = YAML::LoadFile(path);
            mLoadedPath = path;
            return true;
        }
        catch (const YAML::Exception& e)
        {
            std::cerr << "[config] could not load '" << path << "' (" << e.what() << "), using defaults\n";
            mRoot = YAML::Node();
            mLoadedPath.clear();
            return false;
        }
    }

    YAML::Node Config::find(const std::string& key) const
    {
        YAML::Node node;
        node.reset(mRoot);
        std::stringstream parts(key);
        std::string part;
        while (std::getline(parts, part, '.'))
        {
            if (!node.IsMap())
            {
                return YAML::Node();
            }
            const YAML::Node& current = node; // The const operator[] never inserts missing keys
            YAML::Node child = current[part];
            if (!child)
            {
                return YAML::Node();
            }
            node.reset(child); // reset, not =, since = on a YAML::Node writes into the tree
        }
        return node;
    }
}
