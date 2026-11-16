#include "flight/DesignFile.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace tenth
{
    namespace
    {
        constexpr const char* HEADER = "tenth-rocket";
        constexpr int VERSION = 1;
        constexpr const char* ROCKET_FOLDER = "rockets";
        constexpr const char* ROCKET_EXTENSION = ".rocket";

        std::filesystem::path rocketPath(const std::string& name)
        {
            return std::filesystem::path(ROCKET_FOLDER) / (cleanRocketName(name) + ROCKET_EXTENSION);
        }

        // Copies browser storage into the rockets folder, or the folder back out to storage
        void syncBrowserStorage(bool load)
        {
#ifdef __EMSCRIPTEN__
            EM_ASM({ FS.syncfs($0, function(error) {}); }, load ? 1 : 0);
#else
            (void)load;
#endif
        }
    }

    std::string writeDesign(const RocketDesign& design)
    {
        std::ostringstream out;
        out.precision(17);
        out << HEADER << ' ' << VERSION << '\n';
        out << "stages " << design.stageCount() << ' ' << (design.customStaging() ? 1 : 0) << '\n';
        for (PartIndex i = 0; i < design.partCount(); i++)
        {
            const PlacedPart& placed = design.part(i);
            out << "part " << placed.pDefinition->id << ' ';
            switch (placed.attachment)
            {
            case Attachment::Root:
                out << "root";
                break;
            case Attachment::Stack:
                out << "stack " << placed.parent << ' ' << (placed.parentNode == Node::Top ? "top" : "bottom");
                break;
            case Attachment::Radial:
                out << "radial " << placed.parent << ' ' << placed.angle << ' '
                    << placed.position.y - design.part(placed.parent).position.y;
                break;
            }
            out << ' ' << placed.symmetryGroup << ' ' << placed.stage << '\n';
        }
        return out.str();
    }

    std::optional<RocketDesign> readDesign(const std::string& text, const PartCatalog& catalog)
    {
        std::istringstream in(text);
        std::string header;
        int version = 0;
        if (!(in >> header >> version) || header != HEADER || version != VERSION)
        {
            return std::nullopt;
        }
        std::string word;
        int stageCount = 0;
        int custom = 0;
        if (!(in >> word >> stageCount >> custom) || word != "stages")
        {
            return std::nullopt;
        }

        RocketDesign design;
        while (in >> word)
        {
            std::string id;
            std::string attachment;
            if (word != "part" || !(in >> id >> attachment))
            {
                return std::nullopt;
            }
            const PartDefinition* pDefinition = catalog.find(id);
            if (pDefinition == nullptr)
            {
                return std::nullopt;
            }

            // Parents come before children, so each parent is already in place when its child is read
            PartIndex index = NO_PART;
            if (attachment == "root" && design.empty())
            {
                index = design.setRoot(*pDefinition);
            }
            else if (attachment == "stack")
            {
                PartIndex parent = NO_PART;
                std::string node;
                if (in >> parent >> node)
                {
                    index = design.attachStack(parent, node == "top" ? Node::Top : Node::Bottom, *pDefinition);
                }
            }
            else if (attachment == "radial")
            {
                PartIndex parent = NO_PART;
                double angle = 0.0;
                double height = 0.0;
                if (in >> parent >> angle >> height)
                {
                    index = design.attachRadial(parent, angle, height, *pDefinition);
                }
            }

            int group = 0;
            int stage = NO_STAGE;
            if (index == NO_PART || !(in >> group >> stage))
            {
                return std::nullopt;
            }
            design.setSymmetryGroup(index, group);
            design.setStage(index, stage);
        }
        design.setStaging(stageCount, custom != 0);
        design.updateStaging();
        return design;
    }

    RocketDesign defaultRocket(const PartCatalog& catalog)
    {
        RocketDesign design;
        auto find = [&catalog](const char* id) -> const PartDefinition&
        {
            return *catalog.find(id);
        };
        auto single = [&find](const char* id)
        {
            RocketDesign part;
            part.setRoot(find(id));
            return part;
        };

        const PartIndex pod = design.setRoot(find("pod"));
        design.attachStack(pod, Node::Top, find("nose-cone"));
        const PartIndex upperTank = design.attachStack(pod, Node::Bottom, find("tank-large"));
        const PartIndex vacuum = design.attachStack(upperTank, Node::Bottom, find("engine-vacuum"));
        design.attachDesignRadial(vacuum, 0.0, 0.0, single("landing-leg"), 4);
        PartIndex below = design.attachStack(vacuum, Node::Bottom, find("decoupler"));
        for (int i = 0; i < 2; i++)
        {
            below = design.attachStack(below, Node::Bottom, find("tank-large"));
        }
        const PartIndex lifter = design.attachStack(below, Node::Bottom, find("engine-lifter"));

        // Fins sit flush with the bottom of the engine, between the legs' angles
        const double finHeight = find("fin").shape.height;
        const double lifterHeight = find("engine-lifter").shape.height;
        design.attachDesignRadial(lifter, std::numbers::pi / 4.0, (finHeight - lifterHeight) * 0.5, single("fin"), 4);
        design.autoStage();
        return design;
    }

    std::string cleanRocketName(const std::string& name)
    {
        std::string clean;
        for (char c : name)
        {
            const unsigned char u = (unsigned char)c;
            if (std::isalnum(u) || c == ' ' || c == '-' || c == '_')
            {
                clean += c;
            }
        }
        const size_t first = clean.find_first_not_of(' ');
        const size_t last = clean.find_last_not_of(' ');
        return first == std::string::npos ? std::string() : clean.substr(first, last - first + 1);
    }

    void initRocketStorage()
    {
#ifdef __EMSCRIPTEN__
        EM_ASM({
            FS.mkdir('/rockets');
            FS.mount(IDBFS, {}, '/rockets');
        });
        syncBrowserStorage(true);
#endif
    }

    std::vector<std::string> savedRocketNames()
    {
        std::vector<std::string> names;
        std::error_code error;
        for (const auto& entry : std::filesystem::directory_iterator(ROCKET_FOLDER, error))
        {
            if (entry.path().extension() == ROCKET_EXTENSION)
            {
                names.push_back(entry.path().stem().string());
            }
        }
        std::sort(names.begin(), names.end());
        return names;
    }

    bool saveRocket(const std::string& name, const RocketDesign& design)
    {
        if (cleanRocketName(name).empty())
        {
            return false;
        }
        std::error_code error;
        std::filesystem::create_directories(ROCKET_FOLDER, error);
        std::ofstream file(rocketPath(name));
        file << writeDesign(design);
        file.close();
        if (!file)
        {
            return false;
        }
        syncBrowserStorage(false);
        return true;
    }

    std::optional<RocketDesign> loadRocket(const std::string& name, const PartCatalog& catalog)
    {
        std::ifstream file(rocketPath(name));
        if (!file)
        {
            return std::nullopt;
        }
        std::ostringstream text;
        text << file.rdbuf();
        return readDesign(text.str(), catalog);
    }
}
