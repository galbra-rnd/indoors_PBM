#pragma once
#include <unordered_map>

namespace BPM_COMMANDS
{
    enum TOPIC_TYPE
    {
        LAND_TO_FFK,
    };
    static std::unordered_map<TOPIC_TYPE, std::string> TOPICS(
        {{TOPIC_TYPE::LAND_TO_FFK, "/communication_manager/out/command"}});

} // namespace BPM_COMMANDS
