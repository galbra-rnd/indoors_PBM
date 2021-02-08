#pragma once
#include <string>

enum Components
{
    BATTERY_TIME = 0,
    TIME_TO_HOME
};

static const std::string component_to_string(Components component)
{
    switch (component)
    {
    case Components::BATTERY_TIME:
        return "BATTERY_TIME";
    case Components::TIME_TO_HOME:
        return "TIME_TO_HOME";
    default:
        return "NOT_IMPLEMENTED";
    }
};
