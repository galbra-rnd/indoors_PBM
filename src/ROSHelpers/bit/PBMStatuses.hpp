#pragma once
#include <unordered_map>
#include <string>

const static int BASE = 1000;
namespace pbm_statuses
{
    const int STATUS = 0;
    const int WARNING = 1;
    const int ERROR = 2;

    static std::unordered_map<int, std::pair<int, std::string>> IdStatusMap(
        {{BASE, std::make_pair(STATUS, "Everything is OK")},
        {BASE + 1, std::make_pair(WARNING, "Cought an exception in tree root ticking.")}});
} // namespace pbm_statuses
