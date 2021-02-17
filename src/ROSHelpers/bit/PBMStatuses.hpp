#pragma once
#include <unordered_map>
#include <string>

namespace pbm_statuses
{
    const static int BASE = 1000;
    const int STATUS = 0;
    const int WARNING = 1;
    const int ERROR = 2;

    static std::unordered_map<int, std::pair<int, std::string>> IdStatusMap(
        {{BASE, std::make_pair(STATUS, "Everything is OK")},
         {BASE + 1, std::make_pair(WARNING, "Caught an exception in tree root ticking.")},
         {BASE + 2, std::make_pair(WARNING, "Tree tick resulted in FAILURE! (see logs).")},
         {BASE + 3, std::make_pair(WARNING, "No policies loaded. Waiting.")}});
} // namespace pbm_statuses
