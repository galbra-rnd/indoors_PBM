#pragma once

struct Factors
{
    float avg_speed = -1;
};

struct RosParams
{

    float velocity_max_formard = -1;
    Factors factors;
    std::vector<std::string> loaded_policies;
};
