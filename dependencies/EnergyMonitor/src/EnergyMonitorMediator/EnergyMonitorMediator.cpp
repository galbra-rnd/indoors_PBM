#include "EnergyMonitorMediator.hpp"
// #include <bits/stdc++.h>

bool check_estimation_helper(float estimation, const char *calling_func, std::shared_ptr<spdlog::logger> logger)
{
    if (estimation < 0 | std::isinf(estimation))
    {
        logger->warn(std::string(calling_func) + ":: Cant use this estimation: {} \n", estimation);
        return false;
    }
    return true;
}

void EnergyMonitorMediator::LoadThresholdValues(const std::unordered_map<Components, ThresholdValues> &mission_thresholds)
{
    m_MissionPossible.mission_thresholds = mission_thresholds;
}

void EnergyMonitorMediator::LoadMissions(const std::vector<MissionsAvailable> &missions_to_load)
{
    while (!m_MissionPossible.missions.empty())
    {
        m_MissionPossible.missions.pop_back();
    }

    for (auto &mission_to_load : missions_to_load)
    {
        MissionPossible new_mission;
        new_mission.mission = mission_to_load;
        new_mission.state = MISSION::GREEN;

        m_MissionPossible.missions.push_back(new_mission);
    }
}

bool EnergyMonitorMediator::SetMissionAvailability(MissionsAvailable mission, MISSION status)
{
    for (auto &mission_available : m_MissionPossible.missions)
    {
        if (mission_available.mission == mission)
        {
            mission_available.state = status;
            return true;
        }
    }
    m_logger->warn("No such mission: " + mission_to_string(mission) + "\n");
    return false;
}

bool EnergyMonitorMediator::SetTimeToHomeEstimation(float estimation)
{
    m_logger->info(std::string(__PRETTY_FUNCTION__) + " Got estimation: {}", estimation);
    if (!check_estimation_helper(estimation, __PRETTY_FUNCTION__, m_logger))
        return false;
    m_data_monitoring_type[DataMonitoringType::TimeToHome] = estimation;
    return true;
}

bool EnergyMonitorMediator::SetBatteryEstimation(float estimation)
{
    m_logger->info(std::string(__PRETTY_FUNCTION__) + " Got estimation: {}", estimation);
    if (!check_estimation_helper(estimation, __PRETTY_FUNCTION__, m_logger))
        return false;

    m_data_monitoring_type[DataMonitoringType::BatteryTimeLeft] = estimation;
    return true;
}

const ThresholdValues &EnergyMonitorMediator::GetComponentThresholds(const Components component)
{
    return m_MissionPossible.mission_thresholds[component];
}

float EnergyMonitorMediator::GetTimeEstimation(DataMonitoringType data_type)
{
    return m_data_monitoring_type[data_type];
}

Missions &EnergyMonitorMediator::GetMissionsData()
{
    return m_MissionPossible;
}
EnergyMonitorMediator::EnergyMonitorMediator(std::shared_ptr<spdlog::logger> logger)
    : m_logger(logger)
{
}

EnergyMonitorMediator::~EnergyMonitorMediator()
{
}