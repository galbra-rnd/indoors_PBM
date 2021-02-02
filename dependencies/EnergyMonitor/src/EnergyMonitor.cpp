#include "EnergyMonitor.hpp"

void EnergyMonitor::LoadMissions(const std::vector<MissionsAvailable> &missions_to_load)
{
    m_energy_monitor_mediator->LoadMissions(missions_to_load);
}

Missions &EnergyMonitor::GetMissions()
{
    return m_energy_monitor_mediator->GetMissionsData();
}

bool EnergyMonitor::SetBatteryEstimation(float estimation)
{
    return m_energy_monitor_mediator->SetBatteryEstimation(estimation);
}

bool EnergyMonitor::SetTimeToHome(float estimation)
{
    return m_energy_monitor_mediator->SetTimeToHomeEstimation(estimation);
}

EnergyMonitor::EnergyMonitor(/* args */)
{
    m_energy_monitor_mediator = std::make_shared<EnergyMonitorMediator>();
}

EnergyMonitor::~EnergyMonitor()
{
}
