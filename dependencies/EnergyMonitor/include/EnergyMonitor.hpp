#pragma once
#include <memory>
#include "EnergyMonitorConfig.hpp"
#include "EnergyMonitorMediator/EnergyMonitorMediator.hpp"
class EnergyMonitor
{
public:
    EnergyMonitor(/* args */);
    ~EnergyMonitor();

private:
    std::shared_ptr<IEnergyMonitorMediator> m_energy_monitor_mediator;    
};
