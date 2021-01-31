#include "EnergyMonitor.hpp"

EnergyMonitor::EnergyMonitor(/* args */)
{
    m_energy_monitor_mediator =  std::make_shared<EnergyMonitorMediator>();  
    m_energy_monitor_mediator->GetSomeData();  
}

EnergyMonitor::~EnergyMonitor()
{
}
