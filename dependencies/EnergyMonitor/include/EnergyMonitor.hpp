#pragma once
#include <memory>
#include "EnergyMonitorConfig.hpp"
#include "IEnergyMonitor.hpp"
class EnergyMonitor : public IEnergyMonitor
{
public:
    /**
     * @brief IEnergyMonitor::LoadMissions implementation.
     * 
     * @param[in] missions_to_load - const std::vector<MissionsAvailable> & of new missions to monitor.
     */
    void LoadMissions(const std::vector<MissionsAvailable> &missions_to_load) override;

    /**
     * @brief Get the Missions object, for extracting mission name and its current GO/NO-GO status.
     * 
     * @return Missions& 
     */
    Missions &GetMissions() override;

    /**
     * @brief Set the Battery Estimation received from ROS
     * 
     * @param estimation in seconds
     * @return true if estimation set currently
     * @return false otherwise
     */
    bool SetBatteryEstimation(float estimation) override;

    /**
     * @brief Set the Time To Home received from ROS
     * 
     * @param estimation in seconds
     * @return true if estimation set currently
     * @return false otherwise
     */
    bool SetTimeToHome(float estimation) override;

    EnergyMonitor(/* args */);
    ~EnergyMonitor();

private:
    std::shared_ptr<IEnergyMonitorMediator> m_energy_monitor_mediator;
};
