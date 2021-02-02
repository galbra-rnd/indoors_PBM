#pragma once

#include "IEnergyMonitorMediator.hpp"
#include "IEnergyMonitorMediatorDataManager.hpp"
class EnergyMonitorMediator
    : public IEnergyMonitorMediator,
      public IEnergyMonitorMediatorDataManager
{
public:
    /**
     * @brief IEnergyMonitorMediator::LoadMissions implementation.
     * 
     * @param[in] missions_to_load - const std::vector<MissionsAvailable> & of new missions to monitor.
     */
    void LoadMissions(const std::vector<MissionsAvailable> &missions_to_load) override;

    /**
     * @brief Interface implementation for Behavior trees.
     * For setting mission availability according to specific condition.
     * 
     * @param[in] mission - MissionsAvailable for setting its availability.
     * @param[in] status - Setting a new MISSION availability.
     */
    void SetMissionAvailability(MissionsAvailable mission, MISSION status) override;

    /**
     * @brief Set the Battery Estimation object
     * 
     * @param estimation - Operation time left on current battery.
     * @return true if estimation set successfully.
     * @return false otherwise.
     */
    bool SetBatteryEstimation(float estimation) override;

    /**
     * @brief Set the TimeToHome Estimation
     * 
     * @param estimation - Time to home from current position.
     * @return true if estimation set successfully.
     * @return false otherwise.
     */
    bool SetTimeToHomeEstimation(float estimation) override;

    /**
     * @brief Get the Missions Data object
     * Each MissionPossible in Missions contain mission name, and its MISSION
     * 
     * @return Missions& 
     */
    Missions &GetMissionsData() override;

    /**
     * @brief Get the Time Estimation for a DataMonitoringType
     * 
     * @param data_type - data_type for receiving its time estimation.
     * @return float  - time estimation fo requested data-type.
     */
    float GetTimeEstimation(DataMonitoringType data_type) override;

    EnergyMonitorMediator(/* args */);
    ~EnergyMonitorMediator();

private:
    Missions m_MissionPossible;
    std::unordered_map<DataMonitoringType, float> m_data_monitoring_type;
};
