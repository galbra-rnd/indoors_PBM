#pragma once

#include "IEnergyMonitorMediator.hpp"
#include "IEnergyMonitorMediatorDataProvider.hpp"

class EnergyMonitorMediator
    : public IEnergyMonitorMediator,
      public IEnergyMonitorMediatorDataProvider
{
public:
    /**
     * @brief Get the Logger object
     * 
     * @return std::shared_ptr<spdlog::logger> 
     */
    std::shared_ptr<spdlog::logger> GetLogger() override { return m_logger; };
    /**
     * @brief IEnergyMonitorMediator::LoadThresholdValues implementation.
     * 
     * @param mission_thresholds - const std::unordered_map<MissionsAvailable, ThresholdValues> & filled with threshold per each mission
     */
    void LoadThresholdValues(const std::unordered_map<Components, ThresholdValues> &mission_thresholds) override;
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
     * @return true - mission exist in dict
     * @return false - otherwise 
     */
    bool SetMissionAvailability(MissionsAvailable mission, MISSION status) override;

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

    /**
     * @brief Get the Component Thresholds object
     * 
     * @param component 
     * @return const ThresholdValues& 
     */
    const ThresholdValues &GetComponentThresholds(const Components component) override;

    EnergyMonitorMediator(std::shared_ptr<spdlog::logger> logger);
    ~EnergyMonitorMediator();

private:
    Missions m_MissionPossible;
    std::unordered_map<DataMonitoringType, float> m_data_monitoring_type;
    std::shared_ptr<spdlog::logger> m_logger;
};
