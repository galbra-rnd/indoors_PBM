#pragma once

#include "gmock/gmock.h"
#include "EnergyMonitorMediator/EnergyMonitorMediator.hpp"
class MockEnergyMonitorMediator : public EnergyMonitorMediator
{
public:
  /**
 * @brief Construct a new mock method0 object    
 * 
 * void LoadMissions(const std::vector<MissionsAvailable> &missions_to_load) override;
 * void SetMissionAvailability(MissionsAvailable mission, MISSION status) override;
 * bool SetBatteryEstimation(float estimation) override;
 * bool SetTimeToHomeEstimation(float estimation) override;
 * Missions &GetMissionsData() override;
 * float GetTimeEstimation(DataMonitoringType data_type) override;
 */
  MOCK_METHOD(void, LoadMissions, (const std::vector<MissionsAvailable> &missions_to_load));
  // MOCK_METHOD1(LoadMissions, void(const std::vector<MissionsAvailable> &missions_to_load));
  // MOCK_METHOD2(SetMissionAvailability, void(MissionsAvailable mission, MISSION status));
  MOCK_METHOD(void, SetMissionAvailability, (MissionsAvailable mission, MISSION status));
  // MOCK_METHOD1(SetBatteryEstimation, bool(float estimation));
  MOCK_METHOD(bool, SetBatteryEstimation, (float estimation));
  // MOCK_METHOD1(SetTimeToHomeEstimation, bool(float estimation));
  MOCK_METHOD(bool, SetTimeToHomeEstimation, (float estimation));
  // MOCK_METHOD0(GetMissionsData, Missions &());
  MOCK_METHOD(Missions &, GetMissionsData, ());
  // MOCK_METHOD1(GetTimeEstimation, float(DataMonitoringType data_type));
  MOCK_METHOD(float, GetTimeEstimation, (DataMonitoringType data_type));

private:
  float _mock_estimation;
};