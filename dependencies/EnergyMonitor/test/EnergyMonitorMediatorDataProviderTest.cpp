#include "gtest/gtest.h"
#include <memory>
#include "EnergyMonitor.hpp"

/** 
 * EnergyMonitorMediatorDataManagerTest test the IEnergyMonitorMediatorDataProvider.
 * An interface EnergyMonitorMediator expose for the Behavior trees.
 */

/**
 * @test Test Behavior trees interface.
 * This simple test has two parts: \n
 * Test:
 * * Insert new data using the IEnergyMonitorMediator interface. This interface will be exposed to EnergyMonitor. Through it, new data is inserted.
 * * Perform some evaluation inside a TreeNode, and set new mission status using the IEnergyMonitorMediatorDataProvider interface, that EnergyMonitor expose to Behavior trees.
 * 
 */
TEST(EnergyMonitorMediatorDataManagerTest, GetBatteryTimeEstimation)
{
  // Setup
  std::shared_ptr<IEnergyMonitorMediator> em = std::make_shared<EnergyMonitorMediator>(spdlog::basic_logger_mt("EnergyMonitorMediator", "/home/msi/logs/gal_temp2-log.txt"));
  std::vector<MissionsAvailable> new_missions{MissionsAvailable::CAN_FLY, MissionsAvailable::GO_HOME};

  float estimation = 249;
  float DUMMY__MIN_BATTERY_TIME_LEFT_IN_SECS_THRESHOLD = 250;

  // Run
  em->LoadMissions(new_missions);
  em->SetBatteryEstimation(estimation); /** @brief Insert new data. Could be ros data. */

  /**
   * @brief This getter has IEnergyMonitorMediatorDataProvider of EnergyMonitorMediator. It is for Behavior trees.
   * 
   */
  auto getter = std::dynamic_pointer_cast<IEnergyMonitorMediatorDataProvider>(em);

  /**
   * @brief Simple evaluation
   * 
   */
  auto curr_batt_est = getter->GetTimeEstimation(DataMonitoringType::BatteryTimeLeft);
  if (curr_batt_est < DUMMY__MIN_BATTERY_TIME_LEFT_IN_SECS_THRESHOLD)
  {
    getter->SetMissionAvailability(MissionsAvailable::CAN_FLY, MISSION::RED);
  }

  // Test
  bool is_found = false;
  auto missions = em->GetMissionsData();
  for (auto &mission : missions.missions)
  {
    if (mission.mission == MissionsAvailable::GO_HOME)
    {
      ASSERT_EQ(mission.state, MISSION::GREEN);
    }
    if (mission.mission == MissionsAvailable::CAN_FLY)
    {
      ASSERT_EQ(mission.state, MISSION::RED);
      is_found = true;
    }
  }
  ASSERT_TRUE(is_found) << "MESSAGE: If false, the No such mission in side mession mediator";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
