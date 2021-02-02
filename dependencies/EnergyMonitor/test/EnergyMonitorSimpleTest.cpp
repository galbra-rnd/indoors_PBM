#include "gtest/gtest.h"
#include <memory>
#include "EnergyMonitor.hpp"

TEST(EnergyMonitorTest, FirstInitialisation)
{
  std::shared_ptr<IEnergyMonitor> em = std::make_shared<EnergyMonitor>();
}

/**
 * @brief Basic Test.
 * Test if Misions initiated empty.
 * 
 */
TEST(EnergyMonitorTest, GetMissions)
{
  std::shared_ptr<IEnergyMonitor> em = std::make_shared<EnergyMonitor>();
  auto missions = em->GetMissions();

  if (missions.missions.size() > 0)
  {
    auto mission = missions.missions[0];

    auto mission_name = mission_to_string(mission.mission);      /**< Translate to KEY in ROS_DIAGNOSTIC_STATUS_MESSAGE  */
    auto mission_state = mission_state_to_string(mission.state); /**< Translate to VALUE in ROS_DIAGNOSTIC_STATUS_MESSAGE  */
  }

  EXPECT_EQ(missions.missions.size(), 0);
}

/**
 * @brief Set a battery estimation
 * 
 */
TEST(EnergyMonitorTest, SetBatteryEstimation)
{
  // Setup
  std::shared_ptr<IEnergyMonitor> em = std::make_shared<EnergyMonitor>();
  float estimation_in_sec = 350;

  // Run
  EXPECT_TRUE(em->SetBatteryEstimation(estimation_in_sec));
}

/**
 * @test SetBADBatteryEstimation 
 * Set a BAD battery estimation
 * 
 */
TEST(EnergyMonitorTest, SetBADBatteryEstimation)
{
  // Setup
  std::shared_ptr<IEnergyMonitor> em = std::make_shared<EnergyMonitor>();
  float bad_estimation_in_sec = -1;

  // Run
  EXPECT_FALSE(em->SetBatteryEstimation(bad_estimation_in_sec));
}

/**
 * @test LoadMissions.
 * Test if loading missions working properly.
 * 
 * Setup: \n
 * * Create a missions vector.
 * * Initiate IEnergyMonitor
 * 
 * Run: \n
 * * Load missions into IEnergyMonitor.
 * 
 * Test: \n
 * * see if loaded missions apears in IEnergyMonitorMediator
 */
TEST(EnergyMonitorTest, LoadMissions)
{
  // Setup
  std::shared_ptr<IEnergyMonitor> em = std::make_shared<EnergyMonitor>();
  std::vector<MissionsAvailable> missions{MissionsAvailable::FLY};

  // Run
  em->LoadMissions(missions);

  // Test
  bool is_found = false;
  auto loaded_missions = em->GetMissions();
  for (auto &mission : loaded_missions.missions)
  {
    if (mission.mission == MissionsAvailable::FLY)
    {
      ASSERT_EQ(mission.state, MISSION::GREEN);
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
